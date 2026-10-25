# RTA: what WING-Edit expects (T084)

Status: T084 output, 2026-10-26. Source: static analysis of WING-Edit 3.3.4 (arm64 slice of
`WING-Edit.app/Contents/MacOS/WING-Edit`, symbols present), disassembled with capstone and lief.
WING-Edit was never run natively or modified. Confidence: [H] high, [M] medium, [L] guess.

## 1. "RTA in use! Check mixer settings." (root cause) [H]

The three RTA messages are referenced only from `gui_modules::RtaGraph::paint`
(`0x1000e38a0`): `"RTA in use! Check mixer settings."` (xref `0x1000e3b40`), `"RTA in use on another
window!"` (`0x1000e3ad4`) and `"RTA in use! Select INSERTED channel."` (`0x1000e4524`).

`paint` reads the `cfg/rta` node (`AE->cfg [+0x110] -> rta [+0xd8]`). Its children are stored in schema
order at `+0xa8 + 8k`: `$src $tap $dec $det rtasrc rtatap rtadecay rtadet rtarange rtagain rtaauto ...`.
For each it reads the parameter's position (`Cpar +0xac`; `Cpar::setPos` compares against it and
`Cpar::recalcFIB` writes it: position = value - min for ints, index for enums):

```
if !( pos($src) == pos(rtasrc) - 1      // $src (1..76) == rtasrc (0..76)
      && pos($tap) == pos(rtatap)
      && pos($dec) == pos(rtadecay) )
    draw "RTA in use! Check mixer settings."
else if the graph's own (src, tap) != (rtasrc, rtatap)
    draw "RTA in use on another window!" / "... Select INSERTED channel." (FX RTA)
else
    draw the spectrum
```

- `rtasrc`/`rtatap` are written by WING-Edit itself: `RtaGraph::listener_callback` calls
  `Cpar::setInt` on `rtasrc` with the shown strip + 1 and on `rtatap` with the tap its page uses
  (`$ctl/cfg/rta/cheqtap` PRE EQ/POST EQ etc.); `setFxRta` sets `rtasrc` = FX slot, `rtatap` = FXIN.
- `$src $tap $dec $det` are console-owned, read-only status leaves (`$` prefix; not pushed by
  WING-Edit's sync). A console reports the RTA it actually runs there.

**Root cause:** the simulator stored `cfg/rta/$src/$tap/$dec/$det` like any parameter and never
updated them, so they kept their power-on values (`$src` 1, `$dec` SLOW) while `rtasrc`/`rtadecay`
said otherwise (`rtadecay` defaults to MED, so the check failed even before WING-Edit selected a
strip). No meter report or `$`-node availability flag is involved in this message.

**Fix (T084):** `apps/wing-sim/src/rta.rs` `sync_status` mirrors `rtasrc -> $src` (when > 0),
`rtatap -> $tap`, `rtadecay -> $dec`, `rtadet -> $det` (PEAK/RMS only; AVG -> RMS) with the
simulator's origin, so every client gets the change events. It runs every 50 ms on the RTA thread.

## 2. RTA meter data [M]

`nrp::MeterHandler::sendMeterRequest` builds `D4 id DC <type idx...> DE` from its slot table: type
tokens `A0+n` (V2 strip types `AB..AF` for n <= 4), RTA = `AA` index 0. `MeterHandler::rxMeters`
(`0x1000d72d4`) handles type `AA` by copying **120 int16 words** (big-endian, as every meter word)
into the RTA buffer:

- if `pos($det) > 0` (not PEAK) every word is doubled (`shl #1`), else copied as is;
- the auto-gain pass (`rtaauto` on) then computes `max_k(word_k * s + 6 + gain)` with `s = 1/256`
  when `rtadet` is not PEAK (doubled words) and `s = 1/128` for PEAK.

Both paths read a received word as **1/128 dB** (`wing_proto::nrp::meter::RTA_UNITS_PER_DB`), unlike
the 1/256 dB level meters. [M]: inferred from code, not confirmed with a console capture.

Band layout (protocol.md §4.2): semitones from E0 (20.6 Hz) to D#10 (19.9 kHz), A4 = index 53.
The packet rate is the meter transport's (50 ms). The display range (`rtarange` 30/60 dB), gain
(`rtagain`) and auto gain are applied by WING-Edit, not by the console.

## 3. Simulator implementation

- RT: `wing_engine::rta::RtaFeed` (requested/latched selection, lock-free overwrite ring of the
  mono sum). The driver thread latches the selection once per cycle; the selected strip's
  `RtaProbe` (channel/aux/bus/main/matrix: IN, FILT, PRE EQ, POST EQ, keys, PRE FDR, PRE TAP,
  bus/main/matrix POST) or the console's port capture (ch/aux POST, SOLO = monitor 1 solo bus,
  MON.PH/MON.SPK = monitor 1/2 outputs, FXIN = FX slot input) pushes its block. No allocation.
- Non-RT: `wing_engine::rta::RtaAnalyser` (120 1/12-octave 4th-order band-passes, PEAK/RMS/AVG,
  SLOW/MED/FAST decay 6/12/24 dB/s [L]) on the `wing-rta` thread, which writes the `AA` block
  of the meter bank every 50 ms. Cost: ~5.5 us per 64-sample block (~0.4 % of a core at 48 kHz).
- Gaps: FXOUT is not modelled (silence); WING's exact filter shape, AVG detector and decay rates
  are unknown; `$src` cannot express `rtasrc` = 0 (left unchanged).
