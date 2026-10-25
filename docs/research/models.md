# WING processing-block model catalog (T004)

Status: research output for T004 (2026-10-25). Companion: `docs/research/signal-flow.md`.
Scope: every processing block of the WING (filter + filter tool, input gain stage, gate/expander, EQ, dynamics,
inserts, FX rack) and every selectable model of each, with snapshot identifiers, parameter keys, ranges/units and the
behavioural notes needed to emulate them.

## 0. Sources and method (read this first)

| Source | What it gave | Confidence |
|---|---|---|
| **WING-Edit 3.3.4** (`~/Downloads/WING-Edit.app`, x86_64 slice, read-only static analysis) | Model tables (ID, UI name, internal C++ name, availability flags) located as pointer records in `__DATA,__const`; per-model parameter definitions recovered by walking each model's `*_pardef_*` function (otool disassembly): key, UI label, min/step/count, scale (lin/log/int/enum), unit, enum value lists; per-model default strings (`thr=-40,range=40,...`). | High for IDs/keys/enums/ranges. Ranges are what the editor enforces; display tapers are inferred from (min, step, count, lin/log flag). |
| `fixtures/PUBLIC.snap` (console fw **1.13.9**, `snapshot.8`) | Exact JSON paths/keys/value formats actually stored by a console; which models were in use (gate: GATE, WARM, DEQ, DUCK, PSE; dyn: COMP, 76LA, NSTR, F670; aux combo; EQ: STD only; FX: 11 models). | High for fw 1.13; keys of later models are absent. |
| Release notes / reviews (fw 1.12.4, 3.0.5, 3.1) via web: [Drew Brashler fw 3.1 review](https://drewbrashler.com/2025/behringer-wing-firmware-3-1-dynamics-update/), [fw 3.1 newsletter](https://newsletter.drewbrashler.com/p/the-wing-just-got-a-massive-free-dynamics-upgrade), search summaries of the 3.0.5 release notes and behringer.world fw 1.12.4 thread | Firmware history of model additions and the 3.1 "all dynamics in both slots" change. | Medium (secondary sources; the official PDFs on cdn.mediavalet.com returned HTTP 404/500 during this session). |
| Hardware knowledge of the referenced units | "Emulates", behaviour notes, emulation approach. | Medium; every curve/time constant must be validated by measurement in T072. |

The Behringer WING user manual PDF available via web fetch was image-only (no text layer) and the official
"WING Remote Protocols" PDF (P.-G. Maillot) failed to download (HTTP 500), so nothing here is quoted from them.
Re-check the items marked **verify** against those documents when accessible.

Reproduction (all scripts live in the session scratchpad, not committed; T003 owns the committed extractor):
`lipo -thin x86_64` -> `otool -tvV` -> find `(char* id, char* name, int idx, fnptr...)` records in `__DATA,__const` ->
for each model record follow fn[0] (`ae::<blk>_pardef_<name>(ncl::Cpar**, int&)`) and decode the `ncl::Cpar` field
stores: `+0x48` key, `+0x50` label, `+0x5c` flags, `+0xc8` (float min, float step) or int min, `+0xd0` count,
`+0xd8` enum string/float table, `+0xe8` printf format, `+0xf0/+0xf8` end labels.

### 0.1 Parameter flag encoding (WING-Edit `ncl::Cpar` +0x5c, bits 16..27 = `0x4TU`)

- `T` (scale): 1 = linear float (max = min + step*(count-1)), 2 = logarithmic float (max = min*exp(step*(count-1))),
  3 = fader-law level (0..1023 positions, -oo..+10 dB), 4 = integer/bool (min..min+count-1), 5 = enum (string table),
  6 = enum of floats (e.g. ratios).
- `U` (unit): 0 none (knob position), 1 dB, 2 %, 3 ms, 4 Hz, 5 m (distance), 6 s.
- Low 16 bits: UI page/group bits (e.g. 0x10/0x20/0x40 = DEQ band 1/2/3 pages), not needed for DSP.
- Snapshot values are stored in **engineering units** (dB, Hz, ms, %), enums as their string value, booleans as JSON
  booleans; the editor's step grid is only a UI quantisation (snapshot floats such as `rel: 247.0345306` lie on the log
  grid). The engine must accept any value inside the range and clamp.

### 0.2 Conventions in the tables below

`key` = JSON key in the snapshot object of that block (e.g. `ae_data.ch.<n>.gate.<key>`, `...dyn.<key>`,
`...eq.<key>`, `ae_data.fx.<n>.<key>`) - identical to the WING-Edit parameter key. `default` = WING-Edit default
string for that model (blank if none was found). Keys starting with `$` are UI/transient (not stored).
Common wrapper keys not repeated per model: gate block `{on, mdl}`; dyn block `{on, mdl, mix 0..100 %, gain
-6..+12 dB (37 steps)}`; EQ block `{on, mdl, mix 0..125 %}` (note: up to 125 % = exaggerated EQ); FX slot
`{mdl, fxmix 0..100 %}` (ranges confirmed in T003 `wing-tree.json`).

## 1. Counts

| Block | Models | Notes |
|---|---|---|
| Filter (LC/HC) | 1 (standard) | LC slope 6/12/18/24 dB/oct, HC slope 6/12 dB/oct |
| Filter tool (`flt.mdl`) | 4 | TILT, MAX, AP1, AP2 |
| Input gain stage | 1 | preamp gain lives on the *source* (`io.in.<grp>.<n>.g`), channel has trim/balance/polarity/delay |
| Gate + dynamics (shared table) | 32 selectable (+1 hidden) | 13 "gate-family" (idx 0-12) + 19 "compressor-family" (idx 13-31) selectable in ch gate, ch dyn and bus/main/mtx dyn; aux dyn: 28 (no DS902, DEQ, DEQ2, WARM). Table idx 32 `3CMP` exists in WING-Edit but is not offered by any selector (T003 variants) |
| EQ | 9 distinct | ch: 7 (STD, SOUL, E88, E84, F110, PULSAR, MACH4); aux: 6 (no MACH4); bus/main/mtx: 7 (STD-bus, SOUL, E88, E84, F110, PULSAR, PIA) |
| Inserts | 2 insert points per ch/bus/main/mtx (1 on aux) | an insert hosts an FX rack slot (FX1..FX16); post-insert on channels also offers automix X/Y |
| FX rack | 64 (+ NONE) | 16 slots; FX1-8 offer all 64 (+NONE); FX9-16 offer 38 (+NONE): the 26 models HALL..BPLATE (reverbs, modulation, delays, pitch, VSS3) are FX1-8 only (T003 selector lists) |

## 2. Filter block (channels only)

Snapshot path `ae_data.ch.<n>.flt` (fw 1.13 keys: `lc, lcf, hc, hcf, tf, mdl, tilt|low,proc`). Recovered from
`ae::Cchannel` constructor and `ae::Cchfilter::update_direct` in WING-Edit 3.3.4:

| key | UI label | range | unit | notes |
|---|---|---|---|---|
| `lc` | LOW CUT | off/on | | |
| `lcf` | LC FREQ | 20 .. 2000 (log, 641 steps; T003) | Hz | |
| `lcs` | LC SLOPE | 6 / 12 / 18 / 24 | dB/oct | not in fw 1.13 fixture (added later); factory default 24 (T003, fw 3.0.6 factory scene) - a fw 1.13 snapshot without `lcs` should load as 12 (X32-lineage fixed slope, **verify**) |
| `hc` | HIGH CUT | off/on | | |
| `hcf` | HC FREQ | 50 .. 20000 (log, 833 steps) | Hz | |
| `hcs` | HC SLOPE | 6 / 12 | dB/oct | not in fw 1.13 fixture; factory default 12 |
| `tf` | TOOL FILT | off/on | | enables the filter tool |
| `mdl` | TOOL TYPE | TILT / MAX / AP1 / AP2 | | UI: "TILT EQ", "MAXER", "AP 90", "AP 180" |

Filter-tool models:

#### TILT EQ - `TILT`
- Param: `tilt` -6..+6 dB (lin). Behaviour: complementary low/high shelving pair around a fixed pivot (pivot not
  recovered; ~1 kHz typical, **verify**), positive = brighter.
- Emulation approach: 1st-order tilt (two matched shelves). Difficulty **S**.

#### MAXER - `MAX`
- Params: `low` (LOW CONT 0..100), `proc` (HIGH PROC 0..100). Fixture: low 30/40, proc 20/60.
- Emulates: **BBE Sonic Maximizer** (Lo Contour / Process knobs). BBE: 3-band split; low band delayed relative to
  the high band (phase/time alignment ~2.5 ms LF vs HF), Lo Contour = LF shelf boost (~50 Hz), Process = HF boost
  (~5 kHz) via a level-dependent VCA in the original.
- Emulation approach: allpass/delay-based band realignment + LF/HF shelving gains mapped from knobs. Difficulty **M**.

#### AP 90 - `AP1`
- Param: `f` 100..10000 Hz (log). First-order all-pass (90 deg at f), used for phase alignment.
- Emulation approach: 1st-order allpass. Difficulty **S**.

#### AP 180 - `AP2`
- Params: `f` 100..10000 Hz (log), `q` 0.5..5 (lin). Second-order all-pass (180 deg at f).
- Emulation approach: 2nd-order allpass biquad. Difficulty **S**.

Standard LC/HC: Butterworth-type high-pass/low-pass of the selected order is the working assumption (**verify** by
measurement; X32 lineage used 12 dB/oct for LC only). Emulation: cascaded biquads (+1st-order section for odd
orders), Difficulty **S**.

### 2.1 Input stage parameters (channel `in.set`, source `io.in.<grp>.<n>`)

| path | key | range | unit | notes |
|---|---|---|---|---|
| source | `g` | -3 .. 45.5 (local/AES50 preamp gain; fixture values 0..30.5) | dB | analog preamp - in the simulator a pure digital gain |
| source | `vph`, `pol`, `mute`, `mode` (M / ST / M-S) | | | phantom (no-op), polarity, source mute, mono/stereo/mid-side source |
| ch `in.set` | `trim` | -18 .. +18 | dB | digital trim |
| ch `in.set` | `bal` | -9 .. +9 | dB | stereo balance |
| ch `in.set` | `inv` | off/on | | polarity |
| ch `in.set` | `dlyon`, `dly`, `dlymode` | per mode (T003): M 0.1..150 m, FT 0.5..500 ft, MS 0.5..500 ms, SMP 16..500 samples | m/ft/ms/samples | input delay (distance or time) |
| ch `in.set` | `altsrc`, `srcauto` | | | use alternate source; follow global alt switch |

Emulation: gains as smoothed multipliers, delay as integer+fractional delay line (max 500 ms = 24000 samples @48k; 150 m at
343 m/s = 437 ms). Difficulty **S**.

## 3. Gate and dynamics models (one shared model table)

Snapshot paths: `ae_data.ch.<n>.gate` (gate slot, wrapper `{on, mdl}`), `ae_data.ch.<n>.dyn` (dynamics slot, wrapper
`{on, mdl, mix, gain}`), `ae_data.{bus,main,mtx}.<n>.dyn`, `ae_data.aux.<n>.dyn`. Gate side-chain `gatesc`, dynamics
side-chain `dynsc` and dynamics crossover `dynxo` are described in signal-flow.md section 4.

WING-Edit keeps **one** model table for both slots (records `ae::gate_*`, `ae::dyn_*`, `ae::comp_*`). Each record
carries an availability descriptor; the selector lists recovered by T003 (`variants[...]['mdl']`) settle what is
selectable:

- ch `gate`, ch `dyn`, bus/main/mtx `dyn`: the 32 models idx 0-31 (fw 3.x "unidyn": every model in both slots).
- aux `dyn`: 28 models (no DS902, DEQ, DEQ2, WARM). This matches byte 7 of the flag word (`bytes4-7` last element = 0
  for exactly those four), so bytes 4-7 are most likely per-strip-class enables (..., aux).
- `3CMP` (idx 32) has bytes 4-7 all zero and is in no selector: hidden/unreleased in WING-Edit 3.3.4.
- `slot mask` (byte 0) meaning is still unknown (0/1/3/5/7 pattern; perhaps detector/side-chain capabilities or the
  pre-3.x slot assignment). Not needed for emulation.

Firmware history (secondary sources): fw 1.12.4 added the PIA2250 compressor and the aux combo dynamics; fw 3.0.5
added the SOURCE EXTRACTOR/LA LEVELER combo for all channels and in the dynamics slot; fw 3.1 (Nov 2025) added EVEN 88
COMP, LMT COMPRESSOR, ONE KNOB COMP, DUAL DYNAMIC EQ and made every gate/comp model loadable in both slots. Models not
in the fw 1.13.9 fixture and not dated by a source are marked `<=3.0` (present in WING-Edit 3.3.4).

Notes on snapshot value formats (fw 1.13): gate `ratio` stored as string `"1:3"`; 76LA `ratio` as string `"4"`; NSTR
`ratio` as `"2:1"`; COMP `ratio` as number `3`; enum-of-float values (ratios) are therefore stored as **numbers** for
`T=6` params and as **strings** for `T=5` params.


| idx | `mdl` | UI name | internal | ch gate/dyn, bus/main/mtx dyn | aux dyn | byte0 | bytes4-7 | fw |
|---|---|---|---|---|---|---|---|---|
| 0 | `GATE` | WING GATE | `gate_pardef_xseries_gate` | yes | yes | 3 | [1, 1, 1, 1] | <=3.0 |
| 1 | `DUCK` | WING DUCKER | `gate_pardef_xseries_ducker` | yes | yes | 3 | [1, 1, 1, 1] | <=3.0 |
| 2 | `E88` | EVEN 88 GATE | `gate_pardef_neve88rs` | yes | yes | 3 | [1, 1, 1, 1] | <=3.0 |
| 4 | `9000G` | SOUL 9000 GATE | `gate_pardef_ssl9000j` | yes | yes | 3 | [1, 1, 1, 1] | <=3.0 |
| 3 | `D241G` | DRAW MORE 241 | `gate_pardef_drawmer241` | yes | yes | 3 | [1, 1, 1, 1] | <=3.0 |
| 8 | `DS902` | BDX902 DEESSER | `dyn_pardef_dbx902` | yes | no | 0 | [1, 1, 1, 0] | <=3.0 |
| 5 | `DEQ` | DYNAMIC EQ | `dyn_pardef_dyneq` | yes | no | 1 | [1, 1, 1, 0] | <=3.0 |
| 6 | `DEQ2` | DUAL DYNAMIC EQ | `dyn_pardef_dyneq2` | yes | no | 1 | [1, 1, 1, 0] | 3.1 |
| 11 | `WAVE` | WAVE DESIGNER | `dyn_pardef_transient` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 10 | `PSE` | SOURCE EXTRACTOR | `dyn_pardef_pse545` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 9 | `CMB` | PSE/LA COMBO | `dyn_pardef_combo` | yes | yes | 5 | [1, 1, 1, 1] | <=3.0 |
| 12 | `RIDE` | AUTO RIDER | `dyn_pardef_autogain` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 7 | `WARM` | SOUL WARMTH PRE | `dyn_pardef_warmth` | yes | no | 0 | [1, 1, 1, 0] | <=3.0 |
| 13 | `COMP` | WING COMPRESSOR | `comp_pardef_xseries_comp` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 14 | `EXP` | WING EXPANDER | `comp_pardef_xseries_exp` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 15 | `B160` | BDX 160 COMP | `comp_pardef_dbx160` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 16 | `B560` | BDX 560 EASY | `comp_pardef_dbx560` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 17 | `D241C` | DRAW MORE COMP | `comp_pardef_d241c` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 18 | `ECL33` | EVEN COMP/LIM | `comp_pardef_n33609` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 19 | `9000C` | SOUL 9000 | `comp_pardef_ssl9000` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 20 | `SBUS` | SOUL BUS COMP | `comp_pardef_sslgbus` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 21 | `RED3` | RED3 COMPRESSOR | `comp_pardef_d3` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 22 | `76LA` | 76 LIMITER AMP | `comp_pardef_u1176` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 23 | `LA` | LA LEVELER | `comp_pardef_la2a` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 24 | `F670` | FAIR KID | `comp_pardef_f670` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 25 | `BLISS` | ETERNAL BLISS | `comp_pardef_mpress` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 26 | `NSTR` | NO-STRESSOR | `comp_pardef_distress` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 27 | `2250` | PIA2250 RACK | `comp_pardef_api225l` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 28 | `L100` | LTA100 LEVELER | `comp_pardef_tla100a` | yes | yes | 7 | [1, 1, 1, 1] | <=3.0 |
| 29 | `E88C` | EVEN 88 COMP | `comp_pardef_neve88rs` | yes | yes | 7 | [1, 1, 1, 1] | 3.1 |
| 30 | `LMT` | LMT COMPRESSOR | `comp_pardef_lmcplus` | yes | yes | 7 | [1, 1, 1, 1] | 3.1 |
| 31 | `ONEC` | ONE KNOB COMP | `comp_pardef_oneknob` | yes | yes | 7 | [1, 1, 1, 1] | 3.1 |
| 32 | `3CMP` | THREE BAND COMP | `comp_pardef_triband` | no | no | 7 | [0, 0, 0, 0] | hidden (in WING-Edit 3.3.4 table, in no selector) |

### 3.1 Gate-family models (table indices 0-12)

#### WING GATE - `GATE`

- **UI name:** WING GATE  |  **snapshot `mdl`:** `GATE`  |  **WING-Edit internal:** `gate_pardef_xseries_gate`
- **Emulates:** Behringer X32 / WING native gate-expander (internal `xseries_gate`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.gate
- **Behaviour:** Downward expander/gate. `ratio` 1:1.5..1:4 = expander slope below threshold, `gate` = hard gate. `range` = max attenuation (3..60 dB; the top end is labelled "oo dB" = full mute). Attack 0..120 ms (linear), hold 1..1000 ms, release 4..4000 ms (log). `acc` (ACCENT 0..100 %) is a WING addition: hysteresis/transient emphasis on opening, exact law unknown. Detector keyed from the gate side-chain (`gatesc`: filter/source/tap). Standard X32 behaviour: peak detector, open/hold/release state machine, gain smoothing in dB domain.
- **Emulation approach:** Own peak detector + gate state machine (attack ramp, hold counter, exponential release) in log domain; expander gain = (1-ratio)*(thr-level) clamped to -range; acc = threshold hysteresis (open at thr, close at thr-acc*k) until measured.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -80 .. 0 | dB | lin | -40 |
| `range` | RANGE | 3 .. 60 (end label "oo dB") | dB | lin | 40 |
| `att` | ATTACK | 0 .. 120 | ms | lin | 10 |
| `hld` | HOLD | 1 .. 1000 | ms | log | 10 |
| `rel` | RELEASE | 4 .. 4000 | ms | log | 200 |
| `acc` | ACCENT | 0 .. 100 | % | lin | 0 |
| `ratio` | RATIO | 1:1.5 / 1:2 / 1:3 / 1:4 / gate |  | enum | 1:3 |

#### WING DUCKER - `DUCK`

- **UI name:** WING DUCKER  |  **snapshot `mdl`:** `DUCK`  |  **WING-Edit internal:** `gate_pardef_xseries_ducker`
- **Emulates:** X32/WING native ducker (`xseries_ducker`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.gate
- **Behaviour:** Gain is reduced by `range` while the KEY signal (normally an external side-chain source) is above `thr`. Attack/hold/release envelope as GATE; hold range extracted as 30..30000 (log, display "%.0f") - probably ms (30 ms..30 s), verify against console UI. Snapshot fixture uses `rel` 8.02 (value outside 4..4000 if ms: indicates units/taper differ - verify).
- **Emulation approach:** Re-use GATE envelope with inverted sense (attenuate when key above threshold); side-chain source from `gatesc.src`.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -80 .. 0 | dB | lin | -20 |
| `range` | RANGE | 3 .. 60 (end label "oo dB") | dB | lin | 20 |
| `att` | ATTACK | 0 .. 120 | ms | lin | 100 |
| `hld` | HOLD | 30 .. 30000 | ms | log | 100 |
| `rel` | RELEASE | 4 .. 4000 | ms | log | 500 |

#### EVEN 88 GATE - `E88`

- **UI name:** EVEN 88 GATE  |  **snapshot `mdl`:** `E88`  |  **WING-Edit internal:** `gate_pardef_neve88rs`
- **Emulates:** Neve 88RS console channel gate/expander (`neve88rs`)
- **Behaviour:** Threshold -40..0 dB (switch `m40` shifts the threshold range down by 40 dB), hysteresis 0..25 dB (UI end label "EXP"), range 0..60 dB, release 100..3000 ms, `fast` attack switch (88RS: 50 us fast vs ~2 ms). No hold control. Hysteresis = open/close threshold gap, characteristic of the 88RS gate.
- **Emulation approach:** Own gate with hysteresis comparator, fixed attack (fast/slow), exponential release; soft-ish opening curve fitted to 88RS documentation.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -40 .. 0 | dB | lin | -35 |
| `hyst` | HYST | 0 .. 25 (end label "EXP") | dB | lin | 6 |
| `range` | RANGE | 0 .. 60 | dB | lin | 60 |
| `rel` | RELEASE | 100 .. 3000 | ms | log | 350 |
| `fast` | FAST | off/on |  | bool | 0 |
| `m40` | THR -40 | off/on |  | bool | 0 |

#### SOUL 9000 GATE - `9000G`

- **UI name:** SOUL 9000 GATE  |  **snapshot `mdl`:** `9000G`  |  **WING-Edit internal:** `gate_pardef_ssl9000j`
- **Emulates:** SSL 9000 J channel gate/expander (`ssl9000j`)
- **Behaviour:** SSL E/G/J-series dynamics gate section: threshold -40..0, range 0..40 dB, hold 10..4000 ms (extracted range; SSL hold 0..4 s), release 100..4000 ms, `fast` attack (SSL: fast ~100 us vs ~1.5 ms), `mode` GATE (hard, range-limited) or EXP (expander ~1:2 slope).
- **Emulation approach:** Own expander/gate; EXP mode = fixed 2:1 downward expander below threshold, limited by range.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -40 .. 0 | dB | lin | -35 |
| `range` | RANGE | 0 .. 40 | dB | lin | 40 |
| `hld` | HOLD | 10 .. 4000 | ms | log | 50 |
| `rel` | RELEASE | 100 .. 4000 | ms | log | 800 |
| `fast` | FAST | off/on |  | bool | 0 |
| `mode` | MODE | GATE / EXP |  | enum | gate |

#### DRAW MORE 241 - `D241G`

- **UI name:** DRAW MORE 241  |  **snapshot `mdl`:** `D241G`  |  **WING-Edit internal:** `gate_pardef_drawmer241`
- **Emulates:** Drawmer DL241 auto-compressor - gate section (`drawmer241`)
- **Behaviour:** Only threshold (-80..0 dB) and a `slow` switch. DL241 gate: fixed fast attack, programme-dependent release; slow switch lengthens release. Hard gate (full attenuation).
- **Emulation approach:** Own gate with two fixed time-constant sets; programme-dependent release approximated by dual-rate release.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -80 .. 0 | dB | lin | -40 |
| `slow` | SLOW | off/on |  | bool | 0 |

#### BDX902 DEESSER - `DS902`

- **UI name:** BDX902 DEESSER  |  **snapshot `mdl`:** `DS902`  |  **WING-Edit internal:** `dyn_pardef_dbx902`
- **Emulates:** dbx 902 de-esser (`dbx902`)
- **Behaviour:** Frequency 800..8000 Hz, range 0..12 dB (max reduction), mode FULL (wideband gain reduction keyed by HF) or HF (only the band above f is reduced). dbx 902 uses an RMS-ish HF detector relative to broadband level (ratio-free, "range" caps reduction).
- **Emulation approach:** Side-chain HP/BP at f, detector compares HF vs total energy; apply wideband gain or split-band (complementary crossover) reduction.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `f` | FREQ | 800 .. 8000 | Hz | log | 5k |
| `range` | RANGE | 0 .. 12 | dB | lin | 4 |
| `mode` | MODE | FULL / HF |  | enum | hf |

#### DYNAMIC EQ - `DEQ`

- **UI name:** DYNAMIC EQ  |  **snapshot `mdl`:** `DEQ`  |  **WING-Edit internal:** `dyn_pardef_dyneq`
- **Emulates:** WING native dynamic EQ band (`dyneq`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.gate
- **Behaviour:** One filter band (`filt` OFF/BP/LP6/LP12/HP6/HP12 = band shape incl. shelves-as-filters) whose gain `g` (-15..+15 dB) is applied dynamically: `mode` high = act when level is ABOVE `thr`, low = BELOW. Ratio enum 1.2..10, attack 0..200 ms, release 20..4000 ms. Side-chain is the same band-filtered signal.
- **Emulation approach:** Band-filtered detector -> gain computer -> modulate a peaking/shelf biquad (or parallel band split + VCA). Recompute coefficients per block with smoothing (or use SVF with continuous gain).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -60 .. 0 | dB | lin | -20 |
| `ratio` | RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `att` | ATTACK | 0 .. 200 | ms | lin | 50 |
| `rel` | RELEASE | 20 .. 4000 | ms | log | 300 |
| `filt` | FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `g` | GAIN | -15 .. 15 | dB | lin | 0 |
| `f` | FREQ | 20 .. 20000 | Hz | log | 1000 |
| `q` | Q | 0.44 .. 10 |  | log | 1 |
| `mode` | MODE | low / high |  | enum | high |

#### DUAL DYNAMIC EQ - `DEQ2`

- **UI name:** DUAL DYNAMIC EQ  |  **snapshot `mdl`:** `DEQ2`  |  **WING-Edit internal:** `dyn_pardef_dyneq2`
- **Emulates:** WING native dual dynamic EQ (`dyneq2`)
- **Firmware:** added/visible in 3.1
- **Behaviour:** Two independent DEQ bands (`1-*`, `2-*`), `$page` is a UI-only page selector (not stored).
- **Emulation approach:** Two DEQ instances in series.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `1-thr` | 1-THR | -60 .. 0 | dB | lin | -20 |
| `1-ratio` | 1-RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `1-att` | 1-ATT | 0 .. 200 | ms | lin | 50 |
| `1-rel` | 1-REL | 20 .. 4000 | ms | log | 300 |
| `1-filt` | 1-FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `1-g` | 1-GAIN | -15 .. 15 | dB | lin | 0 |
| `1-f` | 1-FREQ | 20 .. 20000 | Hz | log | 150 |
| `1-q` | 1-Q | 0.44 .. 10 |  | log | 1 |
| `1-mode` | 1-MODE | low / high |  | enum | high |
| `2-thr` | 2-THR | -60 .. 0 | dB | lin | -20 |
| `2-ratio` | 2-RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `2-att` | 2-ATT | 0 .. 200 | ms | lin | 50 |
| `2-rel` | 2-REL | 20 .. 4000 | ms | log | 300 |
| `2-filt` | 2-FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `2-g` | 2-GAIN | -15 .. 15 | dB | lin | 0 |
| `2-f` | 2-FREQ | 20 .. 20000 | Hz | log | 3000 |
| `2-q` | 2-Q | 0.44 .. 10 |  | log | 1 |
| `2-mode` | 2-MODE | low / high |  | enum | high |
| `$page` | DEQ PAGE | off/on |  | bool |  |

#### WAVE DESIGNER - `WAVE`

- **UI name:** WAVE DESIGNER  |  **snapshot `mdl`:** `WAVE`  |  **WING-Edit internal:** `dyn_pardef_transient`
- **Emulates:** SPL Transient Designer (`transient`) - UI "WAVE DESIGNER"
- **Behaviour:** Level-independent transient shaper: attack -15..+15 dB, sustain -24..+24 dB, output gain -18..+9 dB. SPL design: difference of fast and slow envelope followers drives a VCA (attack), and difference of slow follower vs release-tail follower (sustain).
- **Emulation approach:** Two envelope followers (fast/slow) on rectified signal, attack gain = k_a*(env_fast-env_slow) in dB, sustain gain = k_s*(env_slow_hold - env_fast); fit time constants to SPL docs.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `att` | ATTACK | -15 .. 15 | dB | lin | 0 |
| `sust` | SUSTAIN | -24 .. 24 | dB | lin | 0 |
| `g` | GAIN | -18 .. 9 | dB | lin | 0 |

#### SOURCE EXTRACTOR - `PSE`

- **UI name:** SOURCE EXTRACTOR  |  **snapshot `mdl`:** `PSE`  |  **WING-Edit internal:** `dyn_pardef_pse545`
- **Emulates:** "Source Extractor" downward expander (`pse545`; hardware reference unclear - possibly an API-style 500-series expander, verify)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.gate
- **Behaviour:** thr -36..+12 dB, depth 0..20 dB (max attenuation), `fast` (time constants), `peak` (peak vs RMS detector). Intended for bleed reduction: soft expander, not a hard gate.
- **Emulation approach:** Own soft-knee downward expander (ratio ~1:2..1:3 inferred) limited by depth; fast/peak select detector and ballistics.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -36 .. 12 | dB | lin | 0 |
| `depth` | DEPTH | 0 .. 20 | dB | lin | 12 |
| `fast` | FAST | off/on |  | bool | 0 |
| `peak` | PEAK | off/on |  | bool | 0 |

#### PSE/LA COMBO - `CMB`

- **UI name:** PSE/LA COMBO  |  **snapshot `mdl`:** `CMB`  |  **WING-Edit internal:** `dyn_pardef_combo`
- **Emulates:** PSE + LA-2A combo (`combo`) - the fixed aux-channel dynamics before fw 3.1
- **Behaviour:** Source Extractor (thr/depth/fast/peak) followed by an LA leveler (`ingain` 0..100, `cpeak` PEAK REDUCTION 0..100, `cmode` COMP/LIM). Snapshot aux `dyn` has no `mdl` key in fw 1.13 (fixed model). fw 3.0.5 notes: combo added to the dynamics slot of all channels.
- **Emulation approach:** Compose PSE + LA models.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -36 .. 12 | dB | lin | -36 |
| `depth` | DEPTH | 0 .. 20 | dB | lin | 12 |
| `fast` | FAST | off/on |  | bool | 0 |
| `peak` | PEAK | off/on |  | bool | 0 |
| `ingain` | GAIN | 0 .. 100 |  | lin | 40 |
| `cpeak` | PEAK | 0 .. 100 |  | lin | 0 |
| `cmode` | CMODE | COMP / LIM |  | enum | comp |

#### AUTO RIDER - `RIDE`

- **UI name:** AUTO RIDER  |  **snapshot `mdl`:** `RIDE`  |  **WING-Edit internal:** `dyn_pardef_autogain`
- **Emulates:** Auto rider (`autogain`) - Waves Vocal Rider-style
- **Behaviour:** Slow automatic gain riding toward `tgt` (-48..0 dB) for signal above `thr` (-54..-18 dB, below = no riding), speed `spd` (int, 50 steps), ratio enum 2/4/8/20/100 (how strongly level error is corrected), hold 0.1..10 s (INF at end), range 1..15 dB (max +/- correction).
- **Emulation approach:** Smoothed RMS level -> error to target scaled by (1-1/ratio) -> slew-limited gain within +/-range; freeze when below threshold, hold timer.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -54 .. -18 | dB | lin | -40 |
| `tgt` | TARGET | -48 .. 0 | dB | lin | -18 |
| `spd` | SPEED | 1 .. 50 |  | int | 10 |
| `ratio` | RATIO | 2 / 4 / 8 / 20 / 100 |  | enum | 4 |
| `hld` | HOLD | 0.1 .. 10 (end label "INF") | s | log | 1.5 |
| `range` | RANGE | 1 .. 15 | dB | lin | 6 |

#### SOUL WARMTH PRE - `WARM`

- **UI name:** SOUL WARMTH PRE  |  **snapshot `mdl`:** `WARM`  |  **WING-Edit internal:** `dyn_pardef_warmth`
- **Emulates:** SSL VHD ("Variable Harmonic Drive") preamp - UI "SOUL WARMTH PRE" (`warmth`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.gate
- **Behaviour:** drv 10..125 (%, drive), hrm -100..+100 (blend of 2nd vs 3rd harmonic dominance: SSL VHD sweeps from 2nd to 3rd), col -1..+1 (tone/colour tilt), trim -18..+6 dB, wmix 0..100 % (parallel mix). fw 1.13 snapshot stores the mix as `mix` (not `wmix`): key changed between firmware versions -> loader must alias.
- **Emulation approach:** Static asymmetric+symmetric waveshaper (polynomial blend of even/odd terms controlled by hrm), drive-dependent gain compensation, 2-4x oversampling or ADAA; pre/post tilt filters for col.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `drv` | DRIVE | 10 .. 125 | % | lin | 33 |
| `hrm` | HARM | -100 .. 100 |  | lin | 0 |
| `col` | COLOR | -1 .. 1 |  | lin | 0 |
| `trim` | TRIM | -18 .. 6 | dB | lin | 0 |
| `wmix` | MIX | 0 .. 100 | % | lin | 100 |

### 3.2 Compressor-family models (table indices 13-32)

#### WING COMPRESSOR - `COMP`

- **UI name:** WING COMPRESSOR  |  **snapshot `mdl`:** `COMP`  |  **WING-Edit internal:** `comp_pardef_xseries_comp`
- **Emulates:** Behringer X32 / WING native compressor (`xseries_comp`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at bus.dyn, ch.dyn, main.dyn, mtx.dyn
- **Behaviour:** Feed-forward VCA-style compressor: thr -60..0 dB, ratio enum 1.1..100 (17 values), knee 0..5 (int; X32 knee steps of ~? dB, X32 doc: soft knee width 0-5 -> 0..10 dB is commonly assumed, verify), det PEAK/RMS, attack 0..120 ms, hold 1..200 ms, release 4..4000 ms, env LIN/LOG (release curve linear-in-dB vs logarithmic), auto (programme-dependent attack/release). Wrapper adds `mix` (0..100 % parallel) and `gain` (make-up, -6..+12 dB) and the `dynxo` crossover (frequency-selective compression) and `dynsc` side-chain.
- **Emulation approach:** Own log-domain compressor (Giannoulis/Massberg/Reiss topology): detector (peak or RMS window), static curve with quadratic soft knee, attack/release smoothing in dB, hold; auto = dual release (fast+slow). Reference: X32 manual curves.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -60 .. 0 | dB | lin | -10 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `knee` | KNEE | 0 .. 5 |  | int | 3 |
| `det` | DETECTOR | PEAK / RMS |  | enum | rms |
| `att` | ATTACK | 0 .. 120 | ms | lin | 50 |
| `hld` | HOLD | 1 .. 200 | ms | lin | 20 |
| `rel` | RELEASE | 4 .. 4000 | ms | log | 150 |
| `env` | ENVELOPE | LIN / LOG |  | enum | log |
| `auto` | AUTO | off/on |  | bool | 1 |

#### WING EXPANDER - `EXP`

- **UI name:** WING EXPANDER  |  **snapshot `mdl`:** `EXP`  |  **WING-Edit internal:** `comp_pardef_xseries_exp`
- **Emulates:** X32 / WING native downward expander (`xseries_exp`)
- **Behaviour:** Same parameter set as COMP but expansion below threshold (ratio enum = expansion ratio), hold up to 1000 ms (log).
- **Emulation approach:** Same engine as COMP with expander static curve.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -60 .. 0 | dB | lin | -10 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `knee` | KNEE | 0 .. 5 |  | int | 3 |
| `det` | DETECTOR | PEAK / RMS |  | enum | rms |
| `att` | ATTACK | 0 .. 120 | ms | lin | 50 |
| `hld` | HOLD | 1 .. 1000 | ms | log | 20 |
| `rel` | RELEASE | 4 .. 4000 | ms | log | 150 |
| `env` | ENVELOPE | LIN / LOG |  | enum | log |
| `auto` | AUTO | off/on |  | bool | 1 |

#### BDX 160 COMP - `B160`

- **UI name:** BDX 160 COMP  |  **snapshot `mdl`:** `B160`  |  **WING-Edit internal:** `comp_pardef_dbx160`
- **Emulates:** dbx 160 compressor (`dbx160`) - UI "BDX 160 COMP"
- **Behaviour:** Two controls: threshold (0.01..5, log, display %.2f - original dbx 160 threshold calibrated in volts 10 mV..3 V) and compression ratio (1.1..50, 16 values). dbx 160: true-RMS detector, hard knee, programme-dependent attack/release (~15 ms attack at 10 dB GR, release ~120 dB/s), VCA gain cell, fixed output (make-up via wrapper gain).
- **Emulation approach:** RMS detector (~5-10 ms window) with programme-dependent ballistics fixed by dbx spec (release rate in dB/s), hard-knee static curve; threshold mapping V-rms -> dBu -> dBFS (WING +4 dBu reference to be chosen).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | 0.01 .. 5 |  | log | 0.5 |
| `comp` | COMP | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 |  | enum | 3 |

#### BDX 560 EASY - `B560`

- **UI name:** BDX 560 EASY  |  **snapshot `mdl`:** `B560`  |  **WING-Edit internal:** `comp_pardef_dbx560`
- **Emulates:** dbx 160X/560A "Easy" (`dbx560`) - UI "BDX 560 EASY"
- **Behaviour:** thr -40..+20, ratio enum 1.1..10, 50, 999 (=inf:1) and negative ratios -1..-5 (dbx "beyond infinity" over-compression), `easy` = OverEasy soft knee. (Enum in binary: 1.1,1.2,1.5,2,3,4,5,7,10,50,999,-5,-3,-2,-1.) dbx 560A/160X: RMS detector, programme-dependent time constants.
- **Emulation approach:** As B160 with OverEasy knee (wide soft knee ~ 10-15 dB) and support for ratio<0 (gain decreases above threshold).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -40 .. 20 | dB | lin | 0 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.5 / 2 / 3 / 4 / 5 / 7 / 10 / 50 / 999 / -5 / -3 / -2 / -1 |  | enum | 3 |
| `easy` | EASY | off/on |  | bool | 1 |

#### DRAW MORE COMP - `D241C`

- **UI name:** DRAW MORE COMP  |  **snapshot `mdl`:** `D241C`  |  **WING-Edit internal:** `comp_pardef_d241c`
- **Emulates:** Drawmer DL241 compressor + peak limiter (`d241c`)
- **Behaviour:** Compressor thr 0..-60 dB (inverted knob), ratio 1.1..100 (17), attack 0.5..100 ms, release 50..5000 ms, `auto` (auto attack/release), separate peak limiter `lim` (-20..0 dB threshold) with its own `lrel`. DL241 compressor is soft-knee "auto" with programme-dependent release.
- **Emulation approach:** Own soft-knee VCA comp + brickwall-ish limiter stage after it; auto = programme-dependent.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | 0 .. -60 | dB | lin | -20 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `att` | ATTACK | 0.5 .. 100 | ms | log | 10 |
| `rel` | RELEASE | 50 .. 5000 | ms | log | 400 |
| `lim` | LIM THR | -20 .. 0 | dB | lin | 20 |
| `lrel` | LIM REL | 50 .. 5000 | ms | log | 400 |
| `auto` | AUTO | off/on |  | bool | 1 |

#### EVEN COMP/LIM - `ECL33`

- **UI name:** EVEN COMP/LIM  |  **snapshot `mdl`:** `ECL33`  |  **WING-Edit internal:** `comp_pardef_n33609`
- **Emulates:** Neve 33609 bus compressor/limiter (`n33609`) - UI "EVEN COMP/LIM"
- **Behaviour:** Limiter section (lon, lthr -12..0 dB, lrec recovery 50/100/200/800 ms/A1/A2, lfast attack) and compressor section (con, cthr -35..-5 dB, ratio 1.5/2/3/4/6, crec 100/400/800/1500 ms/A1/A2, cfast), make-up `cgain` -6..+12. A1/A2 = programme-dependent auto recovery (two time constants). Diode-bridge gain cell -> soft knee, slight distortion, feedback-ish character.
- **Emulation approach:** Two cascaded gain computers (comp then limit) sharing a diode-bridge style gain cell model (soft knee, small even/odd harmonics proportional to GR); A1/A2 = dual-time-constant release. Calibrate thresholds vs 33609 manual.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lon` | LIM ON | off/on |  | bool | 0 |
| `lthr` | LIM THR | -12 .. 0 | dB | lin | -5 |
| `lrec` | LIM REC | 50 / 100 / 200 / 800 / A1 / A2 |  | enum | a1 |
| `lfast` | LIM FAST | off/on |  | bool | 0 |
| `con` | CMP ON | off/on |  | bool | 1 |
| `cthr` | CMP THR | -35 .. -5 | dB | lin | -15 |
| `ratio` | RATIO | 1.5 / 2 / 3 / 4 / 6 |  | enum | 3 |
| `crec` | CMP REC | 100 / 400 / 800 / 1500 / A1 / A2 |  | enum | a1 |
| `cfast` | CMP FAST | off/on |  | bool | 0 |
| `cgain` | GAIN | -6 .. 12 | dB | lin | 0 |

#### SOUL 9000 - `9000C`

- **UI name:** SOUL 9000  |  **snapshot `mdl`:** `9000C`  |  **WING-Edit internal:** `comp_pardef_ssl9000`
- **Emulates:** SSL 9000 J channel compressor (`ssl9000`) - UI "SOUL 9000"
- **Behaviour:** thr -30..+18 dB, ratio 1.3..100 (17 values incl. 1.43, 1.57 SSL-style), `fast` attack (3 ms auto vs fast), release 100..4000 ms, `peak` (peak vs RMS detection). SSL channel comp: VCA, soft-ish knee at low ratios, feed-forward.
- **Emulation approach:** Own VCA compressor with SSL ballistics (auto attack 3-30 ms programme dependent unless fast).
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -30 .. 18 | dB | lin | -10 |
| `ratio` | RATIO | 1.3 / 1.43 / 1.57 / 1.8 / 2 / 2.5 / 2.8 / 3.3 / 4 / 5 / 6 / 7 / 9 / 12 / 20 / 50 / 100 |  | enum | 3 |
| `fast` | FAST ATT | off/on |  | bool | 0 |
| `rel` | RELEASE | 100 .. 4000 | ms | log | 400 |
| `peak` | PEAK | off/on |  | bool | 0 |

#### SOUL BUS COMP - `SBUS`

- **UI name:** SOUL BUS COMP  |  **snapshot `mdl`:** `SBUS`  |  **WING-Edit internal:** `comp_pardef_sslgbus`
- **Emulates:** SSL G-series stereo bus compressor (`sslgbus`) - UI "SOUL BUS COMP"
- **Behaviour:** thr -40..0 dB, ratio 1.5/2/3/4/5/10 (original hardware: 2/4/10), attack enum 0.1/0.3/1/3/10/30 ms, release 0.1/0.2/0.4/0.8/1.6 s/AUTO. Feed-forward VCA (dbx 202 style), RMS-ish detector, soft knee at 2:1, AUTO release is dual time constant.
- **Emulation approach:** Well documented: own VCA bus comp model; reference open designs (e.g. airwindows ButterComp is NOT G-bus; use published G-comp analyses). Stereo-linked detector (max or sum).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -40 .. 0 | dB | lin | -12 |
| `ratio` | RATIO | 1.5 / 2 / 3 / 4 / 5 / 10 |  | enum | 3 |
| `att` | ATTACK | 0.1 / 0.3 / 1 / 3 / 10 / 30 | ms | enum | 10 |
| `rel` | RELEASE | 0.1 / 0.2 / 0.4 / 0.8 / 1.6 / AUTO |  | enum | auto |

#### RED3 COMPRESSOR - `RED3`

- **UI name:** RED3 COMPRESSOR  |  **snapshot `mdl`:** `RED3`  |  **WING-Edit internal:** `comp_pardef_d3`
- **Emulates:** Focusrite Red 3 compressor/limiter (`d3`)
- **Behaviour:** thr -48..0, ratio enum (13 values), attack 1..50 ms (log), release 100..4000 ms, `auto` release. Red 3: VCA-based, smooth soft knee.
- **Emulation approach:** Own VCA compressor with soft knee; auto release dual time constant.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -48 .. 0 | dB | lin | -12 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 |  | enum | 3 |
| `att` | ATTACK | 1 .. 50 | ms | log | 20 |
| `rel` | RELEASE | 100 .. 4000 | ms | log | 400 |
| `auto` | AUTO | off/on |  | bool | 0 |

#### 76 LIMITER AMP - `76LA`

- **UI name:** 76 LIMITER AMP  |  **snapshot `mdl`:** `76LA`  |  **WING-Edit internal:** `comp_pardef_u1176`
- **Emulates:** UREI/Universal Audio 1176LN FET limiter (`u1176`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.dyn
- **Behaviour:** Input -48..0 and output -48..0 (stored as dB-like attenuation of the input/output pots), attack 1..7 and release 1..7 (knob numbers; 1176 attack 20 us..800 us, release 50 ms..1.1 s, 7 = fastest), ratio 4/8/12/20/ALL ("all buttons in": over-compression with nasty knee and changed bias). Fixed threshold - drive via input. FET gain cell: soft knee at 4:1, harder at 20:1, programme-dependent release, significant distortion at fast settings.
- **Emulation approach:** FET model: feedback-ish detector with 1176 time-constant tables, ratio-dependent knee curves (fit to measured 1176 curves), class-A output saturation (2x oversampled waveshaper). ALL mode: special static curve + transient overshoot.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `in` | INPUT | -48 .. 0 | dB | lin | -30 |
| `out` | OUTPUT | -48 .. 0 | dB | lin | -27 |
| `att` | ATTACK | 1 .. 7 |  | lin | 2 |
| `rel` | RELEASE | 1 .. 7 |  | lin | 2 |
| `ratio` | RATIO | 4 / 8 / 12 / 20 / ALL |  | enum | 8 |

#### LA LEVELER - `LA`

- **UI name:** LA LEVELER  |  **snapshot `mdl`:** `LA`  |  **WING-Edit internal:** `comp_pardef_la2a`
- **Emulates:** Teletronix LA-2A optical leveler (`la2a`)
- **Behaviour:** GAIN (ingain 0..100) and PEAK REDUCTION (peak 0..100), mode COMP (~3:1) / LIM (~inf). T4 opto cell: ~10 ms attack, two-stage release (fast ~60 ms to 50 %, then 1..15 s programme/history dependent), soft knee, tube make-up stage.
- **Emulation approach:** T4 cell model: electroluminescent panel + photoresistor with memory (two-state model per published research), feedback topology, tube stage mild saturation.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `ingain` | GAIN | 0 .. 100 |  | lin | 40 |
| `peak` | PEAK | 0 .. 100 |  | lin | 20 |
| `mode` | MODE | COMP / LIM |  | enum | comp |

#### FAIR KID - `F670`

- **UI name:** FAIR KID  |  **snapshot `mdl`:** `F670`  |  **WING-Edit internal:** `comp_pardef_f670`
- **Emulates:** Fairchild 670 variable-mu limiter (`f670`) - UI "FAIR KID"
- **In PUBLIC.snap (fw 1.13.9):** yes, at main.dyn
- **Behaviour:** Input -20..0 dB, threshold 0..10 (knob), time 1..6 (time-constant switch: 1-4 fixed attack/release combos, 5-6 programme-dependent), bias (CAL..1 = DC threshold/bias trim changing knee/ratio). Variable-mu tube: ratio increases with GR (soft knee), push-pull tube distortion, transformer colour.
- **Emulation approach:** Vari-mu gain law (gain = f(control voltage) exponential-ish), feedback sidechain, six documented time-constant settings, tube/transformer waveshaping with oversampling.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `in` | INPUT | -20 .. 0 | dB | lin | -10 |
| `thr` | THR | 0 .. 10 |  | lin | 5 |
| `time` | TIME | 1 .. 6 |  | int | 2 |
| `bias` | BIAS | 0 .. 1 (end label "CAL") |  | lin | 1 |

#### ETERNAL BLISS - `BLISS`

- **UI name:** ETERNAL BLISS  |  **snapshot `mdl`:** `BLISS`  |  **WING-Edit internal:** `comp_pardef_mpress`
- **Emulates:** Elysia mpressor (`mpress`) - UI "ETERNAL BLISS"
- **Behaviour:** thr -50..0, ratio enum 1.2..10 plus negative ratios -0.3..-4 (mpressor "beyond infinity"), attack 0.4..150 ms, release 5..1200 ms, `afast` (Auto Fast: programme-dependent attack), `alog` (Anti Log: linear vs log attack/release curves), `glon/glim` (Gain Reduction Limiter: caps GR at -21..0 dB). Class-A discrete feed-forward VCA.
- **Emulation approach:** Own feed-forward comp with negative-ratio support, auto-fast and anti-log ballistics variants, GR clamp.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -50 .. 0 | dB | lin | -15 |
| `ratio` | RATIO | 1.2 / 1.3 / 1.6 / 2 / 3 / 5 / 10 / -0.3 / -1 / -2 / -3 / -4 |  | enum | 3 |
| `att` | ATTACK | 0.4 .. 150 | ms | log | 10 |
| `rel` | RELEASE | 5 .. 1200 | ms | log | 100 |
| `afast` | AUTO FAST | off/on |  | bool | 0 |
| `alog` | ANTI LOG | off/on |  | bool | 0 |
| `glon` | GR LIMIT ON | off/on |  | bool | 0 |
| `glim` | GR LIMIT | -21 .. 0 | dB | lin | -12 |

#### NO-STRESSOR - `NSTR`

- **UI name:** NO-STRESSOR  |  **snapshot `mdl`:** `NSTR`  |  **WING-Edit internal:** `comp_pardef_distress`
- **Emulates:** Empirical Labs EL8 Distressor (`distress`) - UI "NO-STRESSOR"
- **In PUBLIC.snap (fw 1.13.9):** yes, at ch.dyn
- **Behaviour:** Input 0..10, output 0..10, attack 0..10, release 0..10 (knob scales; Distressor attack ~50 us..30 ms, release ~50 ms..3.5 s), ratio 1.5:1..20:1 and NUKE (brickwall). Distressor curves: ratio-dependent knee (1:1 & 2:1 are "opto-like" soft), 10:1 emulates 1176; distortion modes (Dist 2/3) not exposed.
- **Emulation approach:** Feedback/feedforward hybrid with ratio-specific knees and release shapes; mild harmonic generation tied to ratio. Fit against published Distressor manual curves.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `in` | INPUT | 0 .. 10 |  | lin | -30 |
| `out` | OUTPUT | 0 .. 10 |  | lin | -27 |
| `att` | ATTACK | 0 .. 10 |  | lin | 2 |
| `rel` | RELEASE | 0 .. 10 |  | lin | 2 |
| `ratio` | RATIO | 1.5:1 / 2:1 / 3:1 / 4:1 / 6:1 / 10:1 / 20:1 / NUKE |  | enum | 8 |

#### PIA2250 RACK - `2250`

- **UI name:** PIA2250 RACK  |  **snapshot `mdl`:** `2250`  |  **WING-Edit internal:** `comp_pardef_api225l`
- **Emulates:** API 2500 / 225L (`api225l`) - UI "PIA2250 RACK" (added fw 1.12.4 per release notes)
- **Behaviour:** thr 0..10 (knob), ratio 0..10 (continuous knob), attack FAST/MED/SLOW, release 50..3000 ms, knee HARD/SOFT, type OLD (feedback) / NEW (feed-forward) - the API "Thrust"-era topology switch. VCA (API 2510) gain cell.
- **Emulation approach:** One VCA engine with switchable FF/FB detector path, hard/soft knee; knob->dB/ratio maps from API manual.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | 0 .. 10 |  | lin | 2 |
| `ratio` | RATIO | 0 .. 10 |  | lin | 3 |
| `att` | ATTACK | FAST / MED / SLOW |  | enum | MED |
| `rel` | RELEASE | 50 .. 3000 | ms | log | 200 |
| `knee` | KNEE | HARD / SOFT |  | enum | SOFT |
| `type` | TYPE | OLD / NEW |  | enum | NEW |

#### LTA100 LEVELER - `L100`

- **UI name:** LTA100 LEVELER  |  **snapshot `mdl`:** `L100`  |  **WING-Edit internal:** `comp_pardef_tla100a`
- **Emulates:** Summit Audio TLA-100A tube leveling amp (`tla100a`) - UI "LTA100 LEVELER"
- **Behaviour:** GAIN 0..10, GAIN RED(uction) 0..10, attack FAST/MED/SLOW, release FAST/MED/SLOW. Hybrid FET/opto-ish detector with tube output; very soft knee, programme dependent.
- **Emulation approach:** Own soft-knee leveler with 3x3 ballistics table + tube stage.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `ingain` | GAIN | 0 .. 10 |  | lin | 3 |
| `gr` | GAIN RED | 0 .. 10 |  | lin | 3 |
| `att` | ATTACK | FAST / MED / SLOW |  | enum | MED |
| `rel` | RELEASE | FAST / MED / SLOW |  | enum | MED |

#### EVEN 88 COMP - `E88C`

- **UI name:** EVEN 88 COMP  |  **snapshot `mdl`:** `E88C`  |  **WING-Edit internal:** `comp_pardef_neve88rs`
- **Emulates:** Neve 88RS channel compressor (`neve88rs`)
- **Firmware:** added/visible in 3.1
- **Behaviour:** knee SOFT/HARD, thr +20..-10 (dB, inverted), `thrpull` ("THR -20": extends threshold range by -20 dB), ratio 1..10 (log; min end labelled "LIM"? -> extreme = limit), attack SLOW/FAST, release 10..3000 ms (log; end labelled AUTO). Added fw 3.1 (Drew Brashler 3.1 review).
- **Emulation approach:** Own VCA-style comp with knee option; auto release dual time constant.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `knee` | KNEE | SOFT / HARD |  | enum | soft |
| `thr` | THR | 20 .. -10 | dB | lin | 0 |
| `thrpull` | THR -20 | off/on |  | bool | 0 |
| `ratio` | RATIO | 1 .. 10 (end label "LIM") |  | log | 3 |
| `att` | ATTACK | SLOW / FAST |  | enum | slow |
| `rel` | RELEASE | 10 .. 3000 (end label "AUTO") | ms | log | 350 |

#### LMT COMPRESSOR - `LMT`

- **UI name:** LMT COMPRESSOR  |  **snapshot `mdl`:** `LMT`  |  **WING-Edit internal:** `comp_pardef_lmcplus`
- **Emulates:** SSL 4000E Listen Mic Compressor ("LMC+"-style, `lmcplus`) - UI "LMT COMPRESSOR"
- **Firmware:** added/visible in 3.1
- **Behaviour:** Transient section (`ton`, `tspd` 0..1 speed, `trans` -10..+10 shaping amount, `tgain` +/-12 dB) and compressor section (`con`, `comp` 0..100 amount, `cgain` +/-12 dB). The LMC is a fixed, extremely fast, very high ratio feed-forward comp with heavy colour; the "LMC+" adds a transient shaper. fw 3.1.
- **Emulation approach:** Fixed-ratio ultra-fast VCA comp with saturation + transient shaper (re-use WAVE engine).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `tspd` | T SPEED | 0 .. 1 |  | lin | 0.5 |
| `trans` | TRANS | -10 .. 10 |  | lin | 0 |
| `tgain` | T GAIN | -12 .. 12 | dB | lin | 0 |
| `ton` | T ON | off/on |  | bool | 0 |
| `comp` | COMP | 0 .. 100 |  | lin | 10 |
| `cgain` | C GAIN | -12 .. 12 | dB | lin | 0 |
| `con` | C ON | off/on |  | bool | 1 |

#### ONE KNOB COMP - `ONEC`

- **UI name:** ONE KNOB COMP  |  **snapshot `mdl`:** `ONEC`  |  **WING-Edit internal:** `comp_pardef_oneknob`
- **Emulates:** One-knob compressor (`oneknob`) - WING original (fw 3.1)
- **Firmware:** added/visible in 3.1
- **Behaviour:** Single `gr` 0..10 macro controlling threshold, ratio, attack, release together; `dag` switch (meaning undocumented: likely an alternate voicing/"drag" mode - verify in console UI).
- **Emulation approach:** Map gr through a 1-D table into COMP parameters.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `gr` | GAIN RED | 0 .. 10 |  | lin | 3 |
| `dag` | DAG | off/on |  | bool | 1 |

#### THREE BAND COMP - `3CMP`

- **UI name:** THREE BAND COMP  |  **snapshot `mdl`:** `3CMP`  |  **WING-Edit internal:** `comp_pardef_triband`
- **Emulates:** Three-band compressor (`triband`) - WING native multiband
- **Firmware:** added/visible in hidden (in WING-Edit 3.3.4 table, in no selector)
- **Behaviour:** Global knee 0..25 dB, crossovers lmf 50..500 Hz and mhf 1..5 kHz, per band g +/-15 dB, thr -60..0, ratio (17), att 0.2..50 ms, rel 50..4000 ms; `$page` UI-only. Not offered by any model selector in WING-Edit 3.3.4 (T003) and not in any release note found: treat as hidden/unreleased; implement last.
- **Emulation approach:** LR4 3-way crossover (phase-compensated) + 3 COMP engines, sum.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `knee` | KNEE | 0 .. 25 |  | lin | 5 |
| `lmf` | LO/MID F | 50 .. 500 | Hz | log | 200 |
| `mhf` | MID/HI F | 1000 .. 5000 | Hz | log | 2000 |
| `1-g` | 1-GAIN | -15 .. 15 | dB | lin | 0 |
| `1-thr` | 1-THR | -60 .. 0 | dB | lin | 0 |
| `1-ratio` | 1-RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `1-att` | 1-ATT | 0.2 .. 50 | ms | log | 10 |
| `1-rel` | 1-REL | 50 .. 4000 | ms | log | 350 |
| `2-g` | 2-GAIN | -15 .. 15 | dB | lin | 0 |
| `2-thr` | 2-THR | -60 .. 0 | dB | lin | 0 |
| `2-ratio` | 2-RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `2-att` | 2-ATT | 0.2 .. 50 | ms | log | 10 |
| `2-rel` | 2-REL | 50 .. 4000 | ms | log | 350 |
| `3-g` | 3-GAIN | -15 .. 15 | dB | lin | 0 |
| `3-thr` | 3-THR | -60 .. 0 | dB | lin | 0 |
| `3-ratio` | 3-RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 6 / 8 / 10 / 20 / 50 / 100 |  | enum | 3 |
| `3-att` | 3-ATT | 0.2 .. 50 | ms | log | 10 |
| `3-rel` | 3-REL | 50 .. 4000 | ms | log | 350 |
| `$page` | COMP PAGE | 0 .. 2 |  | int |  |

## 4. EQ models

Snapshot path `ae_data.<strip>.<n>.eq` with wrapper `{on, mdl, mix}`. Three model tables exist in WING-Edit:

- **Table A - input channels** (`ch`): STD (`xseries`, 4 bands + L/H), SOUL, E88, E84, F110, PULSAR, MACH4.
- **Table B - aux channels** (`aux`): STD (`xseries`), SOUL, E88, E84, F110, PULSAR. (Assignment of tables A/B/C to
  strip types is inferred from band counts in the fixture: ch/aux eq have bands 1-4, bus/main/mtx have 1-6 + tilt.)
- **Table C - bus, main, matrix**: STD (`xseries_bus`: 6 bands + L/H + tilt, extended L/H band types), SOUL, E88, E84,
  F110, PULSAR, PIA (table index 7; index 6 = MACH4 is skipped).

Low/high band type enums (T003): ch `leq/heq` = PEQ, SHV; aux = PEQ, SHV, CUT; bus/main/mtx = PEQ, SHV, CUT,
BW6, BW12, BS12, LR12, BW18, BW24, BS24, LR24, BW48, LR48 (from this extraction).

Channels additionally have `peq` (3 bands `1g/1f/1q..3q`, `on`) = "PRE EQ" (signal-flow.md section 2). Monitor outputs
have their own fixed EQ (`cfg.mon.<n>.eq`: `lsg/lsf`, bands 1-6, `hsg/hsf`).

Vintage models store knob positions (0..10, -5..+5) rather than Hz/dB: the knob->physical maps are NOT in the
editor's parameter definitions (display format is plain `%.1f`), so they must come from the hardware front-panel
legends or from measuring a console (**open item**, see section 8).


#### WING EQ - `STD`

- **UI name:** WING EQ  |  **snapshot `mdl`:** `STD`  |  **WING-Edit internal:** `eq_pardef_xseries`  |  tables: ch (A), aux (B)
- **Emulates:** Behringer WING native parametric EQ (`xseries` ch/aux, `xseries_bus` bus/main/mtx)
- **In PUBLIC.snap (fw 1.13.9):** yes, at aux.eq, ch.eq
- **Behaviour:** Channel/aux: low band (`lg/lf/lq` + `leq` type), 4 peaking bands `1..4`, high band (`hg/hf/hq` + `heq`). Bus/main/matrix: low band, 6 peaking bands, high band (= "8-band") plus `tilt` (+/-6 dB). Gains +/-15 dB, f 20..20000 Hz (log, 961 steps), Q 0.44..10 (log). Band type enum for bus/main/mtx L/H: PEQ, SHV (shelf), CUT and filter slopes BW6, BW12, BS12, LR12, BW18, BW24, BS24, LR24, BW48, LR48 (Butterworth/Bessel/Linkwitz-Riley). Channel L/H type enum = PEQ, SHV; aux = PEQ, SHV, CUT (T003). Wrapper: `on`, `mdl`, `mix` 0..100 % (wet/dry), channel also has `peq` = 3-band "PRE EQ" (see signal-flow). Minimum-phase (analog-prototype) filters; WING shelf is RBJ-like; Q definition presumably RBJ bandwidth Q - verify via measurement.
- **Emulation approach:** Cascaded biquads (RBJ cookbook / bilinear with frequency pre-warp, or Vicanek matched-magnitude to avoid cramping at 48 kHz) in TDF-II/SVF; per-block coefficient update with parameter smoothing.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lg` | GAIN L | -15 .. 15 | dB | lin |  |
| `lf` | FREQ L | 20 .. 20000 | Hz | log |  |
| `lq` | Q L | 0.44 .. 10 |  | log |  |
| `leq` | EQ L | (see EQ table header) |  | enum |  |
| `1g` | GAIN 1 | -15 .. 15 | dB | lin |  |
| `1f` | FREQ 1 | 20 .. 20000 | Hz | log |  |
| `1q` | Q 1 | 0.44 .. 10 |  | log |  |
| `2g` | GAIN 2 | -15 .. 15 | dB | lin |  |
| `2f` | FREQ 2 | 20 .. 20000 | Hz | log |  |
| `2q` | Q 2 | 0.44 .. 10 |  | log |  |
| `3g` | GAIN 3 | -15 .. 15 | dB | lin |  |
| `3f` | FREQ 3 | 20 .. 20000 | Hz | log |  |
| `3q` | Q 3 | 0.44 .. 10 |  | log |  |
| `4g` | GAIN 4 | -15 .. 15 | dB | lin |  |
| `4f` | FREQ 4 | 20 .. 20000 | Hz | log |  |
| `4q` | Q 4 | 0.44 .. 10 |  | log |  |
| `hg` | GAIN H | -15 .. 15 | dB | lin |  |
| `hf` | FREQ H | 20 .. 20000 | Hz | log |  |
| `hq` | Q H | 0.44 .. 10 |  | log |  |
| `heq` | EQ H | (see EQ table header) |  | enum |  |

Bus/main/matrix variant (`xseries_bus`, table C) parameters:

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lg` | GAIN L | -15 .. 15 | dB | lin |  |
| `lf` | FREQ L | 20 .. 20000 | Hz | log |  |
| `lq` | Q L | 0.44 .. 10 |  | log |  |
| `leq` | EQ L | PEQ / SHV / CUT / BW6 / BW12 / BS12 / LR12 / BW18 / BW24 / BS24 / LR24 / BW48 / LR48 |  | enum |  |
| `1g` | GAIN 1 | -15 .. 15 | dB | lin |  |
| `1f` | FREQ 1 | 20 .. 20000 | Hz | log |  |
| `1q` | Q 1 | 0.44 .. 10 |  | log |  |
| `2g` | GAIN 2 | -15 .. 15 | dB | lin |  |
| `2f` | FREQ 2 | 20 .. 20000 | Hz | log |  |
| `2q` | Q 2 | 0.44 .. 10 |  | log |  |
| `3g` | GAIN 3 | -15 .. 15 | dB | lin |  |
| `3f` | FREQ 3 | 20 .. 20000 | Hz | log |  |
| `3q` | Q 3 | 0.44 .. 10 |  | log |  |
| `4g` | GAIN 4 | -15 .. 15 | dB | lin |  |
| `4f` | FREQ 4 | 20 .. 20000 | Hz | log |  |
| `4q` | Q 4 | 0.44 .. 10 |  | log |  |
| `5g` | GAIN 5 | -15 .. 15 | dB | lin |  |
| `5f` | FREQ 5 | 20 .. 20000 | Hz | log |  |
| `5q` | Q 5 | 0.44 .. 10 |  | log |  |
| `6g` | GAIN 6 | -15 .. 15 | dB | lin |  |
| `6f` | FREQ 6 | 20 .. 20000 | Hz | log |  |
| `6q` | Q 6 | 0.44 .. 10 |  | log |  |
| `hg` | GAIN H | -15 .. 15 | dB | lin |  |
| `hf` | FREQ H | 20 .. 20000 | Hz | log |  |
| `hq` | Q H | 0.44 .. 10 |  | log |  |
| `heq` | EQ H | PEQ / SHV / CUT / BW6 / BW12 / BS12 / LR12 / BW18 / BW24 / BS24 / LR24 / BW48 / LR48 |  | enum |  |
| `tilt` | TILT | -6 .. 6 | dB | lin |  |

#### SOUL ANALOGUE - `SOUL`

- **UI name:** SOUL ANALOGUE  |  **snapshot `mdl`:** `SOUL`  |  **WING-Edit internal:** `eq_pardef_ssl_g`  |  tables: ch (A), aux (B), bus/main/mtx (C)
- **Emulates:** SSL 4000 G-series channel EQ (`ssl_g`) - UI "SOUL ANALOGUE"
- **Behaviour:** LF (shelf) freq/gain, LMF freq/gain/Q with `lmf3` (/3 range), HMF freq/gain/Q with `hmf3` (x3 range), HF (shelf) freq/gain. All continuous controls stored as knob positions (freq 0..10, gain -5..+5, Q 0..10, display %.1f) - the knob->Hz/dB maps must be taken from the hardware panel legends (G-series: LF 30-450 Hz, LMF 0.2-2.5 kHz, HMF 0.6-7 kHz, HF 1.5-16 kHz, gain +/-15 dB -> likely 3 dB per unit). G-series: proportional-Q (constant-Q-ish) behaviour and shelves with slight overshoot (bell option absent here).
- **Emulation approach:** Biquads with SSL G curve fitting (proportional Q); knob tapers from panel legends; validate against published frequency response plots.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lf` | LO FREQ | 0 .. 10 |  | lin | 2 |
| `lg` | LO GAIN | -5 .. 5 |  | lin | 0 |
| `lmf` | LM FREQ | 0 .. 10 |  | lin | 6 |
| `lmf3` | LM /3 | off/on |  | bool | 1 |
| `lmq` | LM Q | 0 .. 10 |  | lin | 5 |
| `lmg` | LM GAIN | -5 .. 5 |  | lin | 0 |
| `hmf` | HM FREQ | 0 .. 10 |  | lin | 3 |
| `hmf3` | HM x3 | off/on |  | bool | 0 |
| `hmq` | HM Q | 0 .. 10 |  | lin | 5 |
| `hmg` | HM GAIN | -5 .. 5 |  | lin | 0 |
| `hf` | HF FREQ | 0 .. 10 |  | lin | 8 |
| `hg` | HF GAIN | -5 .. 5 |  | lin | 0 |

#### EVEN 88 FORMANT - `E88`

- **UI name:** EVEN 88 FORMANT  |  **snapshot `mdl`:** `E88`  |  **WING-Edit internal:** `eq_pardef_neve88rs`  |  tables: ch (A), aux (B), bus/main/mtx (C)
- **Emulates:** Neve 88RS channel EQ (`neve88rs`) - UI "EVEN 88 FORMANT"
- **Behaviour:** LF band freq/gain, Q LOW/HIGH, type BELL/SHELV; LMF and HMF bands freq/gain/Q (continuous Q); HF band freq/gain, Q LOW/HIGH, BELL/SHELV. Knob positions 0..10 / gain -5..+5 as SOUL. 88RS EQ is a modern Neve (more parametric than 1073/1084).
- **Emulation approach:** Biquads + knob maps from the 88RS manual.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lf` | LF FREQ | 0 .. 10 |  | lin | 2 |
| `lg` | LF GAIN | -5 .. 5 |  | lin | 0 |
| `lq` | LF Q | LOW / HIGH |  | enum | low |
| `lt` | LF TYPE | BELL / SHELV |  | enum | shelv |
| `lmf` | LM FREQ | 0 .. 10 |  | lin | 3 |
| `lmg` | LM GAIN | -5 .. 5 |  | lin | 0 |
| `lmq` | LM Q | 0 .. 10 |  | lin | 5 |
| `hmf` | HM FREQ | 0 .. 10 |  | lin | 5 |
| `hmg` | HM GAIN | -5 .. 5 |  | lin | 0 |
| `hmq` | HM Q | 0 .. 10 |  | lin | 5 |
| `hf` | HF FREQ | 0 .. 10 |  | lin | 7 |
| `hg` | HF GAIN | -5 .. 5 |  | lin | 0 |
| `hq` | HF Q | LOW / HIGH |  | enum | low |
| `ht` | HF TYPE | BELL / SHELV |  | enum | shelv |

#### EVEN 84 - `E84`

- **UI name:** EVEN 84  |  **snapshot `mdl`:** `E84`  |  **WING-Edit internal:** `eq_pardef_neve84`  |  tables: ch (A), aux (B), bus/main/mtx (C)
- **Emulates:** Neve 1084 (`neve84`) - UI "EVEN 84"
- **Behaviour:** Stepped: LF OFF/35/60/110/220 Hz shelf, MF OFF/0.35/0.7/1.6/3.2/4.8/7.2 kHz bell with Q LOW/HIGH, HF 10/12/16 kHz/OFF shelf; gains -5..+5 (knob units; real 1084 is +/-16 dB in 2 dB steps... verify display), `g` -20..+20 dB (input/output gain). Inductor-based passive bands: asymmetric boost/cut, LF shelf with resonant bump, transformer colour.
- **Emulation approach:** Fitted analog-prototype biquads per detent (curves from 1073/1084 measurements), optional transformer saturation.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `g` | GAIN | -20 .. 20 | dB | lin | 0 |
| `lf` | LF FREQ | OFF / 35 / 60 / 110 / 220 |  | enum | 35 |
| `lg` | LF GAIN | -5 .. 5 |  | lin | 0 |
| `mf` | MID FREQ | OFF / 350 / 700 / 1k6 / 3k2 / 4k8 / 7k2 |  | enum | 1k6 |
| `mg` | MID GAIN | -5 .. 5 |  | lin | 0 |
| `mq` | MID Q | LOW / HIGH |  | enum | low |
| `hf` | HF FREQ | 10k / 12k / 16k / OFF |  | enum | 16k |
| `hg` | HF GAIN | -5 .. 5 |  | lin | 0 |

#### FORTISSIMO 110 - `F110`

- **UI name:** FORTISSIMO 110  |  **snapshot `mdl`:** `F110`  |  **WING-Edit internal:** `eq_pardef_isa110`  |  tables: ch (A), aux (B), bus/main/mtx (C)
- **Emulates:** Focusrite ISA 110 (`isa110`) - UI "FORTISSIMO 110"
- **Behaviour:** LF shelf 33/56/95/160/270/460 Hz, HF shelf 3.3/4.7/6.8/10/15/18 kHz (`shv` enables shelves), LMF/HMF sweepable with Q and x3 range switches (`peq` enables), gains -5..+5 knob units, `g` -18..+18 dB output.
- **Emulation approach:** Biquads; knob maps from ISA110 legend.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `peq` | PEQ ON | off/on |  | bool | 1 |
| `lmf` | LM FREQ | 0 .. 10 |  | lin | 7 |
| `lmg` | LM GAIN | -5 .. 5 |  | lin | 0 |
| `lmq` | LM Q | 0 .. 10 |  | lin | 5 |
| `lmf3` | LM x3 | off/on |  | bool | 0 |
| `hmf` | HM FREQ | 0 .. 10 |  | lin | 3 |
| `hmg` | HM GAIN | -5 .. 5 |  | lin | 0 |
| `hmq` | HM Q | 0 .. 10 |  | lin | 5 |
| `hmf3` | HM x3 | off/on |  | bool | 0 |
| `shv` | SHV ON | off/on |  | bool | 1 |
| `lf` | LF FREQ | 33 / 56 / 95 / 160 / 270 / 460 | Hz | enum | 56 |
| `lg` | LF GAIN | -5 .. 5 |  | lin | 0 |
| `hf` | HF FREQ | 3k3 / 4k7 / 6k8 / 10k / 15k / 18k | Hz | enum | 10k |
| `hg` | HF GAIN | -5 .. 5 |  | lin | 0 |
| `g` | GAIN | -18 .. 18 |  | lin | 0 |

#### PULSAR - `PULSAR`

- **UI name:** PULSAR  |  **snapshot `mdl`:** `PULSAR`  |  **WING-Edit internal:** `eq_pardef_pultec_duo`  |  tables: ch (A), aux (B), bus/main/mtx (C)
- **Emulates:** Pultec EQP-1A + MEQ-5 combined (`pultec_duo`) - UI "PULSAR"
- **Behaviour:** EQP-1A section (`eq1`): LF boost & attenuation 0..10 at 20/30/60/100 Hz (simultaneous boost+cut gives the famous shelf-with-dip), HF boost 0..10 with bandwidth `1hw` at 3k..16k, HF attenuation 0..10 shelf at 5/10/20 kHz. MEQ-5 section (`eq5`): low-mid boost (200..1k), mid dip (200..7k), high-mid boost (1.5k..5k). Passive LC network: interacting bands, make-up tube stage.
- **Emulation approach:** Model the passive network (WDF or fitted analog transfer function) so boost/cut interaction is preserved; tube make-up stage light saturation.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `eq1` | EQ1 ON | off/on |  | bool | 0 |
| `1lb` | LF BOOST | 0 .. 10 |  | lin | 0 |
| `1latt` | LF ATT | 0 .. 10 |  | lin | 0 |
| `1lf` | LF FREQ | 20 / 30 / 60 / 100 | Hz | enum | 30 |
| `1hw` | HF WID | 0 .. 10 |  | lin | 5 |
| `1hb` | HF BOOST | 0 .. 10 |  | lin | 0 |
| `1hf` | HF FREQ | 3k / 4k / 5k / 8k / 10k / 12k / 16k | Hz | enum | 10k |
| `1hatt` | HF ATT | 0 .. 10 |  | lin | 0 |
| `1hattf` | HF ATT F | 5k / 10k / 20k | Hz | enum | 20k |
| `eq5` | EQ5 ON | off/on |  | bool | 0 |
| `5lb` | LM BOOST | 0 .. 10 |  | lin | 0 |
| `5lf` | LF FREQ | 200 / 300 / 500 / 700 / 1k | Hz | enum | 300 |
| `5md` | MID DIP | 0 .. 10 |  | lin | 0 |
| `5mf` | MID FREQ | 200 / 300 / 500 / 700 / 1k / 1k5 / 2k / 3k / 4k / 5k / 7k | Hz | enum | 1k |
| `5hb` | HM BOOST | 0 .. 10 |  | lin | 0 |
| `5hf` | HF FREQ | 1k5 / 2k / 3k / 4k / 5k | Hz | enum | 3k |

#### MACH EQ4 - `MACH4`

- **UI name:** MACH EQ4  |  **snapshot `mdl`:** `MACH4`  |  **WING-Edit internal:** `eq_pardef_maag4`  |  tables: ch (A)
- **Emulates:** Maag Audio EQ4 (`maag4`) - UI "MACH EQ4"
- **Behaviour:** Fixed bands: SUB (10 Hz), 40 Hz, 160 Hz, 650 Hz, 2.5 kHz (gains -5..+5 knob units; EQ4 is +/-? dB) and AIR band (`air` 0..10 boost, `airm` OFF/2.5k/5k/10k/20k/40k), `again` auto gain. Air band = very broad high shelf/bell, minimal phase shift reputation. Channel-only in EQ slot (eq table A).
- **Emulation approach:** Broad low-Q bells + Air shelf fitted to Maag curves.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `sub` | SUB | -5 .. 5 |  | lin |  |
| `40` | 40 | -5 .. 5 |  | lin |  |
| `160` | 160 | -5 .. 5 |  | lin |  |
| `650` | 650 | -5 .. 5 |  | lin |  |
| `2k5` | 2k5 | -5 .. 5 |  | lin |  |
| `air` | AIR | 0 .. 10 |  | lin |  |
| `airm` | AIR MODE | OFF / 2k5 / 5k / 10k / 20k / 40k |  | enum |  |
| `again` | AUTO | off/on |  | bool |  |

#### PIA 560 GEQ - `PIA`

- **UI name:** PIA 560 GEQ  |  **snapshot `mdl`:** `PIA`  |  **WING-Edit internal:** `eq_pardef_api560`  |  tables: bus/main/mtx (C)
- **Emulates:** API 560 10-band graphic EQ (`api560`) - bus/main/matrix only
- **Behaviour:** 10 octave bands 31..16k, +/-12 dB, `g` output +/-12 dB. API proportional-Q: bandwidth narrows as gain increases.
- **Emulation approach:** Proportional-Q peaking biquads (Q = f(|gain|)) fitted to API 560 curves.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `g` | GAIN | -12 .. 12 | dB | lin |  |
| `31` | 31 | -12 .. 12 | dB | lin |  |
| `63` | 63 | -12 .. 12 | dB | lin |  |
| `125` | 125 | -12 .. 12 | dB | lin |  |
| `250` | 250 | -12 .. 12 | dB | lin |  |
| `500` | 500 | -12 .. 12 | dB | lin |  |
| `1k` | 1k | -12 .. 12 | dB | lin |  |
| `2k` | 2k | -12 .. 12 | dB | lin |  |
| `4k` | 4k | -12 .. 12 | dB | lin |  |
| `8k` | 8k | -12 .. 12 | dB | lin |  |
| `16k` | 16k | -12 .. 12 | dB | lin |  |

## 5. Inserts

There are no dedicated insert "models": an insert point hosts one FX rack slot. Recovered from `ae::Cinsert`:

| key | UI label | values | notes |
|---|---|---|---|
| `on` | INSERT | off/on | |
| `ins` | FX PROC | NONE, FX1 .. FX16 | slot assignment; a slot can be used by one insert at a time (send use otherwise) |
| `mode` | MODE | FX / AUTO_X / AUTO_Y | **post-insert of input channels only**: AUTO_X/Y route the channel into the Dugan-style automixer group X or Y (`cfg.amix.x/y` enables) instead of an FX slot |
| `w` | WEIGHT | -12 .. +12 dB | automix weight (channel post-insert only) |
| `$stat` | STATE | -, OK, N/A | runtime: N/A when the selected FX model cannot run (e.g. premium model in a slot that is already used) |

Insert points per strip (fixture keys): `ch`: `preins` {on, ins} + `postins` {on, mode, ins, w}; `aux`: `preins` only;
`bus`, `main`, `mtx`: `preins` + `postins` {on, ins}. FX rack slot used as an insert is processed in-line (wet/dry via
`fxmix`), stereo in/out regardless of source width.

Automix (AUTO_X / AUTO_Y): gain-sharing automixer (Dugan style: each channel gain = its level share of the group,
biased by `w`). Emulation approach: per-group RMS levels -> gains g_i = L_i*w_i / sum(L_j*w_j), smoothed; Difficulty
**M** (needs all group members before computing - a cross-channel dependency in the engine DAG).


## 6. FX rack models

Snapshot path `ae_data.fx.<1..16>` = `{mdl, fxmix, <model params>}`; `mdl: "NONE"` for an empty slot. Model table
from WING-Edit (`ae::CfxunitC2`, records `(id, name, class<<32 | algo_id, fnptrs...)`):

- **Slot restriction (T003 selector lists, authoritative):** `fx/1..8` ("full") offer all 64 models + NONE;
  `fx/9..16` ("lite") offer 38 + NONE, i.e. without the 26 table entries HALL..BPLATE (all reverbs, DIMCRS, CHORUS,
  FLANGER, all delays, PITCH, D-PITCH, VSS3, BPLATE).
- `slot class` (high dword of the record's third field): 3 for most of those heavy models (but SUB is class 3 and
  allowed in lite slots; CHORUS/FLANGER/DIMCRS/PITCH are class 2 yet full-only) - so class is a DSP-cost class, not
  the slot rule. **class 4**: EXT (external loop, I/O only).
- `algo id` = internal DSP algorithm number (0x25 GEQ, 0x3b PIA, ...) - useful for T003 cross-reference.

Every slot is stereo in / stereo out. Used as **send FX**: fed by a bus or matrix output routed to the slot input
(`$esrc`/source selection in the FX unit) and returned via a channel/aux source (`FX` source group in routing) -
mix normally 100 %. Used as **insert**: wet/dry `fxmix`. Fixture usage (fw 1.13): BPLATE, V-PLATE, PCORR (x2), HALL
(x2), TAPE-DL, PITCH, DE-S2, DEQ3, *MASTER*, C5-CMB, GEQ (x3).


| # | `mdl` | UI name | internal | slots (T003) | DSP class | algo id |
|---|---|---|---|---|---|---|
| 0 | `EXT` | EXTERNAL | `fx_pardef_external` | FX1-16 | 4 | 1 |
| 1 | `HALL` | HALL REVERB | `fx_pardef_hall_reverb` | FX1-8 only | 3 | 2 |
| 2 | `ROOM` | ROOM REVERB | `fx_pardef_lex_room` | FX1-8 only | 3 | 3 |
| 3 | `CHAMBER` | CHAMBER REVERB | `fx_pardef_lex_chamber` | FX1-8 only | 3 | 4 |
| 4 | `PLATE` | PLATE REVERB | `fx_pardef_lex_plate` | FX1-8 only | 3 | 5 |
| 5 | `CONCERT` | CONCERT REVERB | `fx_pardef_lex_concert` | FX1-8 only | 3 | 6 |
| 6 | `AMBI` | AMBIENCE | `fx_pardef_ambience` | FX1-8 only | 3 | 7 |
| 7 | `V-ROOM` | VINTAGE ROOM | `fx_pardef_vintage_room` | FX1-8 only | 3 | 8 |
| 8 | `V-REV` | VINTAGE REVERB | `fx_pardef_vintage_reverb` | FX1-8 only | 3 | 9 |
| 9 | `V-PLATE` | VINTAGE PLATE | `fx_pardef_vintage_plate` | FX1-8 only | 3 | 10 |
| 10 | `GATED` | GATED REVERB | `fx_pardef_gated_reverb` | FX1-8 only | 3 | 11 |
| 11 | `REVERSE` | REVERSE REVERB | `fx_pardef_reverse_reverb` | FX1-8 only | 3 | 12 |
| 12 | `DEL/REV` | DELAY/REVERB | `fx_pardef_delreverb` | FX1-8 only | 3 | 23 |
| 13 | `SHIMMER` | SHIMMER REVERB | `fx_pardef_shim_reverb` | FX1-8 only | 3 | 13 |
| 14 | `SPRING` | SPRING REVERB | `fx_pardef_spring_reverb` | FX1-8 only | 3 | 14 |
| 15 | `DIMCRS` | DIMENSION CRS | `fx_pardef_dimension_crs` | FX1-8 only | 2 | 17 |
| 16 | `CHORUS` | STEREO CHORUS | `fx_pardef_chorus` | FX1-8 only | 2 | 15 |
| 17 | `FLANGER` | STEREO FLANGER | `fx_pardef_flanger` | FX1-8 only | 2 | 16 |
| 18 | `ST-DL` | STEREO DELAY | `fx_pardef_stereo_delay` | FX1-8 only | 3 | 18 |
| 19 | `TAP-DL` | ULTRATAP DELAY | `fx_pardef_ultratap_delay` | FX1-8 only | 3 | 19 |
| 20 | `TAPE-DL` | TAPE DELAY | `fx_pardef_tape_delay` | FX1-8 only | 3 | 20 |
| 21 | `OILCAN` | OILCAN DELAY | `fx_pardef_telray_delay` | FX1-8 only | 3 | 21 |
| 22 | `BBD-DL` | BBD DELAY | `fx_pardef_bbd_delay` | FX1-8 only | 3 | 22 |
| 23 | `PITCH` | STEREO PITCH | `fx_pardef_stereo_pitch` | FX1-8 only | 2 | 24 |
| 24 | `D-PITCH` | DUAL PITCH | `fx_pardef_dual_pitch` | FX1-8 only | 3 | 25 |
| 25 | `VSS3` | VSS3 REVERB | `fx_pardef_vss3_reverb` | FX1-8 only | 3 | 26 |
| 26 | `BPLATE` | BLUE PLATE | `fx_pardef_plate_reverb` | FX1-8 only | 3 | 27 |
| 27 | `GEQ` | GRAPHIC EQ | `fx_pardef_geq` | FX1-16 | 2 | 37 |
| 28 | `PIA` | PIA 560 GEQ | `fx_pardef_api560` | FX1-16 | 2 | 59 |
| 29 | `DOUBLE` | DOUBLE VOCAL | `fx_pardef_doubler` | FX1-16 | 2 | 38 |
| 30 | `PCORR` | PITCH FIX | `fx_pardef_pitchcorr` | FX1-16 | 2 | 39 |
| 31 | `LIMITER` | PRECISION LIMITER | `fx_pardef_prec_limiter` | FX1-16 | 2 | 28 |
| 32 | `DE-S2` | 2-BAND DEESSER | `fx_pardef_deesser` | FX1-16 | 2 | 29 |
| 33 | `ENHANCE` | ULTRA ENHANCER | `fx_pardef_uenhancer` | FX1-16 | 2 | 31 |
| 34 | `EXCITER` | EXCITER | `fx_pardef_exciter` | FX1-16 | 2 | 32 |
| 35 | `P-BASS` | PSYCHO BASS | `fx_pardef_ren_bass` | FX1-16 | 2 | 30 |
| 36 | `ROTARY` | ROTARY SPEAKER | `fx_pardef_rotary` | FX1-16 | 2 | 33 |
| 37 | `PHASER` | PHASER | `fx_pardef_phaser` | FX1-16 | 2 | 34 |
| 38 | `PANNER` | TREMOLO/PANNER | `fx_pardef_tremolo_pan` | FX1-16 | 2 | 35 |
| 39 | `TAPE` | TAPE MACHINE | `fx_pardef_tapemach` | FX1-16 | 2 | 40 |
| 40 | `MOOD` | MOOD FILTER | `fx_pardef_moodfilter` | FX1-16 | 2 | 45 |
| 41 | `SUB` | SUB OCTAVER | `fx_pardef_suboct` | FX1-16 | 3 | 46 |
| 42 | `RACKAMP` | RACK AMP | `fx_pardef_rack_amp` | FX1-16 | 2 | 36 |
| 43 | `UKROCK` | UK ROCK AMP | `fx_pardef_amp_jcm800` | FX1-16 | 2 | 41 |
| 44 | `ANGEL` | ANGEL AMP | `fx_pardef_amp_engl` | FX1-16 | 2 | 42 |
| 45 | `JAZZC` | JAZZ CLEAN AMP | `fx_pardef_amp_jc120` | FX1-16 | 2 | 43 |
| 46 | `DELUXE` | DELUXE AMP | `fx_pardef_amp_deluxe` | FX1-16 | 2 | 44 |
| 47 | `BODY` | BODYREZ | `fx_pardef_bodyrez` | FX1-16 | 2 | 48 |
| 48 | `SOUL` | SOUL ANALOGUE | `fx_pardef_ssl_g` | FX1-16 | 2 | 53 |
| 49 | `E88` | EVEN 88 FORMANT | `fx_pardef_neve88rs` | FX1-16 | 2 | 54 |
| 50 | `E84` | EVEN 84 | `fx_pardef_neve84` | FX1-16 | 2 | 55 |
| 51 | `F110` | FORTISSIMO 110 | `fx_pardef_isa110` | FX1-16 | 2 | 56 |
| 52 | `PULSAR` | PULSAR | `fx_pardef_pultec_duo` | FX1-16 | 2 | 57 |
| 53 | `MACH4` | MACH EQ4 | `fx_pardef_maag4` | FX1-16 | 2 | 58 |
| 54 | `C5-CMB` | COMBINATOR | `fx_pardef_combinator` | FX1-16 | 2 | 47 |
| 55 | `SUB-M` | SUB MONSTER | `fx_pardef_submonster` | FX1-16 | 2 | 49 |
| 56 | `V-IMG` | VELVET IMAGER | `fx_pardef_velvetimg` | FX1-16 | 2 | 50 |
| 57 | `SPKMAN` | SPEAKER MANAGER | `fx_pardef_spkpro` | FX1-16 | 2 | 51 |
| 58 | `DEQ3` | TRIPLE DEQ | `fx_pardef_trideq` | FX1-16 | 2 | 52 |
| 59 | `*EVEN*` | EVEN CHANNEL | `fx_pardef_evenchannel` | FX1-16 | 2 | 60 |
| 60 | `*SOUL*` | SOUL CHANNEL | `fx_pardef_soulchannel` | FX1-16 | 2 | 61 |
| 61 | `*VINTAGE*` | VINTAGE CHANNEL | `fx_pardef_vintagechannel` | FX1-16 | 2 | 62 |
| 62 | `*BUS*` | BUS CHANNEL | `fx_pardef_buschannel` | FX1-16 | 2 | 63 |
| 63 | `*MASTER*` | MASTERING | `fx_pardef_mastering` | FX1-16 | 2 | 64 |

#### EXTERNAL - `EXT`

- **UI name:** EXTERNAL  |  **snapshot `mdl`:** `EXT`  |  **WING-Edit internal:** `fx_pardef_external`  |  slots: FX1-16
- **Emulates:** External insert/loop (I/O, not DSP)
- **Behaviour:** Routes the slot to an external device: `egrp`/`ein` select the return source group/index, `emode` M/ST/M-S, `lat` latency compensation 0..800 samples(?), `trim` +/-18 dB. Slot class 4.
- **Emulation approach:** Routing node with integer-sample delay compensation.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `egrp` | EXT GRP | OFF / LCL / AUX / A / B / C / SC / USB / CRD / MOD / PLAY / AES |  | enum | OFF |
| `ein` | EXT IN | 1 .. 64 |  | int | 1 |
| `emode` | EXT MODE | M / ST / M/S |  | enum | ST |
| `lat` | LATENCY | 0 .. 800 |  | int | 45 |
| `trim` | TRIM | -18 .. 18 | dB | lin | 0 |

#### HALL REVERB - `HALL`

- **UI name:** HALL REVERB  |  **snapshot `mdl`:** `HALL`  |  **WING-Edit internal:** `fx_pardef_hall_reverb`  |  slots: FX1-8 only
- **Emulates:** WING hall reverb (`hall_reverb`, likely Lexicon/TC-style algorithmic hall)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** pdel 0..200 ms, size 2..100, dcy 0.2..5 s, bass mult 0.5..2, damping 1..20 kHz, lo/hi cut, shape 0..250, spread, diffusion, mod speed.
- **Emulation approach:** FDN (8-16 lines) with input diffusers, size scales delay lengths, two-band decay (mult/damp), modulation.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 50 |
| `size` | SIZE | 2 .. 100 |  | int | 50 |
| `dcy` | DECAY | 0.2 .. 5 | s | log | 1.57 |
| `mult` | BASS MULT | 0.5 .. 2 |  | log | 0.95 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5k7 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 83 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 7200 |
| `shp` | SHAPE | 0 .. 250 |  | lin | 50 |
| `sprd` | SPREAD | 0 .. 50 |  | int | 25 |
| `diff` | DIFFUSION | 1 .. 30 |  | int | 25 |
| `mspd` | MOD SPD | 0 .. 100 |  | int | 30 |

#### ROOM REVERB - `ROOM`

- **UI name:** ROOM REVERB  |  **snapshot `mdl`:** `ROOM`  |  **WING-Edit internal:** `fx_pardef_lex_room`  |  slots: FX1-8 only
- **Emulates:** Lexicon 480L/224-family room (`lex_room`)
- **Behaviour:** Lexicon-style params: spin (modulation), echo L/R time + feedback (early reflection taps), shape/spread/size, bass mult.
- **Emulation approach:** Lexicon-style figure-8 allpass loop (Dattorro family) with echo taps; tune by ear vs references.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 40 |
| `size` | SIZE | 4 .. 76 | m | lin | 50 |
| `dcy` | DECAY | 0.3 .. 25 | s | log | 2.5 |
| `mult` | BASS MULT | 0.25 .. 4 |  | log | 1.2 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5k7 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 83 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 7200 |
| `shp` | SHAPE | 0 .. 250 |  | lin | 0 |
| `sprd` | SPREAD | 0 .. 50 |  | int | 25 |
| `diff` | DIFFUSION | 0 .. 100 |  | int | 40 |
| `spin` | SPIN | 0 .. 100 |  | int | 30 |
| `ecl` | ECHO L | 0 .. 1200 | ms | lin | 250 |
| `ecr` | ECHO R | 0 .. 1200 | ms | lin | 350 |
| `efl` | FEED L | -100 .. 100 | % | lin | 20 |
| `efr` | FEED R | -100 .. 100 | % | lin | 20 |

#### CHAMBER REVERB - `CHAMBER`

- **UI name:** CHAMBER REVERB  |  **snapshot `mdl`:** `CHAMBER`  |  **WING-Edit internal:** `fx_pardef_lex_chamber`  |  slots: FX1-8 only
- **Emulates:** Lexicon-style chamber (`lex_chamber`)
- **Behaviour:** As ROOM with echo levels (`ell/elr`, fader-type).
- **Emulation approach:** As ROOM.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 30 |
| `size` | SIZE | 4 .. 76 | m | lin | 25 |
| `dcy` | DECAY | 0.3 .. 25 | s | log | 1.35 |
| `mult` | BASS MULT | 0.25 .. 4 |  | log | 1.15 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5k |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 122 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 5k |
| `shp` | SHAPE | 0 .. 250 |  | lin | 100 |
| `sprd` | SPREAD | 0 .. 50 |  | int | 10 |
| `diff` | DIFFUSION | 0 .. 100 |  | int | 50 |
| `spin` | SPIN | 0 .. 100 |  | int | 30 |
| `ecl` | ECHO L | 0 .. 300 | ms | lin | 91 |
| `ecr` | ECHO R | 0 .. 300 | ms | lin | 115 |
| `ell` | ECHO L | fader (-oo .. +10 dB) | dB | fader | -25 |
| `elr` | ECHO R | fader (-oo .. +10 dB) | dB | fader | -25 |

#### PLATE REVERB - `PLATE`

- **UI name:** PLATE REVERB  |  **snapshot `mdl`:** `PLATE`  |  **WING-Edit internal:** `fx_pardef_lex_plate`  |  slots: FX1-8 only
- **Emulates:** Lexicon-style plate (`lex_plate`)
- **Behaviour:** As ROOM with `att` (attack) instead of shape.
- **Emulation approach:** Dattorro plate.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 30 |
| `size` | SIZE | 4 .. 76 | m | lin | 20 |
| `dcy` | DECAY | 0.3 .. 25 | s | log | 1.61 |
| `mult` | BASS MULT | 0.25 .. 4 |  | log | 1.2 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 9k7 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 105 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 9k5 |
| `att` | ATTACK | 0 .. 100 |  | lin | 60 |
| `sprd` | SPREAD | 0 .. 50 |  | int | 40 |
| `diff` | DIFFUSION | 0 .. 100 |  | int | 60 |
| `spin` | SPIN | 0 .. 100 |  | int | 30 |
| `ecl` | ECHO L | 0 .. 1200 | ms | lin | 230 |
| `ecr` | ECHO R | 0 .. 1200 | ms | lin | 200 |
| `efl` | FEED L | -100 .. 100 | % | lin | 15 |
| `efr` | FEED R | -100 .. 100 | % | lin | 15 |

#### CONCERT REVERB - `CONCERT`

- **UI name:** CONCERT REVERB  |  **snapshot `mdl`:** `CONCERT`  |  **WING-Edit internal:** `fx_pardef_lex_concert`  |  slots: FX1-8 only
- **Emulates:** Lexicon-style concert hall (`lex_concert`)
- **Behaviour:** Adds reflections (rfl/rfr times, levels), chorus `crs`, mod depth.
- **Emulation approach:** As ROOM + chorus.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 40 |
| `size` | SIZE | 20 .. 76 | m | lin | 50 |
| `dcy` | DECAY | 0.3 .. 29 | s | log | 2.0 |
| `mult` | BASS MULT | 0.25 .. 4 |  | log | 1.2 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5k7 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 83 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 7200 |
| `shp` | SHAPE | 0 .. 50 |  | lin | 15 |
| `sprd` | SPREAD | 0 .. 50 |  | int | 25 |
| `diff` | DIFFUSION | 0 .. 100 |  | int | 25 |
| `depth` | DEPTH | 1 .. 16 |  | int | 12 |
| `rfl` | REFL L | 0 .. 1200 | ms | lin | 270 |
| `rfr` | REFL R | 0 .. 1200 | ms | lin | 330 |
| `rfll` | REFL L | fader (-oo .. +10 dB) | dB | fader | -20 |
| `rflr` | REFL R | fader (-oo .. +10 dB) | dB | fader | -20 |
| `spin` | SPIN | 0 .. 100 |  | int | 30 |
| `crs` | CHORUS | 0 .. 100 |  | int | 10 |

#### AMBIENCE - `AMBI`

- **UI name:** AMBIENCE  |  **snapshot `mdl`:** `AMBI`  |  **WING-Edit internal:** `fx_pardef_ambience`  |  slots: FX1-8 only
- **Emulates:** Ambience (`ambience`, TC/Lexicon "ambience" short reverb)
- **Behaviour:** size, decay 0.2..7.3 s, tail gain, damping, diffusion, mod.
- **Emulation approach:** Short FDN/ER generator.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 4 |
| `size` | SIZE | 2 .. 100 |  | lin | 60 |
| `dcy` | DECAY | 0.2 .. 7.3 | s | log | 0.84 |
| `tail` | TAIL GAIN | 0 .. 100 |  | int | 70 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5k |
| `diff` | DIFFUSION | 1 .. 30 |  | int | 30 |
| `mod` | MOD SPD | 0 .. 100 |  | int | 20 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 70 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 8k |

#### VINTAGE ROOM - `V-ROOM`

- **UI name:** VINTAGE ROOM  |  **snapshot `mdl`:** `V-ROOM`  |  **WING-Edit internal:** `fx_pardef_vintage_room`  |  slots: FX1-8 only
- **Emulates:** Quantec Room Simulator QRS (X32 "Vintage Room") (`vintage_room`)
- **Behaviour:** density, ER level, low/high multipliers, freeze, ER L/R delays, add, level.
- **Emulation approach:** Quantec-like dense FDN; freeze = infinite feedback, input muted.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 20 |
| `size` | SIZE | 1 .. 50 |  | int | 15 |
| `dcy` | DECAY | 0.1 .. 20 | s | log | 1.14 |
| `dens` | DENSITY | 1 .. 30 |  | lin | 30 |
| `erlvl` | ER LEVEL | fader (-oo .. +10 dB) | dB | fader | 30 |
| `lmult` | LO MULT | 0.1 .. 10 |  | log | 1.1 |
| `hmult` | HI MULT | 0.1 .. 10 |  | log | 0.7 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 80 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 10k |
| `frz` | FREEZE | off/on |  | bool | 0 |
| `erl` | ER LEFT | 0 .. 200 | ms | lin | 28 |
| `err` | ER RIGHT | 0 .. 200 | ms | lin | 34 |
| `add` | ADD | off/on |  | bool | 0 |
| `lvl` | LEVEL | -6 .. 6 | dB | lin | 0 |

#### VINTAGE REVERB - `V-REV`

- **UI name:** VINTAGE REVERB  |  **snapshot `mdl`:** `V-REV`  |  **WING-Edit internal:** `fx_pardef_vintage_reverb`  |  slots: FX1-8 only
- **Emulates:** EMT 250 (X32 "Vintage Reverb") (`vintage_reverb`)
- **Behaviour:** decay 0.4..4.5 s, lo/hi mult, mod, front/rear outputs, transformer switch.
- **Emulation approach:** EMT 250 style: modulated delay network + output taps; transformer saturation.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 120 | ms | int | 20 |
| `dcy` | DECAY | 0.4 .. 4.5 | s | lin | 1.5 |
| `lmult` | LO MULT | 0.5 .. 2 |  | log | 1 |
| `hmult` | HI MULT | 0.25 .. 0.67 |  | log | 0.7 |
| `mod` | MOD SPD | 0 .. 100 |  | int | 100 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 75 |
| `hc` | HI CUT | 5000 .. 20000 | Hz | log | 12k |
| `out` | OUTPUT | FRONT / REAR |  | enum | 0 |
| `trans` | XFORMER | off/on |  | bool | 0 |

#### VINTAGE PLATE - `V-PLATE`

- **UI name:** VINTAGE PLATE  |  **snapshot `mdl`:** `V-PLATE`  |  **WING-Edit internal:** `fx_pardef_vintage_plate`  |  slots: FX1-8 only
- **Emulates:** EMT 140 plate (X32 "Vintage Plate") (`vintage_plate`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** pdel 0..250, decay 1..6 s, lo cut, colour.
- **Emulation approach:** Dattorro/plate waveguide with colour tilt.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 250 | ms | int | 20 |
| `dcy` | DECAY | 1 .. 6 | s | log | 2 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 80 |
| `col` | COLOR | -20 .. 20 |  | lin | 8 |

#### GATED REVERB - `GATED`

- **UI name:** GATED REVERB  |  **snapshot `mdl`:** `GATED`  |  **WING-Edit internal:** `fx_pardef_gated_reverb`  |  slots: FX1-8 only
- **Emulates:** AMS RMX16 "Nonlin"-style gated reverb (`gated_reverb`)
- **Behaviour:** attack, decay 0.14..1 s, density, diffusion, spread, hi shelf.
- **Emulation approach:** Dense ER burst with gated envelope.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 8 |
| `att` | ATTACK | 0 .. 30 |  | int | 6 |
| `dcy` | DECAY | 0.14 .. 1 | s | log | 0.3 |
| `dens` | DENSITY | 1 .. 50 |  | int | 22 |
| `diff` | DIFFUSION | 1 .. 30 |  | int | 30 |
| `sprd` | SPREAD | 0 .. 100 |  | int | 60 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 80 |
| `hsf` | HI FREQ | 200 .. 20000 | Hz | log | 5k |
| `hsg` | HI GAIN | -30 .. 0 | dB | lin | -12 |

#### REVERSE REVERB - `REVERSE`

- **UI name:** REVERSE REVERB  |  **snapshot `mdl`:** `REVERSE`  |  **WING-Edit internal:** `fx_pardef_reverse_reverb`  |  slots: FX1-8 only
- **Emulates:** AMS RMX16 "Reverse"-style (`reverse_reverb`)
- **Behaviour:** rise, decay, diffusion, spread, hi shelf.
- **Emulation approach:** Reverse-envelope ER train.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 30 |
| `rise` | RISE | 0 .. 50 |  | int | 40 |
| `dcy` | DECAY | 0.14 .. 1 | s | log | 0.6 |
| `diff` | DIFFUSION | 1 .. 30 |  | int | 24 |
| `sprd` | SPREAD | 0 .. 100 |  | int | 78 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 100 |
| `hsf` | HI FREQ | 200 .. 20000 | Hz | log | 8k |
| `hsg` | HI GAIN | -30 .. 0 | dB | lin | -12 |

#### DELAY/REVERB - `DEL/REV`

- **UI name:** DELAY/REVERB  |  **snapshot `mdl`:** `DEL/REV`  |  **WING-Edit internal:** `fx_pardef_delreverb`  |  slots: FX1-8 only
- **Emulates:** Delay + reverb combo (`delreverb`)
- **Behaviour:** delay time/feedback/HC, delay level, delay->reverb, reverb params, input->reverb.
- **Emulation approach:** Stereo delay + plate/hall into each other.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `time` | TIME | 1 .. 3000 | ms | lin | 240 |
| `feed` | FEED | 0 .. 100 | % | lin | 40 |
| `fhc` | FEED HC | 200 .. 20000 | Hz | log | 7600 |
| `dly` | DLY LVL | 0 .. 100 | % | lin | 40 |
| `d2r` | DLY->REV | 0 .. 100 | % | lin | 70 |
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 24 |
| `size` | SIZE | 2 .. 100 |  | int | 52 |
| `dcy` | DECAY | 0.1 .. 5 | s | log | 1.55 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 5600 |
| `rlc` | REV LCUT | 20 .. 400 | Hz | log | 90 |
| `i2r` | IN->REV | 0 .. 100 | % | lin | 0 |

#### SHIMMER REVERB - `SHIMMER`

- **UI name:** SHIMMER REVERB  |  **snapshot `mdl`:** `SHIMMER`  |  **WING-Edit internal:** `fx_pardef_shim_reverb`  |  slots: FX1-8 only
- **Emulates:** Shimmer reverb (`shim_reverb`)
- **Behaviour:** pdel, size 1..50, decay 1..20 s, cuts, damping, shimmer (pitch +12 in feedback), shine.
- **Emulation approach:** FDN with pitch shifter in feedback path.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 250 | ms | int | 100 |
| `size` | SIZE | 1 .. 50 |  | int | 25 |
| `dcy` | DECAY | 1 .. 20 | s | log | 3 |
| `lc` | LO CUT | 25 .. 250 | Hz | log | 80 |
| `hc` | HI CUT | 500 .. 7000 | Hz | log | 5000 |
| `damp` | DAMPING | 0 .. 100 | % | lin | 50 |
| `shim` | SHIMMER | 0 .. 100 | % | lin | 50 |
| `shine` | SHINE | 0 .. 100 | % | lin | 50 |

#### SPRING REVERB - `SPRING`

- **UI name:** SPRING REVERB  |  **snapshot `mdl`:** `SPRING`  |  **WING-Edit internal:** `fx_pardef_spring_reverb`  |  slots: FX1-8 only
- **Emulates:** Spring reverb (`spring_reverb`, Fender/Accutronics tank)
- **Behaviour:** decay, density, bass, treble.
- **Emulation approach:** Dispersive allpass-chain spring model (Valimaki/Parker).
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `dcy` | DECAY | 1.5 .. 6 | s | log | 3 |
| `dens` | DENSITY | 1 .. 30 |  | lin | 20 |
| `low` | BASS | 1 .. 50 |  | lin | 15 |
| `high` | TREBLE | 1 .. 50 |  | lin | 35 |

#### DIMENSION CRS - `DIMCRS`

- **UI name:** DIMENSION CRS  |  **snapshot `mdl`:** `DIMCRS`  |  **WING-Edit internal:** `fx_pardef_dimension_crs`  |  slots: FX1-8 only
- **Emulates:** Roland SDD-320 Dimension D (`dimension_crs`)
- **Behaviour:** 4 mode buttons sw1..sw4 (combinable), input MONO/STEREO, dry switch.
- **Emulation approach:** BBD-style dual modulated delay with fixed SDD-320 rates/depths per button.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `sw1` | SW 1 | off/on |  | bool | 1 |
| `sw2` | SW 2 | off/on |  | bool | 0 |
| `sw3` | SW 3 | off/on |  | bool | 1 |
| `sw4` | SW 4 | off/on |  | bool | 0 |
| `in` | IN MODE | MONO / STEREO |  | enum | stereo |
| `drysw` | DRY | off/on |  | bool | 1 |

#### STEREO CHORUS - `CHORUS`

- **UI name:** STEREO CHORUS  |  **snapshot `mdl`:** `CHORUS`  |  **WING-Edit internal:** `fx_pardef_chorus`  |  slots: FX1-8 only
- **Emulates:** Stereo chorus (`chorus`)
- **Behaviour:** delay L/R 5..50 ms, depth, speed 0.05..5 Hz, waveform, phase, spread, cuts, mix.
- **Emulation approach:** Modulated fractional delays.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lc` | LO CUT | 20 .. 400 | Hz | log | 76 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 12k |
| `wave` | WAVEFORM | 0 .. 100 |  | lin | 100 |
| `phase` | PHASE | 0 .. 180 |  | lin | 120 |
| `mix` | MIX | 0 .. 100 | % | lin | 50 |
| `dlyl` | DELAY L | 5 .. 50 | ms | lin | 15.1 |
| `dlyr` | DELAY R | 5 .. 50 | ms | lin | 16.6 |
| `depl` | DEPTH L | 0 .. 100 | % | lin | 20 |
| `depr` | DEPTH R | 0 .. 100 | % | lin | 20 |
| `sprd` | SPREAD | 0 .. 100 | % | lin | 100 |
| `spd` | SPEED | 0.05 .. 5 | Hz | log | 0.48 |

#### STEREO FLANGER - `FLANGER`

- **UI name:** STEREO FLANGER  |  **snapshot `mdl`:** `FLANGER`  |  **WING-Edit internal:** `fx_pardef_flanger`  |  slots: FX1-8 only
- **Emulates:** Stereo flanger (`flanger`)
- **Behaviour:** delay 0.5..20 ms, depth, speed, feedback +/-90 % with feedback filters.
- **Emulation approach:** Modulated short delay + feedback.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lc` | LO CUT | 20 .. 400 | Hz | log | 80 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 15k |
| `flc` | FEED LC | 20 .. 400 | Hz | log | 122 |
| `fhc` | FEED HC | 200 .. 20000 | Hz | log | 11k5 |
| `mix` | MIX | 0 .. 100 | % | lin | 50 |
| `dlyl` | DELAY L | 0.5 .. 20 | ms | lin | 4.2 |
| `dlyr` | DELAY R | 0.5 .. 20 | ms | lin | 4.2 |
| `depl` | DEPTH L | 0 .. 100 | % | lin | 80 |
| `depr` | DEPTH R | 0 .. 100 | % | lin | 80 |
| `phase` | PHASE | 0 .. 180 |  | lin | 180 |
| `spd` | SPEED | 0.05 .. 5 | Hz | log | 0.32 |
| `feed` | FEED | -90 .. 90 | % | lin | 70 |

#### STEREO DELAY - `ST-DL`

- **UI name:** STEREO DELAY  |  **snapshot `mdl`:** `ST-DL`  |  **WING-Edit internal:** `fx_pardef_stereo_delay`  |  slots: FX1-8 only
- **Emulates:** Stereo delay (`stereo_delay`)
- **Behaviour:** time 1..3000 ms, mode ST/X(cross)/M, factor, L/R pattern, offset +/-50, feedback + LC/HC in loop.
- **Emulation approach:** Delay lines with feedback filters; tempo factor math.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `time` | TIME | 1 .. 3000 | ms | lin | 333 |
| `mode` | MODE | ST / X / M |  | enum | st |
| `fact` | FACTOR | 1/4 / 1/3 / 1/2 / 2/3 / 3/4 / 1 / 3/8 / 5/4 / 4/3 / 3/2 / 2 |  | enum | 1 |
| `pat` | PATTERN | 1/2:1 / 2/3:1 / 3/4:1 / 7/8:1 / 1:1 / 1:9/8 / 1:5/4 / 1:4/3 / 1:3/2 |  | enum | 1:1 |
| `offs` | OFFSET | -50 .. 50 | ms | int | 0 |
| `feed` | FEED | 0 .. 100 | % | lin | 35 |
| `flc` | FEED LCUT | 20 .. 400 | Hz | log | 100 |
| `fhc` | FEED HCUT | 200 .. 20000 | Hz | log | 8k |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 100 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 10k |

#### ULTRATAP DELAY - `TAP-DL`

- **UI name:** ULTRATAP DELAY  |  **snapshot `mdl`:** `TAP-DL`  |  **WING-Edit internal:** `fx_pardef_ultratap_delay`  |  slots: FX1-8 only
- **Emulates:** Eventide-style UltraTap (`ultratap_delay`)
- **Behaviour:** time 1..2000 ms, repeats 1..16, slope +/-24 dB, factor, pdel, mode MOVE/JUMP/FOCUS/SPREAD, width, diffusion.
- **Emulation approach:** Multi-tap delay with tap gain slope and spacing modes.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `time` | TIME | 1 .. 2000 | ms | lin | 300 |
| `rep` | REPEATS | 1 .. 16 |  | int | 4 |
| `slp` | SLOPE | -24 .. 24 | dB | lin | -3 |
| `fact` | FACTOR | 1/3 / 1/2 / 2/3 / 3/4 / 1 / 5/4 / 4/3 / 3/2 / 2 |  | enum | 1 |
| `pdel` | PRE DLY | 0 .. 500 | ms | lin | 0 |
| `mode` | MODE | MOVE / JUMP / FOCUS / SPREAD |  | enum | move |
| `wid` | WIDTH | -100 .. 100 | % | lin | 70 |
| `diff` | DIFFUSION | 0 .. 100 |  | lin | 0 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 20 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 20k |

#### TAPE DELAY - `TAPE-DL`

- **UI name:** TAPE DELAY  |  **snapshot `mdl`:** `TAPE-DL`  |  **WING-Edit internal:** `fx_pardef_tape_delay`  |  slots: FX1-8 only
- **Emulates:** Tape echo (Roland RE-201/Echoplex-style) (`tape_delay`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** time 60..650 ms, sustain, drive, wow/flutter.
- **Emulation approach:** Delay with modulated read head, saturation and HF loss in feedback.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `time` | TIME | 60 .. 650 | ms | lin | 300 |
| `sust` | SUSTAIN | 0 .. 100 | % | lin | 20 |
| `drv` | DRIVE | 0 .. 100 | % | lin | 0 |
| `wf` | FLUTTER | 0 .. 100 | % | lin | 0 |

#### OILCAN DELAY - `OILCAN`

- **UI name:** OILCAN DELAY  |  **snapshot `mdl`:** `OILCAN`  |  **WING-Edit internal:** `fx_pardef_telray_delay`  |  slots: FX1-8 only
- **Emulates:** Tel-Ray oil-can delay (`telray_delay`)
- **Behaviour:** time, sustain, wobble, tone (0..10 knobs).
- **Emulation approach:** Modulated, heavily filtered feedback delay.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `time` | TIME | 0 .. 10 |  | lin | 6.5 |
| `sust` | SUSTAIN | 0 .. 10 |  | lin | 4 |
| `wb` | WOBBLE | 0 .. 10 |  | lin | 0 |
| `tone` | TONE | 0 .. 10 |  | lin | 5 |

#### BBD DELAY - `BBD-DL`

- **UI name:** BBD DELAY  |  **snapshot `mdl`:** `BBD-DL`  |  **WING-Edit internal:** `fx_pardef_bbd_delay`  |  slots: FX1-8 only
- **Emulates:** Bucket-brigade analog delay (Boss DM-2 style) (`bbd_delay`)
- **Behaviour:** delay 0..100, feedback 0..100.
- **Emulation approach:** BBD model: anti-alias/reconstruction LPF, companding noise, clock-modulated delay.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `dly` | DELAY | 0 .. 100 |  | lin | 65 |
| `fb` | FEED | 0 .. 100 | % | lin | 30 |

#### STEREO PITCH - `PITCH`

- **UI name:** STEREO PITCH  |  **snapshot `mdl`:** `PITCH`  |  **WING-Edit internal:** `fx_pardef_stereo_pitch`  |  slots: FX1-8 only
- **Emulates:** Stereo pitch shifter (`stereo_pitch`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** semitones +/-12, cents +/-50, delay, cuts, mix.
- **Emulation approach:** Granular/overlap-add pitch shift (or Signalsmith Stretch).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `semi` | SEMITONES | -12 .. 12 |  | int | 0 |
| `cent` | CENT | -50 .. 50 |  | int | 0 |
| `dly` | DELAY | 0 .. 500 | ms | lin | 20 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 10 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 20k |
| `mix` | MIX | 0 .. 100 | % | lin | 100 |

#### DUAL PITCH - `D-PITCH`

- **UI name:** DUAL PITCH  |  **snapshot `mdl`:** `D-PITCH`  |  **WING-Edit internal:** `fx_pardef_dual_pitch`  |  slots: FX1-8 only
- **Emulates:** Dual pitch shifter (H3000-style) (`dual_pitch`)
- **Behaviour:** two voices: semi/cent/delay/pan/level.
- **Emulation approach:** Two PITCH engines.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `semi1` | SEMITONES 1 | -12 .. 12 |  | int | 0 |
| `cent1` | CENT 1 | -50 .. 50 |  | int | -10 |
| `dly1` | DELAY 1 | 0 .. 500 | ms | lin | 0 |
| `pan1` | PAN 1 | -100 .. 100 |  | lin | -50 |
| `lvl1` | LEVEL 1 | fader (-oo .. +10 dB) | dB | fader | 0 |
| `semi2` | SEMITONES 2 | -12 .. 12 |  | int | 0 |
| `cent2` | CENT 2 | -50 .. 50 |  | int | 10 |
| `dly2` | DELAY 2 | 0 .. 500 | ms | lin | 0 |
| `pan2` | PAN 2 | -100 .. 100 |  | lin | 50 |
| `lvl2` | LEVEL 2 | fader (-oo .. +10 dB) | dB | fader | 0 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 10 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 20k |

#### VSS3 REVERB - `VSS3`

- **UI name:** VSS3 REVERB  |  **snapshot `mdl`:** `VSS3`  |  **WING-Edit internal:** `fx_pardef_vss3_reverb`  |  slots: FX1-8 only
- **Emulates:** TC Electronic VSS3 (System 6000) (`vss3_reverb`)
- **Behaviour:** 114 presets + ER (type 24 spaces, size, pos, bal, LC, colour, level) and reverb (type, width, pdly, decay, diffuse, 4-band decay multipliers with crossovers, hi soft, lo shelf/damp, hi cut, mod type/rate/width). `load` and `view` are UI actions.
- **Emulation approach:** Separate ER generator (tap tables per room type) + multiband-decay FDN; presets as data tables.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `preset` | PRESET | Build / Small Booth / Home Room / Dialog Alley / Small Wood Room / A Small Room / Intimate Studio / Small Room / Tight & Natural / Room Conversation / Furnished Room 2 / Studio 20x20 ft / Drew Room / Piano Close / Clear Guitar Room / Wide Ambient Chamber / Small Dense Hall / Slap Hall / Acoustic Gtr Ambience / Clear Room / Livingroom / Band Rehearsal Room / The Studio / In The Room / Studio 40x40 ft / Hit Room / Ambient Hall / Stage and Hall / Acoustic Guitar Space / Medium Vocal Hall / Bright Theatre  / Big Empty Club / Venue Warm 1 / Concert 1 / Bright Guitar Hall / Concert Arena / Concert Piano / Piano Hall 1st Row / Empty Arena / Ballad Vocal Hall / Grand Vocal Hall / Large Warm Hall / Back There / WoodHall / Church / Sound Col / 5000 Hall / Cathedral / Large Church / Medium Church / Warm Cathedral / Cologne Cathedral / Drum Plate Stuff / Drum Wood Plate / Piano Plate / Stairway Plate / Slapback Plate / Ambient Plate / Silky Gold Plate / Gold Plate / EMT 141 / Leader Of The Band / VocalDry / Vocal Room / VocalWet / Slapback Vox 1 / Vocal Hall 1 / Vocal Chamber / Bright Male Vox / Vocal Bright / Vocal Deep / Vocal Female / Vocal Deep Male / Large Vocal Hall / Kick & Bass Ambience / Drum Room / Small Perc Room / Drum Room Xpander / Bright Shoe Gaze Snare / Snare Room Bright / Tom-Tom Reverb / Bossa Nova Perc Room / Hard Drum Space / Puk Drum Ambience / Overhead Mics / Dance Snare / Drum Perc Soft 1 / Perc Straight Tail / Store Room / Studio Small / The Alley / Near The Wall / WoodFlr / Large Office / Conference Room / Dance Studio / Forest 2 / StoneWall / Venue 1 / Small Stairway / Forest 1 / Airport PA / Small Tower Hall / On The Street / Dark Tunnel / Empty Nightclub / Parking Garage / Parking Distant / Long Swimmingpool |  | enum |  |
| `load` | LOAD | off/on |  | bool |  |
| `erpdly` | ER PDLY | 0 .. 100 | ms | lin | 11 |
| `ertype` | ER TYPE | ROYAL / THEATRE / CHURCH / GAS / CONCERT / ROYAL2 / V1-NEAR / V2-HARD / V3-SPREAD / V4-BUILD / V5-RANDOM / SLAP / CAR / PHONEBTH / BATHROOM / CONFRM9 / CONFRM30 / GARAGE / SWIMSTDM / AIRPORT / STREET / ALLEY / PIAZA / FOREST |  | enum | church |
| `ersize` | ER SIZE | SML / MED / LRG |  | enum | lrg |
| `erpos` | ER POS | NEAR / DIST |  | enum | near |
| `erbal` | ER BAL | -100 .. 100 |  | lin | 0 |
| `erlc` | ER LCUT | 20 .. 400 | Hz | log | 20 |
| `ercol` | ER COL | -40 .. 40 |  | lin | 40 |
| `erlvl` | ER LVL | fader (-oo .. +10 dB) | dB | fader | -6 |
| `rvtype` | REV TYPE | SMOOTH / NATURAL / ALIVE / FAST / X-WIDE / ALIVE2 |  | enum | smooth |
| `rvwid` | REV WID | NARROW / NORMAL / WIDE / X-WIDE |  | enum | x-wide |
| `rvpdly` | REV PDLY | 0 .. 200 | ms | lin | 0 |
| `dcy` | DECAY | 0.1 .. 20 | s | log | 2.88 |
| `diff` | DIFFUSE | -50 .. 50 |  | lin | 6 |
| `rvbal` | REV BAL | -100 .. 100 |  | lin | 0 |
| `rvlvl` | REV LVL | fader (-oo .. +10 dB) | dB | fader | -6 |
| `ldcy` | LO DCY | 0.1 .. 2.5 |  | lin | 1 |
| `lmdcy` | LOMID DCY | 0.1 .. 2.5 |  | lin | 1 |
| `hmdcy` | HIMID DCY | 0.1 .. 2.5 |  | lin | 1 |
| `hdcy` | HI DCY | 0.1 .. 2.5 |  | lin | 1 |
| `hsoft` | HI SOFT | -50 .. 50 |  | lin | -50 |
| `lxo` | LO X/O | 20 .. 500 | Hz | log | 92 |
| `mxo` | MID X/O | 200 .. 2000 | Hz | log | 774 |
| `hxo` | HI X/O | 500 .. 20000 | Hz | log | 3k8 |
| `lshv` | LO SHV | 20 .. 200 | Hz | log | 178 |
| `lsdmp` | LO DAMP | 0 .. -18 | dB | lin | 0 |
| `hcut` | HI CUT | 20 .. 20000 | Hz | log | 6k5 |
| `mtype` | MOD TYPE | A / B / C / D / E / F |  | enum | c |
| `mrate` | MOD RATE | -100 .. 100 |  | lin | -30 |
| `mwid` | MOD WID | 0 .. 200 |  | lin | 156 |
| `view` | VIEW | off/on |  | bool |  |

#### BLUE PLATE - `BPLATE`

- **UI name:** BLUE PLATE  |  **snapshot `mdl`:** `BPLATE`  |  **WING-Edit internal:** `fx_pardef_plate_reverb`  |  slots: FX1-8 only
- **Emulates:** "Blue Plate" plate reverb (`plate_reverb`; hardware reference unclear)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** pdel, size 2..100, decay, bass mult, damping, cuts, crossover, mod depth/speed, diffusion.
- **Emulation approach:** Modulated Dattorro plate.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pdel` | PRE DLY | 0 .. 200 | ms | int | 32 |
| `size` | SIZE | 2 .. 100 |  | int | 40 |
| `dcy` | DECAY | 0.2 .. 5 | s | log | 1.62 |
| `mult` | BASS MULT | 0.5 .. 2 |  | log | 1.1 |
| `damp` | DAMPING | 1000 .. 20000 | Hz | log | 6k5 |
| `lc` | LO CUT | 20 .. 400 | Hz | log | 60 |
| `hc` | HI CUT | 200 .. 20000 | Hz | log | 7200 |
| `xover` | XOVER | 20 .. 500 | Hz | log | 217 |
| `mdep` | MOD DEPTH | 1 .. 50 |  | lin | 20 |
| `mspd` | MOD SPD | 0 .. 100 |  | int | 20 |
| `diff` | DIFFUSION | 1 .. 30 |  | int | 25 |

#### GRAPHIC EQ - `GEQ`

- **UI name:** GRAPHIC EQ  |  **snapshot `mdl`:** `GEQ`  |  **WING-Edit internal:** `fx_pardef_geq`  |  slots: FX1-16
- **Emulates:** 31-band graphic EQ (`geq`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx; snapshot-only keys: `100`, `10k`, `125`, `12k5`, `160`, `16k`, `1k`, `1k25`, `1k6`, `20`, `200`, `20k`, `25`, `250`, `2k`, `2k5`, `31`, `315`, `3k15`, `40`, `400`, `4k`, `50`, `500`, `5k`, `63`, `630`, `6k3`, `80`, `800`, `8k`
- **Behaviour:** type STD/TRU (TRU = "true response": band interaction compensated so the fader curve equals the actual response), 31 ISO bands 20..20k, +/-15 dB in 0.25 dB steps (121 steps). Keys are the band names `20`,`25`,...,`1k`,...,`20k`.
- **Emulation approach:** Constant-Q 1/3-octave peaking bank; TRU mode = solve band-interaction (least-squares gain correction) on change.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `type` | GEQ TYPE | STD / TRU |  | enum |  |
| `20` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `25` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `31` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `50` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `63` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `80` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `100` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `125` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `160` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `200` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `250` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `315` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `400` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `500` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `630` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `800` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `1k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `1k25` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `1k6` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `2k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `2k5` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `3k15` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `4k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `5k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `6k3` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `8k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `10k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `12k5` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `16k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `20k` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |
| `TRIM` |  | lin [-15.00 .. 15.00 dB], 121 steps (from T003) | dB |  | 0.00 |

#### PIA 560 GEQ - `PIA`

- **UI name:** PIA 560 GEQ  |  **snapshot `mdl`:** `PIA`  |  **WING-Edit internal:** `fx_pardef_api560`  |  slots: FX1-16
- **Emulates:** API 560 graphic (`api560`)
- **Behaviour:** as EQ PIA + `mix`.
- **Emulation approach:** See EQ PIA.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `g` | GAIN | -12 .. 12 | dB | lin |  |
| `31` | 31 | -12 .. 12 | dB | lin |  |
| `63` | 63 | -12 .. 12 | dB | lin |  |
| `125` | 125 | -12 .. 12 | dB | lin |  |
| `250` | 250 | -12 .. 12 | dB | lin |  |
| `500` | 500 | -12 .. 12 | dB | lin |  |
| `1k` | 1k | -12 .. 12 | dB | lin |  |
| `2k` | 2k | -12 .. 12 | dB | lin |  |
| `4k` | 4k | -12 .. 12 | dB | lin |  |
| `8k` | 8k | -12 .. 12 | dB | lin |  |
| `16k` | 16k | -12 .. 12 | dB | lin |  |

#### DOUBLE VOCAL - `DOUBLE`

- **UI name:** DOUBLE VOCAL  |  **snapshot `mdl`:** `DOUBLE`  |  **WING-Edit internal:** `fx_pardef_doubler`  |  slots: FX1-16
- **Emulates:** Vocal doubler (`doubler`)
- **Behaviour:** mode TIGHT/LOOSE/GROUP/DETUNE/THICK, mix, spread.
- **Emulation approach:** Multiple short modulated pitch/delay voices.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mode` | MODE | TIGHT / LOOSE / GROUP / DETUNE / THICK |  | enum | group |
| `mix` | MIX | 0 .. 100 | % | lin | 50 |
| `sprd` | SPREAD | 0 .. 100 | % | lin | 50 |

#### PITCH FIX - `PCORR`

- **UI name:** PITCH FIX  |  **snapshot `mdl`:** `PCORR`  |  **WING-Edit internal:** `fx_pardef_pitchcorr`  |  slots: FX1-16
- **Emulates:** Pitch correction (`pitchcorr`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** speed, amount, A4 410..470 Hz, 12 note enables sw_c..sw_b.
- **Emulation approach:** Pitch detector (YIN/MPM) + PSOLA/phase-vocoder correction toward enabled scale notes.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `spd` | SPEED | 1 .. 100 |  | lin | 50 |
| `amnt` | AMOUNT | 0 .. 100 |  | lin | 25 |
| `a4` | A4 PITCH | 410 .. 470 |  | lin | 440 |
| `sw_c` |  | off/on |  | bool |  |
| `sw_db` |  | off/on |  | bool |  |
| `sw_d` |  | off/on |  | bool |  |
| `sw_eb` |  | off/on |  | bool |  |
| `sw_e` |  | off/on |  | bool |  |
| `sw_f` |  | off/on |  | bool |  |
| `sw_gb` |  | off/on |  | bool |  |
| `sw_g` |  | off/on |  | bool |  |
| `sw_ab` |  | off/on |  | bool |  |
| `sw_a` |  | off/on |  | bool |  |
| `sw_bb` |  | off/on |  | bool |  |
| `sw_b` |  | off/on |  | bool |  |

#### PRECISION LIMITER - `LIMITER`

- **UI name:** PRECISION LIMITER  |  **snapshot `mdl`:** `LIMITER`  |  **WING-Edit internal:** `fx_pardef_prec_limiter`  |  slots: FX1-16
- **Emulates:** Precision limiter (`prec_limiter`, look-ahead brickwall, Waves L2-like)
- **Behaviour:** in gain 0..18, out ceiling -18..0, squeeze 0..100, knee 0..10, auto gain, attack 0.05..1 ms, release 20..2000 ms.
- **Emulation approach:** Look-ahead peak limiter with true-peak option, programme-dependent release.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `gin` | IN GAIN | 0 .. 18 | dB | lin | 0 |
| `gout` | OUT GAIN | -18 .. 0 | dB | lin | -0.5 |
| `sqz` | SQUEEZE | 0 .. 100 |  | int | 0 |
| `knee` | KNEE | 0 .. 10 |  | int | 3 |
| `again` | AUTO GAIN | off/on |  | bool | 1 |
| `att` | ATTACK | 0.05 .. 1 | ms | lin | 0.05 |
| `rel` | RELEASE | 20 .. 2000 | ms | log | 500 |

#### 2-BAND DEESSER - `DE-S2`

- **UI name:** 2-BAND DEESSER  |  **snapshot `mdl`:** `DE-S2`  |  **WING-Edit internal:** `fx_pardef_deesser`  |  slots: FX1-16
- **Emulates:** SPL De-Esser (dual band) (`deesser`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** lo/hi band amount 0..50, solo, gender f/m (moves detection bands), mode st/ms.
- **Emulation approach:** Two band-pass detectors with auto-threshold (SPL is level-independent) -> band gain reduction.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `lo` | LOW | 0 .. 50 |  | lin | 0 |
| `hi` | HIGH | 0 .. 50 |  | lin | 0 |
| `los` | LOW (S) | 0 .. 50 |  | lin | 0 |
| `his` | HIGH (S) | 0 .. 50 |  | lin | 0 |
| `gdr` | GENDER | f / m |  | enum | f |
| `mode` | MODE | st / ms |  | enum | st |

#### ULTRA ENHANCER - `ENHANCE`

- **UI name:** ULTRA ENHANCER  |  **snapshot `mdl`:** `ENHANCE`  |  **WING-Edit internal:** `fx_pardef_uenhancer`  |  slots: FX1-16
- **Emulates:** Behringer Ultrafex/X32 "Enhancer" (`uenhancer`)
- **Behaviour:** stereo & mono level/pan, bass/mid/high gain, bass freq, mid Q, high freq, LMF spread, solo.
- **Emulation approach:** Band-split harmonic enhancer (filtered nonlinear generator added back).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `stlvl` | ST LVL | -100 .. 100 | % | lin | 0 |
| `lmf` | LMF SPRD | -100 .. 100 | % | lin | 0 |
| `mlvl` | MONO LVL | -100 .. 100 | % | lin | 0 |
| `st` | ST PAN | -100 .. 100 | % | lin | 0 |
| `m` | MONO PAN | -100 .. 100 | % | lin | 0 |
| `bass` | BASS GAIN | 0 .. 100 | % | lin | 0 |
| `mid` | MID GAIN | 0 .. 100 | % | lin | 0 |
| `high` | HIGH GAIN | 0 .. 100 | % | lin | 0 |
| `g` | GAIN | -12 .. 12 | dB | lin | 0 |
| `solo` | SOLO | off/on |  | bool | 0 |
| `bassf` | BASS FREQ | 1 .. 50 |  | lin | 15 |
| `midq` | MID Q | 1 .. 50 |  | lin | 15 |
| `highf` | HIGH FREQ | 1 .. 50 |  | lin | 25 |

#### EXCITER - `EXCITER`

- **UI name:** EXCITER  |  **snapshot `mdl`:** `EXCITER`  |  **WING-Edit internal:** `fx_pardef_exciter`  |  slots: FX1-16
- **Emulates:** Aphex Aural Exciter (`exciter`)
- **Behaviour:** tune 1..10 kHz, peak, zero-fill, timbre, harmonics, mix, dry off.
- **Emulation approach:** HP sidechain -> asymmetric waveshaper -> mix (Aphex Type C style).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `tune` | TUNE | 1000 .. 10000 | Hz | log | 3k5 |
| `peak` | PEAK | 0 .. 100 | % | lin | 28 |
| `zero` | ZFILL | 0 .. 100 | % | lin | 24 |
| `tmb` | TIMBRE | -50 .. 50 |  | lin | 14 |
| `hrm` | HARM | 0 .. 100 | % | lin | 30 |
| `mix` | MIX | 0 .. 100 | % | lin | 30 |
| `solo` | DRY OFF | off/on |  | bool | 0 |

#### PSYCHO BASS - `P-BASS`

- **UI name:** PSYCHO BASS  |  **snapshot `mdl`:** `P-BASS`  |  **WING-Edit internal:** `fx_pardef_ren_bass`  |  slots: FX1-16
- **Emulates:** Waves Renaissance Bass / MaxxBass psychoacoustic bass (`ren_bass`)
- **Behaviour:** intensity, bass gain, X/O freq 32..200 Hz, solo.
- **Emulation approach:** LP split -> harmonic generator (full-wave rectifier/polynomial) -> add.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `int` | INTENSITY | -24 .. 6 | dB | lin | -10 |
| `bass` | BASS GAIN | -60 .. 0 | dB | lin | -3 |
| `xf` | X/O FREQ | 32 .. 200 | Hz | log | 60 |
| `solo` | SOLO | off/on |  | bool | 0 |

#### ROTARY SPEAKER - `ROTARY`

- **UI name:** ROTARY SPEAKER  |  **snapshot `mdl`:** `ROTARY`  |  **WING-Edit internal:** `fx_pardef_rotary`  |  slots: FX1-16
- **Emulates:** Leslie rotary speaker (`rotary`)
- **Behaviour:** speed control STOP/SLOW/FAST, lo/hi rotor speeds, balance, mix, distance, drum/horn acceleration.
- **Emulation approach:** Doppler (modulated delay) + AM for horn and drum with inertia.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `sw` | SPD CTL | STOP / SLOW / FAST |  | enum | slow |
| `lo` | LO SPEED | 0.1 .. 4 | Hz | log | 0.66 |
| `hi` | HI SPEED | 4 .. 10 | Hz | log | 5 |
| `bal` | BALANCE | -100 .. 100 |  | lin | -10 |
| `mix` | MIX | 0 .. 100 | % | lin | 80 |
| `dist` | DISTANCE | 0 .. 100 |  | lin | 20 |
| `dac` | DRUM ACCEL | 0 .. 100 | % | lin | 80 |
| `hac` | HORN ACCEL | 0 .. 100 | % | lin | 70 |

#### PHASER - `PHASER`

- **UI name:** PHASER  |  **snapshot `mdl`:** `PHASER`  |  **WING-Edit internal:** `fx_pardef_phaser`  |  slots: FX1-16
- **Emulates:** Stereo phaser with envelope (`phaser`)
- **Behaviour:** speed, phase, wave, range, depth, env mod + AR, mix, stages 2..12, resonance.
- **Emulation approach:** Allpass chain with LFO/envelope modulation.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `spd` | SPEED | 0.05 .. 5 | Hz | log | 0.83 |
| `phase` | PHASE | 0 .. 180 |  | int | 120 |
| `wave` | WAVE | -50 .. 50 |  | int | -25 |
| `range` | RANGE | 2 .. 98 | % | int | 20 |
| `depth` | DEPTH | 0 .. 100 | % | int | 50 |
| `emod` | ENV MOD | -100 .. 100 | % | int | 0 |
| `att` | ATTACK | 10 .. 1000 | ms | log | 20 |
| `hld` | HOLD | 10 .. 2000 | ms | log | 50 |
| `rel` | RELEASE | 10 .. 1000 | ms | log | 200 |
| `mix` | MIX | 0 .. 100 | % | int | 50 |
| `stg` | STAGES | 2 .. 12 |  | int | 6 |
| `reso` | RESO | 0 .. 80 | % | int | 20 |

#### TREMOLO/PANNER - `PANNER`

- **UI name:** TREMOLO/PANNER  |  **snapshot `mdl`:** `PANNER`  |  **WING-Edit internal:** `fx_pardef_tremolo_pan`  |  slots: FX1-16
- **Emulates:** Tremolo/auto-panner (`tremolo_pan`)
- **Behaviour:** envelope AHR, env->speed/depth, speed, phase +/-180, wave, depth.
- **Emulation approach:** LFO AM / pan law.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `att` | ATTACK | 10 .. 1000 | ms | log | 20 |
| `hld` | HOLD | 10 .. 2000 | ms | log | 50 |
| `rel` | RELEASE | 10 .. 1000 | ms | log | 200 |
| `espd` | ENV>SPEED | 0 .. 100 | % | int | 0 |
| `edep` | ENV>DEPTH | 0 .. 100 | % | int | 0 |
| `spd` | SPEED | 0.05 .. 5 | Hz | log | 1.45 |
| `phase` | PHASE | -180 .. 180 |  | int | 180 |
| `wave` | WAVE | -50 .. 50 |  | int | 0 |
| `depth` | DEPTH | 0 .. 100 | % | int | 100 |

#### TAPE MACHINE - `TAPE`

- **UI name:** TAPE MACHINE  |  **snapshot `mdl`:** `TAPE`  |  **WING-Edit internal:** `fx_pardef_tapemach`  |  slots: FX1-16
- **Emulates:** Tape machine saturation (`tapemach`)
- **Behaviour:** drive -5..25 dB, speed 7.5..30 ips, low bump, high shelf, out gain.
- **Emulation approach:** Hysteresis tape model (Chowdhury, reimplemented) or simpler saturation + head-bump/HF EQ per speed.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `drv` | DRIVE | -5 .. 25 | dB | lin | 0 |
| `spd` | SPEED | 7.5 .. 30 |  | log | 15 |
| `low` | LOW BUMP | off/on |  | bool | 0 |
| `hi` | HIGH SHELV | off/on |  | bool | 0 |
| `out` | OUT GAIN | -12 .. 12 | dB | lin | 0 |

#### MOOD FILTER - `MOOD`

- **UI name:** MOOD FILTER  |  **snapshot `mdl`:** `MOOD`  |  **WING-Edit internal:** `fx_pardef_moodfilter`  |  slots: FX1-16
- **Emulates:** Envelope/LFO filter (Mu-Tron style) (`moodfilter`)
- **Behaviour:** base freq, LP/HP/BP/NOTCH, 12/24 dB, reso, drive, env amount + AHR, LFO amount/speed/phase/wave, mix.
- **Emulation approach:** SVF with modulated cutoff.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `fbase` | BASE | 20 .. 15000 | Hz | log | 100 |
| `filt` | TYPE | LP / HP / BP / NOTCH |  | enum | lp |
| `slope` | SLOPE | 12 / 24 |  | enum | 24 |
| `reso` | RESO | 0 .. 10 |  | lin | 0 |
| `drv` | DRIVE | 0 .. 10 |  | lin | 0 |
| `env` | ENV | -100 .. 100 | % | lin | 0 |
| `att` | ATTACK | 10 .. 250 | ms | log | 20 |
| `hld` | HOLD | 1 .. 500 | ms | log | 20 |
| `rel` | RELEASE | 10 .. 500 | ms | log | 100 |
| `mix` | MIX | 0 .. 100 | % | lin | 100 |
| `lfo` | LFO | 0 .. 100 | % | lin | 100 |
| `spd` | SPEED | 0.05 .. 20 | Hz | log | 1 |
| `phase` | PHASE | 0 .. 180 |  | int | 0 |
| `wave` | LFO WAVE | TRI / SIN / SAW+ / SAW- / RMP / SQU / RND |  | enum | sin |

#### SUB OCTAVER - `SUB`

- **UI name:** SUB OCTAVER  |  **snapshot `mdl`:** `SUB`  |  **WING-Edit internal:** `fx_pardef_suboct`  |  slots: FX1-16
- **Emulates:** Boss OC-2 style sub octaver (`suboct`)
- **Behaviour:** range LOW/MID/HIGH, octave-1 and octave-2 levels.
- **Emulation approach:** Zero-crossing flip-flop divider on LP-filtered input, envelope-follow.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `rng` | RANGE | LOW / MID / HIGH |  | enum | low |
| `oct1` | OCTAVE 1 | 0 .. 100 | % | lin | 50 |
| `oct2` | OCTAVE 2 | 0 .. 100 | % | lin | 0 |

#### RACK AMP - `RACKAMP`

- **UI name:** RACK AMP  |  **snapshot `mdl`:** `RACKAMP`  |  **WING-Edit internal:** `fx_pardef_rack_amp`  |  slots: FX1-16
- **Emulates:** Tech 21 SansAmp PSA-1 style (`rack_amp`)
- **Behaviour:** preamp, buzz, punch, crunch, drive, low/high EQ, out, cab.
- **Emulation approach:** Pre-EQ -> waveshaper -> tonestack -> cab IR.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `pre` | PREAMP | 0 .. 10 |  | lin | 5 |
| `buzz` | BUZZ | 0 .. 10 |  | lin | 5 |
| `punch` | PUNCH | 0 .. 10 |  | lin | 5 |
| `crunch` | CRUNCH | 0 .. 10 |  | lin | 5 |
| `drive` | DRIVE | 0 .. 10 |  | lin | 5 |
| `out` | OUT GAIN | 0 .. 10 |  | lin | 5 |
| `leq` | LOW EQ | 0 .. 10 |  | lin | 5 |
| `heq` | HIGH EQ | 0 .. 10 |  | lin | 5 |
| `cab` | CAB SIM | off/on |  | bool | 1 |

#### UK ROCK AMP - `UKROCK`

- **UI name:** UK ROCK AMP  |  **snapshot `mdl`:** `UKROCK`  |  **WING-Edit internal:** `fx_pardef_amp_jcm800`  |  slots: FX1-16
- **Emulates:** Marshall JCM800 (`amp_jcm800`)
- **Behaviour:** gain, bass, middle, treble, presence, master, out, sag, cab.
- **Emulation approach:** Triode stages (WDF/ADAA) + Marshall tonestack (Yeh) + power amp sag + cab IR.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `gain` | GAIN | 0 .. 10 |  | lin | 5 |
| `bass` | BASS | 0 .. 10 |  | lin | 5 |
| `mid` | MIDDLE | 0 .. 10 |  | lin | 5 |
| `treb` | TREBLE | 0 .. 10 |  | lin | 5 |
| `pres` | PRESENCE | 0 .. 10 |  | lin | 5 |
| `mstr` | MASTER | 0 .. 10 |  | lin | 5 |
| `out` | OUT GAIN | 0 .. 10 |  | lin | 5 |
| `sag` | SAG | 0 .. 10 |  | lin | 5 |
| `cab` | CAB SIM | off/on |  | bool | 1 |

#### ANGEL AMP - `ANGEL`

- **UI name:** ANGEL AMP  |  **snapshot `mdl`:** `ANGEL`  |  **WING-Edit internal:** `fx_pardef_amp_engl`  |  slots: FX1-16
- **Emulates:** ENGL high-gain amp (`amp_engl`)
- **Behaviour:** as UKROCK + mid boost, bright, bottom.
- **Emulation approach:** As UKROCK.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `gain` | GAIN | 0 .. 10 |  | lin | 5 |
| `bass` | BASS | 0 .. 10 |  | lin | 5 |
| `mid` | MIDDLE | 0 .. 10 |  | lin | 5 |
| `treb` | TREBLE | 0 .. 10 |  | lin | 5 |
| `pres` | PRESENCE | 0 .. 10 |  | lin | 5 |
| `mstr` | MASTER | 0 .. 10 |  | lin | 5 |
| `out` | OUT GAIN | 0 .. 10 |  | lin | 5 |
| `sag` | SAG | 0 .. 10 |  | lin | 5 |
| `cab` | CAB SIM | off/on |  | bool | 1 |
| `midb` | MID BOOST | off/on |  | bool | 0 |
| `bri` | BRIGHT | off/on |  | bool | 0 |
| `bot` | BOTTOM | off/on |  | bool | 0 |

#### JAZZ CLEAN AMP - `JAZZC`

- **UI name:** JAZZ CLEAN AMP  |  **snapshot `mdl`:** `JAZZC`  |  **WING-Edit internal:** `fx_pardef_amp_jc120`  |  slots: FX1-16
- **Emulates:** Roland JC-120 Jazz Chorus (`amp_jc120`)
- **Behaviour:** volume, bass, middle, treble, out, bright, cab.
- **Emulation approach:** Solid-state clean amp + tonestack + cab.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `vol` | VOLUME | 0 .. 10 |  | lin | 5 |
| `bass` | BASS | 0 .. 10 |  | lin | 5 |
| `mid` | MIDDLE | 0 .. 10 |  | lin | 5 |
| `treb` | TREBLE | 0 .. 10 |  | lin | 5 |
| `out` | OUT GAIN | 0 .. 10 |  | lin | 5 |
| `bri` | BRIGHT | off/on |  | bool | 0 |
| `cab` | CAB SIM | off/on |  | bool | 1 |

#### DELUXE AMP - `DELUXE`

- **UI name:** DELUXE AMP  |  **snapshot `mdl`:** `DELUXE`  |  **WING-Edit internal:** `fx_pardef_amp_deluxe`  |  slots: FX1-16
- **Emulates:** Fender Deluxe Reverb (`amp_deluxe`)
- **Behaviour:** volume, bass, treble, out, sag, cab.
- **Emulation approach:** As UKROCK.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `vol` | VOLUME | 1 .. 10 |  | lin | 5.5 |
| `bass` | BASS | 1 .. 10 |  | lin | 5.5 |
| `treb` | TREBLE | 1 .. 10 |  | lin | 5.5 |
| `out` | OUT GAIN | 1 .. 10 |  | lin | 5.5 |
| `sag` | SAG | 1 .. 10 |  | lin | 5.5 |
| `cab` | CAB SIM | off/on |  | bool | 1 |

#### BODYREZ - `BODY`

- **UI name:** BODYREZ  |  **snapshot `mdl`:** `BODY`  |  **WING-Edit internal:** `fx_pardef_bodyrez`  |  slots: FX1-16
- **Emulates:** TC Electronic BodyRez (`bodyrez`)
- **Behaviour:** single `body` 0..100 amount (acoustic piezo pickup enhancer).
- **Emulation approach:** Fixed EQ/resonance curve scaled by amount.
- **Difficulty:** S

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `body` | BODY | 0 .. 100 |  | lin |  |

#### SOUL ANALOGUE - `SOUL`

- **UI name:** SOUL ANALOGUE  |  **snapshot `mdl`:** `SOUL`  |  **WING-Edit internal:** `fx_pardef_ssl_g`  |  slots: FX1-16
- **Emulates:** SSL G EQ as rack FX
- **Behaviour:** EQ model params (see EQ SOUL) + `mix`. Parameter list is attached from the EQ pardef at runtime (only `mix` extracted here).
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `lf` | LO FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 2.0 |
| `lg` | LO GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lmf` | LM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 6.0 |
| `lmf3` | LM /3 | int [0 .. 1] (from T003) |  |  | 1 |
| `lmq` | LM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `lmg` | LM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hmf` | HM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 3.0 |
| `hmf3` | HM x3 | int [0 .. 1] (from T003) |  |  | 0 |
| `hmq` | HM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmg` | HM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hf` | HF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 8.0 |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |

#### EVEN 88 FORMANT - `E88`

- **UI name:** EVEN 88 FORMANT  |  **snapshot `mdl`:** `E88`  |  **WING-Edit internal:** `fx_pardef_neve88rs`  |  slots: FX1-16
- **Emulates:** Neve 88RS EQ as rack FX
- **Behaviour:** as EQ E88 + mix.
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `lf` | LF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 2.0 |
| `lg` | LF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lq` | LF Q | LOW / HIGH (from T003) |  |  | LOW |
| `lt` | LF TYPE | BELL / SHELV (from T003) |  |  | SHELV |
| `lmf` | LM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 3.0 |
| `lmg` | LM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lmq` | LM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmf` | HM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmg` | HM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hmq` | HM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hf` | HF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 7.0 |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hq` | HF Q | LOW / HIGH (from T003) |  |  | LOW |
| `ht` | HF TYPE | BELL / SHELV (from T003) |  |  | SHELV |

#### EVEN 84 - `E84`

- **UI name:** EVEN 84  |  **snapshot `mdl`:** `E84`  |  **WING-Edit internal:** `fx_pardef_neve84`  |  slots: FX1-16
- **Emulates:** Neve 1084 as rack FX
- **Behaviour:** as EQ E84 + mix.
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `g` | GAIN | lin [-20.0 .. 20.0 dB], 81 steps (from T003) | dB |  | 0.0 |
| `lf` | LF FREQ | OFF / 35 / 60 / 110 / 220 (from T003) |  |  | 35 |
| `lg` | LF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `mf` | MID FREQ | OFF / 350 / 700 / 1k6 / 3k2 / 4k8 / 7k2 (from T003) |  |  | 1k6 |
| `mg` | MID GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `mq` | MID Q | LOW / HIGH (from T003) |  |  | LOW |
| `hf` | HF FREQ | 10k / 12k / 16k / OFF (from T003) |  |  | 16k |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |

#### FORTISSIMO 110 - `F110`

- **UI name:** FORTISSIMO 110  |  **snapshot `mdl`:** `F110`  |  **WING-Edit internal:** `fx_pardef_isa110`  |  slots: FX1-16
- **Emulates:** Focusrite ISA110 as rack FX
- **Behaviour:** as EQ F110 + mix.
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `peq` | PEQ ON | off/on |  | bool | 1 |
| `lmf` | LM FREQ | 0 .. 10 |  | lin | 7 |
| `lmg` | LM GAIN | -5 .. 5 |  | lin | 0 |
| `lmq` | LM Q | 0 .. 10 |  | lin | 5 |
| `lmf3` | LM x3 | off/on |  | bool | 0 |
| `hmf` | HM FREQ | 0 .. 10 |  | lin | 3 |
| `hmg` | HM GAIN | -5 .. 5 |  | lin | 0 |
| `hmq` | HM Q | 0 .. 10 |  | lin | 5 |
| `hmf3` | HM x3 | off/on |  | bool | 0 |
| `shv` | SHV ON | off/on |  | bool | 1 |
| `lf` | LF FREQ | 33 / 56 / 95 / 160 / 270 / 460 | Hz | enum | 56 |
| `lg` | LF GAIN | -5 .. 5 |  | lin | 0 |
| `hf` | HF FREQ | 3k3 / 4k7 / 6k8 / 10k / 15k / 18k | Hz | enum | 10k |
| `hg` | HF GAIN | -5 .. 5 |  | lin | 0 |
| `g` | GAIN | -18 .. 18 |  | lin | 0 |

#### PULSAR - `PULSAR`

- **UI name:** PULSAR  |  **snapshot `mdl`:** `PULSAR`  |  **WING-Edit internal:** `fx_pardef_pultec_duo`  |  slots: FX1-16
- **Emulates:** Pultec as rack FX
- **Behaviour:** as EQ PULSAR + mix.
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `eq1` | EQ1 ON | int [0 .. 1] (from T003) |  |  | 1 |
| `1lb` | LF BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1latt` | LF ATT | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1lf` | LF FREQ | 20 / 30 / 60 / 100 (from T003) | Hz |  | 30 |
| `1hw` | HF WID | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `1hb` | HF BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1hf` | HF FREQ | 3k / 4k / 5k / 8k / 10k / 12k / 16k (from T003) | Hz |  | 10k |
| `1hatt` | HF ATT | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1hattf` | HF ATT F | 5k / 10k / 20k (from T003) | Hz |  | 20k |
| `eq5` | EQ5 ON | int [0 .. 1] (from T003) |  |  | 1 |
| `5lb` | LM BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5lf` | LF FREQ | 200 / 300 / 500 / 700 / 1k (from T003) | Hz |  | 300 |
| `5md` | MID DIP | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5mf` | MID FREQ | 200 / 300 / 500 / 700 / 1k / 1k5 / 2k / 3k / 4k / 5k / 7k (from T003) | Hz |  | 1k |
| `5hb` | HM BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5hf` | HF FREQ | 1k5 / 2k / 3k / 4k / 5k (from T003) | Hz |  | 3k |

#### MACH EQ4 - `MACH4`

- **UI name:** MACH EQ4  |  **snapshot `mdl`:** `MACH4`  |  **WING-Edit internal:** `fx_pardef_maag4`  |  slots: FX1-16
- **Emulates:** Maag EQ4 as rack FX
- **Behaviour:** as EQ MACH4 + mix.
- **Emulation approach:** Re-use EQ model.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin |  |
| `sub` | SUB | -5 .. 5 |  | lin |  |
| `40` | 40 | -5 .. 5 |  | lin |  |
| `160` | 160 | -5 .. 5 |  | lin |  |
| `650` | 650 | -5 .. 5 |  | lin |  |
| `2k5` | 2k5 | -5 .. 5 |  | lin |  |
| `air` | AIR | 0 .. 10 |  | lin |  |
| `airm` | AIR MODE | OFF / 2k5 / 5k / 10k / 20k / 40k |  | enum |  |
| `again` | AUTO | off/on |  | bool |  |

#### COMBINATOR - `C5-CMB`

- **UI name:** COMBINATOR  |  **snapshot `mdl`:** `C5-CMB`  |  **WING-Edit internal:** `fx_pardef_combinator`  |  slots: FX1-16
- **Emulates:** 5-band multiband compressor "Combinator" (X32 Combinator)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** global thr/gain/ratio/slope 24/48 (crossover slope)/attack/release/auto release/SBC (spectral balance control) speed+on/band select, per-band thr/gain offsets +/-10, bypass, stereo width +/-50, mix, band solo.
- **Emulation approach:** 5-band LR crossover + 5 comps + spectral balance control (slow inter-band gain normalisation) + M/S width per band.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `thr` | THR | -40 .. 0 | dB | lin | 0 |
| `gain` | GAIN | -10 .. 10 | dB | lin | 0 |
| `ratio` | RATIO | 1.1 / 1.2 / 1.3 / 1.5 / 1.7 / 2 / 2.5 / 3 / 3.5 / 4 / 5 / 7 / 10 / 100 |  | enum | 2.5 |
| `slope` | SLOPE | 24 / 48 |  | enum | 48 |
| `bandsel` | BAND SELECT | 1 .. 5 |  | int | 3 |
| `att` | ATTACK | 0 .. 20 |  | lin | 5 |
| `rel` | RELEASE | 20 .. 3000 | ms | log | 500 |
| `arel` | AUTO REL | off/on |  | bool | 1 |
| `sbc` | SBC SPEED | 1 .. 10 |  | lin | 5 |
| `sbcon` | SBC ON | off/on |  | bool | 1 |
| `thr_1` | 1-THR | -10 .. 10 | dB | lin | 0 |
| `thr_2` | 2-THR | -10 .. 10 | dB | lin | 0 |
| `thr_3` | 3-THR | -10 .. 10 | dB | lin | 0 |
| `thr_4` | 4-THR | -10 .. 10 | dB | lin | 0 |
| `thr_5` | 5-THR | -10 .. 10 | dB | lin | 0 |
| `gain_1` | 1-GAIN | -10 .. 10 | dB | lin | 0 |
| `gain_2` | 2-GAIN | -10 .. 10 | dB | lin | 0 |
| `gain_3` | 3-GAIN | -10 .. 10 | dB | lin | 0 |
| `gain_4` | 4-GAIN | -10 .. 10 | dB | lin | 0 |
| `gain_5` | 5-GAIN | -10 .. 10 | dB | lin | 0 |
| `byp_1` | 1-BYP | off/on |  | bool | 0 |
| `byp_2` | 2-BYP | off/on |  | bool | 0 |
| `byp_3` | 3-BYP | off/on |  | bool | 0 |
| `byp_4` | 4-BYP | off/on |  | bool | 0 |
| `byp_5` | 5-BYP | off/on |  | bool | 0 |
| `width_1` | 1-WIDTH | -50 .. 50 |  | lin | 0 |
| `width_2` | 2-WIDTH | -50 .. 50 |  | lin | 0 |
| `width_3` | 3-WIDTH | -50 .. 50 |  | lin | 0 |
| `width_4` | 4-WIDTH | -50 .. 50 |  | lin | 0 |
| `width_5` | 5-WIDTH | -50 .. 50 |  | lin | 0 |
| `mix` | MIX | 0 .. 100 | % | lin | 100 |
| `$bdsolo` | BAND SOLO | off/on |  | bool | 0 |

#### SUB MONSTER - `SUB-M`

- **UI name:** SUB MONSTER  |  **snapshot `mdl`:** `SUB-M`  |  **WING-Edit internal:** `fx_pardef_submonster`  |  slots: FX1-16
- **Emulates:** Sub Monster subharmonic synthesizer (dbx 120A-style)
- **Behaviour:** mix, freq 45..67.5 Hz, 5 band levels.
- **Emulation approach:** Band-limited octave-down synthesis per band.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `mix` | MIX | 0 .. 100 | % | lin | 50 |
| `freq` | FREQ | 45 .. 67.5 | Hz | lin | 50 |
| `bd1` | BAND 1 | 0 .. 100 | % | lin | 60 |
| `bd2` | BAND 2 | 0 .. 100 | % | lin | 80 |
| `bd3` | BAND 3 | 0 .. 100 | % | lin | 100 |
| `bd4` | BAND 4 | 0 .. 100 | % | lin | 80 |
| `bd5` | BAND 5 | 0 .. 100 | % | lin | 60 |

#### VELVET IMAGER - `V-IMG`

- **UI name:** VELVET IMAGER  |  **snapshot `mdl`:** `V-IMG`  |  **WING-Edit internal:** `fx_pardef_velvetimg`  |  slots: FX1-16
- **Emulates:** Velvet Imager: K-Stereo (Bob Katz) / "Velvet" stereo imager
- **Behaviour:** width -1..1, stereo 0..100 (ambience recovery amount), gain, mode K-STEREO/VELVET, deep.
- **Emulation approach:** M/S processing + K-Stereo style ambience extraction (decorrelated delayed side).
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `wid` | WIDTH | -1 .. 1 |  | lin | 0 |
| `st` | STEREO | 0 .. 100 | % | lin | 75 |
| `g` | GAIN | -6 .. 6 | dB | lin | 0 |
| `mode` | MODE | K-STEREO / VELVET |  | enum | K-STEREO |
| `deep` | DEEP | off/on |  | bool | 0 |

#### SPEAKER MANAGER - `SPKMAN`

- **UI name:** SPEAKER MANAGER  |  **snapshot `mdl`:** `SPKMAN`  |  **WING-Edit internal:** `fx_pardef_spkpro`  |  slots: FX1-16
- **Emulates:** Speaker manager (`spkpro`)
- **Behaviour:** HP/LP freq + type (FLAT/BW/BS/LR slopes), tilt, phase 0..180, invert, distance 0..5 m(?) and position, dynamic EQ band (deq*), limiter (thr, RMS/peak, release).
- **Emulation approach:** Crossover biquads + allpass phase + delay + DEQ + limiter.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `hpf` | HP FREQ | 20 .. 20000 | Hz | log | 100 |
| `hptype` | HP TYPE | FLAT / BW6 / BW12 / BS12 / LR12 / BW18 / BW24 / BS24 / LR24 / BW48 / LR48 |  | enum | flat |
| `lpf` | LP FREQ | 20 .. 20000 | Hz | log | 2000 |
| `lptype` | LP TYPE | FLAT / BW6 / BW12 / BS12 / LR12 / BW18 / BW24 / BS24 / LR24 / BW48 / LR48 |  | enum | flat |
| `tiltf` | TILT F | 100 .. 10000 | Hz | log | 1000 |
| `tiltg` | TILT | -6 .. 6 | dB | lin | 0 |
| `phase` | PHASE | 0 .. 180 |  | lin | 0 |
| `invert` | INVERT | off/on |  | bool | 0 |
| `dist` | DIST | 0 .. 5 | m | lin | 0 |
| `pos` | POS | -5 .. 5 | m | lin | 0 |
| `dyneq` | DYN EQ | off/on |  | bool | 0 |
| `deqthr` | THR | -60 .. 0 | dB | lin | -20 |
| `deqratio` | RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `deqatt` | ATTACK | 0 .. 200 | ms | lin | 50 |
| `deqrel` | RELEASE | 20 .. 4000 | ms | log | 300 |
| `deqfilt` | FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `deqg` | GAIN | -15 .. 15 | dB | lin | 0 |
| `deqf` | FREQ | 20 .. 20000 | Hz | log | 1000 |
| `deqq` | Q | 0.44 .. 10 |  | log | 1 |
| `deqmode` | MODE | low / high |  | enum | high |
| `lim` | LIMITER | off/on |  | bool | 0 |
| `limthr` | LIM THR | -24 .. 0 | dB | lin | 0 |
| `limrms` | LIMRMS | off/on |  | bool | 1 |
| `limrel` | RELEASE | 50 .. 2000 | ms | log | 150 |

#### TRIPLE DEQ - `DEQ3`

- **UI name:** TRIPLE DEQ  |  **snapshot `mdl`:** `DEQ3`  |  **WING-Edit internal:** `fx_pardef_trideq`  |  slots: FX1-16
- **Emulates:** Triple dynamic EQ (`trideq`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** three DEQ bands (see gate DEQ).
- **Emulation approach:** Three DEQ engines.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `1-thr` | 1-THR | -60 .. 0 | dB | lin | -20 |
| `1-ratio` | 1-RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `1-att` | 1-ATT | 0 .. 200 | ms | lin | 50 |
| `1-rel` | 1-REL | 20 .. 4000 | ms | log | 300 |
| `1-filt` | 1-FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `1-g` | 1-GAIN | -15 .. 15 | dB | lin | 0 |
| `1-f` | 1-FREQ | 20 .. 20000 | Hz | log | 150 |
| `1-q` | 1-Q | 0.44 .. 10 |  | log | 1 |
| `1-mode` | 1-MODE | low / high |  | enum | high |
| `2-thr` | 2-THR | -60 .. 0 | dB | lin | -20 |
| `2-ratio` | 2-RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `2-att` | 2-ATT | 0 .. 200 | ms | lin | 50 |
| `2-rel` | 2-REL | 20 .. 4000 | ms | log | 300 |
| `2-filt` | 2-FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `2-g` | 2-GAIN | -15 .. 15 | dB | lin | 0 |
| `2-f` | 2-FREQ | 20 .. 20000 | Hz | log | 3000 |
| `2-q` | 2-Q | 0.44 .. 10 |  | log | 1 |
| `2-mode` | 2-MODE | low / high |  | enum | high |
| `3-thr` | 3-THR | -60 .. 0 | dB | lin | -20 |
| `3-ratio` | 3-RATIO | 1.2 / 1.3 / 1.5 / 2 / 3 / 5 / 10 |  | enum | 2 |
| `3-att` | 3-ATT | 0 .. 200 | ms | lin | 50 |
| `3-rel` | 3-REL | 20 .. 4000 | ms | log | 300 |
| `3-filt` | 3-FILTER | OFF / BP / LP6 / LP12 / HP6 / HP12 |  | enum | bp |
| `3-g` | 3-GAIN | -15 .. 15 | dB | lin | 0 |
| `3-f` | 3-FREQ | 20 .. 20000 | Hz | log | 4000 |
| `3-q` | 3-Q | 0.44 .. 10 |  | log | 1 |
| `3-mode` | 3-MODE | low / high |  | enum | high |

#### EVEN CHANNEL - `*EVEN*`

- **UI name:** EVEN CHANNEL  |  **snapshot `mdl`:** `*EVEN*`  |  **WING-Edit internal:** `fx_pardef_evenchannel`  |  slots: FX1-16
- **Emulates:** Neve channel strip: 88RS gate (`g_`) + 88RS EQ (E88 params, `eq_on`, EQMIX) + 33609-style comp/limiter (ECL33 params, `d_` prefix)
- **Behaviour:** The EQ band params are attached at runtime (confirmed by T003 variants).
- **Emulation approach:** Compose models.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `g_thr` | THR | -40 .. 0 | dB | lin | -35 |
| `g_hyst` | HYST | 0 .. 25 | dB | lin | 6 |
| `g_range` | RANGE | 0 .. 60 | dB | lin | 60 |
| `g_rel` | RELEASE | 100 .. 3000 | ms | log | 350 |
| `g_fast` | FAST | off/on |  | bool | 0 |
| `g_m40` | THR -40 | off/on |  | bool | 0 |
| `g_on` | G-ON | off/on |  | bool | 0 |
| `eq_on` | EQ-ON | off/on |  | bool |  |
| `mix` | EQMIX | 0 .. 100 | % | lin |  |
| `d_lon` | LIM ON | off/on |  | bool | 0 |
| `d_lthr` | LIM THR | -12 .. 0 | dB | lin | -5 |
| `d_lrec` | LIM REC | 50 / 100 / 200 / 800 / A1 / A2 |  | enum | a1 |
| `d_lfast` | LIM FAST | off/on |  | bool | 0 |
| `d_con` | CMP ON | off/on |  | bool | 0 |
| `d_cthr` | CMP THR | -35 .. -5 | dB | lin | -15 |
| `d_ratio` | RATIO | 1.5 / 2 / 3 / 4 / 6 |  | enum | 3 |
| `d_crec` | CMP REC | 100 / 400 / 800 / 1500 / A1 / A2 |  | enum | a1 |
| `d_cfast` | CMP FAST | off/on |  | bool | 0 |
| `d_gain` | GAIN | -6 .. 12 | dB | lin | 0 |
| `lf` | LF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 2.0 |
| `lg` | LF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lq` | LF Q | LOW / HIGH (from T003) |  |  | LOW |
| `lt` | LF TYPE | BELL / SHELV (from T003) |  |  | SHELV |
| `lmf` | LM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 3.0 |
| `lmg` | LM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lmq` | LM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmf` | HM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmg` | HM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hmq` | HM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hf` | HF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 7.0 |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hq` | HF Q | LOW / HIGH (from T003) |  |  | LOW |
| `ht` | HF TYPE | BELL / SHELV (from T003) |  |  | SHELV |

#### SOUL CHANNEL - `*SOUL*`

- **UI name:** SOUL CHANNEL  |  **snapshot `mdl`:** `*SOUL*`  |  **WING-Edit internal:** `fx_pardef_soulchannel`  |  slots: FX1-16
- **Emulates:** SSL channel strip: 9000G gate (`g_`) + SSL G EQ (SOUL params) + 9000C comp (`d_`)
- **Behaviour:** EQ params attached at runtime (T003).
- **Emulation approach:** Compose models.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `g_thr` | G-THR | -40 .. 0 | dB | lin | -35 |
| `g_range` | G-RANGE | 0 .. 40 | dB | lin | 40 |
| `g_hld` | G-HOLD | 10 .. 4000 | ms | log | 50 |
| `g_rel` | G-REL | 100 .. 4000 | ms | log | 800 |
| `g_fast` | G-FAST | off/on |  | bool | 0 |
| `g_mode` | G-MODE | GATE / EXP |  | enum | gate |
| `g_on` | G-ON | off/on |  | bool | 0 |
| `eq_on` | EQ-ON | off/on |  | bool |  |
| `mix` | EQMIX | 0 .. 100 | % | lin |  |
| `d_on` | D-ON | off/on |  | bool | 0 |
| `d_thr` | D-THR | -30 .. 18 | dB | lin | -10 |
| `d_ratio` | D-RATIO | 1.3 / 1.43 / 1.57 / 1.8 / 2 / 2.5 / 2.8 / 3.3 / 4 / 5 / 6 / 7 / 9 / 12 / 20 / 50 / 100 |  | enum | 3 |
| `d_fast` | D-FAST | off/on |  | bool | 0 |
| `d_rel` | D-REL | 100 .. 4000 | ms | log | 400 |
| `d_peak` | D-PEAK | off/on |  | bool | 0 |
| `lf` | LO FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 2.0 |
| `lg` | LO GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `lmf` | LM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 6.0 |
| `lmf3` | LM /3 | int [0 .. 1] (from T003) |  |  | 1 |
| `lmq` | LM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `lmg` | LM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hmf` | HM FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 3.0 |
| `hmf3` | HM x3 | int [0 .. 1] (from T003) |  |  | 0 |
| `hmq` | HM Q | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `hmg` | HM GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `hf` | HF FREQ | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 8.0 |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |

#### VINTAGE CHANNEL - `*VINTAGE*`

- **UI name:** VINTAGE CHANNEL  |  **snapshot `mdl`:** `*VINTAGE*`  |  **WING-Edit internal:** `fx_pardef_vintagechannel`  |  slots: FX1-16
- **Emulates:** Vintage strip: 1176 (`d_` = 76LA params) + Pultec EQ (PULSAR params `eq1..5hf`, `eq_on`, EQMIX) + LA-2A (`l_`)
- **Behaviour:** EQ params confirmed by T003 variants (not in the pardef walk).
- **Emulation approach:** Compose models.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `d_in` | INPUT | -48 .. 0 | dB | lin | -30 |
| `d_out` | OUTPUT | -48 .. 0 | dB | lin | -27 |
| `d_att` | ATTACK | 1 .. 7 |  | lin | 2 |
| `d_rel` | RELEASE | 1 .. 7 |  | lin | 2 |
| `d_ratio` | RATIO | 4 / 8 / 12 / 20 / ALL |  | enum | 8 |
| `d_on` | D-ON | off/on |  | bool | 0 |
| `l_ingain` | GAIN | 0 .. 100 |  | lin | 40 |
| `l_peak` | PEAK | 0 .. 100 |  | lin | 20 |
| `l_mode` | MODE | COMP / LIM |  | enum | comp |
| `l_on` | LVL-ON | off/on |  | bool | 0 |
| `eq1` | EQ1 ON | int [0 .. 1] (from T003) |  |  | 0 |
| `1lb` | LF BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1latt` | LF ATT | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1lf` | LF FREQ | 20 / 30 / 60 / 100 (from T003) | Hz |  | 30 |
| `1hw` | HF WID | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 5.0 |
| `1hb` | HF BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1hf` | HF FREQ | 3k / 4k / 5k / 8k / 10k / 12k / 16k (from T003) | Hz |  | 10k |
| `1hatt` | HF ATT | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `1hattf` | HF ATT F | 5k / 10k / 20k (from T003) | Hz |  | 20k |
| `eq5` | EQ5 ON | int [0 .. 1] (from T003) |  |  | 0 |
| `5lb` | LM BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5lf` | LF FREQ | 200 / 300 / 500 / 700 / 1k (from T003) | Hz |  | 300 |
| `5md` | MID DIP | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5mf` | MID FREQ | 200 / 300 / 500 / 700 / 1k / 1k5 / 2k / 3k / 4k / 5k / 7k (from T003) | Hz |  | 1k |
| `5hb` | HM BOOST | lin [0.0 .. 10.0], 101 steps (from T003) |  |  | 0.0 |
| `5hf` | HF FREQ | 1k5 / 2k / 3k / 4k / 5k (from T003) | Hz |  | 3k |

#### BUS CHANNEL - `*BUS*`

- **UI name:** BUS CHANNEL  |  **snapshot `mdl`:** `*BUS*`  |  **WING-Edit internal:** `fx_pardef_buschannel`  |  slots: FX1-16
- **Emulates:** Bus strip: SSL VHD warmth (`w_`) + Neve 1084 EQ (E84 params `g,lf,lg,mf,mg,mq,hf,hg`, per T003) + SSL G-bus comp (`d_`)
- **Behaviour:** 
- **Emulation approach:** Compose models.
- **Difficulty:** M

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `w_drv` | DRIVE | 10 .. 125 | % | lin | 33 |
| `w_hrm` | HARM | -100 .. 100 |  | lin | 0 |
| `w_col` | COLOR | -1 .. 1 |  | lin | 0 |
| `w_trim` | W-TRIM | -18 .. 6 | dB | lin | 0 |
| `w_mix` | W-MIX | 0 .. 100 | % | lin | 100 |
| `w_on` | W-ON | off/on |  | bool | 0 |
| `eq_on` | EQ-ON | off/on |  | bool |  |
| `mix` | EQMIX | 0 .. 100 | % | lin |  |
| `d_thr` | THR | -40 .. 0 | dB | lin | -12 |
| `d_ratio` | RATIO | 1.5 / 2 / 3 / 4 / 5 / 10 |  | enum | 3 |
| `d_att` | ATTACK | 0.1 / 0.3 / 1 / 3 / 10 / 30 | ms | enum | 10 |
| `d_rel` | RELEASE | 0.1 / 0.2 / 0.4 / 0.8 / 1.6 / AUTO |  | enum | auto |
| `d_gain` | GAIN | -6 .. 12 | dB | lin | 0 |
| `d_on` | D-ON | off/on |  | bool | 0 |
| `g` | GAIN | lin [-20.0 .. 20.0 dB], 81 steps (from T003) | dB |  | 0.0 |
| `lf` | LF FREQ | OFF / 35 / 60 / 110 / 220 (from T003) |  |  | 35 |
| `lg` | LF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `mf` | MID FREQ | OFF / 350 / 700 / 1k6 / 3k2 / 4k8 / 7k2 (from T003) |  |  | 1k6 |
| `mg` | MID GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |
| `mq` | MID Q | LOW / HIGH (from T003) |  |  | LOW |
| `hf` | HF FREQ | 10k / 12k / 16k / OFF (from T003) |  |  | 16k |
| `hg` | HF GAIN | lin [-5.0 .. 5.0], 101 steps (from T003) |  |  | 0.0 |

#### MASTERING - `*MASTER*`

- **UI name:** MASTERING  |  **snapshot `mdl`:** `*MASTER*`  |  **WING-Edit internal:** `fx_pardef_mastering`  |  slots: FX1-16
- **Emulates:** Mastering chain: tape (`t_`) + Maag EQ4 + Ultra Enhancer (`e_`) + precision limiter (`l_`)
- **In PUBLIC.snap (fw 1.13.9):** yes, at fx
- **Behaviour:** Fixture FX11 uses this model.
- **Emulation approach:** Compose models.
- **Difficulty:** L

| key | UI label | range | unit | scale | default |
|---|---|---|---|---|---|
| `t_drv` | DRIVE | -5 .. 25 | dB | lin | 0 |
| `t_spd` | SPEED | 7.5 .. 30 |  | log | 15 |
| `t_low` | LOW BUMP | off/on |  | bool | 0 |
| `t_hi` | HIGH SHELV | off/on |  | bool | 0 |
| `t_on` | T-ON | off/on |  | bool | 0 |
| `sub` | SUB | -5 .. 5 |  | lin | 0 |
| `40` | 40 | -5 .. 5 |  | lin | 0 |
| `160` | 160 | -5 .. 5 |  | lin | 0 |
| `650` | 650 | -5 .. 5 |  | lin | 0 |
| `2k5` | 2k5 | -5 .. 5 |  | lin | 0 |
| `air` | AIR | 0 .. 10 |  | lin | 0 |
| `airm` | AIR MODE | OFF / 2k5 / 5k / 10k / 20k / 40k |  | enum | 40k |
| `eq_on` | EQ-ON | off/on |  | bool | 0 |
| `e_stlvl` | ST LVL | -100 .. 100 | % | lin | 0 |
| `e_lmf` | LMF SPRD | -100 .. 100 | % | lin | 0 |
| `e_mlvl` | MONO LVL | -100 .. 100 | % | lin | 0 |
| `e_st` | ST PAN | -100 .. 100 | % | lin | 0 |
| `e_m` | MONO PAN | -100 .. 100 | % | lin | 0 |
| `e_bass` | BASS GAIN | 0 .. 100 | % | lin | 0 |
| `e_mid` | MID GAIN | 0 .. 100 | % | lin | 0 |
| `e_high` | HIGH GAIN | 0 .. 100 | % | lin | 0 |
| `e_bassf` | BASS FREQ | 1 .. 50 |  | lin | 15 |
| `e_midq` | MID Q | 1 .. 50 |  | lin | 15 |
| `e_highf` | HIGH FREQ | 1 .. 50 |  | lin | 25 |
| `e_on` | ENH-ON | off/on |  | bool | 0 |
| `l_gin` | IN GAIN | 0 .. 18 | dB | lin | 0 |
| `l_gout` | OUT GAIN | -18 .. 0 | dB | lin | -0.5 |
| `l_sqz` | SQUEEZE | 0 .. 100 |  | int | 0 |
| `l_knee` | KNEE | 0 .. 10 |  | int | 3 |
| `l_again` | AUTO GAIN | off/on |  | bool | 1 |
| `l_att` | ATTACK | 0.05 .. 1 | ms | lin | 0.05 |
| `l_rel` | RELEASE | 20 .. 2000 | ms | log | 500 |
| `l_on` | LIM-ON | off/on |  | bool | 1 |

## 7. PUBLIC.snap cross-reference (fw 1.13.9)

Models present in the fixture and verified against WING-Edit keys (see the per-model "In PUBLIC.snap" line):
gate slot GATE, WARM, DEQ, DUCK, PSE; dyn slot COMP (ch/bus/main/mtx), 76LA, NSTR, F670 (main); aux dyn = CMB (no
`mdl` key in fw 1.13: `{on, thr, depth, fast, peak, ingain, cpeak, cmode}`); EQ STD only; flt TILT and MAX; 11 FX
models. Key mismatches found:

| Location | fw 1.13 snapshot | WING-Edit 3.3.4 | Action for snapshot loader (T011) |
|---|---|---|---|
| gate WARM | `mix` | `wmix` | alias `mix` -> `wmix` |
| aux dyn | no `mdl` | CMB model (and others since 3.1) | default `mdl=CMB` when absent |
| main/mtx out delay | `dly {on, m}` | `dly {on, mode M/FT/MS/SMP, dly 0..100 m}` | map `m` -> `dly` with `mode=M` (monitor `dly.m` is still "DIST 0.1..100 m" in 3.3.4) |
| flt | no `lcs`, `hcs` | present (factory defaults 24 / 12) | default slopes when absent (**verify** what fw 1.13 used) |
| dynxo `type` | `HI6`, `LO6`, `LO12` | `OFF, LO, HI, S6, S12, PEAK` | fw 1.13 encoded band+slope in one token; 3.x splits it - map `LO6`->? (**open**: where the slope lives in 3.x) |
| cfg.solo `mon` | `A` | `PH, SPK, PH+SPK` | map A->PH, B->SPK (inferred) |
| send | `ind` key present | not in `Cchsend` pardef (`pre` present instead) | keep both; `ind` semantics unknown |

## 8. Ambiguities / open items

1. **Knob-unit vintage params** (SOUL, E88, E84, F110, PULSAR, MACH4 gains/freqs; 76LA/NSTR/L100 knobs): physical
   mapping not in the parameter definitions. T003 reports per-model `*_respf_*` (GUI response curves), `*_gain_*`
   (static gain computers) and `*_parchg_*` (parameter -> DSP coefficient) functions in WING-Edit: running those in
   T003's emulator is the best non-hardware source for these maps and for static compressor curves (follow-up).
2. **Byte 0 of the gate/dyn availability word** is unexplained (not needed for DSP).
3. **Hold ranges**: DUCK hold decodes to 30..30000 (log) and fixture DUCK `rel` 8.02 is above the 4 ms minimum but
   the hold range is unusual - verify units on a console.
4. **3CMP** (THREE BAND COMP) is in the WING-Edit table but in no selector: hidden.
5. **PSE "Source Extractor"** hardware reference unknown (internal name `pse545`).
6. **BPLATE "Blue Plate"** hardware reference unknown (internal `plate_reverb`).
7. **dynxo** fw 1.13 -> 3.x value mapping (`HI6` etc.).
8. **Filter-tool TILT pivot**, standard LC/HC filter family (Butterworth assumed).
9. Hardware "Emulates" lines are identifications from internal symbol names (`neve88rs`, `ssl9000j`, `u1176`,
   `la2a`, `f670`, `mpress`, `distress`, `api225l`, `tla100a`, `lmcplus`, `maag4`, `pultec_duo`, `isa110`, `neve84`,
   `ssl_g`, `api560`, `telray_delay`, `ren_bass`, `amp_jcm800`, `amp_engl`, `amp_jc120`, `vss3_reverb`, ...) plus
   general knowledge; Behringer's own manual does not name the originals.

## 9. Reconciliation with T003 (done 2026-10-25)

T003's `data/schema/wing-tree.json` (variants) and `docs/research/wing-edit-schema.md` appeared while this task was
finishing and were cross-checked programmatically against this extraction:

- **Model IDs:** identical for filter tool (4), gate/dyn (32 selectable; this doc additionally lists hidden `3CMP`),
  EQ ch/aux/bus/main/mtx (7/6/7/7/7) and FX (`fx/1` 65 incl. NONE, `fx/9` 39 incl. NONE).
- **Parameter keys/ranges:** all gate/dyn and EQ models match key-for-key and range-for-range (only cosmetic
  differences: Q min 0.442 vs 0.44 rounding; float enums printed as `1.2` here vs `1.2:1` labels in T003).
- **Differences that matter:** (a) EQ-type FX (SOUL, E88, E84, PULSAR) and the strips *EVEN*, *SOUL*, *VINTAGE*,
  *BUS* carry the EQ band parameters in T003 (attached at runtime, missed by the static pardef walk here) - use
  T003's lists; *VINTAGE* = Pultec EQ and *BUS* = Neve 1084 EQ. (b) GEQ: T003 lists the 31 band keys plus
  `TRIM`; this doc only recovered `type`. (c) Fader-law params (`ell/elr/rfll/rflr/lvl1/lvl2/erlvl/rvlvl`) are
  -oo..+10 dB (T003) - shown as "fader" here. (d) T003 omits keys `31` (PIA) and `40` (MACH4, *MASTER*) - likely a
  numeric-name collision on T003's side; both exist in the pardefs.
- **Strip-level values taken from T003:** LC 20..2000 Hz / HC 50..20k, slopes defaults, EQ mix 0..125 %, dyn gain
  -6..+12 dB, ptap/sidechain tap enum long names, solo modes, user-signal taps, output groups, delay ranges.

Rule: T003's generated schema wins for keys/ranges/enums; this doc for behaviour, hardware identification and
emulation approach.

