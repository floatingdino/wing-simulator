# Assumptions to verify against a real WING (coordinator-maintained)

Each item: what was assumed, where, and how to check on a console (or via a measurement). T072 turns these into tests.

| Area | Assumption | Source task |
|---|---|---|
| Comp VCA | 0 dBu = −18 dBFS for B160/B560/9000C thresholds | T04B |
| Comp VCA | dbx OverEasy knee width 6 dB (B560) | T04B |
| Comp VCA | 9000C RMS window 10 ms | T04B |
| Comp VCA | D241C peak detector, 10 ms auto attack; SBUS/D241C auto-release constants | T04B |
| Comp console | ECL33/RED3 thresholds read as dBFS (WING ranges offset from 33609 dBu ranges); E88C/2250 dBu thresholds at 0 dBu = −18 dBFS | T04C |
| Comp console | ECL33 compressor is feedback (detector after the diode bridge), limiter feed-forward ratio 200:1; 6 dB diode soft knee; no 100 Hz side-chain HPF | T04C |
| Comp console | ECL33 A1/A2 two-stage recovery charge constant 0.5 s; colouration = odd (cubic) distortion, 0.1 % THD at 20 dB GR / −6 dBFS out | T04C |
| Comp console | E88C SLOW attack 10 ms (FAST 0.1 ms), 10 dB soft knee, ratio ≥ 9.99 = LIM, rel ≥ 2999 ms = AUTO (0.1 s/1.6 s/0.5 s two-stage) | T04C |
| Comp console | RED3 10 dB soft knee, peak detector, AUTO = 0.1 s/1.6 s/0.5 s two-stage | T04C |
| Comp console | 2250 knob laws (thr 0..10 = +10..−20 dBu linear; ratio slope = knob/10), 10 dB soft knee, NEW RMS 1 ms, OLD peak feedback capped at 31:1 | T04C |
| Comp console | LMT: ratio 20:1, attack 0.3 ms / release 150 ms, `comp` 0..100 = threshold 0..−40 dBFS; transient shaper laws (speed 50→2 ms, ±trans·0.1·Δenv dB); no LMC colouration | T04C |
| Gate | ACCENT = 0–10 dB hysteresis; detector timing | T032 |
| Input | balance law; stereo pair selection takes whole pair; L+R user-signal scaling; oscillator level | T031, T024 |
| Snapshot | header written for migrated files accepted by a real WING; 1.x filter slopes 12 dB/oct; dynxo slope location | T019 |
| Library | `$ctl/lib` semantics, 0-based indices, store/rename/delete actions | T017 |
| Protocol | MIXER->PC root dump reply; ModelRequest reply format; meter source-group sizes | T013, T015 |
| OSC | `?`/`#` dump text format | T014 |
