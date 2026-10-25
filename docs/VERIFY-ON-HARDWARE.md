# Assumptions to verify against a real WING (coordinator-maintained)

Each item: what was assumed, where, and how to check on a console (or via a measurement). T072 turns these into tests.

| Area | Assumption | Source task |
|---|---|---|
| Comp VCA | 0 dBu = −18 dBFS for B160/B560/9000C thresholds | T04B |
| Comp VCA | dbx OverEasy knee width 6 dB (B560) | T04B |
| Comp VCA | 9000C RMS window 10 ms | T04B |
| Comp VCA | D241C peak detector, 10 ms auto attack; SBUS/D241C auto-release constants | T04B |
| Gate | ACCENT = 0–10 dB hysteresis; detector timing | T032 |
| DEQ/DEQ2 | `filt` = band shape (BP peak, LP/HP = 1st/2nd-order shelves, OFF = wide band) rather than a side-chain-only filter; side-chain uses the same shape at f/Q | T048 |
| DEQ/DEQ2 | DEQ bands share the STD EQ laws (mirrored peak/shelf, shelves ignore Q); LP6/HP6 1st-order mirrored law; no STD-style ~50 Hz high-shelf corner clamp; no WING high-frequency de-cramping | T048 |
| DEQ/DEQ2 | `g` = maximum dynamic gain; gain law `min(|g|, max(0, over)·(1−1/ratio))`, mode high = above thr, low = below; thr vs RMS level (5 ms mean square, louder channel) | T048 |
| DEQ/DEQ2 | att/rel are dB-domain time constants (tau, COMP convention), att while moving away from 0 dB; 8-sample control rate; f/Q glide 20 ms | T048 |
| DS902 | detector = HF (12 dB/oct HP at f) / total energy ratio, onset −12 dB, slope 1 dB/dB, capped at range; 1 ms detector, 1 ms attack, 50 ms release | T048 |
| DS902 | HF mode = dynamic WING-law high shelf at f; FULL = wide-band gain; external key replaces the detector input | T048 |
| Input | balance law; stereo pair selection takes whole pair; L+R user-signal scaling; oscillator level | T031, T024 |
| Snapshot | header written for migrated files accepted by a real WING; 1.x filter slopes 12 dB/oct; dynxo slope location | T019 |
| Library | `$ctl/lib` semantics, 0-based indices, store/rename/delete actions | T017 |
| Protocol | MIXER->PC root dump reply; ModelRequest reply format; meter source-group sizes | T013, T015 |
| OSC | `?`/`#` dump text format | T014 |
