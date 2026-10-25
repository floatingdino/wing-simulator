# Assumptions to verify against a real WING (coordinator-maintained)

Each item: what was assumed, where, and how to check on a console (or via a measurement). T072 turns these into tests.

| Area | Assumption | Source task |
|---|---|---|
| Comp VCA | 0 dBu = −18 dBFS for B160/B560/9000C thresholds | T04B |
| Comp VCA | dbx OverEasy knee width 6 dB (B560) | T04B |
| Comp VCA | 9000C RMS window 10 ms | T04B |
| Comp VCA | D241C peak detector, 10 ms auto attack; SBUS/D241C auto-release constants | T04B |
| Gate | ACCENT = 0–10 dB hysteresis; detector timing | T032 |
| Input | balance law; stereo pair selection takes whole pair; L+R user-signal scaling; oscillator level | T031, T024 |
| Snapshot | header written for migrated files accepted by a real WING; 1.x filter slopes 12 dB/oct; dynxo slope location | T019 |
| Library | `$ctl/lib` semantics, 0-based indices, store/rename/delete actions | T017 |
| Protocol | MIXER->PC root dump reply; ModelRequest reply format; meter source-group sizes | T013, T015 |
| OSC | `?`/`#` dump text format | T014 |
| Gate vintage | E88/9000G/D241G: peak detector release 20 ms; release time = time to traverse 60 dB (E88) / 40 dB (9000G) / 80 dB (D241G) linear in dB; attack = time to open the full range | T047 |
| Gate vintage | E88 hysteresis end stop (25 dB, label "EXP") = 2:1 expander without hysteresis; E88 attack 2 ms / 50 us; no hold | T047 |
| Gate vintage | 9000G fixed 2 dB hysteresis; attack 1.5 ms / 100 us; EXP mode 2:1 slope, hold retained in EXP | T047 |
| Gate vintage | D241G fixed 2 dB hysteresis, attack 20 us, depth 80 dB, hold 10/100 ms, programme-dependent release 150..600 ms (SLOW 500..2000 ms), open-time charge tau 0.3 s | T047 |
