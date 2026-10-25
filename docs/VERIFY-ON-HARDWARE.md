# Assumptions to verify against a real WING (coordinator-maintained)

Each item: what was assumed, where, and how to check on a console (or via a measurement). T072 turns these into tests.

| Area | Assumption | Source task |
|---|---|---|
| Comp VCA | 0 dBu = −18 dBFS for B160/B560/9000C thresholds | T04B |
| Comp VCA | dbx OverEasy knee width 6 dB (B560) | T04B |
| Comp VCA | 9000C RMS window 10 ms | T04B |
| Comp VCA | D241C peak detector, 10 ms auto attack; SBUS/D241C auto-release constants | T04B |
| Comp opto | LA: GAIN = output gain (0.4 dB/step, unity at 40) not input gain despite key `ingain`; PEAK = side-chain drive, threshold −30 dBFS at 100 (−16 at 20, off at 0); LIM ratio 20:1; knee 10 dB; EL-panel 5 ms; two-part T4 cell (60 % fast part, 42 ms) and memory constants (3 s charge / 30 s decay) | T04E |
| Comp opto | L100: per-switch times (FAST/MED/SLOW attack 0.18/4.2/100 ms, release 32/219/1500 ms from a retailer span), programme spread ×1/×3/×6, threshold 6−3.6·GR dBFS, ratio 2+0.4·GR, knee 12 dB, GAIN 3 dB/step unity at 3, feedback topology | T04E |
| Comp vari-mu | F670: gm law 10 dB per threshold unit + 0.5 knee, THR map 6−3.6·thr dBFS, fixed amp +10 dB (unity at INPUT −10), BIAS = half gm slope + even-harmonic asymmetry at 0; pos 5/6 stage charge times 0.1 s / 2 s; pos 5 single-peak release 2 s (Thomann prints 0.2 s) | T04E |
| Comp opto/vari-mu | Tube colouration present in WING's LA/L100/F670 at all, and its level (cubic soft clip, ≈ −45 dBc H2/H3 at 0 dBFS) | T04E |
| Comp class A | BLISS: peak detector, hard knee, Anti-Log = linear-gain-domain ballistics, Auto Fast rate ×(1+overshoot/6 dB) | T04E |
| Gate | ACCENT = 0–10 dB hysteresis; detector timing | T032 |
| Input | balance law; stereo pair selection takes whole pair; L+R user-signal scaling; oscillator level | T031, T024 |
| Snapshot | header written for migrated files accepted by a real WING; 1.x filter slopes 12 dB/oct; dynxo slope location | T019 |
| Library | `$ctl/lib` semantics, 0-based indices, store/rename/delete actions | T017 |
| Protocol | MIXER->PC root dump reply; ModelRequest reply format; meter source-group sizes | T013, T015 |
| OSC | `?`/`#` dump text format | T014 |
