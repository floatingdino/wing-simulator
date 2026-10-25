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
| Comp FET | 76LA in/out knobs: 0 dB drive / make-up at the schema defaults -30 / -27 dB (unity below threshold); fixed cell threshold -18 dBFS | T04D |
| Comp FET | 76LA attack/release knob taper geometric between 800..20 µs / 1100..50 ms; release programme-dependent (0.5x after transients, 0.3 s history) | T04D |
| Comp FET | 76LA ratio buttons: threshold offsets 0/+2/+3/+4 dB, knee widths 10/6/4/2 dB; ALL = 20:1 + 1.3 dB output dip, 50 % transient GR overshoot, 3x FET distortion | T04D |
| Comp FET | 76LA colouration: ~0.4 % THD at ~9 dB GR (FET H2), 0.013 % without GR (class-A stage, +12 dBFS soft-clip headroom) | T04D |
| Comp FET | NSTR in/out knobs 4 dB per step, unity at defaults 4 / 3; fixed threshold -18 dBFS; attack 50 µs..30 ms, release 50 ms..3.5 s geometric | T04D |
| Comp FET | NSTR knee widths per ratio (12/12/10/6/4/2/1/0 dB); 1.5/2/3:1 opto-like release (0.5x..2x with GR), 10:1 programme-dependent; WING "1.5:1" replaces the EL8's 1:1 | T04D |
| Comp FET | NSTR normal-mode harmonics: LF-weighted (300 Hz) 2nd + 3rd, 0.3-0.5 % at 100 Hz, < 0.04 % at 1 kHz, scaled up with ratio | T04D |
| Comp FET | WING runs these models with 57 samples latency (2x audio / 16x control oversampling) - real console latency unknown | T04D |
