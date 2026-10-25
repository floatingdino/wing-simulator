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
| Comp console | ECL33/RED3 thresholds read as dBFS (WING ranges offset from 33609 dBu ranges); E88C/2250 dBu thresholds at 0 dBu = −18 dBFS | T04C |
| Comp console | ECL33 compressor is feedback (detector after the diode bridge), limiter feed-forward ratio 200:1; 6 dB diode soft knee; no 100 Hz side-chain HPF | T04C |
| Comp console | ECL33 A1/A2 two-stage recovery charge constant 0.5 s; colouration = odd (cubic) distortion, 0.1 % THD at 20 dB GR / −6 dBFS out | T04C |
| Comp console | E88C SLOW attack 10 ms (FAST 0.1 ms), 10 dB soft knee, ratio ≥ 9.99 = LIM, rel ≥ 2999 ms = AUTO (0.1 s/1.6 s/0.5 s two-stage) | T04C |
| Comp console | RED3 10 dB soft knee, peak detector, AUTO = 0.1 s/1.6 s/0.5 s two-stage | T04C |
| Comp console | 2250 knob laws (thr 0..10 = +10..−20 dBu linear; ratio slope = knob/10), 10 dB soft knee, NEW RMS 1 ms, OLD peak feedback capped at 31:1 | T04C |
| Comp console | LMT: ratio 20:1, attack 0.3 ms / release 150 ms, `comp` 0..100 = threshold 0..−40 dBFS; transient shaper laws (speed 50→2 ms, ±trans·0.1·Δenv dB); no LMC colouration | T04C |
| Gate | ACCENT = 0–10 dB hysteresis; detector timing | T032 |
| DEQ/DEQ2 | `filt` = band shape (BP peak, LP/HP = 1st/2nd-order shelves, OFF = wide band) rather than a side-chain-only filter; side-chain uses the same shape at f/Q | T048 |
| DEQ/DEQ2 | DEQ bands use the STD EQ `wingcoef` sections as `eq.rs` calls them (BP `hqpeq`, LP12 `ls12` Q 0.707 f≤8 kHz, HP12 `hqhs12` f≥50 Hz, shelves ignore Q); LP6/HP6 = `hqls6`/`hqhs6`; side-chain = `hqbp`/`hqlp12`/`lp6`/`hqhp12`/`hqhp6` at f/Q | T048 |
| DEQ/DEQ2 | `g` = maximum dynamic gain; gain law `min(|g|, max(0, over)·(1−1/ratio))`, mode high = above thr, low = below; thr vs RMS level (5 ms mean square, louder channel) | T048 |
| DEQ/DEQ2 | att/rel are dB-domain time constants (tau, COMP convention), att while moving away from 0 dB; 8-sample control rate; f/Q glide 20 ms | T048 |
| DS902 | detector = HF (12 dB/oct HP at f) / total energy ratio, onset −12 dB, slope 1 dB/dB, capped at range; 1 ms detector, 1 ms attack, 50 ms release | T048 |
| DS902 | HF detector = `hqhp12` at f; HF mode = dynamic `hqhs12` high shelf at f; FULL = wide-band gain; external key replaces the detector input | T048 |
| Input | balance law; stereo pair selection takes whole pair; L+R user-signal scaling; oscillator level | T031, T024 |
| Snapshot | header written for migrated files accepted by a real WING; 1.x filter slopes 12 dB/oct; dynxo slope location | T019 |
| Library | `$ctl/lib` semantics, 0-based indices, store/rename/delete actions | T017 |
| Protocol | MIXER->PC root dump reply; ModelRequest reply format; meter source-group sizes | T013, T015 |
| OSC | `?`/`#` dump text format | T014 |
| Gate vintage | E88/9000G/D241G: peak detector release 20 ms; release time = time to traverse 60 dB (E88) / 40 dB (9000G) / 80 dB (D241G) linear in dB; attack = time to open the full range | T047 |
| Gate vintage | E88 hysteresis end stop (25 dB, label "EXP") = 2:1 expander without hysteresis; E88 attack 2 ms / 50 us; no hold | T047 |
| Gate vintage | 9000G fixed 2 dB hysteresis; attack 1.5 ms / 100 us; EXP mode 2:1 slope, hold retained in EXP | T047 |
| Gate vintage | D241G fixed 2 dB hysteresis, attack 20 us, depth 80 dB, hold 10/100 ms, programme-dependent release 150..600 ms (SLOW 500..2000 ms), open-time charge tau 0.3 s | T047 |
