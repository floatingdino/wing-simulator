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
| Comp FET | 76LA in/out knobs: 0 dB drive / make-up at the schema defaults -30 / -27 dB (unity below threshold); fixed cell threshold -18 dBFS | T04D |
| Comp FET | 76LA attack/release knob taper geometric between 800..20 µs / 1100..50 ms; release programme-dependent (0.5x after transients, 0.3 s history) | T04D |
| Comp FET | 76LA ratio buttons: threshold offsets 0/+2/+3/+4 dB, knee widths 10/6/4/2 dB; ALL = 20:1 + 1.3 dB output dip, 50 % transient GR overshoot, 3x FET distortion | T04D |
| Comp FET | 76LA colouration: ~0.4 % THD at ~9 dB GR (FET H2), 0.013 % without GR (class-A stage, +12 dBFS soft-clip headroom) | T04D |
| Comp FET | NSTR in/out knobs 4 dB per step, unity at defaults 4 / 3; fixed threshold -18 dBFS; attack 50 µs..30 ms, release 50 ms..3.5 s geometric | T04D |
| Comp FET | NSTR knee widths per ratio (12/12/10/6/4/2/1/0 dB); 1.5/2/3:1 opto-like release (0.5x..2x with GR), 10:1 programme-dependent; WING "1.5:1" replaces the EL8's 1:1 | T04D |
| Comp FET | NSTR normal-mode harmonics: LF-weighted (300 Hz) 2nd + 3rd, 0.3-0.5 % at 100 Hz, < 0.04 % at 1 kHz, scaled up with ratio | T04D |
| Comp FET | WING runs these models with 57 samples latency (2x audio / 16x control oversampling) - real console latency unknown | T04D |
| WAVE | Differential envelope: fast follower 0.3 ms / 40 ms, slow follower (smoothing the fast one) 15 ms / 300 ms, d = fast_dB - slow_dB, 1 dB dead zone, attack weight full at +9 dB, sustain weight full at -5 dB; `att`/`sust` = dB at full weight; no gain below -80 dBFS (ramp over 20 dB); gain smoothing 0.5 ms; SPL publishes no time constants | T049 |
| WAVE | Attack region lasts ~10-15 ms after an onset, sustain acts on decays faster than ~29 dB/s; steady tones and slower decays pass at unity; detector = max(abs L, abs R), stereo-linked | T049 |
| RIDE | Level = RMS (mean-square one-pole 100 ms, full-scale sine -3 dBFS), `thr`/`tgt` in these RMS dBFS; desired gain = (tgt - level)(1 - 1/ratio) clamped to +-range | T049 |
| RIDE | `spd` 1..50 = 2..200 dB/s geometric slew (default 10 = 4.7 dB/s); gain reduction 2x faster than boost; 20 ms gain smoothing; control rate 16 samples | T049 |
| RIDE | Below `thr`: gain frozen for `hld`, then relaxes to 0 dB at the up rate; `hld` >= 9.99 s = INF (never relaxes) | T049 |
| WARM | Shaper `tanh(u + b) - tanh(b)`, u = 1.5 * drv/100 * x, unity small-signal gain (drive adds harmonics, not level), bias T = tanh(b) = 0.46 (1 - hrm/100)/2 so hrm -100 = 2nd dominant / 3rd largely cancelled, +100 = 3rd only; hrm sign (negative = 2nd) | T049 |
| WARM | Harmonic levels: default ~1.4 % THD at -12 dBFS, 125 % drive ~5 %; WING's actual THD curves unknown | T049 |
| WARM | COLOR = pre tilt +-6 dB (HF-LF, centre 1 kHz, 1st order) with 50 % taken back after the shaper (net +-3 dB tilt); trim applied after the dry/wet mix (both paths), `wmix` blends processed with a 39-sample delay-compensated dry; channels independent (not stereo-linked); no side-chain input | T049 |
| WARM | WING runs WARM with 39 samples latency (2x oversampling) - real console latency unknown | T049 |
