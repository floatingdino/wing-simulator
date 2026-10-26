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
| ONEC | `gr` 0..10 -> COMP engine: thr = -3·gr dB, ratio 1.5..8:1 geometric, att 30..3 ms, rel 300..80 ms (AUTO on), knee 3; WING publishes no curve | T04F |
| ONEC | `dag` = auto make-up switch (meaning undocumented); make-up = 0.5 x static GR at -6 dBFS, <= 12 dB | T04F |
| 3CMP | Hidden model (no schema entry): slot order `knee lmf mhf` then per band `g thr ratio att rel` taken from models.md; ratio default 3:1 | T04F |
| 3CMP | Crossover = LR4 (bilinear Butterworth squared) with allpass phase compensation of the low band; WING's real crossover design unknown | T04F |
| 3CMP | Band comps = standard COMP engine (RMS, log, no AUTO); global knee 0..25 dB rounded to 7.22 dB steps; band gain applied after the comp, 10 ms smoothing; external key ignored | T04F |
| WAVE | Differential envelope: fast follower 0.3 ms / 40 ms, slow follower (smoothing the fast one) 15 ms / 300 ms, d = fast_dB - slow_dB, 1 dB dead zone, attack weight full at +9 dB, sustain weight full at -5 dB; `att`/`sust` = dB at full weight; no gain below -80 dBFS (ramp over 20 dB); gain smoothing 0.5 ms; SPL publishes no time constants | T049 |
| WAVE | Attack region lasts ~10-15 ms after an onset, sustain acts on decays faster than ~29 dB/s; steady tones and slower decays pass at unity; detector = max(abs L, abs R), stereo-linked | T049 |
| RIDE | Level = RMS (mean-square one-pole 100 ms, full-scale sine -3 dBFS), `thr`/`tgt` in these RMS dBFS; desired gain = (tgt - level)(1 - 1/ratio) clamped to +-range | T049 |
| RIDE | `spd` 1..50 = 2..200 dB/s geometric slew (default 10 = 4.7 dB/s); gain reduction 2x faster than boost; 20 ms gain smoothing; control rate 16 samples | T049 |
| RIDE | Below `thr`: gain frozen for `hld`, then relaxes to 0 dB at the up rate; `hld` >= 9.99 s = INF (never relaxes) | T049 |
| WARM | Shaper `tanh(u + b) - tanh(b)`, u = g(drv) x with g = 0.5 * 32^((drv - 33)/92) (0.21 at 10 %, 0.5 default, 16 at 125 %; T088, was 6), small-signal-normalised then RMS make-up so a -18 dBFS RMS sine keeps its level at every drive (T088: quiet passages come up to +6 dB at 125 %, peaks at -12 dBFS go down 5 dB), bias T = tanh(b) = 0.46 (1 - hrm/100)/2 so hrm -100 = 2nd dominant / 3rd largely cancelled, +100 = 3rd only; hrm sign (negative = 2nd) | T049 |
| WARM | Harmonic levels (T088 targets, 1 kHz sine, hrm 0): default ~1 % at -18 dBFS RMS (1.4 % at -12 dBFS peak); 125 % drive 25 % at -18, 14 % at -24, 7.6 % at -30 dBFS RMS (was 10.8 / 5.8 / 3.0 %). Check: THD vs DRIVE at -18 / -24 dBFS RMS and the output level change at 125 % on a console | T049, T088 |
| WARM | COLOR = pre tilt +-6 dB (HF-LF, centre 1 kHz, 1st order) with 50 % taken back after the shaper (net +-3 dB tilt); trim applied after the dry/wet mix (both paths), `wmix` blends processed with a 39-sample delay-compensated dry; channels independent (not stereo-linked); no side-chain input | T049 |
| WARM | WARM's MIX (`wmix`) *is* the block MIX: the gate/dyn slot does not blend with its block `mix` again (one blend, `w = on`). Evidence: WING-Edit 3.3.4's legacy-scene upgrade copies `gate.mix` to `gate.wmix` and keeps the block `mix` (x86_64 code at 0x10026b026: `strcasecmp(mdl, "WARM")` -> `wmix.copyFrom(mix)`), so every upgraded WARM carries `mix == wmix` (user scene: 5/5, 30/30); applying both would square it (5 % -> 0.25 %). Check: WARM at drv 125 / wmix 100 with block `mix` 5 % on a console - fully wet (sim) or 5 % wet? | T085 |
| WARM | WING runs WARM with 39 samples latency (2x oversampling) - real console latency unknown | T049 |
| PSE/CMB | PSE = Waves Primary Source Expander-style soft downward expander (gate slot, `dyn_pardef_pse545`); sources: Waves PSE product pages (reverb.com/woodbrass listings, "reduce bleed/feedback, threshold + attenuation") and a Behringer WING forum post using it on vocals; no WING documentation found. Law: 6 dB knee, 2:1 expansion, capped at `depth`; not a cross-channel bleed canceller | T04A |
| PSE/CMB | PSE timing: `fast` off close 120 ms / open 8 ms, on 30 / 1.5 ms (dB-domain one-pole); `peak` off = 10 ms RMS (reads 3 dB under peak for a sine), on = peak follower with 20 ms release; stereo-linked on the louder channel; external key replaces detector input | T04A |
| PSE/CMB | CMB = PSE then LA (`OptoComp` La) in series; slots 5-7 = LA `ingain cpeak cmode`; CMB defaults thr -36, cpeak 0 (leveler idle); external key drives PSE only; PSE ahead of LA (order unverified) | T04A |
| ENHANCE | Band maps: BASS FREQ 1..50 = 30..400 Hz, HIGH FREQ 1..50 = 2..12 kHz (log), MID Q 1..50 = 0.3..8 (log), mid centre fixed 1 kHz; band = biased-tanh residual harmonics (4x, ADAA1) + linear share 1.0/0.5/0.6, `gain_b` = BASS/MID/HIGH GAIN 0..100 % | T05E |
| ENHANCE | Stereo stage: MONO/ST LVL = M/S gain `1 + pct/100`; MONO PAN linear balance on the mid only; ST PAN output balance; LMF SPRD scales the side of the *enhancement* `1 + pct/100`; SOLO = enhancement only; real Ultrafex/X32 enhancer structure unknown | T05E |
| EXCITER | HP2 at TUNE (Q 0.71..2.5 by PEAK) -> drive 1 + 5 HARM -> tanh bias 0.6 TIMBRE/50 + ZFILL 0.3 tanh(5u) -> /sqrt(k) -> HP2 -> TIMBRE +-6 dB shelf 6 kHz; DRY OFF = no dry (wet still scaled by MIX); aliasing at hard clip -70 dBc (9.7 kHz) | T05E |
| P-BASS | LR4 split at X/O; AGC `u = b/(env + 1e-3)`, harmonics `0.8 abs(u) + 0.6 (u abs(u) - 1.3 u)` x envelope, HP2 at X/O + LP2 at 4 X/O; BASS GAIN = low band level, INTENSITY = harmonic level, SOLO drops the high band; MaxxBass exact law unknown | T05E |
| SUB-M | 5 band-pass analysis bands at FREQ x 2^((i-2)/2) (Q 1.4), Schmitt flip-flop divider (threshold 0.25 x envelope), square -> LP 0.6 f -> BAND i level (acts as a frequency response of the synthesis), mono sum added to both channels, MIX % of sub; dbx 120A / WING band layout unknown | T05E |
| V-IMG | M/S, WIDTH scales S (`1 + wid`; widening above 150 Hz unless DEEP, narrowing full range); STEREO = 0.7 x ambience (HP 250 Hz + 2 allpass + 1 ms; DEEP 100 Hz / 3 ms) added to S only (mono sum exact); VELVET also feeds the high-passed mid; K-Stereo / Velvet real structure unknown | T05E |
| TAPE | Saturator `tanh(u + 0.12)` residual form, K = 1.5, output / (K sqrt(g)) (drive adds half its dB as level); head bump 63 Hz x ips/15, +4 dB (7.5 ips) .. +2 dB (30 ips), Q 1.2; HF loss LP2 at 14 kHz (ips/15)^0.6; HIGH SHELV +6 dB pre / -4 dB post at 8 kHz (net +2 dB); no hysteresis memory | T05E |
| TAPE | Extras not in the WING parameter set (`TapeExtras`, default off): wow 0.6 Hz <= 0.35 ms, flutter 9 Hz <= 0.02 ms (x 15/ips), hiss tilted noise (+3 dB per halving of speed); 1 ms unreported delay when wow/flutter is on | T05E |
| ENHANCE/EXCITER/TAPE | 46 samples latency (4x oversampling, 45.5 rounded up); every FX slot hosts them, so every slot reports 46 samples and strips with inserts pad accordingly | T05E |
| DCA mute | **VERIFIED (user, real WING, 2026-10-25)**: a DCA mute always mutes its members' main (LR/main) sends; `cfg/dcamgrp` ("DCA MUTEGROUPS") ON makes it act as a mute group, additionally muting the members' bus sends incl. PRE-fader sends (full channel mute). Still open: main/matrix strips tagged into a DCA (simulator mutes them outright), and whether DCA mute affects matrix (MXn) sends with dcamgrp off (simulator: no) | T03F |
| Monitor level | `cfg/mon/1/lvl` / `cfg/mon/2/lvl` ("PHONES"/"SPEAKERS") are the physical knobs and the knob wins over a remote set (WING-Edit PC->MIXER pushes -oo); simulator: knob owns the value, client sets are written back. Check: what WING-Edit shows / does for the monitor level on hardware, and the meaning of `$lvl` ("LEVEL") and `$lvlact` | T03F |

## Checking them on a real WING (T072)

The table above is what is assumed. `tools/verify-hardware/` turns the testable part into a short measurement
run: one 50 s test signal and one generated `.snap` scene per assumption (or model). Play the signal through each
scene on the console, record the USB return, and `wing-verify compare` renders the same scene with the simulator
and reports the difference per scene. Procedure and commands: [tools/verify-hardware/README.md](../tools/verify-hardware/README.md).
Models with reference data from WING-Edit are measured automatically instead, see [fidelity-report.md](fidelity-report.md).

Coverage: dynamics (static level curve, step settling times, noise-burst envelopes), saturation (harmonics of
1 kHz and 100 Hz tones) and frequency response are measurable this way. Not covered by the kit (needs the console
UI or other signals): input balance/stereo-pair laws, snapshot header acceptance, `$ctl/lib`, protocol and OSC
rows, meter source groups, DS902/DEQ/DEQ2 (need frequency-selective key signals), 3CMP, delays/reverbs/pitch/mod
FX (time-variant or non-deterministic) and anything in the table that is a latency figure on a model the kit
does not host. A `DEVIATES` row names the assumption to revisit; correct the model, then update this table.

### Scenes and the assumptions they test

<!-- verify:scenes:begin -->

| Scene | Tier | Tests | Area | Assumption under test |
|---|---|---|---|---|
| `00-thru` | quick | fr, stairs, tone | Input | unity path: USB level, 0 dBFS in = 0 dBFS out, pan law, latency (baseline for every other scene) |
| `01-filter` | quick | fr | Filter (exact port) | channel LC 100 Hz / HC 8 kHz match the traced WING-Edit design (sanity check of the whole chain) |
| `02-eq-std` | quick | fr | EQ (exact port) | STD EQ bands match the traced WING-Edit design (sanity check of the whole chain) |
| `03-eq-e88` | full | fr, tone | EQ alt | E88 EQ is purely linear (no transformer / tube colour in WING's version) |
| `04-eq-soul` | full | fr, tone | EQ alt | SOUL EQ is purely linear (no console colour in WING's version) |
| `10-gate-std` | full | stairs, steps, bursts | Gate | WING GATE static law (exact) and ACCENT/detector timing |
| `11-gate-e88` | quick | stairs, steps, bursts | Gate vintage | E88: peak detector release 20 ms, 60 dB release law, 2 ms / 50 us attack, EXP end stop |
| `12-gate-9000g` | full | stairs, steps, bursts | Gate vintage | 9000G: fixed 2 dB hysteresis, 1.5 ms / 100 us attack, 40 dB release law |
| `13-gate-d241g` | full | stairs, steps, bursts | Gate vintage | D241G: 2 dB hysteresis, 20 us attack, 80 dB depth, programme-dependent release |
| `14-gate-pse` | full | stairs, steps, bursts | PSE/CMB | PSE: 6 dB knee, 2:1 expansion capped at depth, fast/slow timing, 10 ms RMS |
| `20-comp-std` | quick | stairs, steps | Comp (reference curve) | WING COMP static law (exact) and attack/release timing |
| `21-comp-b160` | full | stairs, steps | Comp VCA | B160: 0 dBu = -18 dBFS thresholds, RMS detector |
| `22-comp-b560` | full | stairs, steps | Comp VCA | B560: dbx OverEasy knee 6 dB, thresholds at 0 dBu = -18 dBFS |
| `23-comp-9000c` | full | stairs, steps | Comp VCA | 9000C: RMS window 10 ms, 0 dBu = -18 dBFS |
| `24-comp-sbus` | full | stairs, steps | Comp VCA | SBUS: auto-release constants |
| `25-comp-d241c` | full | stairs, steps | Comp VCA | D241C: peak detector, 10 ms auto attack |
| `26-comp-ecl33` | quick | stairs, steps, tone | Comp console | ECL33: threshold read as dBFS, feedback comp, 6 dB soft knee, colouration 0.1 % THD at 20 dB GR |
| `27-comp-e88c` | full | stairs, steps | Comp console | E88C: SLOW attack 10 ms, 10 dB soft knee, 0 dBu = -18 dBFS |
| `28-comp-red3` | full | stairs, steps | Comp console | RED3: 10 dB soft knee, peak detector, thresholds as dBFS |
| `29-comp-2250` | full | stairs, steps | Comp console | 2250: threshold knob law, 10 dB soft knee, NEW RMS 1 ms |
| `2a-comp-lmt` | full | stairs, steps | Comp console | LMT: 20:1, 0.3 ms attack / 150 ms release, transient shaper laws |
| `2b-comp-76la` | quick | stairs, steps, tone | Comp FET | 76LA: fixed -18 dBFS cell threshold, in/out knob scaling, 0.4 % THD at 9 dB GR |
| `2c-comp-nstr` | full | stairs, steps, tone | Comp FET | NSTR: knee widths per ratio, normal-mode harmonics (LF-weighted 2nd + 3rd) |
| `2d-comp-la` | quick | stairs, steps, tone | Comp opto | LA: PEAK = side-chain drive, GAIN = output gain, two-part cell memory, tube colouration |
| `2e-comp-l100` | full | stairs, steps, tone | Comp opto | L100: per-switch times, threshold 6 - 3.6 GR dBFS, knee 12 dB |
| `2f-comp-f670` | quick | stairs, steps, tone | Comp vari-mu | F670: 10 dB per threshold unit, THR map, fixed amp +10 dB, tube colouration |
| `30-comp-bliss` | full | stairs, steps, tone | Comp class A | BLISS: peak detector, hard knee, anti-log ballistics |
| `31-comp-onec` | full | stairs, steps | ONEC | ONEC: `gr` knob mapping onto the COMP engine, auto make-up |
| `32-dyn-wave` | quick | steps, bursts | WAVE | WAVE: differential envelope followers, attack/sustain weights |
| `33-dyn-ride` | full | steps, bursts | RIDE | RIDE: RMS level 100 ms, 2..200 dB/s slew, hold |
| `34-dyn-warm` | quick | fr, stairs, tone | WARM | WARM: shaper law, ~1.4 % THD at -12 dBFS (default), colour tilt |
| `35-fx-tape` | quick | fr, stairs, tone | TAPE | TAPE: saturator law, head bump, HF loss, 46 samples latency |
| `36-fx-exciter` | quick | fr, tone | EXCITER | EXCITER: tanh bias harmonics, TIMBRE shelf |
| `37-fx-enhance` | full | fr, tone | ENHANCE | ENHANCE: band maps, harmonic generator shares |
| `38-fx-pbass` | full | fr, tone | P-BASS | P-BASS: LR4 split, harmonic law |
| `39-fx-ukrock` | full | fr, stairs, tone | FX amps | UKROCK: gain staging and tone stack |
<!-- verify:scenes:end -->

### Results

Not run yet. `wing-verify compare ... --update-doc docs/VERIFY-ON-HARDWARE.md` replaces this block with the
latest run (date and firmware go in the line below it by hand).

<!-- verify:results:begin -->
<!-- verify:results:end -->
