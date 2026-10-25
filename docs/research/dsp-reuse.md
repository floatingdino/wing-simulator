# Reusable DSP survey (T007)

Written from working knowledge plus a limited web check; items marked **(verify)** must be confirmed against the
repo's LICENSE file before code is vendored. Rule of thumb: depend on crates/headers with MIT/BSD/Apache/ISC/0BSD/
public-domain licenses; **GPL/LGPL code is reference-only** (read the algorithm, reimplement; never copy) because the
app may be distributed. Airwindows source headers historically said "all rights reserved" and were later restated as
MIT by the author (web-confirmed via forum posts); keep a copy of the license text per vendored file.

## Library inventory

| Library | Lang | License | Use here | Notes |
|---|---|---|---|---|
| biquad | Rust | MIT/Apache-2.0 | Biquad coeffs (RBJ), DF1/DF2T | Tiny, no_std, RT-safe; we will still own our own SIMD biquad in wing-dsp. |
| fundsp | Rust | MIT/Apache-2.0 | Prototyping, filters (SVF, moog, ladder), reverb (`reverb_stereo`, FDN), delay, dynamics building blocks | Graph API allocates at build time only; process() is alloc-free. Good for reference/prototype; hot paths should be hand-rolled. |
| dasp | Rust | MIT/Apache-2.0 | Sample/frame types, interpolation, ring buffers | Useful glue; lightweight. |
| rubato | Rust | MIT | Async/sync resamplers (sinc, FFT, fast poly) | Supports ratio change at runtime (`set_resample_ratio`) -> drift compensation. Pre-allocate buffers; use `process_into_buffer`. |
| cpal | Rust | Apache-2.0 | Cross-platform audio I/O | CoreAudio backend OK but limited (no aggregate/workgroup control). |
| coreaudio-rs | Rust | MIT/Apache-2.0 | Direct AudioUnit/HAL access | Preferred for host bridge: device selection, IO proc, workgroup. Thin unsafe wrapper. |
| Airwindows | C++ | MIT | Compressors (Pressure4, ButterComp2, Compresaturator, Dyno), de-ess, console emulations, tape, many saturators, reverbs (Verbity, kPlate-style) | Single-file classes, no deps, mostly per-sample float/double; easy to port to Rust. Quality varies; fixed 44.1k assumptions in some (rate-scale needed). Verify exact plugin names. |
| DaisySP | C++ | MIT (core); **LGPL-2.1 modules in separate DaisySP-LGPL repo** (web-confirmed) | Chorus, flanger, phaser, delay line, svf, ladder, overdrive, compressor/limiter, pitch shifter, reverbsc (Csound-derived: LGPL) | Use only MIT repo. ReverbSc is LGPL: avoid. Embedded-style, alloc-free. |
| Signalsmith DSP | C++ header | MIT (0BSD on newer) **(verify)** | Filters, delay, FFT, reverb examples (Basic Reverb, Dattorro-like), mix matrices, oversampling (FIR halfband), envelopes; **Signalsmith Stretch** for time/pitch shift | High quality, clean headers, RT-friendly. Best single C++ source for oversampling + pitch. |
| Faust libraries (`stdfaust.lib`) | Faust | Mixed per-file (mostly MIT/STK-4.3 or LGPL with exception) **(verify per .lib)** | Compressors (`compressors.lib`), filters, reverbs (zita, jcrev, freeverb), phaser/flanger, tonestack | Faust compiler generates C++/Rust with an output license exception, so generated code is usable, but check the .lib file headers (some are LGPL). Good for rapid generation of validated reference implementations; Rust backend exists. |
| Freeverb | C++ | Public domain | Room/hall fallback reverb | Dated sound; acceptable "ROOM" baseline. |
| Dattorro plate | paper + many impls (MIT impls: Signalsmith examples, Faust `dattorro_rev`, DaisySP-like) | algorithm free; check impl | Plate reverb (WING "PLATE", "VINTAGE ROOM" bases) | Reimplement from the 1997 JAES paper; no license issue. |
| Zita-rev1, Dragonfly, MVerb, Rubber Band | C++ | **GPL** | Reference only | Do not vendor. |
| CHOW DSP (ChowTapeModel, ChowPhaser, ChowCentaur, ...) | C++ | Plugins **GPL-3**; `chowdsp_utils` library is **BSD-3** **(verify)** | `chowdsp_utils`: WDF, filters, delay, SIMD, oversampling helpers; plugin code = algorithm reference only | Use utils (BSD) if JUCE dependency can be avoided (it has a JUCE-free core). Otherwise read papers (Jatin Chowdhury, DAFx) and reimplement WDF models. |
| r8brain-free-src | C++ | MIT | Offline/online high-quality resampling, minimum-phase option | Excellent quality; C++ FFI cost. Good for offline renderer. |
| libsamplerate | C | BSD-2 (since 0.2.0) | Resampling | Fixed ratio API with `src_set_ratio` for dynamic. Not needed if rubato suffices. |
| Surge XT / Vital FX | C++ | **GPL-3** | Reference only (chorus, phaser, flanger, reverb, waveshapers) | Flag: do not vendor. |
| Neural Amp Modeler Core | C++ | MIT **(verify)** | Optional amp sim | Heavy; only if neural amps are desired, not for WING parity. |
| Guitarix | C++ | GPL | Reference only | |

## Mapping: WING block / model family -> source

Effort: S = <1 day, M = 1-3 days, L = 1+ week per family, XL = research-heavy.

| Family | Recommended source / algorithm | License | Effort | RT-safety notes |
|---|---|---|---|---|
| Channel LC/HC filters, high/low shelf, parametric EQ (STD, 6-band etc.) | Own biquad (RBJ cookbook, TDF2 / SVF with per-sample coeff smoothing); `biquad` crate for cross-check. Match WING bilinear warping and Q definitions to snapshot params | MIT/own | S | Fixed state arrays; smooth coefficients (or use SVF/Cytomic for modulation); denormal flush (FTZ/DAZ or add dc-offset). |
| Gate / expander / ducker (incl. Gate "Soul/Warm" style modes) | Own: peak/RMS detector + hold/attack/release state machine + range; Airwindows Gatelope/Dyno for ideas; DaisySP dynamics for reference | own / MIT | M | Lookahead via preallocated delay; sidechain filter biquads. |
| Compressor: clean VCA / "Standard" | Own feed-forward log-domain compressor with soft knee; Faust `compressors.lib` as reference (verify) | own | M | Per-sample gain smoothing; branchless. |
| Opto (LA-2A style) | Own two-stage program-dependent release model (published T4 cell models), Airwindows ButterComp2 / Pressure4 for flavour | MIT/own | M-L | Nonlinear state; double precision recommended; bounded loops. |
| FET (1176 style) | Own: FET gain-reduction curve + fast attack + ratio buttons "all-buttons" mode; Airwindows Pressure4 as base | MIT/own | M-L | Optional 2x oversampling for the saturation stage. |
| Vari-mu (Fairchild/Manley style) | Own tube-ish variable-mu gain cell; Airwindows Vari-mu-like (Compresaturator/"Mu") **(verify names)** | MIT/own | L | Oversample 2x; waveshaper aliasing. |
| Vintage/console EQs (Pultec, Neve 1073-ish, API 550-ish, SSL 4000-ish) | Pultec: passive-network model (shelf boost+cut interaction) as cascaded biquads fitted to published curves; Neve/API/SSL: fixed-frequency biquad sets with proportional-Q (API) / Q-shape changes (SSL); Airwindows console + EQ plugins for saturation; CHOW WDF ideas for passives | MIT/own (BSD utils) | L (family) | All linear except optional transformer saturation; coefficient tables precomputed; no heap. |
| De-esser | Own: sidechain bandpass/HP detector -> broadband or split-band gain reduction; Airwindows DeEss as reference | MIT/own | S-M | Use linear-phase-free biquads; lookahead optional. |
| Limiter (brickwall/true-peak) | Own lookahead limiter with 4x true-peak detection (Signalsmith or own halfband) | own | M | Lookahead delay preallocated. |
| Plate reverb | Dattorro plate (paper), tuned by ear/measurement to WING PLATE | free | M | Fixed delay lines allocated at init; modulated taps with interpolation. |
| Hall / Room / Ambience / Vintage rev | FDN (Hadamard/Householder) with per-line damping: Signalsmith Basic Reverb pattern, fundsp FDN, Faust `zita_rev_fdn`-style (algorithm only); Freeverb for cheap ROOM | MIT / PD | M-L | Large buffers (several MB) allocated at init; avoid denormals in feedback. |
| Delays (stereo, tap, ping-pong, BBD/tape/analog flavours) | Own fractional delay (Lagrange/Thiran), feedback filters; Airwindows tape/BBD-ish for colour; CHOW tape ideas (reimplement) | MIT/own | S-M | Max delay buffers preallocated; interpolate for time changes (declick). |
| Chorus / Flanger / Phaser | DaisySP chorus/flanger/phaser (MIT), Faust `phaflangers.lib` (verify), allpass-chain phaser; Surge/Vital = reference only | MIT | S-M | LFO tables precomputed; interpolation required. |
| Pitch shifter / harmonizer / "Pitch" FX | Signalsmith Stretch (MIT) for quality; DaisySP PitchShifter (MIT) or granular/dual-delay for low latency. Rubber Band is GPL: no | MIT | M-L | Signalsmith Stretch is alloc-after-init, latency a few tens of ms; configure block-size up front. |
| Amp / cab / guitar FX | Tonestack (Yeh/Smith models, Faust `tonestack`), WDF triodes (CHOW utils BSD), Airwindows saturators, cab = IR convolution (own partitioned FFT) | mixed | L | Convolution: preallocate partitions; oversample nonlinear stages 2-4x. |
| Multiband (compressor, de-esser split) | Linkwitz-Riley 4th-order crossovers (biquad cascade, phase-aligned) + per-band dynamics | own | M | Fixed band counts; allpass compensation. |
| Exciter / enhancer / saturation | Airwindows (Aura/Tape/Density etc.), own harmonic exciter (HP -> nonlinearity -> mix) | MIT | S-M | Oversample 2-4x or use ADAA (antiderivative anti-aliasing). |
| Utility: gain, pan laws, mid-side, delay comp, meters (RMS/peak/LUFS/true-peak) | Own | n/a | S | Metering via lock-free atomics to UI/proto thread. |

## Resampler for clock drift (host output bridge)

Recommendation: **rubato** (MIT) async sinc resampler (`SincFixedIn` with modest sinc length, linear/cubic interp
between sinc tables) driven by a PI-controller on ring-buffer fill level that adjusts the ratio around 1.0 by a few
ppm via `set_resample_ratio`. Rationale: Rust-native, no FFI, alloc-free after construction, runtime ratio changes,
adequate quality (>110 dB at mild ratios). Practical notes: target 1-2 blocks of buffer fill; smooth ratio changes
(slew < 1 ppm per ms); use a smaller kernel (e.g. 64-128 taps) for the realtime path; run in the output
bridge thread, never on the engine thread. Fallbacks: r8brain-free-src (MIT) for offline renders; libsamplerate
(BSD-2) if rubato's latency/CPU disappoints. Alternative requiring no resampler: aggregate/clock-slaved CoreAudio
device, but not possible for independent devices (e.g. Scarlett), hence the adaptive resampler stays.

## Oversampling

- Default: **2x/4x polyphase half-band FIR** cascades (Signalsmith-style, or own with `wide`/NEON SIMD),
  linear-phase variants for dynamics (latency reported to the delay-compensation graph) and minimum-phase/IIR
  half-band for low-latency saturators. 
- Prefer **ADAA** (1st/2nd order antiderivative anti-aliasing) for memoryless waveshapers (tube/FET/tape
  saturation) to reduce the oversampling factor; reserve 4x for strongly nonlinear amp/vari-mu stages.
- Per-block oversampling selection is a compile-time const generic to keep the hot loops branch-free; all
  filter state preallocated; denormals flushed via FTZ on the audio thread.
- Measure aliasing (sweep + FFT) in tests; each block gets unit test + bench per backlog rule 7.

## Licensing summary / risk

- Safe to vendor: biquad, fundsp, dasp, rubato, cpal, coreaudio-rs, Airwindows (MIT, keep notices), DaisySP (MIT repo
  only), r8brain, libsamplerate (BSD-2), Signalsmith (verify), Freeverb (PD), chowdsp_utils (BSD-3, verify).
- Avoid / reference only: DaisySP-LGPL (ReverbSc), Surge XT, Vital, CHOW plugin sources, Zita-rev1, Dragonfly,
  MVerb, Rubber Band, Guitarix (all GPL/LGPL).
- Faust: allowed with generated code, but audit each `.lib` header.
- Add a `THIRD_PARTY_LICENSES` file and a CI check listing every vendored file and its license.

## Proposed follow-ups

- Verify the **(verify)** items against upstream LICENSE files; write wing-dsp primitives (biquad/SVF/delay/ADAA/
  halfband) first; prototype rubato drift loop with a synthetic 50 ppm clock offset.
