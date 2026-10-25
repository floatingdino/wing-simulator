# WING-Edit reference curves (T00A)

Output: `data/reference-curves/*.json` (14 files, about 13 MB), produced by `tools/curve-extract/curves.py`
(shared helpers in `tools/curve-extract/curvelib.py`). Source: WING-Edit 3.3.4, arm64 slice. The binary was
only emulated function by function in Unicorn (the T003 harness). It was never run natively, debugged or modified.

## TL;DR

- WING-Edit draws its filter, EQ, sidechain and crossover graphs by evaluating **the same biquad coefficients its
  `*_parchg_*`/`update()` code computes for the DSP**. For every curve we export the app's own magnitude, the
  traced **biquad cascade itself** (`[b0,b1,b2,a1,a2]`, float32), and the exact double-precision magnitude and
  phase of that cascade. 100% of the exported EQ, filter, sidechain and crossover curves passed a
  bit-faithful re-evaluation check (max deviation < 1e-5 dB against the app's respf).
- **Dynamics:** only the four WING-native models have a real static curve in WING-Edit: GATE, DUCK, COMP and EXP
  (`gate_respf_xseries_gate`, `gate_respf_xseries_ducker`, `comp_respf_xseries_comp`, `comp_respf_xseries_exp`).
  We export input→output curves over −90…+20 dB and recovered the exact formulas (§4). The other 28 dyn models
  have stub respf functions that return constant 0 or 1, because their GUI shows a gain-reduction meter instead of
  a curve. For all 32 models we export the GR-meter mapping (`*_gain_*`).
- **Priority set for T03C:** STD EQ for ch, aux and bus/main/mtx, including all 13 band types; channel
  LC/HC 6/12/18/24 and HC 6/12; tool filters TILT/MAX/AP1/AP2; GATE/DUCK/COMP/EXP static curves. All of these
  are complete.
- **Alternate EQs:** SOUL, E88, E84, F110, PULSAR, MACH4 and PIA 560 are complete (one-factor-at-a-time band grids).
  MACH4 is a **parallel** filter bank (H = Σ sections), not a cascade.

## 1. Function inventory and calling conventions (arm64, AAPCS64)

Addresses are for this build. All functions are located by symbol (`curvelib.py`), except the globals listed.

| function | address | signature / convention | what it evaluates |
|---|---|---|---|
| `ae::Cchfilter::getResponseCached(float pos)` | 0x100015c74 | x0=this, s0=normalised x-position 0..1 → s0 dB | 4 cascaded biquads at `chdsp[ch]+0x12c` (LC, LC, HC, tool), cached at 500 points `f_i = 20·1.01391^i` Hz, linear interpolation, clamp ±48 dB |
| `ae::Cequalizer::getResponseCached(float pos)` | 0x100043af0 | same | switch on `this+0x1b0` (usage): 0→`f_table_ch`, 1→`f_table_bus`, 3→`f_table_aux` model `respf` at entry +0x30. Usages 2 and 4 are inline 3- and 8-biquad cases that no tree node uses |
| `ae::eq_respf_<m>(int idx, float f_hz, int usage)` | see below | w0=channel/bus index (`this+0x1b8`), s0=Hz, w1=usage (0 ch, 1 bus/main/mtx, 3 aux) → s0 dB | cascade of 5-float biquads in `_AE->chdsp[idx]` (+0x6c0) or `_AE->busdsp[idx]` (+0x6c8), result `10·log10∏|H_i|²` (float32), −144 if 0, clamped to ±48 |
| `ae::Cdynsidechain::getResponse(float f)` | 0x100032164 | x0=this (+0xd8 idx, +0xdc usage), s0=Hz → dB | 1 biquad at `chdsp+0xb4+0x14·usage`, **stored (A1,A2,b1,b0,b2)** |
| `ae::Ccompxover::getResponse(float f)` | 0x1000322dc | really static: w0=ch, w1=usage (2→busdsp), s0=Hz → dB | 1 biquad at `chdsp+0xdc` |
| `ae::<kind>_respf_<m>(float in_dB, Cpar** slots)` | `f_table`+0x38 | s0=input level dB, x0=`Cdynamics+0xe0` (model-slot Cpar* array) → s0 output dB | static curve (4 models) or stub |
| `ae::<kind>_gain_<m>(float, float g)` | `f_table`+0x50 | s1=linear gain (s0 unused) → meter position | GR meter mapping |
| `ae::<kind>_led_<m>`, `_state_<m>` | +0x40, +0x48 | (float, float) → int | threshold LED / state flags (not exported) |

EQ respf per model (`f_table_ch` 0x105088740, `f_table_aux` 0x105088900, `f_table_bus` 0x105088a80; entry = 0x40 bytes
`{name, long, id, pardef, parchg, init, respf, bdsolo}`):

| model | respf | usage(s) | sections |
|---|---|---|---|
| STD (ch, aux) | `eq_respf_xseries` 0x10003dad8 | 0, 3 | 6 cascaded (L, 1–4, H) |
| STD (bus/main/mtx) | `eq_respf_xseries_bus` 0x100034698 | 1 | 10 cascaded (L, 1–6, H, tilt, …); BW48/LR48 use more than one section per band |
| SOUL | `eq_respf_ssl_g` 0x10003652c | all | 6 |
| E88 | `eq_respf_neve88rs` 0x100037c98 | all | 6 |
| E84 | `eq_respf_neve84` 0x1000388f4 | all | 6 |
| F110 | `eq_respf_isa110` 0x100039f04 | all | 6 |
| PULSAR | `eq_respf_pultec_duo` 0x10003b64c | all | 6 |
| MACH4 (ch only) | `eq_respf_maag4` 0x10003ee90 | 0 | 6 **parallel**, summed with complex division (Smith); usage 3 branch cascades |
| PIA 560 (bus) | `eq_respf_api560` 0x10003c7e8 | 1 | 10 |

Dynamics model table: `ae::f_table` 0x105086ff0, entry 0x70 bytes `{name, long, id, pardef, alginit, parchg, init,
respf, led, state, gain, flags…}`. The model selector maps through `ae::Cdynamics::ftblofs` (0x1051dda34, 0x84 bytes
per usage row, usage at `Cdynamics+0x18c`: 0 gate slot, 1 ch dyn, 2 bus/main/mtx, 3 aux). This mapping is the identity
except on aux (28 models). Model ↔ code name: GATE xseries_gate, DUCK xseries_ducker, E88 gate_neve88rs,
9000G ssl9000j, D241G drawmer241, DS902 dbx902, DEQ dyneq, DEQ2 dyneq2, WAVE transient, PSE pse545, CMB combo,
RIDE autogain, WARM warmth, COMP xseries_comp, EXP xseries_exp, B160 dbx160, B560 dbx560, D241C d241c, ECL33 n33609,
9000C ssl9000, SBUS sslgbus, RED3 d3, 76LA u1176, LA la2a, F670 f670, BLISS mpress, NSTR distress, 2250 api225l,
L100 tla100a, E88C comp_neve88rs, LMT lmcplus, ONEC oneknob. `triband` exists as entry 32 but is not selectable.

**Biquad memory layout** (every EQ/filter/crossover site): 5 float32 values `(b2, b0, b1, A2, A1)` with
`y = b0·x + b1·x[-1] + b2·x[-2] + A1·y[-1] + A2·y[-2]`. Exported as `[b0,b1,b2,a1,a2]` with `a = −A`.

**Globals:** `_AE` 0x1051fd9b0 (CaudioEngine; +0x6c0 per-channel DSP structs, +0x6c8 bus DSP structs). `_AEDRV`
0x1051fb748 (CdummyAudioEngine: +8 = fs = 48000.0, **+0xc = 0.0**, +0x10 = π/fs).

## 2. Method

1. Build the tree exactly as T003 does (`extract.Extractor`). `curvelib.py` adds the libm stubs `tan`, `tanf` and
   `frexp`, which the coefficient code needs (bilinear prewarp). It does this by wrapping the hook instead of
   editing T003's `wingemu.py`.
2. For each grid point: `ncl::Cpar::set(float,0,0)` or `setInt`, then the block's own virtual `update_direct(mask)`
   (re-runs pardef on a model change) and `update(mask)` (runs the model's parchg, writes the biquads, posts them to
   the dummy engine, and sets the GUI cache dirty flag). **`update_direct` alone does not compute coefficients.** Every
   block first gets a full-mask `update(0xffff)`, because the initial state has zeroed biquads.
3. Call the GUI evaluator for each of the 270 grid frequencies (10 Hz–24 kHz, 1/24 octave, `freq_hz`). During the
   first call, trace every memory read. The coefficient reads give the cascade WING-Edit evaluates.
4. Re-evaluate that cascade twice. (a) Mimic the app: ω and sincos in float32, the rest in double, then
   `10·log10|H|²`, float32, clamped ±48 dB. This must match the app to < 0.005 dB or the cascade is rejected; the
   actual worst case was < 1e-5 dB. (b) Exact double precision, unclamped, with phase; this is exported. For the
   channel filter, the check uses `Cchfilter::getResponseCached` on its own 500-point grid.
5. Dynamics: call `respf(in_dB, Cdynamics+0xe0)` for each input level (−90…+20 dB, 0.5 dB steps).
6. Parameters are recorded as **read back after `set()`**, so they are quantised to each Cpar's step grid (for
   example 1000 Hz becomes 1002.37 Hz on a log-641-step parameter).

Reproduce (about 45 s, deterministic, byte-identical across runs):
```
python3 -m venv VENV && VENV/bin/pip install unicorn capstone lief
VENV/bin/python -I tools/curve-extract/curves.py --app WING-Edit.app --out data/reference-curves [--only filter,eq-std,dyn,sc,meter,eq-alt]
```

## 3. Files, format and coverage

Common header: `format: wing-reference-curves/1`, `source` (sha256 of the WING-Edit executable), `fs: 48000`,
`freq_hz[270]`, `conventions`. Each curve has `case`, `params` (all params of the block that were set),
`mag_db` (0.001 dB), `phase_deg` (0.01°, wrapped), `biquads`, `check_max_dev_db` and `gui_vs_exact_max_db` (the
GUI's own float32 error, up to about 0.1 dB for narrow low-frequency sections). `dyn-static.json` uses `input_db[]`
and `out_db[]` instead.

| file | block / models | grid | curves |
|---|---|---|---|
| `filter.json` | `ch/*/flt` | flat; LC 6/12/18/24 × 8 freqs (20–2k); HC 6/12 × 9 freqs (50–20k); LC24+HC12; tool TILT ±1/3/6, MAX low×proc 5×5, AP1 × 5 f, AP2 × 5 f × 4 q | 108 |
| `eq-std-ch.json` | `ch/*/eq` STD (`xseries`) | band 2 PEQ: g {±3,±9,±15} × f {30,100,300,1k,3k,10k,18k} × q {0.442,0.7,1,2,4,10}; bands 1/3/4 spot checks; L & H × {PEQ,SHV} × g {±6,±15} × 7 f × q {0.442,1,4,10} | 704 |
| `eq-std-aux.json` | `aux/*/eq` STD (same code, plus CUT) | band 2 grid as above; L/H CUT × 7 f × 5 q | 326 |
| `eq-std-bus.json` | `bus/*/eq` STD (`xseries_bus`), which is also the code for **main and mtx** (usage 1) | band 2 grid; L & H × PEQ/SHV (as ch), CUT × 7 f × 5 q, BW6/BW12/BS12/LR12/BW18/BW24/BS24/LR24/BW48/LR48 × 7 f; tilt ±1/3/6 | 920 |
| `eq-soul.json` … `eq-pia.json` | SOUL 113, E88 121, E84 57, F110 197, PULSAR 148, MACH4 31 (ch), PIA 43 (bus) | one-factor-at-a-time per band: gain × freq knob × Q/type, other bands at 0 | 710 |
| `dyn-static.json` | GATE (thr 6 × range 5 × ratio 5), DUCK (4 × 5), COMP and EXP (thr 5 × all 17 ratios 1.1…100 × knee 0–5) | input −90…+20 dB (221 points) | 1190 |
| `dyn-sidechain-xover.json` | `ch/*/dynsc` (type OFF/LP12/HP12/BP/NOTCH × f × q) and `ch/*/dynxo` crossover (OFF/LO/HI/S6/S12/PEAK) | 5 f, 4 q for BP/NOTCH/PEAK | 51 + 41 |
| `dyn-models.json` | all 32 dyn models | function names, f_table index, respf-is-stub flag, GR-meter mapping at 16 GR values | 32 |

Characteristic results (for quick sanity checks): channel LC and HC are Butterworth: −3.0 dB at fc for every slope,
and 18/24 dB use 2 sections. On the bus, BW6–BW48 are −3 dB at fc, LR12/LR24/LR48 are about −6 dB, BS12/BS24 are
Bessel-like, and CUT with q = 0.707 equals LR24. STD PEQ with q = 1 and +15 dB at 1 kHz gives +10.1 dB at 500 Hz and
+10.3 dB at 2 kHz (bilinear cramping near Nyquist is visible at 10k/18k). A shelf at +6 dB is about +3.9 dB at fc.

## 4. Formulas recovered from disassembly

**EQ/filter display:** ω = (2f)·(π/fs) in float32. For each section, num = b0 + b1·e^{−jω} + b2·e^{−j2ω} and
den = 1 − A1·e^{−jω} − A2·e^{−j2ω}. The result is `10·log10(∏|num/den|²)`, or for MACH4 (ch) `10·log10|Σ num/den|²`,
then −144 if zero and clamped to [−48, +48]. **The GUI curve is exactly the DSP biquad set that WING-Edit computes**,
not an idealised analog prototype. The channel LC/HC use `Cbiquad::hqhp6/hqhp12/hqlp12` with fs-dependent tables
(`hptab44`/`hptab48`, chosen by fs < 46 kHz).

**GATE (`gate_respf_xseries_gate`)**, slots: thr, range, att, hld, rel, acc, ratio. For in < thr:
`out = in + max(k·(in − thr), −R)`, where k = 0.5 / 1 / 2 / 3 for ratio 1:1.5 / 1:2 / 1:3 / 1:4 (the expander slope
is 1+k), and for `gate` k·(in − thr) is replaced by −144. R = trunc(range) (integer dB), **except when range is at its
maximum step (60 dB), where R = 144** (infinite). For in ≥ thr, out = in.

**DUCK (`gate_respf_xseries_ducker`)**: `out = in − range` if in > thr, else in. The GUI plots the key level on the
x axis.

**COMP (`comp_respf_xseries_comp`)**, slots: thr, ratio (fenum, 17 values 1.1…100), knee (int 0–5), …. Work in
"bits": x = thr/6 − in/6.0206 (**note the mixed constants 1/6 and 1/20·log10 2**, so the effective threshold is
thr·1.00343, e.g. −60 → −60.21 dB). With K = knee:
- x < −0.6K (above the knee): s = x
- |x| ≤ 0.6K: s = 0.5·(x²·c + x − 0.3K), where c = −1/(1.2K) (c = 1 if K < 1); this is a quadratic knee
- x > 0.6K: s = 0

Then `out = in + (1/ratio − 1)·(−6.0206)·s`. The knee width is 1.2K bits ≈ 7.2·K dB, centred on the threshold.

**EXP (`comp_respf_xseries_exp`)** mirrors COMP below the threshold: slope ratio (out = thr + ratio·(in − thr)),
with the same quadratic knee construction.

**Detector and time constants (from `comp_parchg_xseries_comp` and `gate_parchg_xseries_gate`; T = `_AEDRV+0xc`,
which by its use is the sample period. The control path runs once per 8 samples, so 8T is the block period).**
- Gate: the side-chain one-pole coefficient is `(1−tan(40π/fs))/(1+tan(40π/fs))` (a 40 Hz corner).
  thr_lin = e^{thr·ln10/20} (0 if thr ≤ −98). The ratio law is stored as `−1/√thr_lin` (1:1.5), `1/thr_lin` (1:2),
  `1/thr_lin²` (1:3), …. Attack: `1 − e^{−2200·T/(att_ms/8)}` (2200 ≈ 1000·ln 9, so att is a 10–90% time; 0 if
  att/8 < 0.01). Hold: `8000·T/hld_ms` (counter increment per block). Release uses the same pow(e, −2200·T/…) form.
- Comp: att clamped to ≥ 0.5 ms. With env = LOG: att_c = 8T·1000/att, rel_c = 8T·1000/rel. With env = LIN:
  att_c = 8T·16.64/att, rel_c = 8T·16.64/rel. Hold = 8T·500/hld. Auto mode replaces these with fixed constants
  (8T·1.197, release 8T·21.24 or 8T·4.2e-4 for RMS/peak). Detector smoothing coefficients are fs-dependent
  constants (≈0.995…0.999, selected by fs < 46 kHz and det = PEAK/RMS). Knee terms are posted as −0.3K, 0.6K and
  −1/(1.2K), and ratio as −(1 − 1/ratio). These match the static formula above, so **the GUI static curve is the
  same law the DSP is parameterised with.**
- **Caveat:** in WING-Edit the dummy engine's T field (+0xc) is 0, so the time coefficients it posts are degenerate.
  Only the formulas, not WING-Edit's posted numbers, are usable. The console presumably fills in T = 1/fs.

**GR meter (`*_gain_*`)**: `m = 20·log10(g)/(−FS)`, clamped at a per-model cap. The FS and cap values are in
`dyn-models.json`: for example GATE/DUCK FS 60 dB, COMP/EXP/9000C/LA… FS 20 dB, E88 cap 2.5, B560 cap 2.0,
D241C 1.5, NSTR 1.3. RIDE, 76LA and L100 are negative (they deflect the other way). B160, SBUS, DEQ, DEQ2 and ONEC
use a log-shaped mapping, WAVE is constant 1, and WARM is 0.

## 5. Caveats and gaps

- **GUI vs DSP:** the curves are WING-Edit's coefficient sets at fs = 48 kHz. We assume the console computes the
  same coefficients, since the parchg code is shared by name and posts them to the engine interface, but this is
  unverified on hardware. Filter structure (DF1/DF2, SVF, double precision) and the 44.1 kHz tables are not
  covered. Dynamic curves are static laws and exclude attack, release and detector behaviour.
- `gui_vs_exact_max_db`: WING-Edit's own drawing has float32 errors of up to about 0.1 dB for high-Q
  low-frequency sections. Use `mag_db`/`biquads` as the oracle, not the GUI clamp.
- **No static curve exists in WING-Edit** for the 28 vintage, character and dynamic-EQ dyn models (stub respf).
  Their behaviour must come from T04x research. DEQ/DEQ2 dynamic-EQ graphs are drawn by
  `gui_modules::DynEqGraph::setDynDeq*` and were not driven (follow-up).
- **FX:** no FX model has a respf. FX EQs (SOUL/E88/E84/F110/PULSAR/MACH4 as FX, DEQ3 via `DynEqGraph::set3DEQFx`,
  SPKMAN via `setSpkPro`) and GEQ/PIA FX were not driven. They probably reuse `eq_parchg_*` through `f_table_all`
  (0x1050890a0), but this is unverified.
- Grids are one-factor-at-a-time for the alternate EQs. Interactions between bands are trivially a product of
  sections for cascaded models, but they are **not** a product for MACH4 (parallel sum). `biquads` let consumers
  evaluate any frequency.
- Main/mtx EQ are not swept separately; they use the same code and usage (1) as bus. EQ bands other than 2 were
  spot-checked only (identical code per band).
- The channel filter's tool section shares the 4th biquad. `tf=0` gives unity there.

## 5a. 44.1 kHz set and the EQ mix law (T04J)

`data/reference-curves/44k1/` (about 330 kB): `filter.json` (LC/HC, tool TILT/MAX/AP1/AP2), `eq-std-{ch,aux,bus}.json`
(small grids incl. `mid-mix` and L/H shelf curves at mix 0/25/50/125 %), `eq-alt.json` (6 random settings per
alternate model, mix cycling 100/50/125/0/75/110 %). Produced by `tools/curve-extract/curves44k1.py`, which patches
`_AEDRV+8` (fs) and `_AEDRV+0x10` (pi/fs, float) to 44.1 kHz before any update; every coefficient routine reads these
two fields on each call (and selects the 44.1 kHz tables via fs < 46 kHz). Each curve records the block's complete
parameter set (exact float32), `biquads` and `mag_db` (1/6-octave grid 20 Hz-20 kHz). Byte-identical on rerun.

**EQ mix law:** `ae::Cequalizer::update` (0x100032554) calls the selected model's `parchg` from `f_table_{ch,bus,aux}`
with `s0 = mix / 100.0f` for usages 0, 1 and 3 (0x1000327b4, 0x100032c48, 0x100032cb8), the same call for STD and every
alternate model; each parchg multiplies its knob gains by it (STD: PEQ/SHV band gains, not cuts or tilt). Nothing
blends dry and wet. The engine's `ModelSlot` therefore forwards `mix` (0..125 %) to every `*/eq` model.

## 6. Proposed follow-ups

- Drive `DynEqGraph::setDynDeq/setDynDeq2/set3DEQFx/setSpkPro` for the DEQ, DEQ2, DEQ3 and SPKMAN graphs.
- Hook `CdummyAudioEngine` vtable+8 with T set to 1/fs to capture every model's posted DSP parameter block per
  parchg (time constants for all 32 dyn models, and FX coefficients).
- ~~Repeat at fs = 44.1 kHz~~ done for filter/EQ (§5a, T04J); sidechain/crossover not yet.
