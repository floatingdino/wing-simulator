# WING-Edit 3.3.4: parameter schema extraction (T003)

Output: `data/schema/wing-tree.json` (about 4.4 MB), produced by `tools/schema-extract/extract.py`.
Source: `WING-Edit.app` 3.3.4, sha256 `2aa6aafc…db530`, arm64 slice. The app was not modified, re-signed, run
natively, or attached to a debugger.

## TL;DR

- WING-Edit builds the **console's whole parameter tree** in memory: 43,008 nodes, 36,499 parameters. This happens in
  `CaudioEngine::CaudioEngine()` (namespace `ncl`: `Cnode`/`Cpar`; namespace `ae`: audio-engine blocks). The binary is
  not stripped, so every class and function name is available.
- We ran **only that constructor** inside a CPU emulator (Unicorn, with every import stubbed in Python). Then we read
  the resulting objects and described each parameter with the app's own `ncl::Cpar::describe()`. That is the same
  routine the console uses to answer a "describe" request. The result is the full tree, built by Behringer's own code.
- **Node hashes are computed, not stored.** Each hash is a MurmurHash3-like function of the node path (§3). Our Python
  reimplementation (`tools/schema-extract/winghash.py`) matches the app on **43,007/43,007** nodes. It also matches the
  documented `/ch/1/mute=F50F69F8`, `/ch/1/fdr=38AE75C2` and all the root hashes in `docs/research/protocol.md`. This
  answers open question 1 there: WING-Edit sends `D7 <hash>` using locally computed hashes.
- **Every model of every block** (filter, unified gate/dynamics, EQ, FX full/lite, insert mode, delay mode) is listed
  with its full parameter definitions and its **per-model defaults**. We got these by switching each model selector
  in the emulator and letting the block's own `update_direct()` run the model's `pardef` function.
- Coverage: **99.99%** of the firmware 3.0.6 factory scene (25,078/25,081 keys; this is the same file as the
  WING-Edit embedded default and T008's `0006211b-snapshot_11.json`). **92.0%** of `fixtures/PUBLIC.snap`
  (19,659/21,368). The PUBLIC.snap gap is version drift: it is firmware 1.13.9 (`snapshot.8`), and §6 lists the
  keys.

## 1. Method

1. `lipo -thin arm64` the executable into a temporary directory. Load the segments into Unicorn at their link
   addresses, so no rebasing is needed. Point all 2,186 dyld binds at trap slots. Python implements `malloc`/`new`,
   `mem*`/`str*`, the libm functions and `__cxa_guard_*`. Other imports return 0, which covers pthread, CF and objc.
2. Recreate what `MainComponent::MainComponent` does before building the tree. This means a
   `CdummyAudioEngine` instance at the globals `0x1051e48b0` and `0x1051fb748`. Then call `CaudioEngine::C2(this)`,
   which takes about 2 s in the emulator, and run `ncl::Cnode::setRoot` twice exactly as the app does. The first pass
   counts the nodes and the second fills the hash table.
3. Walk the tree using `Cnode` +0x70 (first child) and +0x78 (next sibling, circular). For every `Cpar`, call
   `Cpar::describe(Ctextstream*)` with a `Cstr<32768>` stream we build ourselves, read the raw definition fields, and
   call `getPath()`.
4. **Variant probing.** Take one representative instance of every structurally distinct block. For every enum
   parameter whose flags carry an update mask, set each value (`Cpar::setInt`) and call the block's virtual
   `update_direct(mask)`. Diff the subtree. Selectors that change sibling definitions become `variants`. That gives
   `mdl` (filter/gate/dyn/eq/fx), `dlymode`/`dly.mode` (delay units), `postins.mode`, and the `$ctl` user-button and
   layer modes. In total, 1,692 probes.
5. Templating: runs of indexed siblings with identical structure collapse into one `array` template. Examples are
   `ch/1..40` and `fx/1..8` vs `fx/9..16`.
6. Coverage against snapshots (§5). Defaults come from the embedded factory snapshot.

Reproduce:
```
python3 -m venv VENV && VENV/bin/pip install unicorn capstone lief
VENV/bin/python -I tools/schema-extract/extract.py --app WING-Edit.app --snap fixtures/PUBLIC.snap \
   --out data/schema/wing-tree.json --extra-snap data/firmware/json/0006211b-snapshot_11.json \
   [--coverage-report cov.json] [--dump-default-snaps DIR]
```
A run takes about 90 s. The output is deterministic, and the run self-checks the hash function on every node.

## 2. `wing-tree.json` format (`"format": "wing-tree/1"`)

| key | content |
|---|---|
| `source` | app version, binary sha256, slice, extractor |
| `node_count` | 43008 (includes root) |
| `hash` | hash algorithm description + self-check result |
| `units` | unit table from the binary: `dB % ms Hz m s ft` (unit nibble 1..7) |
| `hashes` | flat `{ "ch/1/fdr": "38ae75c2", ... }` for **every** node of the default tree (43,007) |
| `snapshot_sections` | snapshot section → tree prefix: `ae_data`→`/`, `ce_data`→`$ctl`, `ae_globals`→`$globals`, `ce_globals`→`$ctl/$globals` |
| `tree` | hierarchical tree (below) |
| `variants` | model-dependent parameter sets per block (below) |
| `coverage` | per-snapshot coverage stats + sample of missing keys |

**Tree node** (`kind: "node"`): `name`, `long` (the console's display name), `hash` (omitted inside array
templates; use `hashes` or compute it), `flags` (raw 32-bit `Cnode` flags, hex), `indexed: true` (an unnamed numbered
child), `children: [...]`.

**Array** (`kind: "array"`): `first`, `last`, `longs` (per-instance long names, for example `A1..A8`), and
`template` (the node with `name: "*"`). Defaults inside a template are instance `first`'s defaults.

**Parameter** (`kind: "par"`): `name`, `long`, `hash`, `flags`, `class` (`CparInt`, `CparBool`, `CparEnum`,
`CparLinFloat`, `CparLogFloat`, `CparLevel`, `CparStr`, `CparFenum`, `CparTrigger`, `CparVar` (a model slot),
`ae::CparTag`, `ae::Cstringlist`), and `type` from the flags nibble:

| type | fields | value law |
|---|---|---|
| `linf` | `min`, `max`, `step`, `steps`, `fmt`, `unit`, `describe` | `v = min + step*i`, `i = round(pos*(steps-1))` |
| `logf` | `min`, `max`, `logstep` (natural-log), `steps`, `fmt`, `unit` | `v = min * exp(logstep*i)` |
| `fader` | `min=-144` (shown as `-oo`), `max=10`, `steps=1024` | WING fader law, see below |
| `int` | `min`, `max` | |
| `enum` | `values` (wire/snapshot tokens), `long_values` (display names) | snapshot stores the token |
| `fenum` | `values` (floats), `labels` | e.g. `$globals/clkrate` 48000/44100 |
| `string` | `maxlen` | |

`describe` is the app's own text, for example `log [20.0 .. 2k00 Hz], 641 steps`. `default` is the value from the
factory snapshot (firmware 3.0.6). `unused_slot: true` marks a model slot that the current model does not use (named
`n/a` by the app; we name it by its slot index, the same way `getPath()` does).

**Fader law** (`CparLevel`, from `ncl::Cpar::recalcFIB`), for normalised position p in [0,1]:
`p<1/1024: -oo (-144)`; `p<0.0625: 480p-90`; `p<0.25: 160p-70`; `p<0.5: 80p-50`; `p>=0.5: 40p-30` dB.
A centre-detent variant of `linf` (flag byte at +0x100) snaps p in [0.46,0.54] to the centre. This is used by pans.

**Flags** (`Cnode +0x5c`): bits 20–23 = type (1 lin, 2 log, 3 fader, 4 int, 5 enum, 6 fenum, 7 string); bits 16–19
= unit; bits 0–15 = update mask passed to the owning block's `update()`/`update_direct()`; bit 24 = n/a slot; bit
26 (0x04000000) is set on almost all snapshot-persisted params; bit 27 (0x08000000) appears on transient/action
params. The meaning of bits 24–29 is only partly decoded. A leading `$` in a name marks a read-only or live value
that is not stored in snapshots. A leading `~` marks a hidden node, which `describe` skips.

**Variants**: `variants["ch/*/dyn"]["mdl"] = {selector, representative, max_params, variants:[{name, index, long,
params:[...]}]}`. Each variant param is a full parameter definition plus:
- `default`: the model's own default from its `pardef` function, as display text.
- `slot`: the position.
- `hash`: equal to `wing_hash("<block>/<slot>")`, for example `ch/1/dyn/1 = 104b1346`. That is the same id for every
  model (`thr` in GATE, `ingain` in LA), which confirms the "hash by position" behaviour libwing reports.

Parameters that are identical in every variant (`on`, `mdl`, `mix`, `gain`, ...) stay in `tree`. FX slots differ:
`fx/1` covers fx 1–8 ("full", 65 models) and `fx/9` covers fx 9–16 ("lite", 39 models, with no reverbs, delays or
modulation). The other keys are generalised (`*`).

Provenance per field: everything in `tree` and `variants` comes from the WING-Edit binary (emulated code and
data), except `default` in `tree`, which comes from the embedded factory snapshot (byte-identical in content to
T008's firmware scene). **libwing's records were not used.** They are not in the repo, and downloading them needs
user approval. The coordinator can cross-check by joining libwing's `propmap` on the hash. Given the 100% hash
self-check and the matching documented hashes, we expect agreement on ids. Differences should then reflect only
firmware 3.0.5 vs 3.x tree drift.

## 3. Node hash (answers protocol.md open question 1)

`ncl::Cnode::setRoot` (0x10010c9dc) hashes the text of `getPath()`, for example `/ch.1.flt.lcf` (`.` separators, and
`n/a` slots print their index), **without its first character**:
- upper-case a–z, skip spaces;
- pack 4 chars **big-endian** into k (`k=(k<<8)|c`);
- Murmur3 block mix with seed `0xcb21c6be`: `k*=0xcc9e2d51; k=rotl(k,15); k*=0x1b873593; h^=k; h=rotl(h,13);
  h=h*5+0xe6546b64`;
- a trailing partial block gets the **same full mix**;
- finally `h ^= nchars`, with **no fmix**.

This is why the standard CRC/FNV/Murmur variants tried in T002 did not match. Because ids depend only on the path,
**the simulator can compute every id itself** (`winghash.py`). `Croot::find(hash)` does a binary search over the
sorted table.

## 4. Model catalogue (summary; full definitions in `variants`)

- **Filter `ch/*/flt/mdl`** (tool filter; LC/HC are fixed params): TILT (TILT EQ), MAX (MAXER), AP1 (AP 90),
  AP2 (AP 180).
- **Gate slot and dynamics slot**: since firmware 3.x ("unidyn") both slots offer the same 32 models.
  `ch/*/gate/mdl`, `ch/*/dyn/mdl`, `bus|main|mtx/*/dyn/mdl`: GATE, DUCK, E88, 9000G, D241G, DS902, DEQ, DEQ2, WAVE,
  PSE, CMB, RIDE, WARM, COMP, EXP, B160, B560, D241C, ECL33, 9000C, SBUS, RED3, 76LA, LA, F670, BLISS, NSTR, 2250,
  L100, E88C, LMT, ONEC. Long names include WING GATE, BDX902 DEESSER, DUAL DYNAMIC EQ, WAVE DESIGNER,
  SOURCE EXTRACTOR, AUTO RIDER, SOUL WARMTH PRE, 76 LIMITER AMP, LA LEVELER, FAIR KID, NO-STRESSOR, PIA2250 RACK, and
  others. Aux channels get 28 (no DS902, DEQ, DEQ2 or WARM).
- **EQ**: ch has 7 (STD, SOUL, E88, E84, F110, PULSAR, MACH4); aux has 6 (no MACH4); bus/main/mtx have 7 (STD with
  27 params, i.e. more bands, plus PIA 560 GEQ instead of MACH4).
- **Inserts**: `preins` and `postins` select an FX slot (`ins`: NONE, FX1..FX16). `postins/mode` has
  FX/AUTO_X/AUTO_Y.
- **FX 1–8** (65 models): NONE, EXT, HALL, ROOM, CHAMBER, PLATE, CONCERT, AMBI, V-ROOM, V-REV, V-PLATE, GATED,
  REVERSE, DEL/REV, SHIMMER, SPRING, DIMCRS, CHORUS, FLANGER, ST-DL, TAP-DL, TAPE-DL, OILCAN, BBD-DL, PITCH, D-PITCH,
  VSS3, BPLATE, GEQ, PIA, DOUBLE, PCORR, LIMITER, DE-S2, ENHANCE, EXCITER, P-BASS, ROTARY, PHASER, PANNER, TAPE, MOOD,
  SUB, RACKAMP, UKROCK, ANGEL, JAZZC, DELUXE, BODY, SOUL, E88, E84, F110, PULSAR, MACH4, C5-CMB, SUB-M, V-IMG, SPKMAN,
  DEQ3, *EVEN*, *SOUL*, *VINTAGE*, *BUS*, *MASTER*. **FX 9–16** (39 models): the same list minus HALL…BPLATE.
- Also variant-driven: `in/set/dlymode` and `{bus,main,mtx}/*/dly/mode` (M/FT/MS/SMP change the unit and range of
  `dly`/`m`). Also `$ctl` layer strip `type` and user-button/encoder `mode`.

**DSP surprise (for T004 and the T03x/T04x tasks):** the binary also contains, per model, `*_respf_*` (response
curves for the GUI), `*_gain_*` (static gain computers for every compressor and gate) and `*_parchg_*` (parameter
→ DSP coefficient conversion, which posts to `CdummyAudioEngine`/`dsp::Cinputch`). It also has VSS3 reverb tables
(`ae::VSS*Table`, `VSS3_PresetTab`). The same emulator can drive these to produce reference transfer curves without
hardware. This is in scope only for DSP tasks, so it was not done here.

## 5. Coverage

| snapshot | keys | covered | via model variants | % |
|---|---|---|---|---|
| firmware 3.0.6 factory scene (= WING-Edit embedded `wing-fullsize`) | 25,081 | 25,078 | 2,732 | **99.99** |
| embedded `wing-rack` / `wing-compact` defaults | 25,083 / 25,081 | 25,080 / 25,078 | 2,732 | 99.99 |
| `fixtures/PUBLIC.snap` (fw 1.13.9, `snapshot.8`) | 21,368 | 19,659 | 2,914 | **92.0** |

The **union target** is met for 3.x. The 3 keys missing in every 3.x snapshot are `$ctl/$globals/usewheel`,
`timefmt` and `datefmt`. They are local GUI preferences that some other constructor adds (`timefmt` is referenced
at 0x1002109ec), outside `CaudioEngine`.

PUBLIC.snap keys not in the 3.x tree (1,709) are all legacy 1.x schema:
- `ch|aux/*/send/*/ind`, `wid`: 1,536 keys. Per-send independent and width settings were removed in 3.x.
- `mtx/*/dir/*/{on,lvl,inv,in,tap}`: 80 keys. The old matrix direct-input layout.
- `aux/*/dyn/{thr,depth,fast,peak,ingain,cpeak,cmode}`: 56 keys. These are 1.x aux "combo" dynamics parameter names.
- `bus/*/busmode`: 16. `main|mtx/*/dly/m`: 12 (3.x has `dly/mode` + `dly/<unit>`). `fx/{11,13,14,16}/40`: 4 (an
  unused-slot index dumped by 1.x).
- `cfg/talk/indiv`, `$ctl/safes/area/{L,C,R}`, `$ctl/$globals/usewheel`.

Loading a 1.x scene (T011) therefore needs a small migration map. 3.x scenes are fully covered.

## 6. Protocol constants found (answers to protocol.md §9 where possible)

- **Discovery**: `nrp::broadcastDevice` sends the 5-byte string `WING?` to 255.255.255.255, to each interface
  broadcast address, and to `192.168.1.1`. `ConnectPage::findDevices` re-runs every 2,000 ms (`0x7d0`). Port **2222**
  (`0x8ae`) is used for UDP (`UdpConnection::initialiseSocket`) and TCP (`ConnectPage::connect_device`).
- **Firmware gate** (`ConnectPage::connect_device`, `Incompatible Mixer Firmware!`): accept if `fw[0] > '3'`, or if
  `fw[0]=='3'` and `fw[2] >= '1'`. So the **minimum is 3.1**. Only characters [0] and [2] are examined; the
  branch and `-dirty` are ignored. Note that `10.x` would be rejected. The model string must then equal
  `wing-fullsize`, `wing-rack` or `wing-compact`, otherwise the app goes offline. **The simulator must report
  firmware ≥ 3.1**; the 3.0.6 string in the factory scene would be rejected.
- **Value sets** (`nrp::WingHandler::callback_wrapper`): on channel 1 the app sends `D7 <hash BE32>` followed by
  `Cpar::get(binstream)`. Floats are sent as `D5 <f32 BE>`. Ints 0..0x3F are a single byte. Ints that fit int16 are
  `D3 <i16>`, otherwise `D4 <i32>`. Strings use a short form for ≤64 chars and `D1 <len8>` for ≤256 chars (exact
  short-form byte not verified). Params with the n/a flag are skipped. Ids are the local hashes.
- **Data request**: `nrp::requestData` sends `DA DC` on channel 1. `nrp::dumpAllData` serialises the local trees
  with `Cnode::dump(Cbinarystream)` (the PC→MIXER sync).
- **Meters** (`nrp::MeterHandler::sendMeterRequest`, channel 3): `D4 <id BE32> DC` followed by one byte per meter
  group: `0xA0|grp`, `+0x0B` for groups >4, then index bytes. It ends with `DE`. `nrp::setMeterUdpPort` exists. The
  renew interval was not resolved.
- **Channel escape**: `0xDF` is written as the channel-switch prefix in `WingConnector::data_tx`. TCP RX buffer is
  0x5EA bytes; UDP RX buffer is 0xFC8 bytes.
- Not resolved statically (time-boxed): the keepalive interval, the lost-connection timeout, and whether OSC 2223 is
  used. Strings `Lost connection!` / `Auto reconnect is active.` exist (0x10024c7dc).

## 7. Gaps and caveats

- `default` in `tree` comes from the factory scene. Model-slot defaults come from the app's `pardef` (display text,
  not parsed to numbers).
- A `linf` `max` is computed from `min + step*(steps-1)` and is rounded. Use `describe` for the display form. `int`
  min/max are parsed from `describe`.
- Variant probing only covers enum selectors whose block overrides `update_direct`. Dependencies that are not
  expressed this way (for example GUI-only enables) are not captured. Probes run on one instance per distinct
  block; all instances of a template are structurally identical by construction.
- The `$ctl` surface/control tree is included because `CaudioEngine` builds it. `CcontrolEngine` (the GUI fader-layer
  engine) was not run. It is the likely source of the 3 missing GUI preferences.
- Field offsets and type and flag meanings are specific to this build (offsets are listed in `extract.py`). A new
  WING-Edit version needs the symbol and global addresses rechecked.

## 8. Proposed follow-ups
- Cross-join libwing `propmap` (fw 3.0.5) on hash, and record the deltas (needs approval to download libwing).
- T004/T04x: drive `ae::*_respf_*` and `*_gain_*` in the emulator to get reference EQ responses and compressor
  static curves per model.
- T011: write a 1.x→3.x migration map for the PUBLIC.snap legacy keys in §5.
- T012/T013: report firmware ≥ 3.1 in discovery and in the info string.
