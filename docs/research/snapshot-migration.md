# Snapshot (.snap) format and firmware 1.x -> 3.x migration (T011, T019)

Code: `crates/wing-model/src/snapshot.rs` (load/save), `crates/wing-model/src/migrate.rs` (rules),
tests `crates/wing-model/tests/snapshot.rs`, CLI `cargo run -p wing-model --example snap -- load|diff`.

## 1. File format as implemented

- One JSON object. Header fields (`type`, `creator_fw`, `creator_sn`, `creator_model`, `creator_version`,
  `creator_name`, `created`, `active_show`, `active_scene`) then four sections mapped to the tree
  (wing-edit-schema.md §2): `ae_data` -> `/`, `ce_data` -> `$ctl`, `ae_globals` -> `$globals`,
  `ce_globals` -> `$ctl/$globals`.
- Leaves: engineering values as numbers (integral values without a decimal point, e.g. `-144`), enum tokens
  as strings, `CparBool` (and model-slot `CparVar` ints 0..1 such as `auto`, `fast`) as JSON booleans,
  strings as strings. Model-dependent blocks use the active model's parameter names.
- **Which parameters are written**: exactly the parameters the firmware 3.0.6 factory scene persists
  (25,078 tree nodes; there is no reliable flag for this: bit 26 and the `$` prefix both mis-classify
  hundreds of nodes), plus active model slots of those blocks under the current model, plus any other tree
  parameter the loaded file contained. Schema preorder, compact one-line JSON like the console.
- **Floats**: written as the shortest text that round-trips the stored `f32` (console prints ~10 significant
  digits; same value). On load a value already on the parameter's step grid at f32 precision is stored
  bit-exactly; otherwise it is clamped/quantised and listed in `LoadReport::adjusted`.
- **Unknown keys** (not resolvable in the tree) are kept verbatim (raw JSON incl. number text) with their key
  path and re-inserted on save. `$ctl/$globals/{usewheel,timefmt,datefmt}` are real 3.x keys that the
  WING-Edit tree does not model (wing-edit-schema.md §5): kept verbatim and reported as `preserved`, not
  `unmapped`. Empty objects (`ce_data/lib`) are kept too.
- **Load = full recall**: the store is reset to the factory scene first; keys absent from the file keep
  factory values. Selectors (`mdl`, `mode`, `dlymode`, ...) are applied before all other keys (repeated
  while new selectors resolve), so model-dependent names resolve under the file's models.

Round-trip result: factory scene -> store -> JSON is equal key-for-key with every number the identical f32
(test `factory_scene_round_trips_semantically`); header written verbatim.

### Header choices for files we write
- 3.x file loaded: header kept verbatim (including unknown header fields, in order).
- 1.x file migrated: factory-scene layout; `type` = `snapshot.11` and `creator_fw` =
  `3.0.6-193-g60c910ab-dirty:dev_unidyn` (the format/firmware whose key layout we write; a 1.13 `creator_fw`
  next to 3.x keys could make a console try to migrate again); `creator_model` mapped `ngc-full` ->
  `wing-fullsize` (`ngc-rack`/`ngc-compact` -> `wing-rack`/`wing-compact`, **inferred**); `creator_sn`,
  `creator_name`, `created`, `creator_version` kept from the source; `active_show`/`active_scene` `""` when
  absent. New scenes (`Snapshot::factory()`) use the factory header.
- **Unverified**: whether a real WING 3.x / WING-Edit checks `creator_fw` or `creator_model` on scene load.
  Writing the factory header values is the safest choice we have; verify on hardware (T062).

## 2. Legacy detection
`creator_fw` major < 3 (or, without `creator_fw`, `type` older than `snapshot.11`). Fixture
`fixtures/PUBLIC.snap`: `snapshot.8`, fw 1.13.9, model `ngc-full`.

## 3. Migration rules (rule id = `MigrationNote::rule`)

Semantics derived from wing-tree.json (3.3.4 descriptors), models.md §7 alias table, signal-flow.md and the
fixture's own values. PUBLIC.snap result: **0 unmapped, 0 rejected**, 1,755 notes, 8 lossy. The 1,709 keys
absent from the 3.x tree (wing-edit-schema.md §5) are all accounted for: send ind/wid 1,536 + mtx dir 80 + aux
combo dyn 56 (resolve after `aux-dyn-mdl`) + busmode 16 + dly `m` 12 + talk indiv 1 + safes L/C/R 3 + fx `40` 4 +
`usewheel` 1. The other notes are value conversions of keys that exist in both versions.

| rule | 1.x key(s) | 3.x | rationale | lossy when | PUBLIC.snap |
|---|---|---|---|---|---|
| `send-ind` | `ch\|aux/*/send/*/ind` | (dropped) | "independent": the send uses its own `mode` instead of its bus's `busmode`. Evidence: the 4 `ind=true` sends (ch 21/22 -> bus 12/16) are exactly the 4 sends whose `mode` (POST) differs from the bus's `busmode` (PRE); all 764 others match. 3.x stores the effective mode per send. | never | 768 |
| `send-wid` | `ch\|aux/*/send/*/wid` | (dropped) | per-send stereo width has no 3.x key (`ch/*/tapwid` is per channel, not per send) | `wid` != 100 | 768 (2 lossy: ch 4 sends 10/15 at 70 %) |
| `busmode` | `bus/*/busmode` | `ch\|aux/*/send/<bus>/mode` | busmode was the send mode of every non-independent send into that bus; a non-`ind` send whose `mode` disagrees is set to the busmode | never | 16 (no send needed rewriting) |
| `warm-mix` | `ch/*/gate/mix` when `gate/mdl` = WARM | `ch/*/gate/wmix` | models.md §7: WARM's own MIX was `mix` in 1.x, `wmix` in 3.x; 3.x `gate/mix` is the new block-level mix (stays 100 %). Without this rule ch 20-24 (5/30 %) would load as a 5 % block mix. | never | 8 |
| `out-dly-m` | `bus\|main\|mtx/*/dly/m` | `dly/mode` = `M` + `dly/dly` | 3.x output delay = mode (M/FT/MS/SMP) + value; `m` was metres; M mode range 0.1..150 m covers 1.x | never | 12 |
| `dynxo-type` | `*/dynxo/type` `LO6 LO12 HI6 HI12 PC` | `LO`, `HI`, `PEAK` | 1.x token = band + slope; 3.x enum `OFF LO HI S6 S12 PEAK` (long `FLAT LOW HIGH 6 dB/Oct 12 dB/Oct PEAK`) keeps the band. Where the slope lives in 3.x is open (`S6`/`S12` look like slopes but are values of the same enum) | token carried a slope (all non-OFF/PC) | 6 lossy (ch 9 LO12, ch 27 LO6, ch 39/bus 6/bus 7/main 1 HI6) |
| `solo-mon` | `cfg/solo/mon` `A`/`B`/`A+B` | `PH`/`SPK`/`PH+SPK` | monitor outputs renamed (A = phones, B = speakers, models.md §7, inferred) | never | 1 |
| `aux-dyn-mdl` | aux `dyn/*` without `mdl` | add `aux/*/dyn/mdl` = `CMB` | 1.x aux dynamics was the fixed PSE/LA combo `{thr, depth, fast, peak, ingain, cpeak, cmode}` = 3.x CMB params 1:1 (same names and ranges) | never | 8 (56 keys then resolve) |
| `flt-slope` | `*/flt/lc`, `hc` without `lcs`/`hcs` | add `lcs` = `12`, `hcs` = `12` | 1.x had fixed slopes; models.md: load as 12 dB/oct (X32 lineage). Factory default would be 24 for LC. **Verify** on a 1.x console. | never (assumed) | 80 |
| `mtx-dir` | `mtx/*/dir/{1,2}/{on,lvl,inv,in,tap}` | `mtx/*/dir/{on,lvl,inv,in}` | 3.x has one direct input without a tap selector; input 1 maps over | `dir/2/*` or `tap` not at the 1.x default (off, -oo, PRE) | 80 (all default) |
| `talk-indiv` | `cfg/talk/indiv` | `cfg/talk/A/indiv`, `cfg/talk/B/indiv` | the flag moved per talkback channel | never | 2 |
| `talk-mondim` | `cfg/talk/{A,B}/mondim` (bool) | number 40..0 dB | 3.x dim amount; `false` = 0 dB; `true` = the 1.x monitor 1 `dim` (or 20) | `true` (amount inferred) | 2 (false) |
| `safes-area` | `$ctl/safes/area/{L,C,R}` | `LEFT`/`CENTER`/`RIGHT` | renamed; 1.x value `"0"` = no safes -> 3.x default (blank per-strip string). Other 1.x values are an unknown format | value not `"0"`/`""` | 3 |
| `seldblclick` | `$ctl/cfg/seldblclick` (bool) | `OFF`/`HOME`/`BUSFX` | false -> OFF; true -> HOME (**inferred**) | `true` | 1 |
| `mdl-rename` | `gate\|dyn/mdl` = `D241` | `D241G` (gate) / `D241C` (dyn) | the 3.0 D241 model was split (wing-edit-schema.md §9.3) | never | 0 |

Handled without a rule:
- `fx/{11,13,14,16}/40` (models `*MASTER*`, `GEQ`): resolves to the model's `40` parameter in the 3.x schema
  (models.md fills it); not a dumped unused slot after all.
- Type-widened values: send `plink` bool -> int 0..2 (false/true = 0/1), `ce_globals/mousespd` int.
- `$ctl/$globals/usewheel`: preserved verbatim (unmodelled, see §1). Migrated files also get the factory's
  `timefmt`/`datefmt`.
- 3.x-only keys (`flt/lcs|hcs` aside): `gate/{mix,gain}`, `dynxo/q`, `aux/*/dynsc`, `aux/*/in/set/dly*`,
  `bus/*/dly`, send `MX1..MX8`, `cfg/talk/*/MX*`, `cfg/mon/*/dirin`, layers/safes areas etc. keep factory
  defaults (load resets to the factory scene).

## 4. Value adjustments (not key changes)
`LoadReport::adjusted` lists numeric values moved by clamping/quantisation. PUBLIC.snap: 142, all explained by
3.x law changes (wing-edit-schema.md §9.3): `gate|dyn/rel` (130 -> 121 steps, e.g. 247.03 -> 252.38 ms, <= 2.2 %),
`gate/hld` (linear -> log grid, 54 -> 53.09 ms), `ch/*/in/set/dly` 0 m -> 0.1 m (3.x METERS minimum), and 2 EQ
gains with f32 noise. Hardware 3.x would also snap these to its grid; the deviations are below audibility for
time constants but are reported.

## 5. Open items
1. dynxo slope (`LO12`/`HI6`) in 3.x: is `S6`/`S12` a slope for the LO/HI shapes? Needs a 3.x console or
   WING-Edit `respf` run (models.md §8 item 7).
2. 1.x LC/HC slopes (assumed 12/12).
3. `seldblclick` true and `safes/area` non-zero value formats.
4. Header acceptance (`creator_fw`/`creator_model`) on real hardware / WING-Edit file import.
