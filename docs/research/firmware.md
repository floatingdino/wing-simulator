# Firmware container analysis (T005)

Subject: `wing-fullsize-release-3.1.2-20260729.wingfw` (38,874,112 bytes = 0x2502C00; not committed to git).
Method: pure-Python entropy/signature scans (scripts in the session scratchpad `.../scratchpad/t005/`, run with `python -I`),
`strings`, `xxd`. No key recovery, brute force or decryption was attempted.

## Bottom line

- **DSP source/coefficients are NOT recoverable from this file in any practical way.** The file has no outer header or
  container, the first 2.2 MB is statistically indistinguishable from encrypted/compressed data, and the 24 MB bulk
  is compressed with an unidentified LZ-style scheme. The MCU code visible in clear is host/UI/IO glue (SPI, FAT/USB,
  Dante module, FPGA configuration), not the mixing DSP.
- The real DSP lives on the main processor/FPGA images, which are in the high-entropy or compressed regions. Even if
  decompressed, it would be machine code with no parameter names, and reverse engineering it is outside this project's
  scope. Fidelity must come from parameter semantics (WING-Edit, snapshots), manuals and measurement (consistent with
  docs/ARCHITECTURE.md "Fidelity strategy").
- Useful clear-text metadata exists but is small (below).

## Container structure

No magic at offset 0 (starts `6d 9b 05 40 3b d0 ...`); `file` says "data". No ZIP/gzip/xz/zstd/ELF/squashfs/UBI/FDT/
tar/cpio signatures at any offset (the few 3-byte hits for gzip/bzip2/lzma are at the chance rate and none decode).
No trailing padding or footer. Size is a multiple of 16 (and 1024) but not 4096. The format looks like a raw
concatenated SPI-flash style image of several partitions (erased 0xFF pages and zeroed pages are present).

Entropy map (64 KiB-window classification, refined at 4 KiB; E = >7.9 bits/byte):

| Range (hex) | Size | Entropy | Content |
|---|---|---|---|
| 0x000000-0x233FFF | ~2.2 MB | 7.9999 (flat) | Random-looking. No repeated 16-byte blocks (not ECB), no strings. Encrypted or compressed+encrypted. Likely the main application/boot image. |
| 0x234000-0x44FFFF | ~2.2 MB | low | FPGA bitstream. Text header: `Version: 2023.1.150.5.11`, `Generated: Sun Apr 27 19:36:03 2025`, `Project: /home/thomas/.../mc/fpga_sync/efinity_build_T85`, `Family: Trion`, `Device: T85F484`, `Mode: active`, `PADDED_BITS: 0` (Efinix Efinity toolchain, Trion T85, 484-ball). Bit-stream body is dense but not encrypted. |
| 0x450000-0x49BFFF | ~0.3 MB | mid | Cortex-M Thumb firmware(s) in clear (Thumb `__aeabi_uidiv` at ~0x473400, vector-table-like words at 0x460000). See strings below. |
| 0x49C000-0x4A2000, 0x4A3000-0x4A9000, 0x4AA000-0x51E000 | ~0.5 MB | >7.9 | Small high-entropy blobs (encrypted/compressed; second bitstream or MCU image is plausible, unverified). |
| 0x520000-0xEA0000 | ~9 MB | ~6.6-7.1 | Compressed data, literals partly visible (see below). |
| 0xEB0000-0x2502C00 | ~24 MB | ~7.2 | Same compression style; embedded JSON scene text, WAV/FAT remnants, many 4 KiB 0xFF/0x00 pages (e.g. 0x98F000, 0xC02000, 0xEA5000+). |

## Compression of the bulk (not encryption)

Bulk regions show recognisable literal fragments interleaved with binary control bytes and long `FF FF FF` runs,
e.g. a JSON snapshot (`"type": "snapshot.4"`, `"eq":{"lsf":60.13883591,"1f":129.87...`). That is typical of a
byte-oriented LZ77 (LZ4/LZO/LZF/Snappy-like length-extension bytes), but no standard decoder matched and no stream
header was found within the timebox, so the exact scheme (and chunk framing) is **unidentified**. Python `zlib`,
`bz2`, `lzma` fail everywhere. Nothing suggests this region is encrypted. Follow-up idea (cheap, optional): try raw
LZ4/LZO/FastLZ/Snappy block decoders against candidate chunk starts near 0x520000 and 0xEB0000, aiming only at
clear-text assets (default scenes, strings), not DSP.

## Clear-text metadata found

- FPGA toolchain header above (Efinix Efinity 2023.1, T85F484, built 2025-04-27). Suggests the FPGA does
  time-critical routing/AES/stream glue; the Trion T85 is too small to hold the full WING DSP, so DSP is on
  another processor (not confirmed).
- MCU build tags (git-describe style): `F0.2-28-ga171c14c`, `F1.03-0-g7545dd61`, `040-0-g54398c69`,
  `003-0-g5b0af422`. Product/board strings: `WING-DWSG`, `DANTEWSG`, `WING-LIVE-B`, `WING-P`, `WINGE`
  (Dante module / Live / Producer variants; sub-MCU firmware bundled in this image).
- Debug strings: `DBG_MAIN: configuring FPGA`, `INIT HOST SPI`, `message RX:len=%d`, a FAT volume `NO NAME` with
  `0:SE_LOG.bin`, `0000FFFF.wav`; touchscreen/chip IDs `TM:SSD2543`, `TW:GT9271` (display/touch controllers).
- Embedded default/demo scene (compressed but readable fragments): `"type": "snapshot.4"`,
  `creator_fw":"3.0.6-193-g60c910ab-dirty:dev_unidyn4"`, `"model":"wing-fullsize"`, `"sn":"EMU..."`,
  serial `S123456789AAA`, saved `2025-10-02 14:27:23`, `"active_show"`. Confirms the `.snap` JSON schema (cfg /
  ce_data / `eq` keys `lsf,1f,1q,1g,...,hsf`, `mon`, `rta`) matches what T003/T004 should model, and that a
  firmware-resident factory scene exists. The branch name `dev_unidyn4` hints at a "unidyn" (universal dynamics)
  processing module.
- Fragments of the parameter tree names (`INS/PARA`, `OUT`, `MIX`) are visible in compressed text; a proper decoder
  could yield the complete node-name table, which would be valuable for T003 (but derive it more safely from
  WING-Edit/protocol captures).

## Public information

A web search found only user-level material: firmware is distributed as a zip containing a `.wingfw`, installed by
copying it to the console's USB `WING_OS` mass-storage volume; filenames embed `<model>-release-<ver>-<n>-g<hash>-<date>`.
No public reverse-engineering write-ups, unpack tools or format specs were found. Dante module firmware is
separate and must match the mixer firmware.

## Conclusions for the project

1. Do not plan any firmware lifting; keep DSP clean-room, validated by listening/measurement and
   parameter-exact semantics.
2. Optional low-priority follow-up: identify the LZ scheme to harvest the embedded factory scene and node-name
   strings (clear-text assets only). Do not pursue encrypted regions.
3. Never commit the `.wingfw` (backlog rule 6).

---

# T008 follow-up: the "unidentified LZ" region is plain LZ4 (2026-10-25)

Correction to T005: the file size 38,874,112 bytes is **0x2512C00**, not 0x2502C00 (that earlier hex was a typo).

## Scheme (identified)

- **Raw LZ4 block format** (token | literal-length ext | literals | u16le offset | match-length ext; min match 4),
  *no* frame header, magic, block sizes or checksums. It is one continuous sequence stream that runs from file
  offset **0x51EC69** to **0x2512BBD** (67 high-entropy bytes remain before EOF, probably a signature; left alone).
  Matches reach up to 64 KiB back across logical record boundaries (linked-block style), so the stream must be
  decoded in one pass. Decompressed size ~44.76 MB (ratio ~1.35). Compression only, no encryption involved.
- How it was found: literal text in the JSON fragments is followed by `xx 00` 16-bit values and a token byte, i.e. LZ4
  sequences; a structural parse with offset validation decodes the JSON into byte-exact valid documents.
  Decoding is self-synchronising, so the start was located by scoring candidate token boundaries
  (0x51EC49-0x51EC68 sync to a different class; 0x51EC69 gives the fewest unresolved back-references). The true start
  is not marked; the clear Cortex-M tables immediately before it (zlib/inflate length+distance tables, "0123456789ABCDEF")
  end at ~0x51EC10. The first 64 KiB of decoded output may include synthetic history and is not trusted;
  everything used below sits beyond 0x59000 and validates as JSON.
- The bulk of the 44.7 MB is **UI graphics** (RGB565-looking bitmaps with long runs, ~19 JPEG headers, zero pages),
  not DSP code, not parameter tables, no node/model name tables. Searches for FX/EQ/dynamics model names
  (Hall, Vintage, Reverb, E88, ...) and `/ch/`-style node paths in the decoded data found **nothing**. So the model
  catalogue and node-name table are not here (they would live in the unreadable first 2.2 MB / main app).
- Tool: `tools/firmware-extract/wingfw_lz4.py` (pure stdlib, `python3 -I wingfw_lz4.py FW OUTDIR [--dump]`, ~0.4 s).

## Harvested (data/firmware/, 524 KB total, not gitignored; contains no firmware bytes beyond clear JSON)

Length-prefixed JSON records (u32le length then text) found in the decoded stream:

| out offset | bytes | content |
|---|---|---|
| 0x5951B..0x60F1B (8 records, 0x1200 stride) | 3.3-4.4 KB each | `snapshot.4` DAW-button templates (`ce_data.user.daw1..daw4`: transport/marker/auto-mode button maps, `mode DAWBTN`, `btn T1/OT4/SY1/AU1`...). Likely the built-in DAW-control presets (Mackie/Logic/etc.). |
| 0x6211B | 315,265 | **Factory default scene**, `snapshot.11`, `creator_fw 3.0.6-193-g60c910ab-dirty:dev_unidyn`, `creator_model wing-fullsize`, serial `S123456789AAA`, created 2025-10-02. Valid JSON. |

Files: `data/firmware/json/*.json` (raw records), `data/firmware/default-scene-paths.json` (1,143 path patterns with
numeric indices collapsed to `*`: types, counts, min/max, example default value, string enum values seen),
`data/firmware/path-diff-vs-PUBLIC.json` (path-set diff vs `fixtures/PUBLIC.snap`).

Default scene contents (25,090 leaves): `ae_data` (cfg/mon/solo/rta/mtr/talk/amix, io in/out/altsw, ch x40, aux x8, bus x16, main x4, mtx x8, dca x16, mgrp x8, fx x16,
cards wlive/wmadi, play, rec), `ce_data` (cfg, layer L/C/R/CMPCT/RCK/EXT/VRT, user, gpio, safes, daw, midi, osc, lib),
`ae_globals` (clkrate 48000, clksrc INT, usbacfg 48/48, sccfg AUTO, harmt/custsync a-c), `ce_globals` (15 console prefs).
Defaults only: one model per module (`flt/mdl TILT`, `gate/mdl GATE`, `eq/mdl STD`, `dyn/mdl COMP`, `fx/*/mdl NONE`),
no preset library, no FX parameters.

## Diff vs PUBLIC.snap (snapshot.8, fw 1.13.9, ngc-full); wing-tree.json does not exist yet so PUBLIC.snap is the baseline

Present only in firmware 3.0.6 `snapshot.11` (166 path patterns; numeric indices collapsed): `active_show`,
`active_scene`; `ch/aux/bus/main/mtx: dynxo/q`, `ch/*/flt/lcs|hcs` (low/high-cut slope, string), `ch/*/gate/gain`,
`ch/*/tapwid`; full `dynsc/*` sidechain block and `dyn/mdl|mix|gain|range|ratio|att|hld|rel|acc` on aux; `bus|main|mtx/*/dly/{on,mode,dly}`;
`aux/*/in/set/dly*`; send blocks keyed `MX1..MX8` (sends to matrices) on ch/aux; `mtx/*/dir/{on,lvl,inv,in}`;
`cfg/talk/{A,B}/{indiv,MX1..MX8}`, `cfg/mon/*/dirin`; `io/autoaltovr`; `cards/wlive/{auto_play,auto_rec,auto_stop,sdlink}`,
`cards/wmadi/{mode,rxclock}`; `ce_data`: `cfg/{autosel_*,busspill,cscmode,dcacc,dcaspill,fdrscrnlink,mainspill,mfdr,
mfdrkeep,mtxspill,rackmode,showfdr}`, `layer/{CMPCT,EXT,RCK,VRT}/*` and `spidx`, `midi/{enscenes,enscenetx,enshowctl}`,
`safes/area/{CENTER,COMPACT,EXTERN,LEFT,RACK,RIGHT,VIRTUAL}`, user-button maps `D1..D4,U1..U4,MM`; `ce_globals/{datefmt,filesort,showscene,timefmt}`.

Present only in PUBLIC.snap (195 patterns): per-model **FX parameter keys** (`ae_data/fx/*/` ~150 names such as
`dcy,pdel,damp,xover,mdep,mspd,sw_a..sw_gb,e_*,t_*,l_*,gain_1..5,thr_1..5,byp_1..5,1-att..3-thr`), gate/dyn model
keys (`gate/{mode,drv,hrm,col,trim,filt,f,g,q,depth,fast,peak}`, `dyn/{in,out}`, `*/dyn/{cmode,cpeak,depth,fast,peak,ingain}`,
`flt/{low,proc}`, `bus/*/busmode`), per-send `ind/wid`, and older keys that 3.x renamed/moved (`cfg/talk/indiv`,
`mtx/*/dir/*/...`, `main/*/dyn/{in,time,bias}`, `*/dly/m`, `safes/area/{C,L,R}`, `user/*/*/{bd,bu,enc}/{ch,pg,par,fx}`).
Takeaway: the schema must be driven by a *union* of versions; firmware 3.x renamed several keys (dly block, safes areas,
layer set, user buttons) vs 1.13.9.

## Usefulness / what this does NOT give us

- Good: authoritative **default values and types** for the whole snapshot.11 tree (e.g. EQ band default freqs/Q, filter
  defaults, `peq` 3-band, dyn defaults thr -10/ratio 3/knee 3/att 50/rel 150, gate thr -40/range 40, send modes
  `GRP|POST|PRE`, layer `type` enum `OFF|CH|BUS|DCA`, input `mode M|ST`, `proc GEDI`, `mon A`), plus a newer-version
  key set than PUBLIC.snap. Useful for T010/T011 (default scene on first launch, snapshot round-trip).
- Not available: model enumerations, FX/dynamics/EQ parameter names for non-default models, scene/preset libraries,
  DSP coefficients, native-protocol node IDs. Those must come from WING-Edit / protocol captures / PUBLIC.snap.
- Still no DSP: nothing here changes the T005 conclusion.
