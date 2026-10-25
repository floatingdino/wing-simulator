---
id: T008
title: Firmware LZ region: identify compression, harvest factory scene + node names
stage: S0
status: done
owner: research-agent-T008
depends: [T005]
branch: -
---

## Goal
docs/research/firmware.md found 0x520000..EOF (~33 MB) is compressed with an unidentified LZ-style scheme containing clear snapshot JSON fragments. Identify the scheme (try LZ4 block/frame, LZO, LZF, LZSS variants, heatshrink, zstd, brotli, custom Cortex-M decompressor visible in the 0x450000 code region) without any key cracking, write a decompressor in tools/firmware-extract/, and harvest: factory/default scene JSON, node/parameter names, model enum lists, preset libraries. Output to data/firmware/ (gitignored if large) + summary appended to docs/research/firmware.md. Use findings to fill gaps in data/schema/wing-tree.json coverage (report diff only; don't edit the schema file).

## Acceptance criteria
- [x] Scheme identified or documented negative result with what was tried
- [x] If identified: default scene + name lists extracted and summarised

## Log
- 2026-10-25 coordinator: created from T005 follow-up.
- 2026-10-25 research-agent-T008: DONE -> review. Scheme = raw LZ4 block stream (no framing, 64 KiB linked history), file offset 0x51EC69..0x2512BBD, decoded 44.76 MB. Note T005 had the file size hex wrong (it is 0x2512C00). Decoded payload is mostly UI bitmaps; no model/node-name tables. Harvested the factory default scene (snapshot.11, fw 3.0.6 emu, 315,265 B, valid JSON) + 8 snapshot.4 DAW-button templates.
  Files changed: tools/firmware-extract/wingfw_lz4.py (new), data/firmware/{json/*.json, default-scene-paths.json, path-diff-vs-PUBLIC.json} (524 KB, not gitignored), docs/research/firmware.md (T008 section appended), this task file.
  How to test: `python3 -I tools/firmware-extract/wingfw_lz4.py wing-fullsize-release-3.1.2-20260729.wingfw /tmp/out` -> prints 9 JSON records, all json.loads-valid; `--dump` writes decoded.bin (44 MB, keep out of git).
  Gotchas: stream start is unmarked (first 64 KiB of output not trusted); wing-tree.json does not exist yet so the diff was made against fixtures/PUBLIC.snap (snapshot.8, fw 1.13.9). No encryption touched; 67 trailing high-entropy bytes untouched.
  Proposed follow-ups: (1) when wing-tree.json lands, re-run the path diff against it (scripts inline in log; trivial). (2) Optional: carve the RGB565 UI bitmaps/JPEGs only if the control-app wants authentic look (low priority, copyright risk: do not commit). (3) Model enumerations still need WING-Edit/protocol capture (T002/T003).
