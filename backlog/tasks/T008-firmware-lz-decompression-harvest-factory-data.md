---
id: T008
title: Firmware LZ region: identify compression, harvest factory scene + node names
stage: S0
status: todo
owner: -
depends: [T005]
branch: -
---

## Goal
docs/research/firmware.md found 0x520000..EOF (~33 MB) is compressed with an unidentified LZ-style scheme containing clear snapshot JSON fragments. Identify the scheme (try LZ4 block/frame, LZO, LZF, LZSS variants, heatshrink, zstd, brotli, custom Cortex-M decompressor visible in the 0x450000 code region) without any key cracking, write a decompressor in tools/firmware-extract/, and harvest: factory/default scene JSON, node/parameter names, model enum lists, preset libraries. Output to data/firmware/ (gitignored if large) + summary appended to docs/research/firmware.md. Use findings to fill gaps in data/schema/wing-tree.json coverage (report diff only; don't edit the schema file).

## Acceptance criteria
- [ ] Scheme identified or documented negative result with what was tried
- [ ] If identified: default scene + name lists extracted and summarised

## Log
- 2026-10-25 coordinator: created from T005 follow-up.
