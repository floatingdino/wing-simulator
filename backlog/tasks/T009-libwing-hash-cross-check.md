---
id: T009
title: Cross-check schema against libwing node definitions
stage: S1
status: done
owner: research-agent
depends: [T003]
branch: -
---

## Goal
Compare data/schema/wing-tree.json (from WING-Edit 3.3.4) with libwing's embedded node-definition records (fw 3.0.5, Apache-2.0): hashes, types, ranges, enums. Report diffs in docs/research/wing-edit-schema.md (append). Do not vendor libwing data into the repo.

## Acceptance criteria
- [x] Diff report: matched/mismatched/missing counts with examples

## Log
- 2026-10-25 coordinator: created from T003 follow-up.
- 2026-10-25 research-agent: cloned https://github.com/dannydulai/libwing (commit 74c2ff6, crate 1.0.5) into the session scratchpad, not the repo. Wrote tools/schema-extract/compare_libwing.py (parses the binary records in src/propmap.rs, cross-checked against propmap.jsonl; joins on path, variant path and id). Appended section 9 "libwing cross-check" to docs/research/wing-edit-schema.md.
  - Results: 60,748 libwing records. Default-path matches 30,610 with ids 100% equal to winghash; variant-path matches 27,252, ids 100% equal (excluding 104 records with no wing param). Definitions identical for 27,140/30,610 default and 25,844/27,252 variant records; the rest is 3.0.5 vs 3.3.4 drift (unified dynamics, dynxo, plink/colour ranges, labels) plus extractor artefacts. wing-tree-only: 4,909 real paths (4,260 are the WING-Edit-only WEDIT layer) + 7,488 unused slots; libwing-only: 2,886 (mostly $ctl layer $spill strips that CcontrolEngine would create).
  - Files changed: tools/schema-extract/compare_libwing.py (new), docs/research/wing-edit-schema.md (section 9 appended), this task file. No commits.
  - Test: `git clone https://github.com/dannydulai/libwing SCRATCH/libwing && python3 -I tools/schema-extract/compare_libwing.py SCRATCH/libwing` (about 5 s).
  - Proposed follow-ups: (1) extractor: run/emulate CcontrolEngine for $spill and record wire flags incl. read-only per param; (2) fix $ctl/$stat/cnslock maxlen (0 vs 19) and unit nibble 7 naming (ft vs octaves); (3) optional local-only CI conformance check against a libwing checkout.
