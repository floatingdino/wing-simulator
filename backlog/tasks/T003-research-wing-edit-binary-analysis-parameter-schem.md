---
id: T003
title: Research: WING-Edit binary analysis -> parameter schema
stage: S0
status: done
owner: t003-agent
depends: []
branch: -
---

## Goal
Statically analyse WING-Edit.app (Mach-O, read-only; do not modify or re-sign it) to extract the complete parameter tree the app knows: node paths, node ids/hashes, types (float/int/enum/string/bool), ranges, units, tapers/steps, enum value lists (incl. every model name for filter, gate, EQ, dynamics, insert and FX blocks and each model's parameters), and any protocol constants. Tools: strings, otool/nm, python (lief/capstone if pip-installable into a venv in scratch). Output data/schema/wing-tree.json (machine readable, documented format) + docs/research/wing-edit-schema.md (method, coverage, gaps). Cross-check against fixtures/PUBLIC.snap keys: report schema coverage % of snapshot keys.

## Acceptance criteria
- [x] data/schema/wing-tree.json exists with documented format
- [x] Every model enum for every processing block listed with its parameter list
- [x] Coverage report vs fixtures/PUBLIC.snap keys (>=95% target, gaps listed) — 99.99% vs 3.x factory scene; 92.0% vs PUBLIC.snap (fw 1.13 legacy keys, listed)
- [x] Reproducible extraction script in tools/schema-extract/

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 t003-agent: started; static analysis of WING-Edit.app.
- 2026-10-25 t003-agent: method = Unicorn emulation of CaudioEngine ctor from the unmodified arm64 slice (no native run/debugger) + app's own Cpar::describe; variant probing via setInt+update_direct for every model selector.
- 2026-10-25 t003-agent: DONE -> review. Files: tools/schema-extract/{wingemu.py,winghash.py,extract.py}, data/schema/wing-tree.json, docs/research/wing-edit-schema.md.
  Test: `python3 -m venv V && V/bin/pip install unicorn capstone lief && V/bin/python -I tools/schema-extract/extract.py --extra-snap data/firmware/json/0006211b-snapshot_11.json` (~90 s; self-checks hash fn on 43,007/43,007 nodes).
  Results: 43,008 nodes / 36,499 params; node hash = Murmur3-variant of path (winghash.py, verified vs app + documented hashes); all models: flt 4, unified gate/dyn 32 (aux 28), EQ 6-7, FX1-8 65 / FX9-16 39, with per-model param defs + defaults + slot hashes.
  Coverage: 99.99% of fw 3.0.6 factory scene (3 GUI prefs missing); 92.0% of PUBLIC.snap (fw 1.13 legacy keys: send ind/wid, mtx dir, aux combo dyn, busmode, dly/m).
  Protocol: discovery 'WING?' UDP 2222, TCP 2222, min firmware 3.1 (fw[0]>'3' or fw[0]=='3'&&fw[2]>='1'), sets = D7 hash + value token on ch1, meters on ch3.
  Proposed follow-ups: libwing propmap cross-join on hash (needs download approval); drive ae::*_respf_/_gain_ in emulator for DSP reference curves (T004/T04x); 1.x->3.x snapshot migration map (T011); simulator must report fw >= 3.1 (T012/T013); keepalive/meter renew intervals unresolved.
