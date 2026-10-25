---
id: T00A
title: Extract per-model response/gain-computer curves from WING-Edit
stage: S0
status: done
owner: t00a-agent
depends: [T003]
branch: -
---

## Goal
T003 found WING-Edit contains per-model response-curve and gain-computer functions (used for its EQ/dyn graphs). Using the same Unicorn emulation approach (tools/schema-extract/wingemu.py; never run the app natively or modify it), call these functions over parameter sweeps and export reference data: EQ magnitude curves per model/band type, dynamics/gate static gain curves per model, filter curves. Output data/reference-curves/*.json (+ generator script in tools/curve-extract/) and docs/research/reference-curves.md. These become fidelity test oracles for S3/S4.

## Acceptance criteria
- [x] Curves for every EQ/filter/gate/dyn model at representative parameter grids (all models that have a curve function in WING-Edit; 28 vintage dyn models have stub respf -> GR-meter mapping only)
- [x] Documented function list and calling convention

## Log
- 2026-10-25 coordinator: created from T003 surprise finding.
- 2026-10-25 t00a-agent: started. Found GUI curve code by symbol: ae::Cchfilter::getResponseCached, ae::Cequalizer::getResponseCached + per-model ae::eq_respf_* (f_table_ch/aux/bus), ae::*_respf_* for dyn (f_table, 0x70 stride, via Cdynamics::ftblofs), Cdynsidechain::getResponse, Ccompxover::getResponse. Coefficients come from Cblock::update(mask) (not update_direct). wingemu lacks tan/tanf/frexp -> patched in tools/curve-extract/curvelib.py. Next: tools/curve-extract/curves.py generator (STD EQ, filter, GATE/DUCK/COMP/EXP first).
- 2026-10-25 t00a-agent: DONE -> review. Files: tools/curve-extract/{curvelib.py,curves.py}, data/reference-curves/*.json (14 files, 13 MB), docs/research/reference-curves.md.
  Coverage: filter (LC 6-24, HC 6/12, TILT/MAX/AP1/AP2), STD EQ ch/aux/bus(=main/mtx) incl. all 13 band types, SOUL/E88/E84/F110/PULSAR/MACH4/PIA, GATE/DUCK/COMP/EXP static curves (+exact formulas), dyn sidechain filter + dyn crossover, GR-meter mapping for all 32 dyn models. Each curve carries traced biquads [b0,b1,b2,a1,a2] + exact mag/phase; all verified vs app respf (<1e-5 dB with float32-faithful re-eval). MACH4 is a parallel bank.
  Test: `VENV/bin/python -I tools/curve-extract/curves.py --app WING-Edit.app --out data/reference-curves` (~45 s, byte-identical rerun verified).
  Gotchas: coefficients come from block update(mask) not update_direct; wingemu lacked tan/tanf/frexp (patched in curvelib); _AEDRV+0xc (sample period) is 0 in WING-Edit so posted time constants are degenerate (formulas documented from disassembly).
  Proposed follow-ups: drive DynEqGraph (DEQ/DEQ2/DEQ3/SPKMAN) curves; hook CdummyAudioEngine post with T=1/fs to capture all 32 dyn models' time-constant blocks + FX coefficients; repeat at 44.1 kHz; FX EQs via f_table_all unverified.
