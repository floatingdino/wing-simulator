---
id: T037
title: Bus/main/matrix strip processing
stage: S3
status: in_progress
owner: engineer-T037
depends: [T036]
branch: task/T037
---

## Goal
Strip processing on busses/mains/matrices (EQ, dyn, inserts, delay) per signal-flow.md.

## Acceptance criteria
- [ ] Tests per strip type

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineer-T037: started (worktree .claude/worktrees/t037). Plan: `mix/strip.rs` MixStrip (StripHook) =
  trim/bal/inv -> Ins1 -> EQ slot (`<bus|main|mtx>/*/eq`, STD bus + alt + PIA) -> DYN slot (+dynsc filter, dynxo)
  -> Ins2; output delay (`dly`) via new `StripHook::process_post` on POST; bound at blocks >= HOOK_BASE.
