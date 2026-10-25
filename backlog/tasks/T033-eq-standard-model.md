---
id: T033
title: EQ standard model
stage: S3
status: done
owner: agent-T033
depends: [T030, T004]
branch: task/T033
---

## Goal
Standard WING channel/bus/main EQ (shelves + parametric bands per models.md), incl. bypass and band types.

## Acceptance criteria
- [x] Response tests vs spec (magnitude/Q) (analytic RBJ/Butterworth/LR/Bessel; WING-Edit curve comparison pending T00A/T03C)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: scope = model STD + 3-band pre-EQ (peq) (models.md §4).
- 2026-10-25 agent-T033: started (fresh worktree from main 6372f6a after API interruption). Plan: crates/wing-blocks/src/eq.rs (StdEq ch/aux/bus + PreEq), tests/eq_no_alloc.rs, benches/eq.rs.
- 2026-10-25 agent-T033: done -> review.
  Files: crates/wing-blocks/src/eq.rs (new: `StdEq` + `EqParams`/`BandParams`/`BandType`/`EqVariant`, `PreEq` +
  `PreEqParams`), crates/wing-blocks/src/lib.rs (`pub mod eq;` only), crates/wing-blocks/Cargo.toml (`[[bench]] eq`),
  crates/wing-blocks/benches/eq.rs, crates/wing-blocks/tests/eq_no_alloc.rs.
  Scope: STD for Channel (L/H PEQ|SHV + 4 peaks), Aux (L/H + CUT), Bus/main/mtx (L/H all 13 types, 6 peaks, tilt);
  wrapper on (default off) + mix 0..125 % (dry/wet blend); channel PRE EQ `peq` (3 peaks + on) as a separate block
  (placement/mix coverage is open question signal-flow §13.2). Defaults/ranges from wing-tree.json (Q min 0.442).
  RT: f64 TDF-II sections; gain/log2 f/log2 Q one-pole 20 ms, redesign every 16 samples with per-sample linear
  coefficient interpolation; type change = 10 ms crossfade to a fresh chain; on/mix 5 ms; 0 dB bands and off EQ
  are skipped (bit-exact pass-through). No alloc after construction.
  Tests (`cargo test -p wing-blocks eq`): designs vs analog prototypes (<0.01 dB), BW/LR/Bessel closed forms + -3.01/
  -6.02 dB landmarks, Q convention (Q 1.414 = 1 oct between half-gain points), processed impulse vs analytic within
  0.1 dB for all variants, every L/H type x mix 50/100/125 %, 20 Hz +15 dB shelf; variant coercion/clamping;
  exact bypass; zipper test (HF residue >8 kHz on 0->+15 dB / 300->1k jump: smoothed 2e-6 vs abrupt 1e-2);
  type-change crossfade; 3000 random fast-automation blocks (all types, odd block sizes) finite/bounded and
  re-converging; zero-alloc integration test. `make check` passes.
  Bench (`cargo bench -p wing-blocks --bench eq`, M-series): channel all bands 64 smp 0.74 us; bus 8-band +LR48
  +tilt 1.47 us; channel under automation 0.96 us; PRE EQ 0.40 us; 48 channel strips 36.4 us (2.7 % of a 64-smp
  block at 48 kHz).
  Reference curves: data/reference-curves/ did not exist at test time -> tested against analytic RBJ/WING formulas
  only; comparison vs WING-Edit curves is pending (T00A -> T03C).
  Assumptions to verify in T03C: PEQ Q = RBJ bandwidth Q; SHV uses band Q as RBJ shelf Q; CUT = 2nd-order resonant
  HP/LP with band Q; Bessel normalised to -3 dB at f; tilt = 1st-order, pivot 1 kHz, +-tilt/2 at the extremes;
  mix = dry/wet blend (not gain scaling); PRE EQ before the EQ model and outside eq.mix.
  Proposed follow-ups: (1) T03C fit Q/shelf/tilt conventions to reference curves; (2) engine wiring of PreEq/StdEq and
  snapshot key mapping (`lg/lf/lq/leq/...`, `peq.*`); (3) monitor-output EQ (`cfg.mon.<n>.eq`, ls/hs + 6 bands) can
  reuse this module.
