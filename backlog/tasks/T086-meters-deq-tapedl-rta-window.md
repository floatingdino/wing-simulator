---
id: T086
title: BUG: DEQ and TAPE-DL meters static; "RTA in use on another window!"; meter report churn
stage: S4
status: done
owner: agent-T086
depends: [T082, T084]
branch: task/T086
---

## Goal
User (v0.5 build): 76LA GR display works, but dynamic-EQ (DEQ) meters don't move and TAPE-DL VU meters don't move. RTA overlay now says "RTA in use on another window!". Evidence in scratch/wing-sim-v0.5.log: WING-Edit re-defines meter reports continuously (`report 809..820 defined` within ~150 ms, alternating `1 meters: ChannelV2 1` and `22 meters: ChannelV2 1-16, MainV2 1, Fx 7, Source 1,4-5,8, Rta`), and `cfg/rta/rtasrc` sets. Find out (WING-Edit code via strings/symbols/Unicorn emulation only — never run it natively; protocol docs; libwing) (a) why WING-Edit keeps re-defining reports (missing ack/reply? report id handling? renew semantics?) and fix the server; (b) the exact condition for "RTA in use on another window!" (likely RTA ownership tied to a report id / client / window id, or multiple reports containing Rta) and satisfy it; (c) the DEQ meter words (band gain/activity) and TAPE-DL's VU/state words WING-Edit reads for those pages, and populate them from the models. Tests for each.

## Acceptance criteria
- [x] No report-definition churn in a replay of the log's request sequence
- [x] RTA condition documented and satisfied (test)
- [x] DEQ and TAPE-DL meter words populated (tests)

## Log
- 2026-10-26 coordinator: created from user v0.5 test.
- 2026-10-26 agent-T086: done, status review. WING-Edit 3.3.4 arm64, static analysis only (capstone/lief on the
  slice; never run). Details: docs/research/protocol.md §4.3, docs/research/rta.md §1a.
  - **(a) Churn, root cause.** `nrp::MeterHandler::rxMeters` (0x1000d72d4, message thread) treats *every*
    received packet as a sync point: it posts the async meter notification, then compares the wanted masks
    (`+0x38`, OR-ed in by visible components on that notification) with the last request (`+0x90`); if different
    it sends `D4 id+1 DC .. DE` and resets wanted to `ChannelV2 1`; if equal and > 4000 ms it renews. It never
    renews old ids. The server kept every id alive 5 s after its definition, so each 50 ms tick carried several
    packets; the second was compared before the components had re-registered -> `ChannelV2 1` definition, then
    the full set, ... (log: 1847 definitions in ~25 min, "68 live meter reports" at 660 s, 3-8 ms define pairs).
    Fix: one report per client, a definition supersedes the client's earlier ids
    (`wing_proto::server::meters::MeterSubscriptions::apply`).
  - **(a2) Found on the way:** `sendMeterRequest` writes `A9` (monitor) and `AA` (RTA) **without an index**; the
    server produced 0 words for them, so WING-Edit never got RTA data (T084's test used an explicit index 0).
    `MeterRequestParser` now turns an index-less monitor/RTA selection into index 0 (`MeterKind::single_instance`).
  - **(b) RTA condition** (`RtaGraph::paint` 0x1000e3a04): T084's check, then spectrum iff graph's own
    (src `+0x1d8`, tap `+0x1dc`) == (`rtasrc`, `$tap`); else FX graph with selected strip != `fx/N/$a_chn - 1`
    -> "Select INSERTED channel", else "RTA in use on another window!". No report/client/window id involved.
    `rtasrc`/`rtatap` are written only by WING-Edit graphs (strip graphs on selection / display change while
    `eqdisp` is on; insert-view EQ with a GEQ-type insert; FX pages via `setFxRta` only while the selected strip
    is `$a_chn - 1`). The log only shows FX-insert claims (`rtasrc` 12, 14), so strip RTAs afterwards read
    "another window" until they claim again: WING-Edit behaviour. Console side now satisfied: `$` mirror (T084),
    RTA data (a2), and `fx/N/$a_chn`/`$a_pos` + insert `$stat` (WING-Edit computes them only offline:
    `ae::Cinsert::update` returns when connected, 0x10001307c) via `apps/wing-sim/src/rta.rs` `sync_fx_hosts`
    (engine ownership rule, every 50 ms on the RTA thread).
  - **(c) DEQ / TAPE-DL words.** DEQ (gate or dyn slot): `rxMeters` stores gain word / 256, `DynEqGraph::updateCurve`
    draws the band at 15x that dB -> word = 256 dB / 15 (already so since T082); `ae::dyn_gain_dyneq` is the same
    law (`clamp(20 log10(1+g)/15, ±1)`, the T082 doc misread it) and `dyn_led_dyneq`/`dyn_state_dyneq` return 0
    -> LED/state word now 0 (was the GR>0.5 dB flag). DEQ3: words 4/6/8 = `2048 (g_k - 1)` (unchanged). TAPE-DL:
    `FxTapeDelay::listener_callback` reads FX words 2/3 (out L/R, 1/256 dB), needle `668 + 551 (dB+6)/6` above
    -6 dBFS, `/17` below, clamped (pins below about -23 dBFS); no state words. Both were populated; the static
    meters are explained by the churn (values are applied only for the current id) and by the scene: ch 26
    (the only DEQ) has `altsrc` on with alt input OFF (silent), FX7 TAPE-DL on bus 2 is fed only by ch 21.
  - **Files:** crates/wing-proto/src/server/meters.rs, crates/wing-proto/src/nrp/meter.rs,
    crates/wing-proto/tests/meter_churn_replay.rs (+ fixtures/t086-meter-defines.txt, the log's definitions),
    apps/wing-sim/src/rta.rs, apps/wing-sim/tests/meters.rs, crates/wing-blocks/src/meter.rs,
    docs/research/protocol.md §4.3, docs/research/rta.md §1a.
  - **Tests:** `meter_churn_replay` (model of MeterHandler from the disassembly vs the server, 100 page sets from
    the log: 96 definitions for 99 page changes, max 1 live report, no `ChannelV2 1` definition; the same model
    against the pre-T086 server reproduces the storm: 557 definitions for 2 page changes); wing-proto unit tests
    `definition_supersedes_the_clients_earlier_reports`, `monitor_and_rta_blocks`,
    `monitor_and_rta_without_index_select_their_block`; wing-sim `rta::tests::wing_edit_rta_overlay_condition`
    (paint + claim model), `fx_host_status`; `tests/meters.rs` `deq_band_gain_words` (gate + dyn slot, console end
    to end), `deq3_and_tape_delay_fx_words`. `make check`: pass. Not run: `make check-full`, live WING-Edit check.
  - **Proposed follow-ups:** live check with WING-Edit (meter log line should now show one definition per page
    change); confirm `$a_pos` 0 = Ins1 [M] and the `$a_chn` numbering on hardware; log `cfg/rta` claims per
    client to diagnose remaining "another window" reports.
