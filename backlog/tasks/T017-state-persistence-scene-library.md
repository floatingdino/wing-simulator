---
id: T017
title: State persistence + scene library
stage: S1
status: done
owner: engineering-agent
depends: [T011, T013]
branch: task/T017
---

## Goal
Autosave current state, load/save named scenes, support WING-Edit library/show operations that a console supports (store/recall/scenes) as far as protocol allows.

## Acceptance criteria
- [x] Restart restores state (autosave `state.snap`, restored on start unless `--scene`/`--recall`)
- [x] Scenes can be saved/recalled from WING-Edit if supported (`$ctl/lib` served from the library;
  recall via `$action` GO/GONEXT/GOPREV; store via the simulator extension `STORE:name`. Not yet
  exercised with the WING-Edit library page, see follow-ups)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineering-agent: done, ready for review.
  - **Persistence** (`apps/wing-sim/src/persist.rs`): state dir (default
    `~/Library/Application Support/WingSimulator`, `--state-dir`), `state.snap` autosaved by a
    background thread 2 s after the last change and on exit; atomic write (temp + fsync + rename
    + dir fsync). The store hook only bumps an atomic counter, so no IO touches the control or RT
    path. On start: `--scene` > `--recall` > `state.snap` > factory. An unreadable `state.snap` is
    moved to `state.snap.bad`. `--no-autosave` turns it off. `--save-on-exit` now writes atomically too.
  - **Scene library** (`apps/wing-sim/src/library.rs`): `<state dir>/scenes/NNN Name.snap` (NNN =
    show order; hand-copied unnumbered `.snap` files are listed after them). One show, `SIMULATOR`.
    `wing_proto::server::library::SceneLibrary` trait + `LiveInfo::library`: with a library
    configured, the native session routes `$ctl/lib/$action` sets to it (string or enum index),
    reads `$actionidx` as the argument, and drops client sets of the other console-owned
    `$ctl/lib/$*` leaves (the WING-Edit push includes `$actshow`). `$scenes` defs come from the
    library. Actions run on a worker thread (file IO off the client threads), and recall goes
    through `Snapshot::load` on the shared store, so all clients get change events. Afterwards
    `$active`/`$activeid`/`$actshow` are published and `$syscfg/$cnsmdl` is re-applied (a load
    resets it). Semantics, inferred from the schema (`$action` enum IDLE GOPREV GONEXT GO PREV NEXT
    GOTAG; `$actionidx` 0..16384; `$activeid` 0..128) and WING-Edit strings ("PLEASE OPEN SHOWFILE
    ON CONNECTED MIXER", "requestNode: CCE->lib->scenes"), all **[H], unverified**: GO recalls
    `$actionidx` (0-based); GONEXT/GOPREV recall relative to the active scene; NEXT/PREV move
    `$actionidx`; GOTAG is unsupported. Store/rename/delete have no schema node, so they are
    **simulator extensions** as `$action` strings: `STORE[:name]` (no name: update the active scene),
    `RENAME:name`, `DELETE` (moves the file to `scenes/.deleted/`). The scene header
    `active_show`/`active_scene` is written, and the active scene survives restarts through `state.snap`.
  - **CLI**: `--list-scenes` (prints and exits before binding any socket), `--recall NAME|#N`,
    `--state-dir DIR`, `--no-autosave`.
  - Files: `apps/wing-sim/src/{main,persist,library}.rs`, `crates/wing-proto/src/server/{library,defs,session,mod}.rs`.
  - Tests (`make check` passes): `persist::tests::{atomic_write_replaces_and_leaves_no_temp,
    autosave_debounces_and_restores}`, `library::tests::{store_recall_rename_delete,
    listing_order_and_lookup, native_store_and_recall_loopback}` (TCP loopback on an ephemeral
    port: STORE x2, GO, GONEXT by enum index, `$scenes` DD, change events), and
    `wing-proto session::tests::library_actions_are_routed` plus `library::tests::ids_and_action_text`.
    Manual: `wing-sim --state-dir DIR --list-scenes`. I did not start the daemon, because port 2222 is in use by the live instance.
  - Proposed follow-ups:
    - Verify against WING-Edit's library page: what it sends on store/recall/next, whether it
      re-requests `$scenes` after changes, and whether `$activeid`/`$actionidx` are 0- or 1-based.
      A capture with a real console would settle this.
    - Route `$ctl/lib/$action` from OSC too (today OSC sets the enum in the store, which has no effect).
    - Notify clients when the `$scenes` list changes (mechanism unknown).
    - Multiple shows (`.show` files, `$actshow` switching), scene tags for GOTAG, and safes/scope
      on recall.
