# A/B compare and export to a real console

## A/B compare
Two in-memory scene slots in the daemon. Over OSC (UDP 2223) send:

| address | effect |
|---|---|
| `/wingsim/ab/store_a`, `/wingsim/ab/store_b` | capture the current state into the slot and make it the active side |
| `/wingsim/ab/a`, `/wingsim/ab/b` | switch to that side |
| `/wingsim/ab/toggle` | switch to the other side (A when none is active) |

Leaving a side captures its current state first, so edits made while on A are still there when you return;
selecting the side you are already on reverts it to its stored state. A switch sets only the parameters that
differ, through the normal parameter path: the audio engine gets them like any other change (smoothed, no clicks)
and WING-Edit / OSC subscribers see them through the usual fan-out. Also available as `$ctl/lib/$action` =
`AB:store_a` ... The CLI can preload slots: `wing-sim --ab-a a.snap --ab-b b.snap` (starts on A).

## Export
`/wingsim/export ,s /path/mix.snap` (or `wing-sim --scene in.snap --export /path/mix.snap`, which exports and
exits; `--export-on-exit PATH` exports at shutdown) writes a firmware 3.x `.snap` for loading on a console, plus
`/path/mix.changes.md`: a wing-diff report of the changes since the scene was loaded. The file is re-loaded and
checked before writing (0 rejected keys, 3.x `snapshot.11`, content round-trips). Header: see
docs/research/snapshot-migration.md; `--export-header factory` writes the 3.0.6 factory header instead of the
advertised firmware. Loading on real hardware / WING-Edit offline is still unverified.
