# Backlog protocol (read before working a task)

Every task is one file `backlog/tasks/<ID>-<slug>.md` with a YAML header:

```
id: T0xx
title: ...
stage: S0..S7
status: todo | in_progress | blocked | review | done
owner: <agent name or ->
depends: [T0yy, ...]
branch: <git branch or ->
```
followed by **Goal**, **Context**, **Acceptance criteria** (testable), **Notes/Log** (append-only, dated).

Rules for engineering agents
1. Only work tasks whose `depends` are all `done`. Set `status: in_progress` and `owner` before starting.
2. Atomic + resumable: append to the Log after each meaningful step (what's done, what's next, gotchas) so another
   agent can resume from the file alone. Commit early on the task branch `task/<ID>`.
3. Code tasks work in their own git worktree/branch; research tasks write only to their named output files.
4. Finish with `status: review` + a Log entry listing files changed, how to test, test results. The coordinator
   merges and sets `done`.
5. If you discover new required work, do not expand scope: add a `Proposed follow-ups` list in the Log.
6. Never commit WING-Edit.app, firmware binaries, or large audio. Never run sudo; if something needs root,
   write the exact command into the Log and set `status: blocked`.
7. RT code rules: no alloc/lock/syscall/logging on audio threads; every DSP block has a unit test and a bench.
