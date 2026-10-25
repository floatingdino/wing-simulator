# Backlog index

Source of truth for status is each task file's header. Regenerate this index with `python3 tools/backlog.py`.


## S0 — Research & foundation

| ID | Title | Depends |
|---|---|---|
| T001 | Repo scaffold: Rust workspace + build/test harness | - |
| T002 | Research: WING discovery, native binary protocol, OSC, metering | - |
| T003 | Research: WING-Edit binary analysis -> parameter schema | - |
| T004 | Research: WING signal flow + DSP model catalog | - |
| T005 | Research: firmware container analysis (timeboxed 1h) | - |
| T006 | Spike: macOS virtual 48x48 audio device (HAL plugin) | - |
| T007 | Research: reusable DSP libraries + licensing | - |

## S1 — v0.1 WING-Edit controls the simulator (no audio)

| ID | Title | Depends |
|---|---|---|
| T010 | wing-model: generated parameter tree | T001, T003 |
| T011 | Snapshot (.snap) load/save round-trip | T010 |
| T012 | Discovery responder (UDP 2222) | T001, T002 |
| T013 | Native protocol server (TCP 2222) | T010, T012 |
| T014 | OSC server (UDP 2223) | T010 |
| T015 | Meter transport (synthetic) | T013 |
| T016 | Milestone v0.1: WING-Edit control loop test | T011, T013 |
| T017 | State persistence + scene library | T011, T013 |

## S2 — v0.2 Virtual device + audio passthrough

| ID | Title | Depends |
|---|---|---|
| T020 | wing-shm: shared-memory ring protocol | T001, T006 |
| T021 | HAL driver production build | T006, T020 |
| T022 | Engine RT host + parallel scheduler | T001 |
| T023 | Host output bridge with drift compensation | T022 |
| T024 | I/O routing: sources, USB alt inputs, output patching | T010, T022 |
| T025 | Milestone v0.2: audio passthrough | T021, T023, T024 |

## S3 — v0.3 Full mix with standard models

| ID | Title | Depends |
|---|---|---|
| T030 | wing-dsp primitives | T001, T007 |
| T031 | Input stage + filter block (standard) | T030, T004 |
| T032 | Gate/Expander standard model | T030, T004 |
| T033 | EQ standard model | T030, T004 |
| T034 | Dynamics standard compressor | T030, T004 |
| T035 | Channel strip assembly + process order | T031, T032, T033, T034 |
| T036 | Pan, fader, mute, sends, busses, mains, matrices | T035 |
| T037 | Bus/main/matrix strip processing | T036 |
| T038 | DCA, mute groups, solo/monitor | T036 |
| T039 | Real metering | T015, T035 |
| T03A | Control->RT parameter pipeline | T010, T022 |
| T03B | Milestone v0.3: full standard mix | T036, T037, T038, T039, T03A, T025 |

## S4 — v0.4 Alternate processing models

| ID | Title | Depends |
|---|---|---|
| T040 | EPIC: alternate filter models (decompose after T004) | T004, T035 |
| T041 | EPIC: alternate gate/expander models | T004, T035 |
| T042 | EPIC: alternate EQ models | T004, T035 |
| T043 | EPIC: alternate dynamics models | T004, T035 |
| T044 | EPIC: channel insert models | T004, T035 |

## S5 — v0.5 FX rack + all FX models

| ID | Title | Depends |
|---|---|---|
| T050 | FX rack framework (16 slots, insert/send, stereo) | T036, T03A |
| T051 | EPIC: FX models (decompose after T004) | T004, T050 |

## S6 — v0.6 Engineer workflow features

| ID | Title | Depends |
|---|---|---|
| T060 | Offline renderer (wing-render) | T03B |
| T061 | Record/bounce outputs from the daemon | T03B |
| T062 | A/B scenes + save back to console-compatible .snap | T011, T03B |
| T063 | Scene diff report | T011 |
| T064 | Control app (menu bar) for config/status | T023, T025 |
| T065 | DAW sync: scene recall via OSC/MIDI from REAPER | T014, T017 |

## S7 — v1.0 Performance, fidelity, packaging

| ID | Title | Depends |
|---|---|---|
| T070 | Perf suite: full scene 48ch + 16 FX | T03B, T051 |
| T071 | Soak test 8h | T03B |
| T072 | Fidelity test suite | T03B |
| T073 | Packaging + user docs | T064 |
