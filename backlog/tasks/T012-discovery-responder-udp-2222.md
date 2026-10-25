---
id: T012
title: Discovery responder (UDP 2222)
stage: S1
status: done
owner: agent-T012-T018
depends: [T001, T002]
branch: task/T012-T018
---

## Goal
Answer WING-Edit discovery so the simulator appears in WING-Edit's console list with configurable name/serial/model/fw. Support binding to a chosen interface/IP (and loopback alias if WING-Edit needs a non-127 address — document).

## Acceptance criteria
- [x] WING-Edit lists 'Wing Simulator' (manual/screenshot check logged)
- [x] Unit test for packet format

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent: done, status review. Files: crates/wing-proto/src/discovery.rs, tests/discovery_loopback.rs,
  examples/discovery.rs; workspace Cargo.toml gains `socket2 = { version = "0.6", features = ["all"] }` (SO_REUSEPORT
  before bind).
  - Networking choice for wing-proto: **std blocking sockets + socket2, a thread per socket/client** (not tokio).
    Reasons: at most 24 TCP clients plus a few UDP sockets, so threads are cheap; no async runtime next to the RT
    engine; one small dependency instead of tokio's tree; simple deterministic tests. T013/T014/T015 should follow it.
  - `Responder::bind(ResponderConfig{bind (default 0.0.0.0:2222), identity, advertise_ip})` with SO_REUSEADDR,
    SO_REUSEPORT and SO_BROADCAST; `spawn(on_reply)` returns a `ResponderHandle` (stops on drop). Answers exactly
    `WING?` (tolerates trailing NUL/CR/LF) to the probe's source ip:port from the bound port, no trailing NUL.
  - IP field: `advertise_ip` override, else a specific bind IP, else the local IPv4 the OS routes toward the prober
    (connected UDP socket's local_addr; equals the arrival interface on a normal network; no cmsg/libc needed).
    Binding a specific unicast IP on macOS stops broadcast reception, so use 0.0.0.0 plus `advertise_ip` for aliases.
  - Identity defaults: name WINGSIM, model ngc-full, serial WSIM00000001, fw `3.3.4-0-g0c2b9d4a:release` (per
    coordinator: WING-Edit gates CONNECT on fw major.minor >= 3.1 only). All configurable; fields validated (printable
    ASCII, no commas). Run: `cargo run -p wing-proto --example discovery -- --seconds 10 [--name --model --serial --fw
    --ip --bind]`.
  - Live check (2026-10-25 ~16:00, WING-Edit 3.3.4 open, no UI interaction): the example ran 8 s, received WING-Edit's
    probes from 192.168.20.130:61447 every 2 s (2 per burst) and replied `WING,192.168.20.130,WINGSIM,ngc-full,...`.
    A passive screenshot showed SELECT CONSOLE listing `WING  WINGSIM  192.168.20.130  3.1.2-0-g...` (that run used
    the earlier 3.1.2 default). Caveat: the coordinator's fake_wing.py may also have been answering as WINGSIM on 2222
    then, so the entry is not attributable to this responder alone. The process exited by itself.
  Tests: unit (reply format, libwing example parse, probe detection, identity validation, override, loopback route);
  loopback integration with ephemeral ports (reply ip 127.0.0.1, reply from the responder port, non-probe ignored,
  configured identity and IP override, two binds of one port co-exist, invalid identity rejected).
  How to test: `make check` (passes).
