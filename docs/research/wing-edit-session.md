# WING-Edit 3.3.4 connect session: decoded (T00B)

Status: T00B output, 2026-10-25. Feeds T013 (native TCP server), T015 (meters) and T016 (WING-Edit control loop).

Sources:
- **Capture A** (`tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz`): user-initiated
  connects of WING-Edit 3.3.4 to `fake_wing.py --tcp-mode end`, fw string `3.1.2-0-g00000000:release`. It holds three
  TCP connections. The log was gzipped while the fake was still running. Connection 3 has no close, and its TCP
  traffic stops at 63.7 s, although discovery lines continue to 151.8 s.
- **Capture B** (`fixtures/protocol/wing-edit-connect-tokens-mode.txt.gz`): a passive re-run with a token-aware fake
  (`tokens` mode, described below), fw `3.3.4-0-g0c2b9d4a:release`. Nobody clicked anything: WING-Edit was still
  auto-reconnecting from capture A and connected 0.9 s after the fake started. The connection stayed up for 114 s,
  until the fake was stopped.
- Decoder: `cargo run -p wing-proto --example decode_capture` (`crates/wing-proto/examples/decode_capture.rs`). It
  reassembles each connection's byte stream from the hexdumps and decodes it with `wing_proto::nrp::NrpDecoder` and
  `MeterRequestParser`. It maps hashes through `data/schema/wing-tree.json` `hashes`.
- Commands:
  ```
  gzip -dc tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz | \
    cargo run -p wing-proto --example decode_capture -- [--tokens] [--out-dir fixtures/protocol] \
      [--scene-conn 1 --scene-out fixtures/protocol/pc-to-mixer-scene.json --source <capture>]
  gzip -dc fixtures/protocol/wing-edit-connect-tokens-mode.txt.gz | cargo run -p wing-proto --example decode_capture -- \
      --out-dir fixtures/protocol --prefix wing-edit-connect-tokens --server-bins
  ```

Tags: **[A]** seen in capture A, **[B]** seen in capture B, **[H]** hypothesis, not verified.

---

## 1. Sequence (what WING-Edit sends, what a server must reply)

```
WING-Edit (client)                                           console / simulator
  | UDP  WING?  every 2 s, also while connected                 |
  |------------------------------------------------------------>|  reply info string (fw >= 3.1)
  | TCP connect :2222 (no banner either way)                     |
  | ch3: DF D3  D3 <udpPort:u16>  DE                             |  (1) store meter port; no reply
  | ch3: D4 00000000 DC  AB 00..0F  AE 00  A7 00 03 04 07  DE    |  (2) define report 0; no TCP reply;
  |                                                              |      start UDP meter data
  |   ...0.05 s [B] / 2.4-3.1 s [A conn 1,2] / 0.05 s [A conn 3] |
  | ch1: DF D1  D7 <h> <value>  x ~28.7k-30.2k (whole scene)     |  (3) apply sets; no reply
  |      tree order, sent as fast as TCP allows (~0.5 s, 229 kB) |
  | ch1: D7 <h> DC     x 25, one at a time                       |  (4) MUST reply D7 <h> <value> DE to
  |      (next one sent only after the previous reply)           |      each one
  | ch1: D7 e96ea488 DE DD   (play/$songs)                       |  (5) reply: definition record(s) + DE
  | ch1: D7 ebd73de7 DE DD   ($ctl/lib/$scenes)                  |      (DE alone was accepted)
  | ch1: D7 985729a2 8B "ModelRequest"  ($syscfg/$cnsmdl)        |  (6) [H] reply D7 985729a2 <model str>
  | ch3: DF D3 D4 00000001 DC AB 00..0F AE 00 A7 00 03 04 07 DE  |  (7) define report 1 (same as 0)
  | ch3: D4 00000002 DC AB 00 DE                                 |      define report 2 (ch 1 V2)
  | ch3: D4 00000003 DC AB 00..0F AE 00 A7 00 03 04 07 DE        |      define report 3 (same as 0)
  | ch3: D4 00000003   every 4.04 s (3.99-4.08)                  |  (8) renew report 3 (5 s lifetime)
  |   ... no ch1 traffic while idle; the ch3 renew is the only keep-alive ...
```

Byte-level detail:
- The first segment is `df d3 d3 f0 07 de` [A] or `df d3 d3 eb df de de` [B]. That is ch3 `D3 <port>` followed by a
  `DE`. In [B] the port `0xEBDF` contains `DF`, and the client escapes it as `DF DE` (Behringer tx routine). The port
  is the UDP socket WING-Edit also uses for its `WING?` probes: 61447 in [A], and 60383 in [B], where later probes came
  from 60383.
- The client tx channel stays selected across segments. The 31-byte meter definition carries no `DF D3` because ch3
  is already current. The renews are bare `d4 00 00 00 03` (5 bytes).
- Every get and every `DD` request goes in its own TCP segment. Gets are sent about 2 ms apart, the reply latency
  of the fake. That looks like strict request/response.
- Meter type tokens: `AB` = channel V2, `AE` = main V2, `A7` = source groups. Indices: ch 1-16, main 1, and source
  groups 0, 3, 4, 7. In the protocol.md §4.2 group order those are local, AES50 B, AES50 C and card (zero-based).
  Report ids are small sequential integers (0, 1, 2, 3), not libwing's `(seq<<16)|port`.

### 1.1 The 25 data requests (`D7 h DC`), in order (identical in every connection)
`play/$pos $total $actlist $actidx $format $channels $resolution $rate $song $actfile $actstate`,
`rec/$time $actstate $actfile`, `$syscfg/$usbspd_act`,
`$stat/A/stat A/dev B/stat B/dev C/stat C/dev sc_stat sc_devices`, `cards/$type`, `$syscfg/opt_mod`.
All of these are read-only/live (`$`) leaves that are not part of the push. Capture B answered each with
`D7 <h> <schema-typed value> DE`: strings and enums as a string token, ints as small int, floats as `D5`.

### 1.2 Server replies needed to keep WING-Edit connected

| Step | Client sends | Reply | Evidence |
|---|---|---|---|
| discovery | `WING?` | `WING,<ip>,<name>,<model>,<serial>,<fw>`, fw `3.1`+ | [A][B], T003 |
| (1)(2) meter port/define | ch3 `D3`, `D4 .. DC .. DE` | **no TCP reply.** Send UDP `id.l + int16[]` to `client_ip:port` every 50 ms | [B] sent 50 ms packets of `-90 dB`; the connection stayed up. [A] replied `DF D3 DE` on ch3 (spurious) |
| (3) scene sets | `D7 h value` | none (no echo needed) | [A][B] |
| (4) gets | `D7 h DC` | **`D7 h <value> DE`** for every get | [B] continued to (6)-(8). [A] answered `DE` only, and WING-Edit never sent ModelRequest or periodic renews (see §3) |
| (5) list defs | `D7 h DE DD` | definition record(s) + `DE`; `DE` alone is tolerated | [A][B] |
| (6) model | set `$syscfg/$cnsmdl = "ModelRequest"` | [H] `D7 985729a2 <"wing-fullsize"/"wing-rack"/"wing-compact">` (as an event, no `DE`) | Not answered in [B], and the connection still stayed up. T003 found that WING-Edit goes offline unless the model is one of those 3 strings. Verify in T013/T016 |
| (8) keep-alive | ch3 `D4 00000003` every ~4 s | keep sending report 3 for 5 s after each renew | [B]: 28 renews in 114 s |

The server must not treat unknown hashes as fatal. Hash `00000000` arrives about 1.4k times per push (§2.3).

---

## 2. The PC → MIXER push

### 2.1 Shape
- Every connection (A1, A2, A3 and B) starts with a full push, without any request from the console. WING-Edit
  never sent `DA DC` (MIXER → PC) in these sessions, so the sync direction was PC → MIXER every time. A1 and A2 waited
  2.4-3.1 s before pushing. A3 and B pushed 0.05 s after the meter definition. The delay was probably a user dialog
  [H].
- Each value is `D7 <hash BE32> <value token>` on ch1, back to back, with no `DE` and no terminator. The push is
  229 kB of wire bytes in 1400-byte TCP segments over about 0.5 s.
- Order is the schema tree order of the `ae` tree: `cfg io ch aux bus main mtx dca mgrp fx cards play rec`, then
  `$syscfg/usbh_spd`. Next comes the `$ctl` (`ce`) tree: `layer`, `user`, `cfg/mfdr`, ... Last come a few `$` leaves:
  `$stat/usbstate`, `cards/$type`, `cards/wlive/{1,2}/$stat/{sdstate,sessions}` and `$ctl/lib/$actshow`.
  `$globals` is not pushed.
- A1 also contains three `cfg/rta/$src`, `$tap` and `$dec` sets, interleaved with the gets at 38.529 s. These are
  probably GUI-driven (RTA view) [H].

### 2.2 Counts and mapping

| Connection | sets | unique hashes | mapped unique (non-zero) | sets to hash 0 |
|---|---|---|---|---|
| A1 | 29,914 | 28,523 | 28,522 / 28,522 (100%) | 1,392 |
| A2 | 30,031 | 28,520 | 28,519 / 28,519 (100%) | 1,512 |
| A3 | 30,151 | 28,520 | 28,519 / 28,519 (100%) | 1,632 |
| B | 28,715 | 28,524 | 28,523 / 28,523 (100%) (incl. `$cnsmdl`) | 192 |

- **Every non-zero hash maps to a schema path (100%).** Counting hash 0 as one unique id gives 28,522/28,523
  (99.996%) for A1. A2 and A3 carry the same 28,519 params minus the 3 RTA sets. Their values are identical to A1.
- Schema coverage: the push carries 28,522 of the schema's 36,499 params. All of them are schema params, not nodes.
  The 7,977 not pushed are:
  - **4,502 unused model slots.** Slots marked `unused_slot` for the default model, which the current scene's
    models also leave unused. WING-Edit skips n/a params; this confirms T003. 2,986 slots marked `unused_slot`
    in the default tree *were* pushed, because the scene uses other models.
  - **3,349 params with a `$` (or `~`) path component below the root.** These are live/read-only values: `$stat/*`,
    `io/in/$BUS|$SEND|$MTX|$MAIN|$MON/*`, `cards/wlive/*/$stat|$ctl/*`, `cfg/mon/*/$lvl`, `*/$ha` and so on. The only
    pushed exceptions are the 11 listed in §2.1.
  - **126 `$ctl` console-local surface settings**: `$ctl/cfg/*` (except `mfdr`), `$ctl/safes/*`, `$ctl/daw/*`,
    `$ctl/midi/*`, `$ctl/osc/ronly`, `$ctl/gpio/*`, `$ctl/user/{sel,mode,cmode,cuser}`.
  - Pushed params without flag bit 26 (2,443, mostly `$ctl/layer/WEDIT/*`) show that bit 26 alone does not decide
    what is pushed.

### 2.3 Hash `00000000` (the only "unmapped" id)
All the hash-0 sets come in runs of 66 or 72. Each run directly follows the last slot param of a `$ctl/user/*` button
or encoder block whose `mode` is not `OFF`, for example after `$ctl/user/user/1/bu/2` (mode FXPAR) or after
`$ctl/user/1/1/enc/2` (mode FX). There are 20 runs in A1. The values are small ints, for example
`0 0 0 0 0 0 1 1 1 …` or `0 4 0 4 … 4`.
Interpretation [H]: these are sub-nodes that WING-Edit creates when a user-control mode is applied, after
`Cnode::setRoot` built the hash table, so they never got a hash. Supporting evidence:
- The count grows by 120 on every reconnect (1,392, 1,512, 1,632), which looks like nodes leaking on each scene
  apply.
- They are not in the `CaudioEngine` tree (T003).
- A real console cannot resolve id 0.

**Simulator rule**: an unknown hash (including 0) must invalidate the cursor. The following value token is then
dropped, and does not land on the previously selected node. The fixture lists the runs (`hash0_runs`) with the
preceding path, for reference.

### 2.4 Value encodings (A1, by schema class)

| Schema class/type | Token(s) on the wire | Notes |
|---|---|---|
| `CparLevel` (fader), `CparLinFloat`, `CparLogFloat`, `CparFenum` | `D5 f32` (engineering units: dB, Hz, ms, %) | Never `D6` (normalised) |
| `CparBool`, `CparInt` | `00..3F`, `D3 i16` for ≥64 | `D3` only for `*/icon` ids (54×) and a few others. Never `D4` |
| `CparEnum` | **string token with the enum value name** (`"OFF"`, `"COMP"`, `"MAIN.1"`) | Not an index. The server must map name → index |
| `CparStr`, `ae::CparTag` | `D0` (empty) or `80+len-1` + bytes | `D1` (long) never seen; the max length pushed is ≤64 |
| `CparVar` (model slot) | the current model's type: `D5`, small int, `D3` or string | e.g. `ch/1/dyn/1 = -23.5` with `ch/1/dyn/mdl = "COMP"` |

Totals for A1: small int 14,751; `D5` 7,435; `80+n` strings 6,488; `D0` 1,180; `D3` 60. No `D1`, `D4`, `D6`, `D8`,
`D9`, name/index navigation, or `DA` root navigation was seen anywhere.

---

## 3. Why each connection ended

| Conn | Life | Last client traffic | End |
|---|---|---|---|
| A1 | 34.913-40.267 s | `D7 … DE DD` at 38.538, then a single ch3 renew `DF D3 D4 00000000` at 40.264 | client closed 3 ms after the renew. A2 opened 18 ms later (auto-reconnect) |
| A2 | 40.285-57.134 s | last `DD` at 43.266, then a single renew at 57.132 | client closed 2 ms after the renew |
| A3 | 63.107 s- | last `DD` at 63.718. No renews and no traffic until the log ends (151.8 s, 88 s idle) | not closed in the log (truncated) |
| B | 0.919-114.7 s | ModelRequest, then meter defines 1-3, then 28 renews every 4.04 s | stayed up until the fake was stopped |

What differs between A and B:
1. In A the fake answered every get with a bare `DE` and no value.
2. In A the fake scanned *bytes* rather than tokens. It replied `DE` to every `0xDC`/`0xDD` byte, including bytes
   inside floats and hashes: 187 spurious `DF D1 DE` replies per connection, plus `DF D3 DE` on the meter channel.
3. In A no meter data was sent.

In A, WING-Edit never progressed to the ModelRequest or the periodic renew. It stays in a half-connected state.
The single renew followed by a close in A1 and A2 looks like WING-Edit's disconnect path, either user-triggered or a
connect timeout [H]. The close times (5.4 s and 16.8 s after connect) are not consistent with a fixed timer.
Sending value replies to the gets (B) is enough to reach the steady state. Run B used proper get replies *and* UDP
meter data, so it does not say which of the two is required. A follow-up run with get replies but no meters was
attempted: WING-Edit had not reconnected by itself after the previous fake stopped (§4).

Keep-alive behaviour [B]:
- In the steady state WING-Edit sends no ch1 keep-alive at all. The only periodic traffic is the ch3 `D4 <id>`
  renew every 4.04 s.
- That is well inside the console's 10 s inactivity timeout (protocol.md §2.1). The simulator should count ch3
  traffic as activity.
- `WING?` discovery probes continue at 2 s intervals while connected, from the meter UDP socket.

Lost-connection detection [B]: after the fake was killed, WING-Edit's socket stayed in `CLOSED` state, and it did not
reconnect to a new fake listening on the same port for at least 4.5 min (two restarts: 90 s and 180 s; `lsof` still showed the dead socket). It does not appear to auto-reconnect after a
server-side close, which needs a user click. Auto-reconnect after A2 was fast (18 ms) only because the client closed
the connection itself.

---

## 4. Answers to protocol.md §9 open questions

1. Node ids: local hashes (T003), confirmed on the wire. 100% of non-zero ids match `wing_hash(path)`.
3. Channels: only ch1 and ch3 are used. No ch0/ch2. Post-connect sequence: §1. The PC → MIXER sync is a flat
   `D7 h v` stream, not a `DC` dump. MIXER → PC (`DA DC`) was not exercised.
4. Value tokens: §2.4. `D5` for all floats, enum by name, no `D6`. No echo is required: the push completes without
   any reply.
5. Meters: report ids 0-3. The definitions are in §1. The renew interval is 4.04 s. The expected packet sizes are
   still unverified: [B] sent `11×16 + 11 + 32` words and WING-Edit kept renewing.
6. Keep-alive: the ch3 renew every ~4 s. Lost-connection timeout: not observed (§3).
7. OSC 2223: no OSC traffic in either capture (the fake in A listened on 2223).
10. Echo: not needed by WING-Edit. Knock: WING-Edit always probes first, so this is moot for WING-Edit.

New open questions (for T013/T016):
- What exact reply does a real console send to `D7 985729a2 "ModelRequest"`, and what does WING-Edit do with it?
- What does the console return for `D7 <enum-par> DE DD` (dynamic enum lists `play/$songs`, `$ctl/lib/$scenes`)?
  Most likely a single definition record of the enum with the current list, then `DE`. The leading `DE` from the
  client is unexplained [H: it resets a pending response state].
- Is the UDP meter stream required to stay connected? Run with get replies but no meters.
- MIXER → PC sync: `DA DC` is expected (T003). That needs a user-initiated connect with that direction selected.

---

## 5. Fixtures (`fixtures/protocol/`)

| File | Content |
|---|---|
| `wing-edit-connect-{1,2,3}.bin` | raw client→server wire bytes (escaped, as on TCP) of capture A connections 1-3 (229,085 / 229,781 / 230,494 B). Connection 3 is cut short by the log truncation |
| `wing-edit-connect-tokens-1.bin` | raw client bytes of capture B (222,108 B): push, gets, `DD`, ModelRequest, meter defines, 28 renews |
| `wing-edit-connect-tokens-1.server.bin` | the token-mode fake's TCP replies in B (275 B): 25 × `D7 h v DE` + 2 × `DE`. A minimal reply set that WING-Edit accepted |
| `wing-edit-connect-tokens-mode.txt.gz` | the full capture-B log (`fake_wing.py` format) |
| `pc-to-mixer-scene.json` | capture A connection 1 push: `params` (28,522 `path → value`; floats as numbers, ints as numbers, enums and strings as strings), `hash0_runs` (20 runs, with the preceding path), `encodings`, `set_count` |

Suggested T013 tests:
- Feed `wing-edit-connect-1.bin` into the server's decoder. Expect 29,914 sets, 25 gets, 2 `DD`, 0 decode errors,
  and the resulting state equal to `pc-to-mixer-scene.json`.
- Replay `wing-edit-connect-tokens-1.bin` and assert the server's replies contain one `D7 h … DE` per get.

## 6. Codec validation (wing-proto NRP)

- All 4 sessions (911 kB) decode with **zero errors**: no unknown tokens, no truncated tokens at stream end, no bytes
  on channels other than 1 and 3, and no bytes before the first channel select.
- Payload `DF` escaping inside hashes (`DF DE`, for example `D7 FF 53 68 DF DE …`) and inside the meter port
  (`D3 EB DF DE`) is handled.
- An independent Python demux/tokeniser (Behringer receive routine) gives identical `D7` counts: 29,941, 30,058 and
  30,178 for A1-A3.
- No codec bugs were found. Notes for the server side:
  - `NrpDecoder` drains ch1 and ch3 from separate queues, so the relative order of ch1 and ch3 tokens inside one
    `feed` is lost. This is harmless for WING-Edit, but a server that logs a single transcript would want the order
    kept.
  - Client-originated `DE` (ch1 `D7 h DE DD`, ch3 `D3 port DE`) decodes as `Token::End` / `MeterToken::End`. The
    server must ignore it, not treat it as an error. `MeterRequestParser` already ignores an `End` outside a
    definition.

## 7. Tool used for capture B (not committed; recipe)

Capture B came from a scratch variant of `tools/protocol-probe/fake_wing.py`. It has the same discovery and logging,
plus:
- a real ch1 tokeniser. `D7 h DC` → `D7 h <value by schema type> DE`. Standalone `DC`/`DD` → `DE`.
- no TCP reply on ch3. UDP meter packets go from :2222 to `client_ip:<D3 port>` every 50 ms for 5 s after each
  `D4`.
- an optional `--model-reply` for ModelRequest (not exercised).

It ran passively (no UI interaction). WING-Edit connected by itself through auto-reconnect. Porting this reply mode
into `fake_wing.py`, or straight into T013, is a proposed follow-up.
