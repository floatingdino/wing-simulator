# WING remote protocols: implementable spec (T002)

Status: research output for T002, 2026-10-25. This is the reference for `crates/wing-proto` (T012 discovery,
T013 native TCP, T014 OSC, T015 meters).

Confidence tags used below:
- **[LIVE]**: observed on this Mac against WING-Edit 3.3.4 (captures in `tools/protocol-probe/captures/`).
- **[DOC]**: Behringer/Maillot *WING Remote Protocols* PDF. Two editions were read: V3.0.5-1 (bundled in libwing
  1.0.5) and V3.1.2-a (current Google Drive copy linked from Maillot's site). Where they differ, both are noted.
- **[LIB]**: open-source client source code, mainly libwing 1.0.5 (`src/console.rs`, `src/node.rs`).
- **[DERIVED]**: worked out from the above but not seen on the wire. Verify it before relying on it.

All multi-byte integers on the wire are **big-endian**, and floats are IEEE-754 binary32, also big-endian.

---

## 0. Summary for implementers

| Port | Transport | Purpose | Server behaviour the simulator needs |
|---|---|---|---|
| 2222 | UDP | Discovery | Reply to `WING?` with an ASCII CSV identity string, sent to the requester's ip:port |
| 2222 | TCP | Native binary protocol ("NRP") | Escape-framed multi-channel byte stream. Token-based get/set/navigate/node-definition. Change events are pushed to connected clients. 10 s inactivity timeout. Max 16 (fw 3.0.5) or 24 (fw 3.1.2) clients |
| (client-chosen) | UDP | Meter data | Requested on NRP channel 3. The console sends `reportId(4) + int16[]` about every 50 ms to client ip:port. Each request lives 5 s and is renewed by the client |
| 2223 | UDP | OSC 1.0 (messages only, no bundles) | Paths, `,s/,i/,f/,b` args, node listings, `/*S` event subscription (one subscriber, 10 s) |

---

## 1. Discovery (UDP 2222)

### 1.1 Probe sent by WING-Edit 3.3.4 [LIVE]

Captured with `tools/protocol-probe/capture_discovery.py`, which binds 0.0.0.0:2222 with SO_REUSEADDR/SO_REUSEPORT and
reads the destination address with IP_RECVDSTADDR:

```
[  0.549s] #1 from 192.168.20.130:61075 to ['192.168.20.255'] len=5 repr=b'WING?'
00000000  57 49 4e 47 3f                                   |WING?|
[  0.549s] #2 from 192.168.20.130:61075 to ['255.255.255.255'] len=5 repr=b'WING?'
00000000  57 49 4e 47 3f                                   |WING?|
[  2.553s] #3 from 192.168.20.130:61075 to ['192.168.20.255'] len=5 repr=b'WING?'
...
```

- Payload: exactly 5 bytes, `57 49 4E 47 3F` ("WING?"), with no terminator.
- Cadence: one burst every **2.0 s** while the CONNECT page is open (or always; this was not tested). Each burst is 2
  datagrams: one to the **subnet-directed broadcast** (192.168.20.255) and one to **255.255.255.255**.
- Source port: an ephemeral UDP port. It stayed fixed for minutes (50677), then started changing every cycle (52164,
  52370, 61075) once our fake console had answered. Reply to the source ip:port of *each* probe.
- WING-Edit had no other sockets open: no TCP, and no OSC 2223 socket, while it was not connected.

### 1.2 Reply [DOC][LIB][LIVE]

Send the reply to the probe's source ip:port. Our responder sent it from port 2222, as a real console does.

```
WING,<ip>,<name>,<model>,<serial>,<firmware>
```

The reply is plain ASCII, comma-separated, with no trailing NUL. A NUL-less reply was accepted live. Example from
libwing's `Discovery.md` (real Wing Compact):
`WING,192.168.1.19,WING-PP-93892216,wing-compact,01XXXUX06X3AEX,3.0.5-0-g0c2b9d4a:release`.
Example from the PDF (via OSC `/?`): `WING,192.168.1.71,PGM,ngc-full,NO_SERIAL,1.07.2-40-g1b1b292b:develop`.

| Field | Values seen | Notes |
|---|---|---|
| ip | dotted quad | **WING-Edit displays (and presumably connects to) this field, not the UDP source address** [LIVE: a reply carrying `10.9.9.9` from 192.168.20.130 was listed as 10.9.9.9] |
| name | any ASCII, e.g. `PGM`, `WING-PP-93892216` | Shown as "Name" and "CONSOLE NAME" |
| model | `ngc-full` (full-size WING), `wing-compact`, `wing-rack`, `wing-bk` (black full-size; unconfirmed) | [LIVE] `ngc-full` is shown as "WING". `wing-compact` is shown raw ("wing..."). The WING-Edit strings include `ngc-full`, `wing-bk`, `wing-rack`, `wing-compact`, `wing-fullsize` (the "ACTIVE WING MODEL" UI value) |
| serial | ASCII | not shown in the list |
| firmware | `<maj>.<min>.<patch>-<n>-g<githash>[-dirty]:<branch>`, e.g. `3.0.5-0-g0c2b9d4a:release` | The list shows it truncated; "CONSOLE INFORMATION" shows `3.0.5-0`. **Discovery does not check it**: `1.07.2-40-...:develop` was listed without complaint |

Firmware compatibility [LIVE strings, not exercised]: WING-Edit contains `Incompatible Mixer Firmware!` /
`Please update to the latest version`, with `-dirty`, `release` and `%d.%d.%d` nearby. So it parses the firmware
string and checks it **at connect time**, not at discovery. Behringer release notes show each WING-Edit release
requiring a minimum console firmware (for example, WING-Edit 3.2 requires fw 3.0.6). The minimum for 3.3.4 is unknown
and is a T003 question. Recommendation: make the advertised firmware string configurable, and default it to the
newest firmware string T003 finds referenced. Until then, something like `3.1.2-0-g00000000:release` is a guess. The
snapshot embedded in WING-Edit has `"creator_fw":"3.0.6-193-g60c910ab-dirty:dev_unidyn"`,
`"creator_model":"wing-rack"`, `"creator_sn":"EMU"`, `"creator_version":"emulation"`.

Other observed WING-Edit behaviour [LIVE]:
- Entries stay in the "SELECT CONSOLE" list after the console stops answering (stale entries persist until RESCAN).
- The default "AUTO CONNECT: ON" did **not** auto-connect to a newly discovered console. It probably applies only to
  reconnecting a previously connected console, and the prefs plist stores no console. A TCP connection therefore needs
  a user click on CONNECT (§7).
- Maillot's `wingemul` README says that from v0.03 it "accepts direct TCP without a prior UDP knock". That implies
  real consoles may only accept TCP 2222 from hosts that sent `WING?` first. The simulator should **not** enforce
  this.

---

## 2. Native protocol (TCP 2222): framing layer ("NRP")

### 2.1 Connection

- The client connects to TCP 2222 with no banner or greeting from either side.
- libwing and behringer-wing-typescript send `DF D1` (select channel 1) immediately after connecting [LIB].
- Inactivity timeout: the console closes a connection after **10 s without receiving anything** [DOC]. libwing
  re-sends `DF D1` every 7 s as a keep-alive, and that alone is enough [LIB]. wapi says to request some data
  periodically [DOC].
- Max simultaneous TCP clients: 16 (doc V3.0.5) or 24 (doc V3.1.2). Further connections are rejected.
- Simulator: implement the 10 s timeout, but make it configurable or disable it during bring-up.

### 2.2 Channels and escape byte [DOC]

The byte stream is multiplexed into 14 logical channels. `0xDF` is the escape byte.

| Wire sequence | Meaning |
|---|---|
| `DF D0`…`DF DD` | switch the current channel (rx or tx direction independently) to ChID 0…13 |
| `DF DE` | literal data byte `DF` |
| `DF DF` | literal `DF`. The receiver stays in the escape state, so the next byte is interpreted again after a `DF` |
| `DF xx` (xx not in D0…DF) | literal `DF` followed by literal `xx`. The 3.1.2 doc example: `D7 DF E0 66 C1 D3 00 7F` is passed through unchanged |
| a trailing `DF` at the end of a send buffer | the transmitter appends `DE` |

The channel table in the doc is confusingly numbered (1-based "channel #" alongside 0-based "ChID"). What actually
goes on the wire:

| ChID (wire `DF Dx`) | Use |
|---|---|
| 1 (`DF D1`) | Audio engine and control requests: all get/set/node-def traffic (`ae_data` and `ce_data` trees) |
| 3 (`DF D3`) | Meter subscription requests (the data arrives over UDP) |
| 0, 2, 4…13 | "n/u" per the doc table. The prose says "control engine = channel #1". Whether WING-Edit uses 0/2 is open (T003) |

Normative receive routine (Behringer sample, transcribed):

```c
// state: escf=false, ch=-1
void rx(byte b) {
  if (b == 0xDF && !escf) { escf = true; return; }
  if (escf) {
    if (b != 0xDF) {
      escf = false;
      if (b == 0xDE) b = 0xDF;                       // escaped literal DF
      else if (b >= 0xD0 && b < 0xD0+14) { ch = b-0xD0; return; }  // channel switch
      else if (ch >= 0) deliver(ch, 0xDF);           // DF + ordinary byte => both literal
    }
    // b == 0xDF while escaped: falls through, delivers DF, escf stays true
  }
  if (ch >= 0) deliver(ch, b);
}
```

Normative transmit routine (Behringer sample): on a channel change, emit `DF D<ch>`. Then for each byte: if the
previous byte was `DF` and this byte is in `D0…DE`, emit `DE` before it. If the buffer ends in `DF`, emit `DE`.

Simulator encoder [DERIVED]: always emit a payload `DF` as `DF DE`. This is valid under the receive routine,
unambiguous, and what libwing does for hashes.
Simulator decoder: implement the routine above exactly. Clients differ: libwing escapes `DF` in hashes but not in
strings.

Doc examples:
- 3.0.5: `D7 02 DF DE AF 0E 02` decodes to `D7 02 DF AF 0E 02`.
- 3.1.2: `D7 DF DE D4 AB CF 08` decodes to `D7 DF D4 AB CF 08`.
- With the current tx channel 3, sending `d702dfaf0e02` to "channel 1" transmits `df d0 d7 02 df af 0e 02`.

Bytes received before any channel has been selected are dropped (`ch == -1`).

---

## 3. Native protocol: token layer (channel 1)

### 3.1 Token table [DOC Table 2]

| Token | Payload | Meaning |
|---|---|---|
| `00` | none | value false / off / 0 |
| `01` | none | value true / on / 1 |
| `02`…`3F` | none | small int value 2…63 |
| `40`…`7F` | none | **navigate**: child at node index 1…64 (`index = tok-0x3F`) |
| `80`…`BF` | n bytes | string value, length 1…64 (`n = tok-0x7F`) |
| `C0`…`CF` | n bytes | **navigate**: child by node name, length 1…16 (`n = tok-0xBF`) |
| `D0` | none | empty string value |
| `D1` | u8 L, then L+1 bytes | string value, length 1…256 |
| `D2` | u16 | navigate: child index 1…65536 (libwing reads `u16+1`) |
| `D3` | i16 | int16 value |
| `D4` | i32 | int32 value |
| `D5` | f32 | float value in engineering units (dB, Hz, ms, ...) |
| `D6` | f32 | raw float value, normalized 0.0…1.0 |
| `D7` | u32 | **navigate**: select node by hash (absolute) |
| `D8` | none | click (toggle) the current node |
| `D9` | i8 | step the current node by ±n |
| `DA` | none | navigate to the root node |
| `DB` | none | navigate one level up |
| `DC` | none | **data request**: current node |
| `DD` | none | **node definition request**: children of the current node |
| `DE` | none | end of a data/definition response |
| `DF` | u16 len (or `0000` + u32 len) | node definition record follows (len bytes) |
| `E0`…`FF` | | unused |

Semantics: the token stream is a tiny stack machine with a **current node** cursor per connection.
- Navigation tokens (`D7`, `DA`, `DB`, `40-7F`, `C0-CF`, `D2`) move the cursor.
- A **value token** (`00-3F`, `80-BF`, `D0`, `D1`, `D3-D6`) sets the current node's value. This is a set when
  client→console, and a change/answer when console→client.
- `D8`/`D9` act on the current node. `DC`/`DD` request data/definitions.
- Console→client value messages have the same shape: `D7 <hash> <value token>` [DOC OSC-blob example; LIB].

Integer value encoding used by clients [LIB]: 0…63 as a single byte, else `D3 i16` if it fits, else `D4 i32`. String
encoding: empty → `D0`, 1…64 → `0x7F+len` + bytes, 65…256 → `D1 (len-1)` + bytes. Fader/float sets use `D5 f32` [LIB].
WING-Edit's own choice (D5 vs D6) is a T003/capture question. The simulator must accept every value form for every
node type and convert, as the console does: int → float, string → enum, etc.

### 3.2 Worked examples (payload bytes on channel 1, before escaping)

Hashes come from the PDF and libwing's propmap: `/ch/1/mute = F50F69F8`, `/ch/1/fdr = 38AE75C2`,
`/ch = 70B10139`, `/ch/1 = 878392CF`, `/$stat = 97A00439`, `/$ctl/daw = DF17C242`, `/$ctl/daw/$on = BEEFAEAB`.

| Purpose | Client → console | Console → client |
|---|---|---|
| connect / keepalive | `DF D1` | (none) |
| get a leaf [DOC] | `D7 BE EF AE AB DC` | `D7 BE EF AE AB 01 DE` |
| set mute on [DOC] | `D7 F5 0F 69 F8 01` | change events to other clients: `D7 F5 0F 69 F8 01` (the sender: see `D8` below and §9 Q10) |
| set ch1 fader to −3 dB [DERIVED] | `D7 38 AE 75 C2 D5 C0 40 00 00` | |
| set ch1 fader by raw value 0.675 [DERIVED] | `D7 38 AE 75 C2 D6 3F 2C CC CD` | |
| toggle mute [DERIVED] | `D7 F5 0F 69 F8 D8` | the new value must reach the clicking client: `D7 F5 0F 69 F8 01` (+ `$mute`) [I, T083] |
| path navigation, same as `/ch/1/mute` get [DERIVED] | `DA C1 63 68 40 C3 6D 75 74 65 DC` (root, name "ch", index 1, name "mute", data request) | |
| hash containing DF (`/$ctl/daw` defs) | `D7 DF DE 17 C2 42 DD` (libwing form) or `D7 DF 17 C2 42 DD` (also legal) | def records + `DE` |
| change events seen via OSC `/*b` [DOC] | | `D7 38AE75C2 D5 C3100000  D7 7E463474 D5 C3100000` (fdr and `$fdr` = −144 dB), `D7 F50F69F8 01  D7 26855CD3 01` (mute and `$mute`) |

Unsolicited events [DOC wapi]: "if a connection is kept active, changes made directly at the console ... will
generate data the application will continuously receive". So on TCP **every connected client gets change events
without subscribing**. The OSC `/*b` stream proves the event encoding is identical. Note the paired `$`-prefixed
shadow nodes (`$fdr`, `$mute`) that are emitted alongside the real node.

### 3.3 Node definition records [DOC][LIB]

Request: `DA DD` gives the root's children, and `D7 <hash> DD` gives the children of that node (libwing uses id 0 to
mean root). Response: one `DF` record per child, then `DE`.

```
DF  len.w  [len.l if len.w==0]
    parent.l  hash.l  index.w  namelen.b name  longnamelen.b longname  flags.w  <type-specific>
type-specific:
  node (0), fader level (3):  -
  lin float (1), log float (2): min.f max.f steps.l
  int (4):                    min.l max.l
  string enum (5):            count.w { itemlen.b item  longitemlen.b longitem } * count
  float enum (6):             count.w { value.f  longitemlen.b longitem } * count
  string (7):                 maxlen.w
flags: bits0-3 unit (0 none,1 dB,2 %,3 ms,4 Hz,5 m,6 s,7 oct); bits4-7 type (above); bit9 read-only
```

`len` counts the bytes after the length field (from `parent` onward), measured after unescaping. Numbered children
(for example `/ch/1`) have an **empty name and index ≥ 1**. Named children have index 0.

Example 1, the first root child [DOC, raw bytes from the OSC blob dump of `/` `,b dd`]:

```
df 0018 00000000 97a00439 0000 05 2473746174 05 5354415445 0000
   len=24 parent=root  hash   idx=0 "$stat"   "STATE"     flags=0000 (node)
```

Example 2, `/$ctl/daw/$on` [DOC]:

```
df 001f df17c242 beefaeab 0000 03 246f6e 06 444157204f4e 0040 00000000 00000001 de
   len=31 parent    hash    idx  "$on"    "DAW ON"      int   min=0    max=1   end
```

Root children, in order: `$stat cfg $syscfg io ch aux bus main mtx dca mgrp fx cards play rec $ctl`. Hashes:
`/ch=70b10139`, `/aux=8fa3078d`, `/bus=f46c185e`, `/main=04d3a3a8`, `/mtx=f82a5af2`, `/dca=e313aeff`,
`/mgrp=d252398b`, `/fx=473c9134`, `/cards=b4296fc9`, `/play=57297a28`, `/rec=fab1762c`, `/$ctl=cbb95143`,
`/cfg=edca7af9`, `/$syscfg=f89818a6`, `/io=294f7794`.

**Dynamic subtrees**: children of nodes that carry a `mdl` enum (channel filter/gate/eq/dyn, insert slots, FX slots,
and so on) change when `mdl` changes. Model-specific parameters share hashes by **position** (wapi says "setting #7
of EQ STD (`1q`) and of EQ SOUL (`lmg`) have the same token"). libwing's propmap shows 60,748 path entries and only
36,216 unique ids for this reason. The simulator must regenerate definitions and push values when `mdl` changes. How
the console announces a structure change to clients is open.

### 3.4 Hashes

- Every node and leaf has a 32-bit id the docs call a "hash". The id depends on the full path (`/ch/1/mute` ≠
  `/ch/2/mute`).
- CRC-32 (zlib/BE/C), FNV-1/1a, djb2 and Murmur3 were tested over name, path and parent-seeded variants: **no
  match**.
- None of the known hashes appears as a literal (BE or LE) in the WING-Edit binary.
- So WING-Edit either computes them with an unknown function, or **learns them at runtime** by walking the tree with
  `DD` (or by name/index navigation). Either way the simulator should serve the real ids.
- Source: libwing `src/propmap.rs` embeds **60,748 complete raw definition records** (wire format above, including
  parent id and flags/RO bit; 2,664 read-only; 27,896 model-variant entries). They were captured from a Wing Compact
  on fw 3.0.5. License Apache-2.0, so this can seed `data/schema` with attribution. Newer firmware may add nodes; T003
  should reconcile.

### 3.5 Data dumps

- `D7 <hash> DC` on a **leaf** returns `D7 <hash> <value> DE` [DOC].
- `DC` on a **node** (tree dump) presumably returns `D7 <childhash> <value>` for every descendant leaf and then `DE`
  [DERIVED; unverified]. WING-Edit's "Synchronize MIXER→PC" uses this or a node walk (open question).
- Over OSC, the textual `,s *` node dump returns snapshot-style `key=value` text (§5).

---

## 4. Meters (request on NRP channel 3, data over UDP)

### 4.1 Request tokens [DOC Table 4]

| Token | Payload | Meaning |
|---|---|---|
| `D3` | u16 | client UDP port (set at least once per connection) |
| `D4` | u32 | report id. Repeating `D4 <id>` alone renews that report's 5 s lifetime |
| `DC` | | start of a meter collection definition (applies to the preceding `D4` id) |
| `A0` ch 1-40, `A1` aux 1-8, `A2` bus 1-16, `A3` main 1-4, `A4` mtx 1-8, `A5` dca 1-8, `A6` fx 1-16, `A7` source group 1-16, `A8` output group 1-11, `A9` monitor, `AA` RTA, `AB` ch V2, `AC` aux V2, `AD` bus V2, `AE` main V2, `AF` mtx V2 | | meter type selector |
| `00`…`7F` | | index 1…128 (**zero-based**), repeatable after one type token |
| `DE` | | end of collection |

Examples:
- `DC A0 00 01 08 A6 04 DE` selects ch 1, 2, 9 and FX 5 [DOC].
- Full sequence (doc 3.1.2):

```
DF D3 D3 37 37                  ; meter port 0x3737 = 14135
DF D3 D4 00 00 00 02 DC A0 01 DE ; report id 2 = channel 2
DF D3 D4 00 00 00 02            ; renew id 2 (send within 5 s; libwing renews every 3 s)
```

  (Doc 3.0.5 appends `DF D1` after each, switching back to channel 1. libwing does the same.)
- libwing's report id is `(seq<<16)|udpPort`, sent as `D4 SS SS PP PP`.

### 4.2 Data packets (console → client UDP ip:port) [DOC][LIB]

```
reportId.l  { int16 }*      ; big-endian, one block per selected meter, in request order
```

Group order (doc 3.1.2 footnotes):
- `A7` source groups 1…16: local, aux, AES50 A, AES50 B, AES50 C, StageConnect, USB audio, card, module, USB player,
  AES/EBU, user signals, oscillator, buses, mains, matrices.
- `A8` output groups 1…11: local, aux, AES50 A, AES50 B, AES50 C, StageConnect, USB audio, card, module, USB
  recorder, AES/EBU.

| Type | Words per index | Contents |
|---|---|---|
| channel/aux/bus/main/matrix | 8 | in L, in R, out L, out R, gate key, gate gain, dyn key, dyn gain |
| dca | 4 | pre L, pre R, post L, post R |
| fx | 10 | in L, in R, out L, out R, 6 state meters |
| source | n | group levels (local ins: 8 in doc 3.0.5, 24 in doc 3.1.2; oscillator 2) |
| output | n | group levels (local outs: 8) |
| monitor | 6 | solo L/R, mon1 L/R, mon2 L/R |
| rta | 120 | RTA bins, semitones from E0 (20.6 Hz) to D#10; A4 at index 53 |
| * V2 | 11 | in L/R, out L/R, gate key, gate gain, gate LED(0/1), dyn key, dyn gain, dyn state(0/1), automix gain |

Value scale:
- Levels are in **1/256 dB** (signed): `0xF100` = −15.0 dB.
- Gate and dyn gain are scaled so that 256 ≈ 20 dB GR for most models (the standard gate is 60 dB).
- FX band GR: dB = v × 6/2048.
- T082 (WING-Edit 3.3.4 `nrp::MeterHandler::rxMeters`, static analysis): the gate/dyn gain word is a GR-meter
  **position × 256** computed with the slot's selected model's law (WING-Edit `ae::*_gain_<model>`; 60 dB full
  scale for GATE/DUCK, 20 dB for most compressors, negative for 76LA/L100/RIDE, VU-style for B160/SBUS, ...).
  DEQ2 is special: the gain word and the following LED/state word are its two band gains `256 (g_k − 1)`.
  FX blocks: words 0–3 levels, words 4–9 model-specific state read by a few FX pages (channel-strip FX LEDs,
  DEQ3 bands, C5-CMB band GR, limiter/de-esser GR). The full table is in `crates/wing-blocks/src/meter.rs`.

Timing: sent about every **50 ms** for 5 s after the last `D4` renew. WING-Edit has a "REDUCED METER UPDATE RATE
(50%)" option. The UDP source port the console sends from is undocumented; use 2222. libwing parses the first 2
bytes as the id because it encodes the port in the low half. Real consoles use 4 bytes.

Simulator: one meter scheduler per TCP client and report id. Expire after 5 s. Use the latest port. Keep
DSP-produced levels in lock-free snapshots (T039/T015).

---

## 5. OSC (UDP 2223) [DOC]

- OSC 1.0 messages only. **No bundles.**
- The type-tag string may be omitted. Types are `i`, `f`, `s`, `b`, big-endian and 4-byte padded.
- Max packet 32 KB. Replies go to the sender's ip:port, unless a `/%<port>/...` prefix redirects them.
- `/?` returns `/? ,s "WING,<ip>,<name>,<model>,<serial>,<fw>"` (the same string as discovery).
- **Get leaf**: send the address with no args.
  - Float leaf: `,sff` `<text> <raw 0..1> <value>`, e.g. `/ch/1/fdr ,sff "-oo" 0.0 -144.0`.
  - Int leaf: `,sfi`, e.g. `/ch/1/mute ,sfi "1" 1.0 1`.
  - String or enum leaf: `,s`.
- **Get node**: `/ch/1` returns `,sss…` with the child names (`/` returns the root list).
- **Set**: the address plus `,s` (text, enum name), `,i` (int, enum index) or `,f`.
  - The console converts as needed. Writes are **not echoed**; errors may come back as an event.
- **Node set**: `/ ,s "/ch.1.fdr=-1,mute=0,.2.fdr=0,mute=1"` (`.` goes down or up, `/` goes to root, `,` separates).
  - Reply: `/* ,s "OK"`, or `/<node>* ,s "OK"` for node-relative sets like `/ch/1 ,s "fdr=3"`.
  - Errors: `NODE NOT FOUND`, `VALUE ERROR`, `BUFFER OVERFLOW`, `NODE IS NOT PAR`, `INCOMPLETE DATA`, `STACK EMPTY`.
- **Node special args**:
  - `,s "*"` returns a snapshot-style dump (`mdl=NONE,fxmix=100,`).
  - `,s "?"` returns definitions (`mdl list [NONE, EXT, ...]~ fxmix lin [0 .. 100 %], 101 steps`).
  - `,s "#"` returns definitions with values.
- **Hash addressing**: `/#f50f69f8` is equivalent to `/ch/1/mute`.
- **Native passthrough**: a `,b` blob carries NRP tokens, either at root (`/ ,b d7beefaeabdc`) or relative
  (`/$ctl/daw ,b dd`). The reply blob contains the native response; the root def dump example is in §3.3.
- **Event subscription**: one subscriber at a time (the last requester wins), renewed every < 10 s. Each form takes an
  optional `/%port` prefix.
  - `/*b` sends native events as blobs.
  - `/*s` sends `,sff`/`,sfi`/`,s` triplets.
  - `/*S` sends single native-typed args (round-trippable).
- Address space: identical to the native tree (`ae_data` and `ce_data` of `.snap`), so T010's generated tree serves
  both.

OSC is not used by WING-Edit for discovery (no 2223 socket was open) [LIVE]. Whether it is used after connect is
open. OSC is mainly for third-party tools (Companion, TouchOSC, REAPER bridges) and T065.

---

## 6. Firmware/version expectations of WING-Edit 3.3.4

- Discovery accepts any firmware string [LIVE].
- At connect, WING-Edit can report "Incompatible Mixer Firmware! Please update to the latest version" [strings]. Users
  report that WING-Edit refuses to open a desk with older firmware.
- The parse involves `%d.%d.%d`, `-dirty` and `release` tokens. T003 must extract the minimum version and whether the
  branch suffix matters.
- Model string drives the UI surface: `ngc-full` / `wing-bk` → full WING, `wing-rack`, `wing-compact`. The simulator
  should advertise `ngc-full`, since our target is the full console (48 ch USB).

---

## 7. Live experiment status and the next capture to take

Done [LIVE]:
- Discovery probe capture.
- Fake-console replies accepted by WING-Edit (listed with name, IP and firmware).
- IP-field semantics.
- Model display.
- Firmware not checked at discovery.

Logs are in `tools/protocol-probe/captures/2026-10-25-wing-edit-3.3.4-discovery.txt`.

Not done: the **first TCP bytes WING-Edit sends**. WING-Edit did not auto-connect, and clicking CONNECT means
operating the user's live WING-Edit session. With "SYNC DIRECTION: MIXER → PC" selected, that could overwrite the
open mix, so it was intentionally not done unattended. To capture it (about 2 minutes, by a human):

1. In WING-Edit, save the current session (LOAD/SAVE), or set SYNC DIRECTION to PC → MIXER.
2. Run `python3 tools/protocol-probe/fake_wing.py --seconds 120 --tcp-mode silent | tee tools/protocol-probe/captures/tcp-silent.txt`.
3. Select WINGSIM in SELECT CONSOLE and click CONNECT. Wait for the timeout or error, then note any dialog text
   (firmware warning?).
4. Repeat with `--tcp-mode end`, which answers every `DC`/`DD` with `DE`, to see the next request in the sequence.
5. Repeat with `--fw` variants (`3.0.5-...`, `3.1.2-...`, `3.3.4-...`) to bracket the firmware check.

The capture script logs raw hexdumps and NRP-decoded per-channel payloads.

Note: our experiments left a stale "ALTNAME 10.9.9.9" entry in WING-Edit's console list. RESCAN clears it.

---

## 8. Reusable libraries and prior art

| Name | Lang | License | Coverage | Quality / notes |
|---|---|---|---|---|
| [libwing](https://github.com/dannydulai/libwing) (crates.io `libwing` 1.0.5) | Rust (+C FFI, Dart) | Apache-2.0 | Discovery, NRP client (escape decoding, all tokens), node-def parser, set/get, meters (UDP), keepalive; `propmap.rs`/`.jsonl` = **60,748 raw node definitions + ids from fw 3.0.5 Compact** | Best reference and **best reusable asset (schema data + codec test vectors)**. Client-only, blocking, `Arc<Mutex>`. Minor bugs: ignores `len.l`, strings >256 unhandled, no DF escaping in strings, meter id parsed as u16. Bundles the V3.0.5-1 PDF |
| [behringer-wing-typescript](https://github.com/matterlobby/behringer-wing-typescript) (npm `behringer-wing`) | TypeScript | MIT | Port of libwing (discovery, NRP, meters, propmap) | Useful as a second client for interop tests (Node) |
| [errere/behringer_wing_binary_data_interface_library](https://github.com/errere/behringer_wing_binary_data_interface_library) | C | MIT | `nrpc.c` escape codec + binary interface + test | Small; check against the Behringer sample routine |
| [errere/my_wing_binary_protocol_research](https://github.com/errere/my_wing_binary_protocol_research) | C | Unlicense | nrpc routine | Tiny |
| [pmaillot/wapi](https://github.com/pmaillot/wapi) → x32ram.com/wapi | C | closed binary lib (free download); source withdrawn over GPL concerns | Full native API, meters, tokens (`wapi.h`) | Reference only; do not copy |
| Maillot `wingemul` (sites.google.com/site/patrickmaillot/wing) | Windows exe | closed, free | WING **server** emulator (fw 3.1.1, 16 clients, fake meters, `WingEmulData.bin` state) | Very valuable as a **differential-test oracle** in a Windows VM: point WING-Edit or our client at it and capture. Not runnable on macOS directly |
| [bitfocus/companion-module-behringer-wing](https://github.com/bitfocus/companion-module-behringer-wing) | TypeScript | MIT | OSC client (actions, subscriptions) | Real-world OSC usage; interop test client for T014 |
| [ralfferreira/behringer-wing-mcp](https://github.com/ralfferreira/behringer-wing-mcp), [vanmobe/WINGuard](https://github.com/vanmobe/WINGuard) | TS / C++ | MIT | OSC clients | Interop only |
| [joelhooks/channel-strip](https://github.com/joelhooks/channel-strip) | Zig | unverified | `spike/fixtures/wing-tree-hardware.json` (3.6 MB hardware tree dump), protocol notes, wingemul bring-up notes | Data source to cross-check T003 (license unknown, so reference only) |
| [OpenMixerProject/OpenWING](https://github.com/OpenMixerProject/OpenWING) | C | GPL-3.0 | Custom Linux firmware for the WING hardware | No remote-protocol server; not useful for T002 |
| `rosc` (crates.io) | Rust | MIT/Apache-2.0 (verify) | Generic OSC 1.0 codec | Candidate for the OSC codec in `wing-proto` (T014); WING's subset is small enough to hand-roll too |

Recommendation:
- Hand-write the NRP codec in `crates/wing-proto`. It is about 300 lines.
- Use the Behringer sample routines as the normative spec and the PDF/libwing byte strings as test vectors.
- Import libwing's raw definition records (Apache-2.0, with NOTICE attribution) as the seed schema, keyed by hash,
  and reconcile with T003.
- Use libwing (`wingprop`, `wingmon`, `wingschema`) and behringer-wing-typescript as black-box clients for integration
  tests before WING-Edit is in the loop.

---

## 9. Open questions

For **T003** (WING-Edit binary analysis):
1. How does WING-Edit obtain node ids? Does it compute a hash (find the function: look for code building `D7` +
   u32), navigate by name/index tokens (`C0-CF`, `40-7F`), or learn ids from `DD` walks? This decides whether we must
   reproduce exact ids. We should anyway.
2. What is the exact firmware compatibility check: the minimum version, the parse of
   `X.Y.Z-n-gHASH[-dirty]:branch`, and whether `:release` vs other branches or `-dirty` matter? Also the meaning of the
   `ModelRequest` string.
3. Which channels does WING-Edit use (`DF D0`/`D2` "control engine"?) and what is its post-connect sequence: initial
   requests, sync (MIXER→PC) traversal strategy, and whether it uses node-level `DC` dumps?
4. Which value tokens does it send for sets (`D5` vs `D6` vs ints/strings)? Does it rely on an echo of its own sets?
5. Which meter request sets does it issue per view (V1 vs V2 types, RTA, source/output groups), with what report ids
   and renew interval? What packet sizes does it expect?
6. What are its keepalive interval and lost-connection timeout ("Lost connection! Auto reconnect...")?
7. Does it ever use OSC 2223 (for example `/*S`)?
8. The embedded "emulation" snapshot (`creator_sn: EMU`): is it a complete default state usable as the simulator's
   power-on state? Is there an offline/emulation mode whose internal tree we could dump?
9. How does it handle dynamic `mdl` subtree changes (re-requesting `DD` after a model change)?

For a live capture or hardware owner:
10. Does a real console echo a set back to the setting client? Does it require the UDP knock before TCP?
    *Partly answered (T083, no real-console capture):* WING-Edit 3.3.4 needs no echo for plain value sets (its
    push and DCA mute/solo buttons work without one), but it toggles strip mutes with `D8` (no `ch/N/mute` set ever
    appears in scratch/wing-sim-v0.4.log while the user clicked them and the audio muted) and redraws the button only
    from the console's reply; strips also carry a `$mute` indicator (int 0..2) that DCAs lack. The simulator echoes
    the result of every `D8`/`D9` and of value sets it adjusted (clamp, form change) to the setter, and drives
    `*/$mute`. Still open: the exact real-console behaviour.
11. What does a node-level `DC` response look like? What is the UDP source port of meter packets?
12. Discovery: does the real reply include a trailing NUL? Does WING-Edit require the reply's source port to be 2222?

---

## Sources

- *WING Remote Protocols* V3.0.5-1 (P.-G. Maillot, published with Behringer's authorization), bundled in libwing 1.0.5.
  Mirror: https://cdn.mediavalet.com/aunsw/musictribe/M7u1XZvRGkWxJAVwCKE5BA/7qQ-l43rn0SEnqyb5ZcETg/Original/WING%20Remote%20Protocols_V3.0.6-30.pdf
- *WING Remote Protocols* V3.1.2-a, Google Drive id `1-iptgd2Uxw4qPEbmegG2Sqccf8AbRRfk`, linked from
  https://sites.google.com/site/patrickmaillot/wing
- libwing 1.0.5 source (crates.io download), `Discovery.md`, `src/console.rs`, `src/node.rs`, `src/propmap.rs`.
- behringer-wing-typescript `src/wing.ts`. joelhooks/channel-strip `.brain/resources/wing-protocol-notes.svx`,
  `emulator/RUN.md`.
- Behringer WING-Edit 3.2.1 release notes (minimum-firmware coupling); behringer.world forum threads on meters
  (p=14622).
- Live observation: `tools/protocol-probe/` scripts and captures (2026-10-25).

## Firmware compatibility check

WING-Edit 3.3.4 rejects our fake console on CONNECT with "Incompatible Mixer
Firmware! / Please update to the latest version" because of a version gate in
the arm64 slice. Static analysis (otool/capstone, no execution of the app).

### The rule

The gate reads the device record's firmware string field (`[x19+0x178]`) and
checks ONLY the major and minor version characters of the string:

- character index 0 (major) must be `'3'` (0x33). `< '3'` fails; `> '3'` passes
  outright (skips the minor check).
- character index 2 (minor) must be `>= '1'` (0x31). `< '1'` fails.

So the accepted set is firmware **major.minor >= 3.1** (e.g. `3.1.x`, `3.3.4`).
`3.0.x` FAILS — index 2 is `'0'` (0x30) < `'1'`. That is exactly why our old
discovery fw `3.0.5-0-g0c2b9d4a:release` was rejected. The suffix (`-dirty`,
`:release`, git hash) is NOT part of the gate — it is only parsed by a separate
display/formatting routine that truncates the string at the first `-` for the
CONSOLE INFORMATION panel. The patch number (3rd component) is not checked.

### Which data is checked

The check runs inside the CONNECT handler (`0x10024d690` -> compat fn
`0x10024ede8`) and reads the firmware string already stored in the device
record — the 6th field of the discovery reply `WING,<ip>,<name>,<model>,
<serial>,<fw>`. It is evaluated before the session is established (the fn
returns early if already connected), so it is the **discovery fw field**, not
anything read over TCP after connecting.

### Evidence (addresses in the arm64 slice, base 0x100000000)

String `"Incompatible Mixer Firmware!"` at `0x1005ea157`,
`"Please update to the latest version"` at `0x1005ea174`.

Compat function `0x10024ede8`; the decision:

```
0x10024ef50  ldr   x0, [x19, #0x178]   ; x0 = firmware string (from device record)
0x10024ef54  ldrb  w8, [x0]            ; first codepoint (UTF-8 decoded inline)
...
0x10024f150  cmp   w9, #0x33           ; major char == '3' ?
0x10024f154  b.ne  #0x10024f3fc        ;   no  -> fail (Incompatible dialog)
0x10024f158  mov   w1, #2
0x10024f15c  bl    #0x1003d4294        ; char-at(index=2) -> minor codepoint
0x10024f160  cmp   w0, #0x31           ; minor char >= '1' ?
0x10024f164  b.lt  #0x10024f3fc        ;   no  -> fail
0x10024f168  ...                       ; pass: continue connect/sync
```

Fail path `0x10024f3fc` builds the two dialog strings from `0x1005ea157` /
`0x1005ea174` and shows the Incompatible message. `0x1003d4294` is a UTF-8
char-at(index) helper returning the codepoint. (An earlier formatting routine
near `0x10024e93c`/`0x10024e974` compares the string against `"-dirty"`
(`0x1005ea198`) and `"release"` (`0x1005ea19f`) purely to format the FIRMWARE
display field — it does not gate the connection.)

### Recommended fw string

Use a firmware whose major is `3` and minor `>= 1`, keeping the real WING
suffix format so the display routine is happy:

```
3.3.4-0-g0c2b9d4a:release
```

`tools/protocol-probe/fake_wing.py` default `--fw` has been updated to this
value. Confidence: high (the compare constants, string addresses, and fail-path
dialog strings are unambiguous). The only residual uncertainty is whether the
patch component or suffix matter in a later code path, but the gate itself does
not inspect them.
