# WING signal flow, tap points and routing (T004)

Status: research output for T004 (2026-10-25). Companion: `docs/research/models.md` (every model of every block).

Evidence levels used below (`[E/T003]` = confirmed in T003's schema):
- **[S]** = seen in `fixtures/PUBLIC.snap` (console fw 1.13.9) - exact key names and value formats.
- **[E]** = recovered from WING-Edit 3.3.4 parameter definitions (static analysis, see models.md section 0).
- **[D]** = public documentation / release notes / marketing (secondary, see Sources).
- **[I]** = inferred from the above; must be verified on a console or in the manual (T072 measurement tests).

The official WING manual PDF that was reachable from this session is image-only and the WING Remote Protocols PDF
returned HTTP 500, so the processing *order* inside strips is partly [I]. Each such point is listed in section 12.

## 1. Engine overview

```
                         +-------------------- 40 input channels (ch.1..40) ---------------+
 SOURCES (io.in.*)  ---> |  per channel: main src / alt src -> input -> filter -> Ins1 ->   |
 LCL 8  AUX 8            |  [G E D I2 in user order] -> mute/fader/pan -> sends            |
 A/B/C 48 each (AES50)   +------------------------------------------------------------------+
 SC 32 (StageConnect)    +-------------------- 8 aux channels (aux.1..8) -----------------+
 USB 48  CRD 64  MOD 64  |  input -> EQ / dyn / Ins (fixed order) -> mute/fader/pan -> sends |
 PLAY 4  AES 2           +------------------------------------------------------------------+
 USR 24 (user signals)          | main sends (4)            | bus sends (16)
 OSC 2 (oscillators)            v                           v
                         +---------------+    +----------------------------------+
                         | 4 MAIN busses |    | 16 busses (bus.1..16)            |
                         | (main.1..4)   |<---| Ins1 -> EQ -> Dyn -> Ins2 -> fdr  |
                         | Ins1 EQ Dyn   |    | sends: main 1-4, bus 1-8, MX 1-8 |
                         | Ins2 fdr dly  |    +----------------------------------+
                         +---------------+                   | MX sends
                                 | MX sends (8)              v
                                 +------------> +----------------------------------+
                                                | 8 matrices (mtx.1..8)            |
                                                | + 2 direct inputs each           |
                                                | Ins1 EQ Dyn Ins2 fdr out-delay   |
                                                +----------------------------------+
 OUTPUTS (io.out.<grp>.<n> = {grp, in}): LCL 8, AUX 8, A/B/C 48, SC 32, USB 48, CRD 64, MOD 64, REC 4, AES 2.
 Any output can take any source signal (pre-processing) or any bus/main/matrix/monitor output.
 FX rack: 16 stereo slots, used ONLY as inserts (a "send effect" = FX inserted on a bus).
```

Everything is stereo internally [D/S]: channels, aux, busses, mains, matrices and FX slots are stereo; a mono
source feeds both sides (`mode` M / ST / M-S on the source [S][E]). Counts [S]: 40 ch, 8 aux, 16 bus, 4 main,
8 mtx, 16 DCA, 8 mute groups, 16 FX slots, 2 monitor outputs (A/B), 2 talkback paths (A/B).

## 2. Input channel strip (ch.1..40)

```
 io.in.<grp>.<n>  (source: g preamp gain -3..45.5 dB, vph, pol, mute, mode M/ST/M-S, name...)     [S][E]
   |  main: in.conn.{grp,in}       alt: in.conn.{altgrp,altin}                                   [S]
   |  select = in.set.altsrc OR (in.set.srcauto AND io.altsw)   (global ALT switch, see 6)         [S][E][I]
   v
 +-- INPUT (in.set) ---------------------------------------------------------------------------+
 | trim -18..+18 dB -> balance -9..+9 dB (stereo) -> polarity inv -> input delay                 |
 | (dlyon, dly: M 0.1..150 m | FT ..500 ft | MS 0.5..500 ms | SMP 16..500)                                       [S][E]          |
 +--------------------------------------------------------------------------------- TAP "IN" --+
   v
 +-- FILTER (flt) -----------------------------------------------------------------------------+
 | low cut (lc, lcf, lcs 6/12/18/24) -> high cut (hc, hcf, hcs 6/12) -> filter tool (tf, mdl     |
 | TILT/MAX/AP1/AP2)                                                       [S][E] (order [I])   |
 +------------------------------------------------------------------------------- TAP "FILT" --+
   v
 +-- INSERT 1 (preins: on, ins NONE|FX1..16) ---------------------------------------- [S][E][I]
   v
 +-- PROCESS CHAIN: 4 blocks in the order given by `proc` (24 permutations of G,E,D,I) ---------+
 |   default "GEDI": G = gate slot (gate, gatesc)                                               |
 |                   E = EQ slot (peq "PRE EQ" 3-band + eq model, eq.mix)                        |
 |                   D = dynamics slot (dyn, dynsc, dynxo, dyn.mix, dyn.gain)                     |
 |                   I = INSERT 2 (postins: on, mode FX|AUTO_X|AUTO_Y, ins, w)                    |
 |   fixture uses GEDI (36x), EDIG (3x), IGED (1x)                                    [S][E]   |
 |                                                                                            |
 |   block1 --TAP "3"--> block2 --TAP "4"--> block3 --TAP "5"--> block4 --TAP "PFL"-->           |
 +--------------------------------------------------------------------------------------------+
   v
 mute (mute, mute groups, DCA mutes) -> fader (fdr dB, + DCA offsets) ---------- TAP "AFL"
   v
 pan (-100..100) / width (wid -150..150 %) --------------------------------------- TAP "POST"
   v
 +-> main sends  main.1..4 {on, lvl dB, pre}            (pre = take signal at the channel tap) [S]
 +-> bus sends   send.1..16 {on, lvl, mode PRE|POST|GRP, pon, ind, plink, pan, wid}           [S][E]
 +-> solo bus (cfg.solo.chtap), meters, RTA, direct outs via output patch / user signals
```

- `ptap` (UI "TAP POINT") [S][E] enum (long names from T003): `IN` INPUT, `FILT` FILTER, `3` TAP 3, `4` TAP 4,
  `5` TAP 5, `PFL` PRE FDR, `AFL` POST FDR, `POST` POST PROC; factory default `5`. It is the channel's tap used by
  sends in PRE mode (and main sends with `pre`=true) [I]. Meaning of 3/4/5 [I]: positions are numbered 1 = IN,
  2 = FILT, 3/4/5 = after the 1st/2nd/3rd block of the `proc` chain, PFL = after the 4th block (= pre fader),
  AFL = post fader. `POST` is labelled "POST PROC" - placed last in the list, so most likely post fader + pan
  (the user-signal tap uses the same wording: PRE = "PRE FADER", POST = "POST PROCESSING") [I]. Fixture: `4`
  (26x), `5` (7x), PFL, POST, AFL. The **same 8-value enum** is used for gate/dynamics side-chain taps
  (`gatesc.tap`, `dynsc.tap`) [E/T003].
- `proc` order strings [E]: GEDI GEID GIED IGED GDEI GDIE GIDE IGDE EGDI EGID EIGD IEGD EDGI EDIG EIDG IEDG DEGI DEIG
  DIEG IDEG DGEI DGIE DIGE IDGE.
- Since fw 3.1 the G slot and D slot each accept any gate or dynamics model [D] - the slot names stay but the
  processor type is free (two compressors in series possible).
- `peq` (3 bands `1..3` g/f/q + `on`) [S] is the "PRE EQ" [E label]; placement assumed directly before the EQ model
  inside the E block [I]. It lets a vintage EQ model be combined with surgical parametric bands.
- EQ `mix` 0..125 % (above 100 % exaggerates the EQ) and dynamics `mix` 0..100 % [S][E/T003]; dynamics `gain`
  -6..+12 dB = make-up after the dynamics model [I].
- Side-chains [S][E]:
  - `gatesc` = gate key: `type` OFF/LP12/HP12/BP/NOTCH, `f`, `q` 0.44..10, `src` (SELF or any CH.n/AUX.n/BUS.n/
    MAIN.n/MTX.n), `tap` (IN, ... where on the key source the key is taken; fixture: IN), `$solo` (listen).
  - `dynsc` = dynamics key, same fields (fixture: SELF/IN on channels, SELF/BUS on busses, external BUS.5, CH.15).
  - `dynxo` = dynamics crossover ("DYN XOVER"): 3.x `type` OFF (FLAT) / LO / HI / S6 / S12 / PEAK, `f` 20..20k Hz,
    `depth` -30..+30 dB (default 6) [E/T003]; fw 1.13 fixture stores `HI6`, `LO6`, `LO12` (band+slope in one token)
    [S]. Frequency-selective dynamics: the dynamics gain is applied only to the band selected by the crossover
    (low or high split with 6/12 dB slope, or a peak band); `depth` scales/limits that band's change [I].
- Pan -100..+100 (201 steps, centre detent), width -150..+150 % (61 steps) [E/T003]. Pan law not recovered; assume
  constant-power (-3 dB centre) [I] - verify by measurement. Fader law (T003, `CparLevel`): for position p in
  [0,1]: p<1/1024 -> -oo; p<0.0625: 480p-90; p<0.25: 160p-70; p<0.5: 80p-50; else 40p-30 dB (max +10 dB).

## 3. Aux channel strip (aux.1..8)

```
 source (main/alt, srcauto as ch) -> in.set {trim, bal, inv} -> INSERT (preins) -> EQ (4-band STD or EQ table B
 model, mix) -> DYN (fw<=3.0: fixed PSE/LA combo, keys thr/depth/fast/peak/ingain/cpeak/cmode with no mdl;
 fw 3.1: model choice) --TAP PRE (PRE sends, main pre, PFL solo)--> mute -> fader --TAP AFL (POST/GRP sends,
 AFL solo)--> pan/width --TAP POST--> main sends 1..4 {on,lvl,pre} + bus sends 1..16
```
[S] for keys; block order [I] (aux has no `proc`, no filter, no gate, no `ptap`; marketing: "single FX insert
slot, expander/limiter and 4-band EQ" [D]). Aux sends: same object as channel sends. With no `ptap`, PRE sends are
assumed to tap pre-fader (post-processing) [I].

## 4. Bus strip (bus.1..16)

```
 sum of: channel/aux sends (mode per bus), bus->bus sends from other busses (into busses 1-8 only)
   v
 in.set {trim, bal, inv}  [S]
   v
 INSERT 1 (preins) -> EQ (STD-bus: L + 6 bands + H + tilt, or EQ table C model; mix)
   -> DYN (any compressor-family/gate model with slot mask; mix, gain, dynsc key: SELF/BUS taps or external,
      dynxo crossover) -> INSERT 2 (postins {on, ins})                                 [S] keys, order [I]
   v                                          ---- TAP "PRE" (PFL solo, sends with pre=true, user-signal PRE) [I]
 mute -> fader ------------------------------------- TAP "AFL"/post-fader (AFL solo, sends with pre=false) [I]
   v
 pan / width (busmono = sum to mono) -------------- TAP "POST" (bus output to output patch, meters POST)   [S][I]
   v
 +-> main sends  main.1..4 {on, lvl, pre}             [S]
 +-> bus sends   send.1..8 {on, lvl, pre}  (bus -> bus 1..8)   [S][I]
 +-> matrix sends send.MX1..MX8 {on, lvl, pre}        [S]
```
- `busmode` PRE / POST / GRP [S]: the send mode of every channel/aux send *into* this bus. In the fixture every
  channel's `send.<n>.mode` equals `bus.<n>.busmode` (bus 1-4 POST, 5-7 GRP, 8-16 PRE), so the bus mode drives the
  per-send mode [S][I]. Semantics [I]:
  - PRE: send taken at the channel tap (`ptap`), independent of channel fader; typical monitor mixes.
  - POST: post channel fader (and mute), pre or post pan per `plink`; typical FX sends.
  - GRP: subgroup: post fader + post pan, send `on` acts as the bus assignment, channel is usually removed from the
    mains manually; send level still exists in the snapshot.
- `plink` (PAN LINK, 0..2) [E]: send pan independent / linked to channel pan (exact 3 states unknown) [I]. `pon`
  (PRE ON) and `ind` semantics unknown (fixture almost always false) [S].
- **Bus->bus** [S]: fixture busses 1 and 2 (FX returns VOX VERB/VOX DLY) send to bus 5 (VOX, GRP), 3->7, 4->6, 6->7.
  All observed sends go to a higher-numbered bus; whether the console forbids lower-numbered targets (loop
  prevention) is unknown [I]. The engine must compute busses in dependency order and reject cycles.
- Send effects: the fixture's FX returns are busses with the FX inserted (`bus.1.preins.ins = FX1` = BPLATE,
  fxmix 100) [S]. There are no FX return channels; an FX slot is always an insert (section 8).

## 5. Main strip (main.1..4) and matrix strip (mtx.1..8)

```
 MAIN n:  sum of ch/aux/bus main sends -> in.set {trim, bal, inv} -> Ins1 -> EQ (table C) -> DYN -> Ins2
          --TAP PRE (PFL solo, MX sends pre=true)--> mute -> fader --TAP AFL (AFL solo, MX sends pre=false)-->
          pan/width/busmono -> OUT DELAY (dly) --TAP POST--> output patch (MAIN 2k-1/2k), meters

 MTX n:   sum of bus/main MX sends + direct inputs dir.1, dir.2 {on, lvl, inv, in, tap}
          -> in.set {trim, bal, inv} -> Ins1 -> EQ -> DYN -> Ins2 --TAP PRE (PFL solo; fixture mtxtap PFL)-->
          mute -> fader --TAP AFL--> pan/width -> OUT DELAY --TAP POST--> output patch (MTX 2k-1/2k)
```
[S] keys; block order [I]. Direct inputs `dir.<1|2>.in`: OFF, AES, MON.PH, MON.SPK, MON.BUS (list possibly
truncated) [E]; `tap` PRE (fixture) [S]. Output delay: fw 1.13 `dly {on, m}` (fixture main 0.1, mtx 5) vs WING-Edit
3.3.4 `dly {on, mode M/FT/MS/SMP, dly 0..100}` [S][E] - snapshot loader must map. `cfg.mainlink` OFF | 2 | 2-3 | 2-4 ("1 > 2", "1 > 2-3", "1 > 2-4": main 1 controls
mains 2..4) [S][E/T003].

Fixture example: main.1 "TO FOH" Ins1 = FX12 (C5-CMB) -> F670 dyn with dynxo HI6; main.4 "STREAM" Ins1 = FX5
(HALL) and Ins2 = FX11 (*MASTER*); mtx.1 "FOH" Ins1 = FX16 (GEQ).

## 6. Input routing, main/alt sources, USB

- Sources [S][E]: groups `LCL` 8 (local preamps), `AUX` 8 (local aux line ins), `A`/`B`/`C` 48 each (AES50 ports),
  `SC` 32 (StageConnect), `USB` 48 (USB audio from the computer), `CRD` 64 (expansion card), `MOD` 64 (module),
  `PLAY` 4 (USB player), `AES` 2, `USR` 24 (user signals), `OSC` 2 (oscillators: `osc.lvl`, `osc.mode`
  SINE/PINK/WHITE, `osc.f`). A channel's `conn.grp` enum additionally allows `BUS`, `MAIN`, `MTX` (re-input of mix
  outputs) [E].
- Source objects carry preamp gain `g`, phantom `vph`, polarity `pol`, mute, name/colour/icon, `mode` M/ST(/M-S).
  Stereo sources occupy two consecutive indices both flagged ST (fixture: AUX.1/2 "PC" ST, AUX.3/4 "SPOTIFY" ST);
  a channel connected to the first index of an ST pair is stereo [S][I].
- Alt source: each ch/aux has `conn.altgrp/altin`. Effective source = alt when `in.set.altsrc` (per channel ALT
  INPUT) is on, or when `in.set.srcauto` (ALT AUTOSW) is on and the global switch `io.altsw` (ALT INPUT) is on
  [S][E][I]. `cfg ... autoaltovr` "GLOBAL INPUT SELECT OVERRIDE" exists [E]. Fixture: ch1-4 main A.1-4 (stage AES50)
  with alt USB.1-4 (DAW playback = virtual soundcheck), `srcauto` true, `io.altsw` false.
- **USB**: `ae_globals.usbacfg` "48/48" = USB audio channel configuration (in/out) [S]. USB *inputs* to the console
  are the `USB` source group; USB *outputs* to the computer are `io.out.USB.1..48`, each patched from any signal:
  fixture USB.1 <- A.1 (raw input = recording split), USB.45 <- MTX.3. For the simulator this group is the
  48-channel CoreAudio device (DAW->WING = USB source group, WING->DAW = USB outputs).
- **Output patch** `io.out.<grp>.<n> = {grp, in}` [S]: `grp` = OFF, any source group (LCL, AUX, A, B, C, SC, USB,
  CRD, MOD, PLAY, AES, USR, OSC = direct, pre-processing) or mix outputs BUS, MAIN, MTX, SEND ("FX SEND"), MON
  [E/T003]. `SEND` ("FX SEND") is an output source whose meaning is not yet clear (possibly the FX-slot input feed
  for the EXT model) [I]. Mix outputs are addressed as mono legs:
  `in` = 2*k-1 / 2*k for the L/R of stereo bus k [I] (fixture: LCL out 1 <- BUS 15 = bus 8 L "VOX 1 IEM";
  AUX out 1/2 <- MAIN 7/8 = main 4 L/R "STREAM"; A.23/24 <- MTX 1/2 = mtx 1 L/R "FOH").
- **User signals** `io.in.USR.<n>.user = {grp OFF|CH|AUX|BUS|MAIN|MTX, in, tap PRE ("PRE FADER") | POST ("POST
  PROCESSING"), lr L+R (mono sum) | L | R}` [S][E/T003] turn any strip's pre-fader or post-processing signal into a
  routable source (e.g. post-fader channel direct outs to USB).
- Output groups [S]: LCL 8, AUX 8, A/B/C 48, SC 32, USB 48, CRD 64, MOD 64, REC 4 (USB/SD recorder), AES 2.

## 7. Send taps summary

| Sender | Destinations | Modes / taps |
|---|---|---|
| ch | main 1-4 (`on`,`lvl`,`pre`), bus 1-16 | PRE (= `ptap`), POST, GRP (= bus mode) |
| aux | main 1-4, bus 1-16 | as ch (PRE tap fixed pre-fader [I]) |
| bus | main 1-4, bus 1-8, mtx 1-8 | `pre` flag per send (pre/post bus fader) |
| main | mtx 1-8 | `pre` flag |
| mtx | (outputs only) + 2 direct inputs in | dir tap PRE/... |

Send level `lvl` in dB (-144 = -oo .. +10), fader law shared with faders [S][E].

## 8. FX rack

- 16 stereo slots `fx.1..16 = {mdl, fxmix, params}` [S]. A slot is engaged by assigning it to exactly one insert
  point (`preins.ins` / `postins.ins` = `FXn`) of a ch/aux/bus/main/mtx [S]. Inserts are stereo in/out.
- "Send" effects are made by inserting the FX into a bus (fixture busses 1-4) and returning the bus to the mains
  (and bus->bus into groups) [S]. "Insert" effects: FX in a channel/main insert with `fxmix` as wet/dry.
- Premium slots: models of slot class 3 (all reverbs, delays, D-PITCH, SUB) are restricted to FX1-FX8 [E][D][I];
  fixture places HALL/V-PLATE/BPLATE/TAPE-DL only in FX1-7, consistent.
- Insert `$stat` OK / N/A reports whether the slot can run [E]. Channel post-insert can instead route to the
  automixer (AUTO_X / AUTO_Y, weight `w` -12..+12 dB; groups enabled by `cfg.amix.x/y`) [S][E].
- EXT model: the slot becomes an external hardware loop: send via the slot's output patch, return from `egrp/ein`,
  with latency compensation `lat` [E].
- FX processing order in the engine: an FX slot runs inline in its host strip; a bus-inserted reverb therefore
  sits between bus sum and bus EQ (Ins1) or after dynamics (Ins2).

## 9. DCA and mute groups

- 16 DCAs `dca.<n> = {name, col, icon, led, mute, fdr, mon}` and 8 mute groups `mgrp.<n> = {name, mute}` [S].
- Membership is stored in each strip's `tags` string [S]: `#Dn` = member of DCA n, `#Mn` = member of mute group n
  (fixture: ch tags "#M1,DCA.1", bus "#D5,#D7"; plain words such as `DCA.1`, `INSTRUM` are free user tags) [S][I].
- DCA: member effective fader (dB) = own fader + sum of its DCA faders (X32 semantics, [I]); DCA mute mutes members;
  `cfg.dcamgrp` ("DCA MUTEGROUPS") makes DCA mute buttons behave as mute groups [E][I].
- Mute group: effective mute = own mute OR any active mute group it belongs to OR any muted DCA.
  `cfg muteovr` "MUTEGROUP/SIP OVERRIDE" exists [E].

## 10. Solo and monitoring

- `cfg.solo = {mode LIVE|STUDIO|SIP (IN-PLACE), mon PH|SPK|PH+SPK (fw 1.13 fixture: "A"), mute, chtap, bustap,
  maintap, mtxtap (each PFL "PRE FDR" | AFL "POST FDR"), srcsolo OFF|CH39|AUX7}` [S][E/T003]: per strip-type solo
  tap (fixture: ch/bus/main AFL, mtx PFL); `mon` = which monitor path solo goes to; strip `mon` A/B/A+B chooses monitor bus per strip; `solosafe` exempts a strip from
  solo-in-place muting [S][I]. LIVE = solo does not affect mix; STUDIO/IN-PLACE (SIP) mutes non-soloed strips [I].
  Exclusive solo / solo-follows-select are control-surface prefs (`ce_data.cfg.soloexcl/selfsolo`) [S].
- Monitor outputs `cfg.mon.1|2` (= PH phones / SPK speakers, inferred from solo `mon` values and the strings
  "MON.PH", "MON.SPK") [S][E]; `src` options OFF, MAIN.1-4, MTX.1-8, BUS.1-16, AUX.1-8 [T003]:
  `src` (e.g. MAIN.1), `srclvl`, `srcmix`, 8-band EQ (`lsg/lsf`, 1-6 `g/f/q`, `hsg/hsf`), `lim` limiter threshold,
  `dly {on, m}` (m = distance 0.1..100 m), `dim`, `pfldim` (PFL attenuation), `eqbdtrim`, `inv`, `pan`, `wid`. Solo bus replaces/mixes into the
  monitor source while any solo is active [I].
- Talkback `cfg.talk.A|B = {mode AUTO|PUSH|LATCH, mondim, busdim, B1..B16, M1..M4}`, `assign` (source, fixture
  CH40), `indiv` per-destination levels [S][E].
- RTA/meters: `cfg.rta` and `cfg.mtr` select sources/taps (IN, PRE/POST etc.) - metering only [S].

## 11. Sample rate, clock, latency

- `ae_globals.clkrate` 48000 in fixture; editor offers 44.1 kHz / 48 kHz ("44K1", "48K") [S][E]. `clksrc` INT
  (or external AES50/card) [S]. WING processes at the console sample rate; no internal oversampling is visible in
  the editor (emulations needing it must oversample internally).
- Console latency figures were not obtainable from the manual in this session (**verify**; Behringer marketing
  quotes sub-millisecond local-in to local-out). Delay-compensation: WING does not auto-align parallel paths through
  inserts [I]; the EXT model has manual `lat` compensation [E].
- Simulator (ARCHITECTURE.md): engine adds one block of latency; FX/look-ahead models (precision limiter, pitch,
  PCORR) add their own internal latency that the console presumably does not compensate either [I].

## 12. Engine DAG implied by the above

```
 sources (USB in, oscillators, players, re-inputs from previous block for BUS/MAIN/MTX sources)
   -> 40 ch + 8 aux strips (parallel; each runs its own inserted FX slots inline; automix groups X/Y need a
      cross-channel gain stage before the channel post-insert completes)
   -> 16 bus sums -> bus strips in dependency order (bus->bus sends 1-8; busses with external side-chain keys
      from other busses also impose order)
   -> 4 main sums -> main strips -> 8 mtx sums (+direct inputs) -> mtx strips
   -> monitor/solo, user signals, output patch (USB outs 1..48 -> CoreAudio; e.g. USB 45/46 -> host device)
```
Side-chain keys can point at any strip (`gatesc.src`/`dynsc.src` = CH.n/AUX.n/BUS.n/MAIN.n/MTX.n). A key from a
strip computed later than the keyed strip needs one block of delay (or a reordering pass); cycles are possible
(ch keyed by a bus it feeds) and must be broken with a one-block delay [I].

## 13. Open questions (to verify on a console / in the manual)

1. Exact position of Insert 1 (pre-insert): after filter (assumed) or before filter.
2. Placement of `peq` (PRE EQ) relative to the EQ model, and whether `eq.mix` covers it.
3. Meaning of `ptap` 3/4/5 (assumed: after process blocks 1/2/3) and whether PRE sends are post-mute.
4. Aux, bus, main, matrix block order (no `proc` key; assumed Ins1 -> EQ -> Dyn -> Ins2).
5. `send.pon`, `send.ind`, `plink` states; GRP mode exact behaviour (pan, level).
6. Bus->bus allowed targets (only 1-8 seen as keys; only higher-numbered targets used).
7. Pan/width laws, `busmono`, fader law table (0..1023 positions <-> dB).
8. Monitor 1/2 = phones/speakers mapping; solo modes list; dim semantics.
9. Output delay key change (`m` in fw 1.13 vs `mode`/`dly` in WING-Edit 3.3.4); `lcs`/`hcs` defaults when absent;
   dynxo `HI6`-style tokens (fw 1.13) vs `type` LO/HI/S6/S12 (3.x).
10. Console processing latency figures.

## Sources

- `fixtures/PUBLIC.snap` (WING fw 1.13.9-0-g2d999d50, snapshot.8).
- T003 output `data/schema/wing-tree.json` / `docs/research/wing-edit-schema.md` (enum long names, ranges,
  factory defaults from the fw 3.0.6 factory scene) - cross-checked 2026-10-25.
- WING-Edit 3.3.4 for macOS (`ae::Cchannel`, `ae::Cchsend`, `ae::Cinsert`, `ae::Cdynsidechain`, `ae::Cchfilter`,
  `ae::Cfxunit` parameter definitions; static analysis only).
- [Drew Brashler - WING firmware 3.1 dynamics update](https://drewbrashler.com/2025/behringer-wing-firmware-3-1-dynamics-update/)
  and [newsletter](https://newsletter.drewbrashler.com/p/the-wing-just-got-a-massive-free-dynamics-upgrade).
- Behringer product/marketing copy as summarised by web search (B&H/retailer listings: "5 variable plug-in slots",
  "dual inserts, 8-band PEQ, stereo imager and full dynamics" on busses; aux "single FX insert slot,
  expander/limiter and 4-band EQ").
- Attempted but unavailable: WING user manual PDF (image-only via adorama/manualslib; mediavalet HTTP 500),
  "WING Remote Protocols V3.0.6" (P.-G. Maillot, mediavalet HTTP 500), WING fw 3.0.5 release notes PDF (HTTP 404).
