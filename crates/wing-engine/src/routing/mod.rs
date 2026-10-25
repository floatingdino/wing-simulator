//! WING I/O routing: source address space, channel main/alt source selection, output patching.
//!
//! # Model
//!
//! * **Sources** ([`SrcGroup`], [`SourceAddr`]): every group the schema defines under `io/in`,
//!   addressed as mono legs `GRP.n` (flat index `0..SOURCE_COUNT`):
//!
//!   | group | legs | in the simulator |
//!   |---|---|---|
//!   | `USB` | 48 | **live**: driver `USB In n` = REAPER hardware out n = engine external input n-1 |
//!   | `OSC` | 2 | **live**: internal [`Oscillators`] (sine / pink / white, `osc.lvl` dBFS RMS, `osc.f`) |
//!   | `USR` 1..24 | 24 | derived: strip tap (`user.grp` CH/AUX/BUS/MAIN/MTX, `in`, `tap` PRE/POST, `lr` L+R/L/R) |
//!   | `USR` 25..56 | 32 | derived: alias of another source (`user.grp` LCL..AES, `in`) |
//!   | `BUS` `MAIN` `MTX` `SEND` `MON` | 32/8/16/32/4 | derived: mix output legs (2k-1/2k = L/R of strip k) |
//!   | `LCL` `AUX` `A` `B` `C` `SC` `CRD` `MOD` `PLAY` `AES` | 24/8/48/48/48/32/64/64/4/2 | silent but routable |
//!
//!   Derived signals resolve to a [`Tap`] (dense [`TapId`]); the mix graph
//!   ([`crate::mix::build_mix_graph`]) wires a `Port` per tap ([`TapPorts`]); SEND/MON legs read as
//!   silence until the FX rack / monitor section exist. Taps used as *channel sources* are
//!   feedback paths and arrive one block late ([`feedback`]).
//! * **Source settings** (`io/in/<grp>/<n>`): preamp gain `g` (LCL/A/B/C/SC) applies as linear
//!   gain, `mute` zeroes the leg, `pol` inverts it, `mode` M/ST/M-S sets the width of a channel
//!   connected to that leg (ST/M-S: the odd-aligned pair containing n, so `AUX.4` of a stereo
//!   `AUX.3/4` gives AUX.3 + AUX.4 [I]; M-S is decoded L = M+S, R = M-S). Phantom
//!   (`vph`), remote HA, names/colours are ignored.
//! * **Input strips** (ch 1..40, aux 1..8): `in.conn.{grp,in}` (main) and
//!   `in.conn.{altgrp,altin}` (alt). The alt source is used when `in.set.altsrc` is on, or when
//!   `in.set.srcauto` is on and the global `io/altsw` (ALT INPUT) is on. The usual
//!   virtual-soundcheck setup (and the user's REAPER session) feeds the desk through the alt
//!   sources (`USB.n`) with `srcauto` set, so flipping `io/altsw` swaps the stage inputs for
//!   REAPER playback. `io/autoaltovr` is not modelled. A mix output used as a channel source
//!   (e.g. `BUS.1`) is always the stereo pair (bus k = legs 2k-1/2k) [I].
//! * **Outputs** (`io/out/<grp>/<n>.{grp,in}`, [`OutGroup`]): every output group is resolved
//!   (LCL 8, AUX 8, A/B/C 48, SC 32, USB 48, CRD 64, MOD 64, REC 4, AES 2) to a [`Leg`]; a
//!   source group gives the pre-processing source (with its gain/mute/polarity), BUS/MAIN/MTX/
//!   SEND/MON give mix taps. Only `USB` outputs leave the simulator: [`UsbOutRouter`] renders
//!   them to the driver's `USB Out 1..48` (REAPER inputs).
//!
//! # Threading
//!
//! [`RoutingControl`] (control thread) owns a [`RoutingBuilder`] that resolves every routing
//! parameter of the [`wing_model::ParamStore`] once, a dirty flag raised by a store hook, and a
//! [`TableSender`]. `refresh` rebuilds the immutable [`RoutingTable`] and publishes it; each RT
//! processor holds a [`TableReader`] and adopts the new table at the start of its next block
//! without allocating or freeing (see [`swap`]). Route changes are not crossfaded.

mod addr;
pub mod feedback;
mod nodes;
pub mod swap;
mod table;

pub use addr::{
    Liveness, MixGroup, OutAddr, OutGroup, Side, SourceAddr, SrcGroup, Strip, StripPoint, Tap,
    TapId, OUTPUT_COUNT, SOURCE_COUNT, STRIP_COUNT, TAP_COUNT,
};
pub use feedback::{FeedbackWriter, TapFeedback};
pub use nodes::{
    add_input_stage, add_input_stage_with_feedback, add_usb_output, InputRouter, IoNodes,
    Oscillators, TapPorts, UsbOutRouter, NO_TAPS,
};
pub use swap::{TableReader, TableSender};
pub use table::{
    Feed, InputRoute, Leg, OscMode, OscParams, RoutingBuilder, RoutingControl, RoutingTable, Width,
    INPUT_STRIPS, USB_CHANNELS,
};

/// RT-side reader of routing tables.
pub type RoutingReader = TableReader<RoutingTable>;
