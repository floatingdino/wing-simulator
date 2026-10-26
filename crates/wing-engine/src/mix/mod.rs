//! Mixing (T036): fader/mute/pan/width, sends with tap points and modes, 16 busses, 4 mains,
//! 8 matrices, summing, and the complete engine graph ([`build_mix_graph`]).
//!
//! # Signal flow per strip
//!
//! ```text
//! ch/aux:  router -> ChannelStrip (T035) -PFL-> mute*fader -AFL-> width,pan -POST->
//!          PRE  = ptap signal * mute, `tapwid` (aux: pre-fader)       PRE sends
//!          PRE' = PRE through the channel pan                       PRE sends w/ pan link, main `pre`
//!          AFL                                                      POST sends without pan link
//!          POST                                                     main sends, GRP, POST w/ pan link
//! bus/main/mtx: sum of sends -> StripHook (T037 [`MixStrip`]: trim, Ins1, EQ, dyn, Ins2) -PFL->
//!          mute*fader, width, pan, busmono -> StripHook::process_post (output delay) -POST->
//!          PRE  = PFL * mute                                        sends with `pre`
//!          POST                                                     sends without `pre`, outputs
//! ```
//!
//! | sender | into | parameters | taken from |
//! |---|---|---|---|
//! | ch/aux | main 1-4 | `on lvl pre` | `pre` ? PRE' : POST |
//! | ch/aux | bus 1-16, mtx 1-8 (`MXn`) | `on lvl mode plink pan` | PRE: link ? PRE' : PRE + send pan; POST: link ? POST : AFL + send pan; GRP: POST at unity (`lvl` ignored, [I]) |
//! | bus | main 1-4, bus 1-16, mtx 1-8 | `on lvl pre` | `pre` ? PRE : POST |
//! | main | mtx 1-8 | `on lvl pre` | `pre` ? PRE : POST |
//!
//! * **Bus modes**: WING 1.x stored a per-bus `busmode` (PRE/POST/GRP) plus a per-send `ind`
//!   (independent) flag; 3.x stores the effective `mode` per send and wing-model's snapshot
//!   migration resolves `busmode`/`ind` into it (snapshot-migration.md R1/R10), so the mix only
//!   reads `send/*/mode`. `pon` (PRE ON) is not modelled [I].
//! * **Laws** ([`law`]): fader `10^(dB/20)` with -144 dB = -oo (the position taper lives in
//!   wing-model); width = mid/side scaling; pan law configurable per strip type
//!   ([`MixConfig`]), defaults constant-power -3 dB for channels and sends, balance (0 dB centre)
//!   for bus/main/matrix [I]. `busmono` sums the strip to mono after pan [I].
//! * **Mute** zeroes AFL/POST and the PRE taps (sends are post-mute, [I]). External mutes and
//!   DCA offsets (T038) arrive as [`param::EXT_MUTE`]/[`param::EXT_GAIN`]; [`param::MUTE_SAFE`]
//!   exempts a strip from the external mute.
//! * **Smoothing**: every gain path (fader, mute, pan, width, each send) is a 2x2 matrix ramped
//!   with a 5 ms one-pole per block and linear interpolation inside the block ([`law::Ramp`]).
//!   Mode changes crossfade (each tap signal has its own ramp).
//! * **Bus -> bus** sends feed only higher-numbered busses (loop prevention [I]). The graph
//!   computes busses in *tiers* ([`BusOrder`]): by default from the scene's active bus->bus sends
//!   at build time, so independent busses run in parallel. A bus reads every lower-numbered bus of
//!   a lower tier; a send that does not fit the built order is ignored until the graph is rebuilt
//!   ([`MixGraph::bus_tiers`]).
//! * **DCAs, mute groups, solo and monitors** (T038): [`logic`] resolves `dca`, `mgrp`, strip
//!   `tags` and `cfg/solo` on the control thread into per-strip external gain / mute / solo flags
//!   ([`MixShared`], polled by every strip node once per block); [`monitor`] is the monitor
//!   section node (solo busses with priority over the monitor source, `cfg/mon/1|2`), whose
//!   outputs are the `MON` legs 1..4.
//! * **Matrices** take ch/aux `MXn` sends, bus and main sends. Matrix direct inputs (`mtx/*/dir`:
//!   AES / monitor) have no source in the simulator yet.
//! * **Inserts / FX rack** (T050/T044, [`crate::fx`]): every strip's Ins1/Ins2 run their FX slot inline
//!   through the rack handle the builder installs; the rack node after the matrices publishes the FX
//!   SEND legs and sums the automix groups.
//! * **Side-chain keys** (T080, [`keys`]): every gate/dynamics slot's `src`/`tap` is resolved through a
//!   shared key bank (one block late, any strip may key any other, cycles included); key listen
//!   (`gatesc`/`dynsc` `$solo`) plays the slot's key on the solo bus.
//! * Mix outputs (BUS/MAIN/MTX legs), strip taps (user signals PRE FADER / POST PROCESSING) feed
//!   the USB output router directly and the channel sources (re-inputs) one block late
//!   ([`crate::routing::feedback`]).

pub mod keys;
pub mod law;
pub mod logic;
pub mod monitor;
mod nodes;

mod build;
pub mod strip;

pub use build::{bind_mix_params, build_mix_graph, bus_tiers, MixError, MixGraph, MixNodes};
pub use law::PanLaw;
pub use logic::{LogicState, MixLogic, MixShared, SoloMode};
pub use monitor::{MonitorLevels, MonitorNode};
pub use nodes::{
    InputSignal, InputStripNode, MixSignal, MixStripNode, NoProcessing, SendMode, SendParams,
    SlotKind, SlotSpec, StripHook, INPUT_OUTPUTS, MIX_OUTPUTS,
};
pub use strip::{bind_mix_strip, MixStrip, MixStripTelemetry, MixTap};

use std::sync::Arc;

use crate::routing::Strip;

/// Global source index of every strip that can send: `0..40` ch, `40..48` aux, `48..64` bus,
/// `64..68` main.
pub const SRC_COUNT: usize = 68;

/// Source index of a sending strip (`None` for matrices or out of range).
pub fn source_index(s: Strip) -> Option<u8> {
    let i = s.index()?;
    (i < SRC_COUNT).then_some(i as u8)
}

/// Parameter block ids.
pub mod block {
    /// Fader section of an input strip node (after the [`crate::channel::block`] ids).
    pub const INPUT_MIX: u8 = 32;
    /// Fader section of a bus/main/matrix node.
    pub const OWN: u8 = 0;
    /// Send of source `s` into a bus/main/matrix node: `SEND_BASE + s` ([`super::SRC_COUNT`]).
    pub const SEND_BASE: u8 = 1;
    /// Blocks `>= HOOK_BASE` of a bus/main/matrix node go to its [`super::StripHook`] (T037).
    pub const HOOK_BASE: u8 = 128;
}

/// Parameter ids within the mix blocks.
pub mod param {
    // fader section
    pub const MUTE: u16 = 0;
    pub const FDR: u16 = 1;
    pub const PAN: u16 = 2;
    pub const WID: u16 = 3;
    pub const TAPWID: u16 = 4;
    pub const BUSMONO: u16 = 5;
    /// External mute (mute groups, DCA mutes; T038). Not bound to the schema.
    pub const EXT_MUTE: u16 = 8;
    /// External gain offset in dB (DCA faders; T038). Not bound to the schema.
    pub const EXT_GAIN: u16 = 9;
    /// Exempt the strip from [`EXT_MUTE`]. Not bound to the schema.
    pub const MUTE_SAFE: u16 = 10;
    // sends
    pub const ON: u16 = 0;
    pub const LVL: u16 = 1;
    pub const MODE: u16 = 2;
    pub const PLINK: u16 = 3;
    pub const SEND_PAN: u16 = 4;
    /// `pre` flag (bus/main senders, main sends): true = [`super::SendMode::Pre`], false = Post.
    pub const PRE: u16 = 5;
}

/// How bus->bus dependencies are scheduled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BusOrder {
    /// Tiers from the scene's active (`on`) bus->bus sends at build time (minimal depth).
    #[default]
    FromScene,
    /// Bus `j` may receive from every bus `i < j` (16 sequential tiers, always complete).
    Chain,
    /// No bus->bus sends.
    Flat,
}

/// Factory for bus/main/matrix strip processing (T037).
pub type HookFactory = Box<dyn FnMut(Strip) -> Box<dyn StripHook>>;

/// Hook factory without strip processing (every bus/main/matrix sums straight into its fader).
pub fn no_processing() -> HookFactory {
    Box::new(|_| Box::new(NoProcessing) as Box<dyn StripHook>)
}

/// Mix/graph configuration.
pub struct MixConfig {
    pub block_size: usize,
    pub sample_rate: f32,
    /// Channel/aux pan.
    pub input_pan: PanLaw,
    /// Send pan (independent send pans).
    pub send_pan: PanLaw,
    /// Bus/main/matrix pan.
    pub bus_pan: PanLaw,
    /// Apply the send level to GRP sends (default false, see [`SendMode::Grp`]).
    pub grp_uses_level: bool,
    pub bus_order: BusOrder,
    /// Strip processing for busses/mains/matrices (`None` = the standard [`MixStrip`]; see
    /// [`no_processing`] for the bare T036 mix).
    pub hooks: Option<HookFactory>,
    /// DCA / mute-group / solo state published by a [`MixLogic`] (default: a private, never
    /// updated state = no DCAs, mute groups or solos).
    pub logic: Option<Arc<MixShared>>,
    /// Level bank the input strips meter into (T039; IN/PFL/keys/GR). The mix-level meters
    /// (strip outs, bus/main/matrix in/out, mix-output sources) are port meters:
    /// [`crate::meters::PortMeters::for_mix`].
    pub meters: Option<std::sync::Arc<crate::meters::LevelBank>>,
    /// RTA feed the strips' probes push the selected tap into (T084; ch/aux POST, solo, monitor
    /// and FX taps are graph ports the caller captures: [`crate::rta`]).
    pub rta: Option<std::sync::Arc<crate::rta::RtaFeed>>,
    /// Model-slot probe the input strips capture into on request (T088, [`crate::probe`]).
    pub probe: Option<std::sync::Arc<crate::probe::SlotProbeFeed>>,
    /// FX rack hosting the strips' inserts (T050; `None` = a new [`crate::fx::FxRack`] with every
    /// catalog model). Tests pass one built with [`crate::fx::FxRack::with_factory`].
    pub fx_rack: Option<crate::fx::FxRack>,
    /// Monitor level knobs (T03F): when set, the monitor section takes its levels from them and
    /// ignores `cfg/mon/1|2/lvl` (the daemon mirrors the knobs into the store).
    pub monitor_levels: Option<Arc<monitor::MonitorLevels>>,
}

impl Default for MixConfig {
    fn default() -> Self {
        Self {
            block_size: 64,
            sample_rate: 48_000.0,
            input_pan: PanLaw::ConstPower,
            send_pan: PanLaw::ConstPower,
            bus_pan: PanLaw::Balance,
            grp_uses_level: false,
            bus_order: BusOrder::FromScene,
            hooks: None,
            logic: None,
            meters: None,
            rta: None,
            probe: None,
            fx_rack: None,
            monitor_levels: None,
        }
    }
}

impl std::fmt::Debug for MixConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MixConfig")
            .field("block_size", &self.block_size)
            .field("sample_rate", &self.sample_rate)
            .field("input_pan", &self.input_pan)
            .field("send_pan", &self.send_pan)
            .field("bus_pan", &self.bus_pan)
            .field("grp_uses_level", &self.grp_uses_level)
            .field("bus_order", &self.bus_order)
            .field("hooks", &self.hooks.is_some())
            .field("logic", &self.logic.is_some())
            .field("meters", &self.meters.is_some())
            .field("rta", &self.rta.is_some())
            .field("probe", &self.probe.is_some())
            .field("fx_rack", &self.fx_rack.is_some())
            .field("monitor_levels", &self.monitor_levels)
            .finish()
    }
}
