//! Construction of the complete engine graph and its parameter bindings.

use std::fmt;

use wing_model::ParamStore;

use crate::channel::{bind_aux_strip, bind_channel_strip, ChannelStrip};
use crate::graph::{Graph, GraphBuilder, GraphError, NodeId, Port};
use crate::params::{BindError, BlockLayout, ParamMap, ParamMapBuilder};
use crate::routing::{
    add_input_stage_with_feedback, add_usb_output, FeedbackWriter, InputRouter, IoNodes, MixGroup,
    RoutingControl, Side, Strip, StripPoint, Tap, TapFeedback, TapPorts, INPUT_STRIPS, NO_TAPS,
    USB_CHANNELS,
};

use super::nodes::{
    InputSignal, InputStripNode, MixSignal, MixStripNode, SlotKind, SlotSpec, INPUT_OUTPUTS,
    MIX_OUTPUTS,
};
use super::strip::{bind_mix_strip, MixStrip};
use super::{block, param, BusOrder, MixConfig};

/// Error from [`build_mix_graph`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MixError {
    Graph(GraphError),
    Bind(BindError),
}

impl fmt::Display for MixError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MixError::Graph(e) => write!(f, "graph: {e}"),
            MixError::Bind(e) => write!(f, "binding: {e}"),
        }
    }
}

impl std::error::Error for MixError {}

impl From<GraphError> for MixError {
    fn from(e: GraphError) -> Self {
        MixError::Graph(e)
    }
}

impl From<BindError> for MixError {
    fn from(e: BindError) -> Self {
        MixError::Bind(e)
    }
}

/// Node ids of the mix graph.
#[derive(Clone, Debug)]
pub struct MixNodes {
    /// Oscillators and input router.
    pub io: IoNodes,
    /// Input strips: `0..40` = ch 1..40, `40..48` = aux 1..8 ([`InputStripNode`]).
    pub inputs: [NodeId; INPUT_STRIPS],
    /// Busses 1..16 ([`MixStripNode`]).
    pub bus: [NodeId; 16],
    /// Mains 1..4.
    pub main: [NodeId; 4],
    /// Matrices 1..8.
    pub mtx: [NodeId; 8],
    /// One-block feedback writer for re-inputs.
    pub feedback: NodeId,
    /// USB output router (48 outputs = driver `USB Out 1..48`).
    pub usb_out: NodeId,
}

impl MixNodes {
    /// Node of a strip.
    pub fn node(&self, s: Strip) -> Option<NodeId> {
        let i = s.index()?;
        Some(match s {
            Strip::Ch(_) | Strip::Aux(_) => self.inputs[i],
            Strip::Bus(n) => self.bus[n as usize - 1],
            Strip::Main(n) => self.main[n as usize - 1],
            Strip::Mtx(n) => self.mtx[n as usize - 1],
        })
    }

    /// An input strip signal (`strip` 0..48).
    pub fn input_port(&self, strip: usize, sig: InputSignal, side: Side) -> Port {
        Port::node(self.inputs[strip], sig as u32 * 2 + side as u32)
    }

    /// A bus/main/matrix signal.
    pub fn mix_port(&self, s: Strip, sig: MixSignal, side: Side) -> Option<Port> {
        if matches!(s, Strip::Ch(_) | Strip::Aux(_)) {
            return None;
        }
        Some(Port::node(self.node(s)?, sig as u32 * 2 + side as u32))
    }

    /// A strip's user-signal tap ("PRE FADER" = PFL, "POST PROCESSING" = POST).
    pub fn strip_port(&self, s: Strip, point: StripPoint, side: Side) -> Option<Port> {
        match s {
            Strip::Ch(_) | Strip::Aux(_) => {
                let sig = match point {
                    StripPoint::PreFader => InputSignal::Pfl,
                    StripPoint::PostProc => InputSignal::Post,
                };
                Some(self.input_port(s.index()?, sig, side))
            }
            _ => {
                let sig = match point {
                    StripPoint::PreFader => MixSignal::Pfl,
                    StripPoint::PostProc => MixSignal::Post,
                };
                self.mix_port(s, sig, side)
            }
        }
    }

    /// The output (POST) of any strip.
    pub fn post(&self, s: Strip, side: Side) -> Option<Port> {
        self.strip_port(s, StripPoint::PostProc, side)
    }

    /// Every tap the routing layer can address, wired to its port (SEND/MON legs stay `None`).
    pub fn tap_ports(&self) -> Box<TapPorts> {
        let mut taps = Box::new(NO_TAPS);
        for (id, slot) in taps.iter_mut().enumerate() {
            let Some(t) = Tap::from_id(crate::routing::TapId(id as u16)) else {
                continue;
            };
            *slot = match t {
                Tap::Mix { grp, leg } => {
                    let k = leg.div_ceil(2);
                    let side = if leg % 2 == 1 { Side::L } else { Side::R };
                    let s = match grp {
                        MixGroup::Bus => Some(Strip::Bus(k as u8)),
                        MixGroup::Main => Some(Strip::Main(k as u8)),
                        MixGroup::Mtx => Some(Strip::Mtx(k as u8)),
                        MixGroup::Send | MixGroup::Mon => None,
                    };
                    s.and_then(|s| self.post(s, side))
                }
                Tap::Strip { strip, point, side } => self.strip_port(strip, point, side),
            };
        }
        taps
    }
}

/// The mix graph, ready to extend (more nodes, more bindings) or [`MixGraph::finish`].
pub struct MixGraph {
    /// Every node added; the daemon may add more (e.g. host-bridge taps) before building.
    pub graph: GraphBuilder,
    /// Bindings of every mix and strip parameter (`ch`, `aux`, sends, `bus|main|mtx` faders);
    /// incl. `bus|main|mtx` processing (T037) at blocks `>=` [`block::HOOK_BASE`].
    pub params: ParamMapBuilder,
    pub nodes: MixNodes,
    /// Tap ports (also given to the USB output router and the feedback writer).
    pub taps: Box<TapPorts>,
    /// Bus tier (0-based scheduling stage) per bus; bus `j` receives bus `i` iff `i < j` and
    /// `tier[i] < tier[j]`.
    pub bus_tiers: [u8; 16],
}

impl MixGraph {
    /// Compile the graph and the parameter map.
    pub fn finish(self) -> Result<(Graph, ParamMap, MixNodes), GraphError> {
        Ok((self.graph.build()?, self.params.build(), self.nodes))
    }

    /// Whether bus `from` (1-based) feeds bus `to` in this graph.
    pub fn bus_feeds_bus(&self, from: usize, to: usize) -> bool {
        bus_wired(&self.bus_tiers, from - 1, to - 1)
    }
}

fn bus_wired(tiers: &[u8; 16], i: usize, j: usize) -> bool {
    i < j && tiers[i] < tiers[j]
}

/// Scheduling tiers of the 16 busses for `order`.
pub fn bus_tiers(store: &ParamStore, order: BusOrder) -> [u8; 16] {
    let mut t = [0u8; 16];
    match order {
        BusOrder::Flat => {}
        BusOrder::Chain => {
            for (j, v) in t.iter_mut().enumerate() {
                *v = j as u8;
            }
        }
        BusOrder::FromScene => {
            for j in 0..16 {
                for i in 0..j {
                    let on = store
                        .lookup(&format!("bus/{}/send/{}/on", i + 1, j + 1))
                        .and_then(|id| store.get_f64(id))
                        .is_some_and(|v| v != 0.0);
                    if on {
                        t[j] = t[j].max(t[i] + 1);
                    }
                }
            }
        }
    }
    t
}

const CH_MIX: BlockLayout = BlockLayout {
    params: &[
        ("mute", param::MUTE),
        ("fdr", param::FDR),
        ("pan", param::PAN),
        ("wid", param::WID),
        ("tapwid", param::TAPWID),
    ],
    targets: &[],
};

const AUX_MIX: BlockLayout = BlockLayout {
    params: &[
        ("mute", param::MUTE),
        ("fdr", param::FDR),
        ("pan", param::PAN),
        ("wid", param::WID),
    ],
    targets: &[],
};

const OWN_MIX: BlockLayout = BlockLayout {
    params: &[
        ("mute", param::MUTE),
        ("fdr", param::FDR),
        ("pan", param::PAN),
        ("wid", param::WID),
        ("busmono", param::BUSMONO),
    ],
    targets: &[],
};

/// `ch|aux/*/send/<bus|MXn>`.
const MODE_SEND: BlockLayout = BlockLayout {
    params: &[
        ("on", param::ON),
        ("lvl", param::LVL),
        ("mode", param::MODE),
        ("plink", param::PLINK),
        ("pan", param::SEND_PAN),
    ],
    targets: &[],
};

/// `*/main/<n>`, `bus/*/send/*`, `main/*/send/*`.
const PRE_SEND: BlockLayout = BlockLayout {
    params: &[("on", param::ON), ("lvl", param::LVL), ("pre", param::PRE)],
    targets: &[],
};

fn send_block(src: usize) -> u8 {
    block::SEND_BASE + src as u8
}

/// Bind every channel/aux strip (T035 blocks + fader section), every send and every
/// bus/main/matrix fader section and strip processing (T037, [`bind_mix_strip`]) of `nodes`.
pub fn bind_mix_params(b: &mut ParamMapBuilder, nodes: &MixNodes) -> Result<(), BindError> {
    for i in 0..INPUT_STRIPS {
        let node = nodes.inputs[i];
        let (p, layout) = if i < 40 {
            bind_channel_strip(b, i as u32 + 1, node)?;
            (format!("ch/{}", i + 1), &CH_MIX)
        } else {
            bind_aux_strip(b, i as u32 - 39, node)?;
            (format!("aux/{}", i - 39), &AUX_MIX)
        };
        b.bind_block(&p, node, block::INPUT_MIX, layout)?;
        for m in 0..4 {
            b.bind_block(
                &format!("{p}/main/{}", m + 1),
                nodes.main[m],
                send_block(i),
                &PRE_SEND,
            )?;
        }
        for k in 0..16 {
            b.bind_block(
                &format!("{p}/send/{}", k + 1),
                nodes.bus[k],
                send_block(i),
                &MODE_SEND,
            )?;
        }
        for k in 0..8 {
            b.bind_block(
                &format!("{p}/send/MX{}", k + 1),
                nodes.mtx[k],
                send_block(i),
                &MODE_SEND,
            )?;
        }
    }
    for j in 0..16 {
        let p = format!("bus/{}", j + 1);
        let src = 48 + j;
        b.bind_block(&p, nodes.bus[j], block::OWN, &OWN_MIX)?;
        bind_mix_strip(b, Strip::Bus(j as u8 + 1), nodes.bus[j])?;
        for m in 0..4 {
            b.bind_block(
                &format!("{p}/main/{}", m + 1),
                nodes.main[m],
                send_block(src),
                &PRE_SEND,
            )?;
        }
        for k in 0..16 {
            b.bind_block(
                &format!("{p}/send/{}", k + 1),
                nodes.bus[k],
                send_block(src),
                &PRE_SEND,
            )?;
        }
        for k in 0..8 {
            b.bind_block(
                &format!("{p}/send/MX{}", k + 1),
                nodes.mtx[k],
                send_block(src),
                &PRE_SEND,
            )?;
        }
    }
    for m in 0..4 {
        let p = format!("main/{}", m + 1);
        b.bind_block(&p, nodes.main[m], block::OWN, &OWN_MIX)?;
        bind_mix_strip(b, Strip::Main(m as u8 + 1), nodes.main[m])?;
        for k in 0..8 {
            b.bind_block(
                &format!("{p}/send/MX{}", k + 1),
                nodes.mtx[k],
                send_block(64 + m),
                &PRE_SEND,
            )?;
        }
    }
    for k in 0..8 {
        b.bind_block(
            &format!("mtx/{}", k + 1),
            nodes.mtx[k],
            block::OWN,
            &OWN_MIX,
        )?;
        bind_mix_strip(b, Strip::Mtx(k as u8 + 1), nodes.mtx[k])?;
    }
    Ok(())
}

/// Build the complete engine graph from `store` (bus order) and `routing`:
///
/// ```text
/// level 0  osc                         (+ 48 external inputs = USB In 1..48)
/// level 1  input router                (USB/osc sources, re-inputs from the previous block)
/// level 2  40 ch + 8 aux strips        (InputStripNode: T035 strip + fader/pan/taps)
/// 3..      16 bus nodes in tiers       (MixStripNode: sum -> hook -> fader/pan)
/// next     4 mains                     (sum of ch/aux/bus main sends)
/// next     8 matrices                  (sum of ch/aux/bus/main MX sends)
/// last     feedback writer, USB output router (every tap wired)
/// ```
///
/// Every node is independent within its level, so the engine's workers split strips, busses,
/// mains and matrices. Graph external inputs: 48 (USB In). Strip and send parameters are bound in
/// [`MixGraph::params`]; build a [`crate::params::ParamPipeline`] from it and `sync_all` before
/// the first cycle. Allocates (control thread).
pub fn build_mix_graph(
    store: &ParamStore,
    routing: &mut RoutingControl,
    mut cfg: MixConfig,
) -> Result<MixGraph, MixError> {
    let fs = cfg.sample_rate as f64;
    let bs = cfg.block_size;
    let tiers = bus_tiers(store, cfg.bus_order);
    let mut g = GraphBuilder::new(bs, cfg.sample_rate, USB_CHANNELS as u32);
    let fb = TapFeedback::new(bs);
    let io = add_input_stage_with_feedback(&mut g, routing, fb.clone())?;

    let mut inputs = [NodeId(0); INPUT_STRIPS];
    for (i, id) in inputs.iter_mut().enumerate() {
        let (name, mut strip) = if i < 40 {
            (format!("ch{}", i + 1), ChannelStrip::channel(fs, bs))
        } else {
            (format!("aux{}", i - 39), ChannelStrip::aux(fs, bs))
        };
        // input strip i (ch 1..40, aux 1..8) = the meters' flat strip index
        strip.set_meter(
            cfg.meters
                .clone()
                .map(|b| crate::meters::StripMeter::at(b, i)),
        );
        let src = [
            InputRouter::port(io.inputs, i, 0),
            InputRouter::port(io.inputs, i, 1),
        ];
        *id = g.add_node(name, INPUT_OUTPUTS, &src, InputStripNode::new(strip, &cfg))?;
    }
    // ports of an Input slot: Pre, PreLinked, Afl, Post (output channels 2..10)
    let input_ports = |n: NodeId| (2..10).map(move |c| Port::node(n, c));
    // ports of a Mix slot: Pre, Post (output channels 2..6)
    let mix_ports = |n: NodeId| (2..6).map(move |c| Port::node(n, c));

    let mut add_mix = |g: &mut GraphBuilder,
                       strip: Strip,
                       srcs: &[(u8, NodeId, SlotKind)]|
     -> Result<NodeId, GraphError> {
        let specs: Vec<SlotSpec> = srcs
            .iter()
            .map(|&(src, _, kind)| SlotSpec { src, kind })
            .collect();
        let ports: Vec<Port> = srcs
            .iter()
            .flat_map(|&(_, n, kind)| -> Vec<Port> {
                match kind {
                    SlotKind::Input => input_ports(n).collect(),
                    SlotKind::Mix => mix_ports(n).collect(),
                }
            })
            .collect();
        let hook = match cfg.hooks.as_mut() {
            Some(f) => f(strip),
            None => Box::new(MixStrip::new(strip, fs, bs)),
        };
        let node = MixStripNode::new(strip, &specs, hook, &cfg);
        debug_assert_eq!(node.num_inputs(), ports.len());
        let name = match strip {
            Strip::Bus(n) => format!("bus{n}"),
            Strip::Main(n) => format!("main{n}"),
            Strip::Mtx(n) => format!("mtx{n}"),
            _ => unreachable!(),
        };
        g.add_node(name, MIX_OUTPUTS, &ports, node)
    };

    let input_srcs: Vec<(u8, NodeId, SlotKind)> = inputs
        .iter()
        .enumerate()
        .map(|(i, &n)| (i as u8, n, SlotKind::Input))
        .collect();
    let mut bus = [NodeId(0); 16];
    for j in 0..16 {
        let mut srcs = input_srcs.clone();
        for (i, &n) in bus.iter().enumerate().take(j) {
            if bus_wired(&tiers, i, j) {
                srcs.push(((48 + i) as u8, n, SlotKind::Mix));
            }
        }
        bus[j] = add_mix(&mut g, Strip::Bus(j as u8 + 1), &srcs)?;
    }
    let bus_srcs: Vec<(u8, NodeId, SlotKind)> = bus
        .iter()
        .enumerate()
        .map(|(i, &n)| ((48 + i) as u8, n, SlotKind::Mix))
        .collect();
    let mut main = [NodeId(0); 4];
    for (m, id) in main.iter_mut().enumerate() {
        let srcs: Vec<_> = input_srcs.iter().chain(&bus_srcs).copied().collect();
        *id = add_mix(&mut g, Strip::Main(m as u8 + 1), &srcs)?;
    }
    let main_srcs: Vec<(u8, NodeId, SlotKind)> = main
        .iter()
        .enumerate()
        .map(|(i, &n)| ((64 + i) as u8, n, SlotKind::Mix))
        .collect();
    let mut mtx = [NodeId(0); 8];
    for (k, id) in mtx.iter_mut().enumerate() {
        let srcs: Vec<_> = input_srcs
            .iter()
            .chain(&bus_srcs)
            .chain(&main_srcs)
            .copied()
            .collect();
        *id = add_mix(&mut g, Strip::Mtx(k as u8 + 1), &srcs)?;
    }

    let mut nodes = MixNodes {
        io,
        inputs,
        bus,
        main,
        mtx,
        feedback: NodeId(0),
        usb_out: NodeId(0),
    };
    let taps = nodes.tap_ports();
    let (writer, fb_inputs) = FeedbackWriter::new(
        fb,
        routing.subscribe(),
        InputRouter::port(io.inputs, 0, 0),
        &taps,
    );
    nodes.feedback = g.add_node("feedback", 1, &fb_inputs, writer)?;
    nodes.usb_out = add_usb_output(&mut g, routing, io.osc, &taps)?;

    let mut params = ParamMapBuilder::new();
    bind_mix_params(&mut params, &nodes)?;
    Ok(MixGraph {
        graph: g,
        params,
        nodes,
        taps,
        bus_tiers: tiers,
    })
}
