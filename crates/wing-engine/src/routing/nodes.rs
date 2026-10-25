//! RT processors that apply a [`RoutingTable`]: the two oscillators, the input router (USB +
//! oscillators -> 48 stereo input strips) and the USB output router (-> driver `USB Out 1..48`).

use crate::graph::{
    GraphBuilder, GraphError, InputList, NodeId, NodeIo, Port, ProcessCtx, Processor,
};

use super::addr::{TapId, TAP_COUNT};
use super::table::{Feed, InputRoute, OscMode, RoutingTable, Width, INPUT_STRIPS, USB_CHANNELS};
use super::RoutingReader;

/// Engine ports for taps, indexed by [`TapId`]. `None` (all of them until T036 adds the mixing
/// nodes) reads as silence.
pub type TapPorts = [Option<Port>; TAP_COUNT];

/// No taps wired yet.
pub const NO_TAPS: TapPorts = [None; TAP_COUNT];

const OSC_INPUT_BASE: usize = USB_CHANNELS;

#[inline]
fn feed_slice<'a>(ins: &InputList<'a>, feed: Feed) -> Option<&'a [f32]> {
    match feed {
        Feed::Usb(k) if (k as usize) < USB_CHANNELS => Some(ins.get(k as usize)),
        Feed::Osc(k) if k < 2 => Some(ins.get(OSC_INPUT_BASE + k as usize)),
        _ => None,
    }
}

#[inline]
fn write_scaled(out: &mut [f32], x: Option<&[f32]>, gain: f32) {
    match x {
        Some(x) if gain != 0.0 => {
            for (o, &v) in out.iter_mut().zip(x) {
                *o = v * gain;
            }
        }
        _ => out.fill(0.0),
    }
}

#[derive(Clone, Copy)]
struct OscState {
    phase: f64,
    rng: u32,
    pink: [f32; 7],
}

/// The two WING oscillators (`io/in/OSC/1..2`): sine, pink or white noise at an RMS level of
/// `lvl` dBFS [I]. Two output channels. Source mute/polarity are applied downstream by the
/// routers (they are part of the leg gain).
pub struct Oscillators {
    reader: RoutingReader,
    st: [OscState; 2],
}

impl Oscillators {
    pub fn new(reader: RoutingReader) -> Self {
        let s = |seed| OscState {
            phase: 0.0,
            rng: seed,
            pink: [0.0; 7],
        };
        Self {
            reader,
            st: [s(0x1234_5678), s(0x9e37_79b9)],
        }
    }
}

#[inline]
fn white(rng: &mut u32) -> f32 {
    // xorshift32 -> uniform [-1, 1)
    let mut x = *rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *rng = x;
    (x as i32) as f32 * (1.0 / 2_147_483_648.0)
}

/// Uniform [-1,1) has RMS 1/sqrt(3).
const WHITE_NORM: f32 = 1.732_050_8;
/// Approximate RMS normalisation of Paul Kellet's refined pink filter driven by unit-RMS white.
const PINK_NORM: f32 = 0.33;

impl Processor for Oscillators {
    fn process(&mut self, ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        self.reader.update();
        let t = self.reader.get();
        for k in 0..2 {
            let p = t.osc[k];
            let st = &mut self.st[k];
            let out = io.output(k);
            match p.mode {
                OscMode::Sine => {
                    let amp = p.level * std::f32::consts::SQRT_2;
                    let inc = (p.freq as f64 / ctx.sample_rate as f64).fract();
                    for o in out.iter_mut() {
                        *o = amp * (st.phase * std::f64::consts::TAU).sin() as f32;
                        st.phase += inc;
                        if st.phase >= 1.0 {
                            st.phase -= 1.0;
                        }
                    }
                }
                OscMode::White => {
                    let g = p.level * WHITE_NORM;
                    for o in out.iter_mut() {
                        *o = g * white(&mut st.rng);
                    }
                }
                OscMode::Pink => {
                    let g = p.level * PINK_NORM;
                    let b = &mut st.pink;
                    for o in out.iter_mut() {
                        let w = white(&mut st.rng) * WHITE_NORM;
                        b[0] = 0.99886 * b[0] + w * 0.0555179;
                        b[1] = 0.99332 * b[1] + w * 0.0750759;
                        b[2] = 0.96900 * b[2] + w * 0.153_852;
                        b[3] = 0.86650 * b[3] + w * 0.3104856;
                        b[4] = 0.55000 * b[4] + w * 0.5329522;
                        b[5] = -0.7616 * b[5] - w * 0.0168980;
                        let y = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362;
                        b[6] = w * 0.115926;
                        *o = g * y;
                    }
                }
            }
        }
    }
}

/// Input router: inputs = USB In 1..48 then oscillators 1..2; outputs = 48 input strips x L/R
/// (`2*i` / `2*i+1`, strips 0..40 = ch 1..40, 40..48 = aux 1..8).
///
/// Mix-output and user-signal sources (feeds [`Feed::Tap`]/[`Feed::TapMono`]) read as silence
/// here: re-inputting a bus into a channel is a feedback path that needs a one-block delayed
/// tap, which comes with the mixing nodes (T036).
pub struct InputRouter {
    reader: RoutingReader,
}

impl InputRouter {
    pub fn new(reader: RoutingReader) -> Self {
        Self { reader }
    }

    /// The input ports the router expects, given the oscillator node.
    pub fn inputs(osc: NodeId) -> Vec<Port> {
        (0..USB_CHANNELS as u32)
            .map(Port::External)
            .chain((0..2).map(|k| Port::node(osc, k)))
            .collect()
    }

    /// Output port of strip `i` (0..48), side `side` (0 = L, 1 = R).
    pub fn port(router: NodeId, strip: usize, side: usize) -> Port {
        Port::node(router, (strip * 2 + side) as u32)
    }
}

#[inline]
fn route_strip(r: &InputRoute, ins: &InputList<'_>, l: &mut [f32], rr: &mut [f32]) {
    write_scaled(l, feed_slice(ins, r.legs[0].feed), r.legs[0].gain);
    write_scaled(rr, feed_slice(ins, r.legs[1].feed), r.legs[1].gain);
    if r.width == Width::MidSide {
        for (a, b) in l.iter_mut().zip(rr.iter_mut()) {
            let (m, s) = (*a, *b);
            *a = m + s;
            *b = m - s;
        }
    }
}

impl Processor for InputRouter {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        self.reader.update();
        let t: &RoutingTable = self.reader.get();
        let ins = io.inputs();
        let b = io.block_size();
        let outs = io.outputs();
        for (i, pair) in outs.chunks_exact_mut(2 * b).take(INPUT_STRIPS).enumerate() {
            let (l, r) = pair.split_at_mut(b);
            route_strip(t.input(i), &ins, l, r);
        }
    }
}

/// USB output router: inputs = USB In 1..48, oscillators 1..2, then every wired tap; outputs =
/// USB Out 1..48 (to the driver). Unwired taps read as silence.
pub struct UsbOutRouter {
    reader: RoutingReader,
    /// Input index per [`TapId`], `u16::MAX` = not wired.
    tap_input: Box<[u16]>,
}

impl UsbOutRouter {
    /// Router plus its input list for the given tap ports.
    pub fn new(reader: RoutingReader, osc: NodeId, taps: &TapPorts) -> (Self, Vec<Port>) {
        let mut inputs = InputRouter::inputs(osc);
        let mut tap_input = vec![u16::MAX; TAP_COUNT].into_boxed_slice();
        for (i, p) in taps.iter().enumerate() {
            if let Some(p) = p {
                tap_input[i] = inputs.len() as u16;
                inputs.push(*p);
            }
        }
        (Self { reader, tap_input }, inputs)
    }

    #[inline]
    fn tap<'a>(&self, ins: &InputList<'a>, id: TapId) -> Option<&'a [f32]> {
        match self.tap_input.get(id.0 as usize) {
            Some(&k) if k != u16::MAX => Some(ins.get(k as usize)),
            _ => None,
        }
    }
}

impl Processor for UsbOutRouter {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        self.reader.update();
        let ins = io.inputs();
        for n in 0..USB_CHANNELS {
            let leg = *self.reader.get().usb_out(n + 1);
            let out = io.output(n);
            match leg.feed {
                Feed::Tap(id) => write_scaled(out, self.tap(&ins, id), leg.gain),
                Feed::TapMono(id) => {
                    let l = self.tap(&ins, id);
                    let r = self.tap(&ins, TapId(id.0 + 1));
                    match (l, r) {
                        (Some(l), Some(r)) => {
                            let g = 0.5 * leg.gain;
                            for ((o, &a), &b) in out.iter_mut().zip(l).zip(r) {
                                *o = (a + b) * g;
                            }
                        }
                        (Some(x), None) | (None, Some(x)) => {
                            write_scaled(out, Some(x), 0.5 * leg.gain)
                        }
                        (None, None) => out.fill(0.0),
                    }
                }
                f => write_scaled(out, feed_slice(&ins, f), leg.gain),
            }
        }
    }
}

/// Node ids of the routing stage in a graph.
#[derive(Clone, Copy, Debug)]
pub struct IoNodes {
    /// Oscillators (2 outputs).
    pub osc: NodeId,
    /// Input router (96 outputs, see [`InputRouter::port`]).
    pub inputs: NodeId,
}

/// Add the oscillators and the input router. The graph must have at least 48 external inputs
/// (USB In 1..48).
pub fn add_input_stage(
    g: &mut GraphBuilder,
    ctl: &mut super::RoutingControl,
) -> Result<IoNodes, GraphError> {
    let osc = g.add_node("osc", 2, &[], Oscillators::new(ctl.subscribe()))?;
    let inputs = g.add_node(
        "input-router",
        (INPUT_STRIPS * 2) as u32,
        &InputRouter::inputs(osc),
        InputRouter::new(ctl.subscribe()),
    )?;
    Ok(IoNodes { osc, inputs })
}

/// Add the USB output router (48 outputs = driver `USB Out 1..48`). Add it after every node
/// whose ports appear in `taps`.
pub fn add_usb_output(
    g: &mut GraphBuilder,
    ctl: &mut super::RoutingControl,
    osc: NodeId,
    taps: &TapPorts,
) -> Result<NodeId, GraphError> {
    let (router, inputs) = UsbOutRouter::new(ctl.subscribe(), osc, taps);
    g.add_node("usb-out-router", USB_CHANNELS as u32, &inputs, router)
}
