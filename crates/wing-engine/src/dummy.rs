//! Synthetic processors for tests and benchmarks (stand-ins for real strips/FX until the
//! blocks crate is wired in).

use crate::graph::{NodeIo, ProcessCtx, Processor};

/// Deterministic CPU load: input 0 (or silence) through `stages` cascaded biquads per output
/// channel. Roughly 0.25 µs per stage per 64-sample channel on an M-series P-core.
pub struct DummyWork {
    stages: usize,
    state: Vec<[f32; 2]>,
}

impl DummyWork {
    /// `stages` biquads per output channel, `channels` outputs.
    pub fn new(stages: usize, channels: usize) -> Self {
        Self {
            stages,
            state: vec![[0.0; 2]; stages * channels],
        }
    }
}

impl Processor for DummyWork {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        // Fixed low-pass-ish coefficients (stable).
        let (b0, b1, b2, a1, a2) = (0.2f32, 0.4f32, 0.2f32, -0.5f32, 0.3f32);
        let input = (io.num_inputs() > 0).then(|| io.input(0));
        for ch in 0..io.num_outputs() {
            let out = io.output(ch);
            match input {
                Some(x) => out.copy_from_slice(x),
                None => out.fill(0.0),
            }
            for st in &mut self.state[ch * self.stages..(ch + 1) * self.stages] {
                let [mut s1, mut s2] = *st;
                for v in out.iter_mut() {
                    // Transposed direct form II.
                    let x = *v;
                    let y = b0 * x + s1;
                    s1 = b1 * x - a1 * y + s2;
                    s2 = b2 * x - a2 * y;
                    *v = y;
                }
                *st = [s1, s2];
            }
        }
    }
}

/// Sums inputs into outputs: output `j` = sum of inputs `j, j + n_out, j + 2 n_out, ...`
/// (so stereo sources wired L,R,L,R,... sum into a stereo bus). Gain per input fixed at `gain`.
pub struct Sum {
    /// Gain applied to every input.
    pub gain: f32,
}

impl Processor for Sum {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        let n_out = io.num_outputs();
        let ins = io.inputs();
        let n_in = ins.len();
        for j in 0..n_out {
            let out = io.output(j);
            out.fill(0.0);
            let mut i = j;
            while i < n_in {
                let x = ins.get(i);
                for (o, &v) in out.iter_mut().zip(x) {
                    *o += self.gain * v;
                }
                i += n_out;
            }
        }
    }
}

/// Mixer-shaped test scene:
/// level 0: `strips` stereo strips (`DummyWork`, `strip_stages` biquads/ch) fed by external ch i;
/// level 1: `fx` stereo bus sums, each summing all strips (gain 1/strips);
/// level 2: `fx` stereo FX (`DummyWork`, `fx_stages`) on the busses;
/// level 3: main stereo sum of the FX returns.
/// Returns the graph and the main node.
pub fn scene(
    strips: usize,
    fx: usize,
    strip_stages: usize,
    fx_stages: usize,
    block: usize,
) -> (crate::Graph, crate::NodeId) {
    use crate::{GraphBuilder, Port};
    let mut g = GraphBuilder::new(block, 48_000.0, strips as u32);
    let strip_ids: Vec<_> = (0..strips)
        .map(|i| {
            let ext = [Port::External(i as u32)];
            g.add_node(
                format!("strip{i}"),
                2,
                &ext,
                DummyWork::new(strip_stages, 2),
            )
            .unwrap()
        })
        .collect();
    let strip_ports: Vec<Port> = strip_ids
        .iter()
        .flat_map(|&n| [Port::node(n, 0), Port::node(n, 1)])
        .collect();
    let gain = 1.0 / strips.max(1) as f32;
    let fx_ids: Vec<_> = (0..fx)
        .map(|i| {
            let bus = g
                .add_node(format!("bus{i}"), 2, &strip_ports, Sum { gain })
                .unwrap();
            let ins = [Port::node(bus, 0), Port::node(bus, 1)];
            g.add_node(format!("fx{i}"), 2, &ins, DummyWork::new(fx_stages, 2))
                .unwrap()
        })
        .collect();
    let fx_ports: Vec<Port> = fx_ids
        .iter()
        .flat_map(|&n| [Port::node(n, 0), Port::node(n, 1)])
        .collect();
    let main = g.add_node("main", 2, &fx_ports, Sum { gain: 1.0 }).unwrap();
    (g.build().unwrap(), main)
}
