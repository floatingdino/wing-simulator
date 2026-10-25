//! Multi-channel biquad bank: `channels` x `stages` TDF-II sections in structure-of-arrays layout.
//!
//! Channels are grouped in lanes of [`LANES`] (8 = two NEON f32x4 registers). Coefficients and state for one
//! (group, stage) live in `[f32; LANES]` arrays so the per-sample update is a straight-line vector op across
//! channels. Audio is transposed into a small stack tile (`TILE` frames x `LANES` channels), all stages run on
//! the tile with state held in registers, then the tile is transposed back. The public API stays per-channel:
//! [`BiquadBank::set_coeffs`] takes `(channel, stage, Coeffs)`.
//!
//! Allocation happens only in [`BiquadBank::new`].

use crate::biquad::Coeffs;

/// Channels per SIMD lane group.
pub const LANES: usize = 8;
/// Frames per transposition tile.
const TILE: usize = 32;

#[derive(Clone, Copy, Debug)]
struct StageLanes {
    b0: [f32; LANES],
    b1: [f32; LANES],
    b2: [f32; LANES],
    a1: [f32; LANES],
    a2: [f32; LANES],
    s1: [f32; LANES],
    s2: [f32; LANES],
}

impl Default for StageLanes {
    fn default() -> Self {
        Self {
            b0: [1.0; LANES],
            b1: [0.0; LANES],
            b2: [0.0; LANES],
            a1: [0.0; LANES],
            a2: [0.0; LANES],
            s1: [0.0; LANES],
            s2: [0.0; LANES],
        }
    }
}

impl StageLanes {
    #[inline(always)]
    fn run(&mut self, tile: &mut [[f32; LANES]]) {
        let (b0, b1, b2, a1, a2) = (self.b0, self.b1, self.b2, self.a1, self.a2);
        let mut s1 = self.s1;
        let mut s2 = self.s2;
        for frame in tile.iter_mut() {
            for l in 0..LANES {
                let x = frame[l];
                let y = b0[l] * x + s1[l];
                s1[l] = b1[l] * x - a1[l] * y + s2[l];
                s2[l] = b2[l] * x - a2[l] * y;
                frame[l] = y;
            }
        }
        self.s1 = s1;
        self.s2 = s2;
    }
}

/// `channels` independent cascades of `stages` biquads each (e.g. 48 channels x 6 EQ bands).
pub struct BiquadBank {
    channels: usize,
    stages: usize,
    groups: usize,
    /// Indexed `[group * stages + stage]`.
    lanes: Vec<StageLanes>,
}

impl BiquadBank {
    /// All sections start as pass-through.
    pub fn new(channels: usize, stages: usize) -> Self {
        let groups = channels.div_ceil(LANES);
        Self {
            channels,
            stages,
            groups,
            lanes: vec![StageLanes::default(); groups * stages],
        }
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    pub fn stages(&self) -> usize {
        self.stages
    }

    /// Set the coefficients for one channel's stage (state kept).
    #[inline]
    pub fn set_coeffs(&mut self, channel: usize, stage: usize, c: Coeffs) {
        assert!(channel < self.channels && stage < self.stages);
        let st = &mut self.lanes[(channel / LANES) * self.stages + stage];
        let l = channel % LANES;
        st.b0[l] = c.b0 as f32;
        st.b1[l] = c.b1 as f32;
        st.b2[l] = c.b2 as f32;
        st.a1[l] = c.a1 as f32;
        st.a2[l] = c.a2 as f32;
    }

    pub fn reset(&mut self) {
        for st in &mut self.lanes {
            st.s1 = [0.0; LANES];
            st.s2 = [0.0; LANES];
        }
    }

    /// Zero state values below 1e-15 (portable denormal fallback; call once per block).
    pub fn flush_denormals(&mut self) {
        for st in &mut self.lanes {
            for v in st.s1.iter_mut().chain(st.s2.iter_mut()) {
                if v.abs() < 1e-15 {
                    *v = 0.0;
                }
            }
        }
    }

    #[inline]
    fn run_tile(&mut self, group: usize, tile: &mut [[f32; LANES]]) {
        let base = group * self.stages;
        for st in &mut self.lanes[base..base + self.stages] {
            st.run(tile);
        }
    }

    /// Process planar buffers in place: `bufs[ch]` is channel `ch`'s block. All blocks must have equal length;
    /// `bufs.len()` must equal `channels`.
    pub fn process_planar(&mut self, bufs: &mut [&mut [f32]]) {
        assert_eq!(bufs.len(), self.channels);
        let len = bufs.first().map_or(0, |b| b.len());
        assert!(bufs.iter().all(|b| b.len() == len));
        let mut tile = [[0.0f32; LANES]; TILE];
        for g in 0..self.groups {
            let ch0 = g * LANES;
            let active = (self.channels - ch0).min(LANES);
            let mut start = 0;
            while start < len {
                let n = (len - start).min(TILE);
                for l in 0..active {
                    let src = &bufs[ch0 + l][start..start + n];
                    for (t, &x) in tile.iter_mut().zip(src) {
                        t[l] = x;
                    }
                }
                self.run_tile(g, &mut tile[..n]);
                for l in 0..active {
                    let dst = &mut bufs[ch0 + l][start..start + n];
                    for (t, y) in tile.iter().zip(dst) {
                        *y = t[l];
                    }
                }
                start += n;
            }
        }
    }

    /// Process interleaved frames in place (`data.len() == frames * channels`).
    pub fn process_interleaved(&mut self, data: &mut [f32]) {
        let ch = self.channels;
        assert_eq!(data.len() % ch, 0);
        let frames = data.len() / ch;
        let mut tile = [[0.0f32; LANES]; TILE];
        for g in 0..self.groups {
            let ch0 = g * LANES;
            let active = (ch - ch0).min(LANES);
            let mut start = 0;
            while start < frames {
                let n = (frames - start).min(TILE);
                for (i, t) in tile[..n].iter_mut().enumerate() {
                    let f = (start + i) * ch + ch0;
                    t[..active].copy_from_slice(&data[f..f + active]);
                }
                self.run_tile(g, &mut tile[..n]);
                for (i, t) in tile[..n].iter().enumerate() {
                    let f = (start + i) * ch + ch0;
                    data[f..f + active].copy_from_slice(&t[..active]);
                }
                start += n;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::biquad::Biquad;

    fn design(ch: usize, st: usize) -> Coeffs {
        let fs = 48_000.0;
        let f = 50.0 * (1.0 + ch as f64) * (1.0 + st as f64);
        match st % 3 {
            0 => Coeffs::peak(fs, f, 1.0 + ch as f64 * 0.1, 6.0 - ch as f64 * 0.3),
            1 => Coeffs::highpass(fs, f * 0.5, 0.707),
            _ => Coeffs::high_shelf(fs, f * 4.0, 0.707, -4.0),
        }
    }

    #[test]
    fn bank_matches_scalar_biquads() {
        let (channels, stages, len) = (13, 3, 100);
        let mut bank = BiquadBank::new(channels, stages);
        let mut scalar: Vec<Vec<Biquad>> = (0..channels)
            .map(|c| (0..stages).map(|s| Biquad::new(design(c, s))).collect())
            .collect();
        for c in 0..channels {
            for s in 0..stages {
                bank.set_coeffs(c, s, design(c, s));
            }
        }
        let input = |c: usize, n: usize| ((n * (c + 3)) as f32 * 0.071).sin() * 0.5;
        let mut planar: Vec<Vec<f32>> = (0..channels)
            .map(|c| (0..len).map(|n| input(c, n)).collect())
            .collect();
        let mut interleaved: Vec<f32> = (0..len)
            .flat_map(|n| (0..channels).map(move |c| input(c, n)))
            .collect();
        let mut bank2 = BiquadBank::new(channels, stages);
        for c in 0..channels {
            for s in 0..stages {
                bank2.set_coeffs(c, s, design(c, s));
            }
        }
        {
            let mut refs: Vec<&mut [f32]> = planar.iter_mut().map(|v| v.as_mut_slice()).collect();
            bank.process_planar(&mut refs);
        }
        bank2.process_interleaved(&mut interleaved);
        for c in 0..channels {
            for n in 0..len {
                let mut x = input(c, n);
                for s in &mut scalar[c] {
                    x = s.process(x);
                }
                assert!((planar[c][n] - x).abs() < 1e-6, "planar c={c} n={n}");
                assert!(
                    (interleaved[n * channels + c] - x).abs() < 1e-6,
                    "interleaved c={c} n={n}"
                );
            }
        }
    }
}
