//! Standard gate-family models WING GATE (`GATE`) and WING DUCKER (`DUCK`) (models.md §3.1) with the gate
//! side-chain `gatesc` (signal-flow.md §2: filter type/freq/Q, source, tap, key listen).
//!
//! Signal path (stereo-linked): key (self or external) -> side-chain filter -> peak detector (instant
//! attack, [`DETECTOR_RELEASE_MS`] one-pole release, max of both sides) -> level in dB -> open/hold/release
//! state machine -> gain in dB, slewed linearly in dB -> applied to both sides.
//!
//! GATE:
//! - Opens when the key level exceeds `thr`; closes (after `hld`) once it falls below `thr - hysteresis`,
//!   where hysteresis = `acc` (ACCENT 0..100 %) x [`ACCENT_MAX_HYST_DB`] (models.md: "acc = threshold
//!   hysteresis until measured").
//! - `ratio` `gate`: closed gain = `-range`. `1:1.5 .. 1:4`: downward expander, closed gain =
//!   `(r - 1) * (level - thr)` clamped to `-range`.
//! - Attack = time for the gain to traverse the full range upward (linear in dB, 0 = instant); release =
//!   time to traverse the full range downward.
//!
//! DUCK: attenuates by `range` while the key is above `thr` (no hysteresis); attack = time to duck the full
//! range, hold keeps it ducked after the key drops, release = time to recover.
//!
//! Range: 3 .. 60 dB; 60 dB is the "oo" end of the control (full mute): the internal floor is then
//! [`MUTE_FLOOR_DB`] and the output is exactly zero once there.
//!
//! Parameter changes need no extra smoothing: gain only moves at the attack/release slew rates.
//! The side-chain filter is a TPT SVF (safe under coefficient changes). Detector release, the accent law
//! and the "time = full-range traverse" definition are assumptions to **verify** in T072.

use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::one_pole_coeff;
use wing_dsp::svf::SvfMode;
use wing_dsp::Svf;

/// Detector release time constant (ms).
pub const DETECTOR_RELEASE_MS: f64 = 10.0;
/// Hysteresis at ACCENT = 100 %.
pub const ACCENT_MAX_HYST_DB: f32 = 10.0;
/// `range` at or above this value means full mute ("oo dB").
pub const RANGE_INF_DB: f32 = 60.0;
/// Internal gain floor used for the "oo" range.
pub const MUTE_FLOOR_DB: f32 = -120.0;

/// Gate-slot model handled by [`Gate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GateModel {
    /// WING GATE (`GATE`): gate / downward expander.
    #[default]
    Gate,
    /// WING DUCKER (`DUCK`).
    Duck,
}

/// GATE `ratio` enum (snapshot strings `"1:1.5"`, `"1:2"`, `"1:3"`, `"1:4"`, `"gate"`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GateRatio {
    R1_5,
    R2,
    #[default]
    R3,
    R4,
    Gate,
}

impl GateRatio {
    /// Expansion ratio `r` (output dB per input dB below threshold), `None` for a hard gate.
    pub fn expansion(self) -> Option<f32> {
        match self {
            GateRatio::R1_5 => Some(1.5),
            GateRatio::R2 => Some(2.0),
            GateRatio::R3 => Some(3.0),
            GateRatio::R4 => Some(4.0),
            GateRatio::Gate => None,
        }
    }

    /// Parse the snapshot string.
    pub fn from_snap(s: &str) -> Option<Self> {
        Some(match s {
            "1:1.5" => GateRatio::R1_5,
            "1:2" => GateRatio::R2,
            "1:3" => GateRatio::R3,
            "1:4" => GateRatio::R4,
            "gate" | "GATE" => GateRatio::Gate,
            _ => return None,
        })
    }
}

/// GATE / DUCK model parameters (keys `thr range att hld rel acc ratio`), engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateParams {
    pub model: GateModel,
    /// `thr` -80 .. 0 dB.
    pub threshold_db: f32,
    /// `range` 3 .. 60 dB (60 = oo).
    pub range_db: f32,
    /// `att` 0 .. 120 ms.
    pub attack_ms: f32,
    /// `hld` GATE 1 .. 1000 ms, DUCK 30 .. 30000 ms.
    pub hold_ms: f32,
    /// `rel` 4 .. 4000 ms.
    pub release_ms: f32,
    /// `acc` 0 .. 100 % (GATE only).
    pub accent_pct: f32,
    /// `ratio` (GATE only).
    pub ratio: GateRatio,
}

impl Default for GateParams {
    fn default() -> Self {
        Self::gate_default()
    }
}

impl GateParams {
    /// WING-Edit default string for `GATE`: thr=-40, range=40, att=10, hld=10, rel=200, acc=0, ratio=1:3.
    pub fn gate_default() -> Self {
        Self {
            model: GateModel::Gate,
            threshold_db: -40.0,
            range_db: 40.0,
            attack_ms: 10.0,
            hold_ms: 10.0,
            release_ms: 200.0,
            accent_pct: 0.0,
            ratio: GateRatio::R3,
        }
    }

    /// WING-Edit default for `DUCK`: thr=-20, range=20, att=100, hld=100, rel=500.
    pub fn duck_default() -> Self {
        Self {
            model: GateModel::Duck,
            threshold_db: -20.0,
            range_db: 20.0,
            attack_ms: 100.0,
            hold_ms: 100.0,
            release_ms: 500.0,
            accent_pct: 0.0,
            ratio: GateRatio::Gate,
        }
    }

    pub fn clamped(mut self) -> Self {
        self.threshold_db = self.threshold_db.clamp(-80.0, 0.0);
        self.range_db = self.range_db.clamp(3.0, RANGE_INF_DB);
        self.attack_ms = self.attack_ms.clamp(0.0, 120.0);
        self.hold_ms = match self.model {
            GateModel::Gate => self.hold_ms.clamp(1.0, 1000.0),
            GateModel::Duck => self.hold_ms.clamp(30.0, 30_000.0),
        };
        self.release_ms = self.release_ms.clamp(4.0, 4000.0);
        self.accent_pct = self.accent_pct.clamp(0.0, 100.0);
        self
    }

    /// Hysteresis in dB (GATE only).
    pub fn hysteresis_db(&self) -> f32 {
        match self.model {
            GateModel::Gate => self.accent_pct.clamp(0.0, 100.0) * 0.01 * ACCENT_MAX_HYST_DB,
            GateModel::Duck => 0.0,
        }
    }

    /// Attenuation depth in dB used internally (positive; [`MUTE_FLOOR_DB`] magnitude for "oo").
    pub fn depth_db(&self) -> f32 {
        if self.range_db >= RANGE_INF_DB {
            -MUTE_FLOOR_DB
        } else {
            self.range_db.max(3.0)
        }
    }
}

/// `gatesc.type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScFilter {
    #[default]
    Off,
    Lp12,
    Hp12,
    Bp,
    Notch,
}

/// `gatesc.src`: the key source. The engine resolves the buffer and passes it to [`Gate::process`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScSource {
    #[default]
    SelfKey,
    /// `CH.n` (1..=40).
    Channel(u8),
}

/// `gatesc.tap`: where on the key source the key is taken (resolved by the engine).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScTap {
    #[default]
    In,
    Filt,
    Tap3,
    Tap4,
    Tap5,
    Pfl,
    Afl,
    Post,
}

/// Gate side-chain parameters (`gatesc`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SidechainParams {
    pub filter: ScFilter,
    /// `f` 20 .. 20000 Hz.
    pub freq: f32,
    /// `q` 0.44 .. 10.
    pub q: f32,
    pub source: ScSource,
    pub tap: ScTap,
    /// `$solo` (SC SOLO / key listen): the engine sends [`Gate::key_signal`] to the solo bus while set.
    pub key_listen: bool,
}

impl Default for SidechainParams {
    fn default() -> Self {
        Self {
            filter: ScFilter::Off,
            freq: 1_002.374_4,
            q: 1.995_882,
            source: ScSource::SelfKey,
            tap: ScTap::In,
            key_listen: false,
        }
    }
}

impl SidechainParams {
    pub fn clamped(mut self) -> Self {
        self.freq = self.freq.clamp(20.0, 20_000.0);
        self.q = self.q.clamp(0.44, 10.0);
        self
    }
}

/// GATE / DUCK processor (stereo-linked).
pub struct Gate {
    params: GateParams,
    sc: SidechainParams,
    fs: f64,
    // Derived per-parameter values.
    thr: f32,
    thr_close: f32,
    depth: f32,
    up_rate: f32,
    down_rate: f32,
    hold_samples: u32,
    // State.
    /// Peak envelope (instant attack, one-pole release; holds exactly on a constant input).
    env: f32,
    env_rel: f32,
    sc_filt: [Svf; 2],
    gain_db: f32,
    engaged: bool,
    hold_ctr: u32,
    key_l: Vec<f32>,
    key_r: Vec<f32>,
    key_len: usize,
}

impl Default for Gate {
    fn default() -> Self {
        Self::new()
    }
}

/// dB per sample to traverse `depth` dB in `ms`; 0 ms = instant.
fn rate(depth: f32, ms: f32, fs: f64) -> f32 {
    if ms <= 0.0 {
        f32::INFINITY
    } else {
        (depth as f64 / (ms as f64 * 1e-3 * fs)) as f32
    }
}

impl Gate {
    pub fn new() -> Self {
        let mut g = Self {
            params: GateParams::default(),
            sc: SidechainParams::default(),
            fs: 48_000.0,
            thr: 0.0,
            thr_close: 0.0,
            depth: 0.0,
            up_rate: 0.0,
            down_rate: 0.0,
            hold_samples: 0,
            env: 0.0,
            env_rel: 0.0,
            sc_filt: [Svf::default(); 2],
            gain_db: 0.0,
            engaged: false,
            hold_ctr: 0,
            key_l: Vec::new(),
            key_r: Vec::new(),
            key_len: 0,
        };
        g.prepare(48_000.0, 64);
        g
    }

    /// Allocate key scratch for blocks up to `max_block` samples and reset state.
    pub fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        let n = max_block.max(1);
        self.key_l = vec![0.0; n];
        self.key_r = vec![0.0; n];
        self.env_rel = one_pole_coeff(DETECTOR_RELEASE_MS * 1e-3, sample_rate) as f32;
        let (p, s) = (self.params, self.sc);
        self.set_params(&p);
        self.set_sidechain(&s);
        self.reset();
    }

    /// Clear detector/filter state and return to the idle gain (GATE closed, DUCK released).
    pub fn reset(&mut self) {
        self.env = 0.0;
        for f in &mut self.sc_filt {
            f.reset();
        }
        self.engaged = false;
        self.hold_ctr = 0;
        self.gain_db = self.idle_gain();
        self.key_len = 0;
    }

    fn idle_gain(&self) -> f32 {
        match self.params.model {
            GateModel::Gate => -self.depth,
            GateModel::Duck => 0.0,
        }
    }

    pub fn params(&self) -> &GateParams {
        &self.params
    }

    pub fn sidechain(&self) -> &SidechainParams {
        &self.sc
    }

    /// Update model parameters (RT-safe). Values are clamped per model.
    pub fn set_params(&mut self, p: &GateParams) {
        let p = p.clamped();
        self.params = p;
        self.thr = p.threshold_db;
        self.thr_close = p.threshold_db - p.hysteresis_db();
        self.depth = p.depth_db();
        let (a, r) = (
            rate(self.depth, p.attack_ms, self.fs),
            rate(self.depth, p.release_ms, self.fs),
        );
        (self.up_rate, self.down_rate) = match p.model {
            GateModel::Gate => (a, r),
            GateModel::Duck => (r, a),
        };
        self.hold_samples = (p.hold_ms as f64 * 1e-3 * self.fs).round() as u32;
        self.gain_db = self.gain_db.clamp(-self.depth, 0.0);
    }

    /// Update side-chain parameters (RT-safe).
    pub fn set_sidechain(&mut self, s: &SidechainParams) {
        let s = s.clamped();
        self.sc = s;
        let mode = match s.filter {
            ScFilter::Off => return,
            ScFilter::Lp12 => SvfMode::Lowpass,
            ScFilter::Hp12 => SvfMode::Highpass,
            ScFilter::Bp => SvfMode::Bandpass,
            ScFilter::Notch => SvfMode::Notch,
        };
        for f in &mut self.sc_filt {
            f.set(mode, self.fs, s.freq as f64, s.q as f64, 0.0);
        }
    }

    /// Current gain in dB (0 = open / not ducking). For gain-reduction metering.
    pub fn gain_db(&self) -> f32 {
        self.gain_db
    }

    /// GATE: open (or holding). DUCK: ducking (or holding).
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Filtered key of the most recent block (or last `max_block` chunk): the key-listen signal.
    pub fn key_signal(&self) -> (&[f32], &[f32]) {
        (&self.key_l[..self.key_len], &self.key_r[..self.key_len])
    }

    /// Process a stereo block in place. `key` is the external side-chain (already taken at `gatesc.tap` of
    /// `gatesc.src`); it is used only when `source != SelfKey`, otherwise (or if `None`) the block keys
    /// from its own input. Blocks longer than `max_block` are processed in chunks.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32], key: Option<(&[f32], &[f32])>) {
        debug_assert_eq!(left.len(), right.len());
        let key = if self.sc.source == ScSource::SelfKey {
            None
        } else {
            key
        };
        let chunk = self.key_l.len();
        let mut start = 0;
        while start < left.len() {
            let end = (start + chunk).min(left.len());
            let k = key.map(|(kl, kr)| (&kl[start..end], &kr[start..end]));
            self.process_chunk(&mut left[start..end], &mut right[start..end], k);
            start = end;
        }
    }

    fn process_chunk(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        key: Option<(&[f32], &[f32])>,
    ) {
        let n = left.len();
        let (src_l, src_r): (&[f32], &[f32]) = match key {
            Some((kl, kr)) => (kl, kr),
            None => (left, right),
        };
        self.key_l[..n].copy_from_slice(src_l);
        self.key_r[..n].copy_from_slice(src_r);
        self.key_len = n;
        if self.sc.filter != ScFilter::Off {
            self.sc_filt[0].process_block(&mut self.key_l[..n]);
            self.sc_filt[1].process_block(&mut self.key_r[..n]);
            for f in &mut self.sc_filt {
                f.flush_denormals();
            }
        }
        let p = self.params;
        let expansion = match p.model {
            GateModel::Gate => p.ratio.expansion(),
            GateModel::Duck => None,
        };
        let floor = -self.depth;
        for i in 0..n {
            let k = self.key_l[i].abs().max(self.key_r[i].abs());
            self.env = if k >= self.env {
                k
            } else {
                self.env * self.env_rel
            };
            let level = lin_to_db_fast(self.env);
            // Open/hold state machine.
            if level > self.thr {
                self.engaged = true;
                self.hold_ctr = self.hold_samples;
            } else if self.engaged {
                if level >= self.thr_close {
                    self.hold_ctr = self.hold_samples;
                } else if self.hold_ctr > 0 {
                    self.hold_ctr -= 1;
                } else {
                    self.engaged = false;
                }
            }
            let target = match p.model {
                GateModel::Gate => {
                    if self.engaged {
                        0.0
                    } else {
                        match expansion {
                            None => floor,
                            Some(r) => ((r - 1.0) * (level - self.thr)).clamp(floor, 0.0),
                        }
                    }
                }
                GateModel::Duck => {
                    if self.engaged {
                        floor
                    } else {
                        0.0
                    }
                }
            };
            let g = self.gain_db;
            self.gain_db = if target > g {
                (g + self.up_rate).min(target)
            } else {
                (g - self.down_rate).max(target)
            };
            let lin = if self.gain_db <= MUTE_FLOOR_DB + 1e-3 {
                0.0
            } else {
                db_to_lin_fast(self.gain_db)
            };
            left[i] *= lin;
            right[i] *= lin;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_dsp::gain::db_to_lin;

    const FS: f64 = 48_000.0;

    fn gate(p: GateParams) -> Gate {
        let mut g = Gate::new();
        g.set_params(&p);
        g.prepare(FS, 64);
        g
    }

    /// Run `n` samples of a constant (DC) key at `level_db` through the gate keyed on itself.
    fn run_dc(g: &mut Gate, level_db: f32, n: usize) -> f32 {
        let x = db_to_lin(level_db);
        let mut l = vec![x; n];
        let mut r = vec![x; n];
        g.process(&mut l, &mut r, None);
        l[n - 1]
    }

    /// Per-sample gain trace for a constant key at `level_db` (one-sample blocks).
    fn trace(g: &mut Gate, level_db: Option<f32>, n: usize) -> Vec<f32> {
        let x = level_db.map_or(0.0, db_to_lin);
        (0..n)
            .map(|_| {
                let (mut l, mut r) = ([x], [x]);
                g.process(&mut l, &mut r, None);
                g.gain_db()
            })
            .collect()
    }

    fn ms(n: f64) -> f64 {
        n * 1e-3 * FS
    }

    #[test]
    fn hard_gate_static_curve_and_range() {
        for range in [3.0f32, 20.0, 40.0, 59.0] {
            let mut g = gate(GateParams {
                ratio: GateRatio::Gate,
                range_db: range,
                ..GateParams::gate_default()
            });
            for lvl in [-70.0f32, -50.0, -41.0, -39.0, -20.0, -1.0] {
                g.reset();
                let y = run_dc(&mut g, lvl, 48_000);
                let want = if lvl > -40.0 { 0.0 } else { -range };
                assert!(
                    (g.gain_db() - want).abs() < 1e-3,
                    "range {range} lvl {lvl}: gain {}",
                    g.gain_db()
                );
                let out_db = 20.0 * y.abs().log10();
                assert!((out_db - (lvl + want)).abs() < 1e-3);
            }
        }
        // "oo" range = exact silence.
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            range_db: 60.0,
            ..GateParams::gate_default()
        });
        assert_eq!(run_dc(&mut g, -60.0, 48_000), 0.0);
    }

    #[test]
    fn expander_static_curve() {
        for (ratio, r) in [
            (GateRatio::R1_5, 1.5f32),
            (GateRatio::R2, 2.0),
            (GateRatio::R3, 3.0),
            (GateRatio::R4, 4.0),
        ] {
            let mut g = gate(GateParams {
                ratio,
                range_db: 50.0,
                release_ms: 4.0,
                hold_ms: 1.0,
                ..GateParams::gate_default()
            });
            for lvl in [-80.0f32, -60.0, -50.0, -45.0, -40.5, -30.0] {
                g.reset();
                run_dc(&mut g, lvl, 48_000);
                let want = ((r - 1.0) * (lvl + 40.0)).clamp(-50.0, 0.0);
                assert!(
                    (g.gain_db() - want).abs() < 2e-3,
                    "ratio {r} lvl {lvl}: {} vs {want}",
                    g.gain_db()
                );
            }
        }
    }

    #[test]
    fn accent_hysteresis() {
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            accent_pct: 50.0, // 5 dB
            attack_ms: 0.0,
            hold_ms: 1.0,
            release_ms: 4.0,
            ..GateParams::gate_default()
        });
        run_dc(&mut g, -35.0, 4800);
        assert!(g.is_engaged());
        run_dc(&mut g, -43.0, 4800); // inside the hysteresis band: stays open
        assert!(g.is_engaged() && g.gain_db() == 0.0);
        run_dc(&mut g, -46.0, 4800); // below thr - 5: closes
        assert!(!g.is_engaged() && g.gain_db() == -40.0);
        run_dc(&mut g, -41.0, 4800); // re-opening needs thr
        assert!(!g.is_engaged());
        run_dc(&mut g, -39.0, 4800);
        assert!(g.is_engaged());
    }

    #[test]
    fn attack_hold_release_timing() {
        let (att, hld, rel) = (10.0f64, 50.0f64, 100.0f64);
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            range_db: 40.0,
            attack_ms: att as f32,
            hold_ms: hld as f32,
            release_ms: rel as f32,
            ..GateParams::gate_default()
        });
        // Attack: closed (-40) -> open (0) linearly in dB over `att`.
        let t = trace(&mut g, Some(-20.0), ms(200.0) as usize);
        let open_at = t.iter().position(|&x| x == 0.0).unwrap() as f64;
        assert!((open_at - ms(att)).abs() <= 2.0, "attack {open_at}");
        let mid = t[ms(att / 2.0) as usize];
        assert!((mid + 20.0).abs() < 0.2, "attack midpoint {mid}");
        // Key off: the detector falls from -20 dB to thr (-40) in tau*ln(10); then hold; then release.
        let fall = DETECTOR_RELEASE_MS * (10f64).ln();
        let t = trace(&mut g, None, ms(400.0) as usize);
        let rel_start = t.iter().position(|&x| x < 0.0).unwrap() as f64;
        assert!(
            (rel_start - ms(fall + hld)).abs() <= ms(0.2),
            "release start {rel_start} vs {}",
            ms(fall + hld)
        );
        let closed_at = t.iter().position(|&x| x <= -40.0).unwrap() as f64;
        assert!(
            (closed_at - rel_start - ms(rel)).abs() <= 2.0,
            "release {}",
            closed_at - rel_start
        );
    }

    #[test]
    fn ducker_follows_external_key() {
        let mut g = Gate::new();
        g.set_params(&GateParams {
            attack_ms: 20.0,
            hold_ms: 30.0,
            release_ms: 200.0,
            ..GateParams::duck_default()
        });
        g.set_sidechain(&SidechainParams {
            source: ScSource::Channel(5),
            ..Default::default()
        });
        g.prepare(FS, 64);
        assert_eq!(g.gain_db(), 0.0, "idle ducker passes");
        let n = 64;
        let run = |g: &mut Gate, key_db: Option<f32>, blocks: usize| -> f32 {
            let k = key_db.map_or(0.0, db_to_lin);
            let key = vec![k; n];
            let mut out = 0.0;
            for _ in 0..blocks {
                // Loud programme (0 dBFS) must not duck itself: only the key counts.
                let mut l = vec![1.0f32; n];
                let mut r = vec![1.0f32; n];
                g.process(&mut l, &mut r, Some((&key, &key)));
                out = l[n - 1];
            }
            out
        };
        assert!((run(&mut g, None, 100) - 1.0).abs() < 1e-6);
        // Key above thr (-20): ducks by range (20 dB) within attack.
        run(&mut g, Some(-10.0), (ms(20.0) as usize).div_ceil(n) + 1);
        assert!((g.gain_db() + 20.0).abs() < 1e-3, "{}", g.gain_db());
        let y = run(&mut g, Some(-10.0), 10);
        assert!((y - 0.1).abs() < 1e-4);
        // Key off: held, then released over 200 ms.
        run(&mut g, None, (ms(30.0) as usize) / n);
        assert!(g.gain_db() <= -19.99, "held: {}", g.gain_db());
        run(&mut g, None, (ms(400.0) as usize) / n);
        assert_eq!(g.gain_db(), 0.0);
        // SELF source ignores the external key.
        g.set_sidechain(&SidechainParams::default());
        let mut l = vec![db_to_lin(-40.0); n];
        let mut r = l.clone();
        let key = vec![1.0f32; n];
        g.process(&mut l, &mut r, Some((&key, &key)));
        assert_eq!(g.gain_db(), 0.0);
    }

    #[test]
    fn sidechain_filter_and_key_listen() {
        // 100 Hz key at -30 dB, threshold -40: opens unfiltered, stays closed with HP12 at 4 kHz.
        let key: Vec<f32> = (0..4920)
            .map(|i| {
                db_to_lin(-30.0) * (i as f32 * 2.0 * core::f32::consts::PI * 100.0 / 48_000.0).sin()
            })
            .collect();
        for (filter, open) in [
            (ScFilter::Off, true),
            (ScFilter::Hp12, false),
            (ScFilter::Lp12, true),
        ] {
            let mut g = Gate::new();
            g.set_params(&GateParams {
                ratio: GateRatio::Gate,
                ..GateParams::gate_default()
            });
            g.set_sidechain(&SidechainParams {
                filter,
                freq: 4000.0,
                q: 0.707,
                key_listen: true,
                ..Default::default()
            });
            g.prepare(FS, 8192);
            let mut l = key.clone();
            let mut r = key.clone();
            g.process(&mut l, &mut r, None);
            assert_eq!(g.is_engaged(), open, "{filter:?}");
            let (kl, _) = g.key_signal();
            assert_eq!(kl.len(), key.len());
            let kl = &kl[kl.len() - 480..];
            let peak = kl.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            if filter == ScFilter::Hp12 {
                assert!(
                    peak < db_to_lin(-60.0),
                    "key listen hears filtered key: {peak}"
                );
            } else {
                assert!(peak > db_to_lin(-31.0));
            }
        }
    }

    #[test]
    fn ratio_strings_and_clamps() {
        assert_eq!(GateRatio::from_snap("1:3"), Some(GateRatio::R3));
        assert_eq!(GateRatio::from_snap("gate"), Some(GateRatio::Gate));
        let p = GateParams {
            hold_ms: 5.0,
            release_ms: 8.02,
            ..GateParams::duck_default()
        }
        .clamped();
        assert_eq!(p.hold_ms, 30.0);
        assert_eq!(p.release_ms, 8.02);
    }
}
