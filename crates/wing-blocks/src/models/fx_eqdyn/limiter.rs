//! `LIMITER` (PRECISION LIMITER): look-ahead brickwall peak limiter with an inter-sample-peak estimate.
//!
//! Slots: 1 `gin` IN GAIN 0..18 dB, 2 `gout` OUT GAIN (ceiling) -18..0 dB, 3 `sqz` SQUEEZE 0..100,
//! 4 `knee` KNEE 0..10, 5 `again` AUTO GAIN, 6 `att` ATTACK 0.05..1 ms, 7 `rel` RELEASE 20..2000 ms.
//! Defaults (`fx_init_prec_limiter`): 0, -0.5, 0, 3, on, 0.05, 502.
//!
//! What WING-Edit shows (`fx_parchg_prec_limiter`): the input is scaled by `10^((gin-18)/20)` and the output
//! by `10^((gout+18)/20)` (18 dB internal headroom, so the limiter's own threshold is fixed and `gout` is the
//! output ceiling); attack and release are posted as per-sample rates `T*1000/ms`; `again` is a flag.
//!
//! ```text
//! in * gin --> look-ahead delay L ----------------------------------------> * g --> * makeup --> out
//!          \-> detector tap (L - A + 1 earlier) -> |peak| (incl. 4x inter-sample estimate)
//!               -> required gain (soft knee) -> running min over A -> A-tap moving average
//!               -> release (one-pole up, instant down) -> g
//! ```
//!
//! * **Brickwall guarantee**: the required gain of every sample is in the running minimum for the A samples
//!   before it reaches the output, and the moving average of A values that are each <= that requirement is
//!   <= it too; the release only lets the gain rise towards that bound. So `|out| <= ceiling` for every
//!   sample (tested). `A` = attack time in samples (the gain ramps down linearly over `att`).
//! * **Latency** is constant (`L` = 1 ms + 2 samples, 50 at 48 kHz) whatever `att`, so the FX slot's
//!   alignment never changes ([`Model::latency`]).
//! * **Inter-sample peaks** [I]: the detector also evaluates a 4-point cubic (Catmull-Rom) interpolation at
//!   1/4, 1/2 and 3/4 between samples ("true-peak-ish"; reconstructed overs are mostly caught).
//! * **Knee** [I]: `knee` is the soft-knee width in dB (quadratic knee centred on the ceiling: limiting starts
//!   `knee/2` dB below it and the curve meets the ceiling `knee/2` dB above it, so it is never exceeded).
//! * **Squeeze** [I]: programme-dependent release; the release rate is multiplied by `1 + 9 sqz/100` while
//!   the gain reduction is less than 3 dB (quick recovery from short peaks, louder result), as WING posts
//!   `sqz/100` and `sqz/200 + 0.5` weights.
//! * **Auto gain** [I]: on (default) = maximiser: `gin` raises the signal into the ceiling (`gout`). Off =
//!   `gin` only lowers the threshold (`gout - gin`) and the output is compensated by `-gin`, so material below
//!   the threshold passes at unity gain. Either way the ceiling `gout` is never exceeded.

use crate::model::{Model, ModelTelemetry, ParamValue};

/// Maximum attack (look-ahead) time.
pub const MAX_ATTACK_S: f64 = 0.001;
/// Extra look-ahead for the inter-sample estimator (it needs two newer samples).
const ISP_AHEAD: usize = 2;
/// Start of the sample clock (so `t - window` never wraps).
const T0: u64 = 1 << 32;

/// Schema slots.
pub mod slot {
    pub const GIN: u16 = 1;
    pub const GOUT: u16 = 2;
    pub const SQZ: u16 = 3;
    pub const KNEE: u16 = 4;
    pub const AGAIN: u16 = 5;
    pub const ATT: u16 = 6;
    pub const REL: u16 = 7;
}

/// WING-Edit defaults in slot order.
pub const DEFAULTS: [f32; 7] = [0.0, -0.5, 0.0, 3.0, 1.0, 0.05, 502.0];
const RANGES: [(f32, f32); 7] = [
    (0.0, 18.0),
    (-18.0, 0.0),
    (0.0, 100.0),
    (0.0, 10.0),
    (0.0, 1.0),
    (0.05, 1.0),
    (20.0, 2000.0),
];

/// Static gain (linear, <= 1) that keeps a peak `x` (linear, >= 0) at or below the threshold `t` (linear)
/// with a soft knee of `knee_db` dB.
#[inline]
pub fn static_gain(x: f64, t: f64, knee_db: f64) -> f64 {
    if x <= 0.0 {
        return 1.0;
    }
    if knee_db <= 0.0 {
        return if x > t { t / x } else { 1.0 };
    }
    let l = 20.0 * x.log10();
    let td = 20.0 * t.log10();
    let w = knee_db * 0.5;
    let lo = td - w;
    if l <= lo {
        return 1.0;
    }
    // quadratic knee over [td - w, td + w] (width `knee_db`): out = l - (l - lo)^2 / (4 w), slope 1 -> 0,
    // reaching exactly td at l = td + w; above: out = td. The output never exceeds td.
    let out = if l < td + w {
        l - (l - lo) * (l - lo) / (4.0 * w)
    } else {
        td
    };
    10f64.powf((out - l) / 20.0)
}

/// Fixed-capacity ring of f64.
#[derive(Clone, Debug, Default)]
struct Ring {
    buf: Vec<f64>,
    pos: usize,
}

impl Ring {
    fn new(n: usize, v: f64) -> Self {
        Self {
            buf: vec![v; n.max(1)],
            pos: 0,
        }
    }
    #[inline]
    fn push(&mut self, v: f64) {
        self.pos += 1;
        if self.pos == self.buf.len() {
            self.pos = 0;
        }
        self.buf[self.pos] = v;
    }
    /// `d = 0` = newest.
    #[inline]
    fn get(&self, d: usize) -> f64 {
        let n = self.buf.len();
        self.buf[(self.pos + n - (d % n)) % n]
    }
    fn fill(&mut self, v: f64) {
        self.buf.iter_mut().for_each(|x| *x = v);
    }
}

/// PRECISION LIMITER.
pub struct PrecisionLimiter {
    fs: f64,
    v: [f32; 7],
    dirty: bool,
    // derived
    gin: f64,
    makeup: f64,
    thr: f64,
    knee: f64,
    attack: usize,
    rel_c: f64,
    sqz: f64,
    // state
    la: usize,
    xl: Ring,
    xr: Ring,
    /// Detector input (abs of the newest samples, for the interpolator): 4 per channel.
    hist: [[f64; 4]; 2],
    req: Ring,
    /// Monotonic deque (indices into `req` time, values) for the running minimum.
    dq_t: Vec<u64>,
    dq_v: Vec<f64>,
    dq_head: usize,
    dq_len: usize,
    t: u64,
    avg: Ring,
    sum: f64,
    sum_n: usize,
    g: f64,
    gr_db: f32,
}

impl Default for PrecisionLimiter {
    fn default() -> Self {
        Self::new()
    }
}

impl PrecisionLimiter {
    pub fn new() -> Self {
        let mut m = Self {
            fs: 48_000.0,
            v: DEFAULTS,
            dirty: true,
            gin: 1.0,
            makeup: 1.0,
            thr: 1.0,
            knee: 3.0,
            attack: 1,
            rel_c: 0.0,
            sqz: 0.0,
            la: 1,
            xl: Ring::default(),
            xr: Ring::default(),
            hist: [[0.0; 4]; 2],
            req: Ring::default(),
            dq_t: Vec::new(),
            dq_v: Vec::new(),
            dq_head: 0,
            dq_len: 0,
            t: T0,
            avg: Ring::default(),
            sum: 0.0,
            sum_n: 0,
            g: 1.0,
            gr_db: 0.0,
        };
        m.prepare(48_000.0, 64);
        m
    }

    /// Current gain reduction (dB, >= 0).
    pub fn gain_reduction_db(&self) -> f32 {
        self.gr_db
    }

    /// Output ceiling (linear).
    pub fn ceiling(&self) -> f64 {
        10f64.powf(self.v[1].clamp(-18.0, 0.0) as f64 / 20.0)
    }

    fn max_attack(&self) -> usize {
        ((MAX_ATTACK_S * self.fs).round() as usize).max(1)
    }

    fn update(&mut self) {
        self.dirty = false;
        let v = self.v;
        let p = |i: usize| v[i] as f64;
        let gin = db_to_lin(p(0));
        let ceil = db_to_lin(p(1));
        self.gin = gin;
        if self.v[4] != 0.0 {
            self.thr = ceil;
            self.makeup = 1.0;
        } else {
            self.thr = ceil;
            self.makeup = 1.0 / gin;
        }
        self.knee = p(3);
        self.sqz = p(2) / 100.0;
        let a = ((p(5) * 1e-3 * self.fs).round() as usize).clamp(1, self.max_attack());
        if a != self.attack {
            self.attack = a;
            self.rebuild();
        }
        // release: per-sample one-pole towards the required gain (time constant `rel`)
        self.rel_c = 1.0 - (-1.0 / (p(6) * 1e-3 * self.fs)).exp();
    }

    /// Rebuild the running-min deque and the moving-average sum for a new attack length from history.
    fn rebuild(&mut self) {
        let a = self.attack;
        self.dq_head = 0;
        self.dq_len = 0;
        let t0 = self.t;
        for k in (0..a).rev() {
            let tk = t0.wrapping_sub(k as u64);
            let v = self.req.get(k);
            self.dq_push(tk, v);
        }
        // moving average over the last `a` running-min values: approximate from the current minimum
        let m = self.dq_front();
        self.avg.fill(m);
        self.sum = m * a as f64;
        self.sum_n = 0;
    }

    #[inline]
    fn dq_push(&mut self, t: u64, v: f64) {
        let cap = self.dq_v.len();
        while self.dq_len > 0 {
            let back = (self.dq_head + self.dq_len - 1) % cap;
            if self.dq_v[back] >= v {
                self.dq_len -= 1;
            } else {
                break;
            }
        }
        let i = (self.dq_head + self.dq_len) % cap;
        self.dq_v[i] = v;
        self.dq_t[i] = t;
        self.dq_len += 1;
    }

    #[inline]
    fn dq_front(&self) -> f64 {
        if self.dq_len == 0 {
            1.0
        } else {
            self.dq_v[self.dq_head]
        }
    }

    #[inline]
    fn dq_expire(&mut self, oldest: u64) {
        let cap = self.dq_v.len();
        while self.dq_len > 0 && self.dq_t[self.dq_head] < oldest {
            self.dq_head = (self.dq_head + 1) % cap;
            self.dq_len -= 1;
        }
    }
}

#[inline]
fn db_to_lin(db: f64) -> f64 {
    10f64.powf(db / 20.0)
}

/// Peak of the segment between `h[1]` and `h[2]` (Catmull-Rom over `h[0..4]`, oldest first) at 1/4, 1/2,
/// 3/4, together with the sample `h[2]` itself.
#[inline]
fn isp_peak(h: &[f64; 4]) -> f64 {
    let (p0, p1, p2, p3) = (h[0], h[1], h[2], h[3]);
    let a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
    let b = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
    let c = -0.5 * p0 + 0.5 * p2;
    let mut m = p2.abs();
    for t in [0.25, 0.5, 0.75] {
        let y = ((a * t + b) * t + c) * t + p1;
        m = m.max(y.abs());
    }
    m
}

impl Model for PrecisionLimiter {
    fn id(&self) -> &'static str {
        "LIMITER"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let amax = self.max_attack();
        self.la = amax + ISP_AHEAD;
        let n = self.la + 2;
        self.xl = Ring::new(n, 0.0);
        self.xr = Ring::new(n, 0.0);
        self.req = Ring::new(amax + 2, 1.0);
        self.avg = Ring::new(amax + 2, 1.0);
        self.dq_t = vec![0; amax + 2];
        self.dq_v = vec![1.0; amax + 2];
        self.attack = 0;
        self.update();
        self.reset();
    }

    fn latency(&self) -> usize {
        self.la
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let (la, a) = (self.la, self.attack);
        // detector tap: the sample whose gain must be final when it leaves the delay is `la` old; it enters
        // the min window `a - 1` samples before that.
        let det = la - (a - 1);
        let (thr, knee, gin, mk) = (self.thr, self.knee, self.gin, self.makeup);
        let mut gr_max = 0.0f64;
        for i in 0..n {
            let (xl, xr) = (left[i] as f64 * gin, right[i] as f64 * gin);
            self.xl.push(xl);
            self.xr.push(xr);
            // detector samples around the tap: det+1 (older) .. det-2 (newer)
            let mut pk = 0.0f64;
            for (ch, ring) in [&self.xl, &self.xr].into_iter().enumerate() {
                let h = [
                    ring.get(det + 1),
                    ring.get(det),
                    ring.get(det - 1),
                    ring.get(det - 2),
                ];
                self.hist[ch] = h;
                // segment between det and det-1 (h[1]..h[2]); h[2] is the newer sample
                pk = pk.max(isp_peak(&h)).max(h[1].abs());
            }
            let req = static_gain(pk, thr, knee);
            self.t = self.t.wrapping_add(1);
            self.req.push(req);
            self.dq_push(self.t, req);
            self.dq_expire(self.t.wrapping_sub(a as u64 - 1));
            let m = self.dq_front();
            // moving average of the running minimum over `a` samples
            let old = self.avg.get(a - 1);
            self.avg.push(m);
            self.sum += m - old;
            self.sum_n += 1;
            if self.sum_n >= 4096 {
                self.sum_n = 0;
                self.sum = (0..a).map(|k| self.avg.get(k)).sum();
            }
            let target = (self.sum / a as f64).min(m.max(0.0)).min(1.0);
            // release: rise slowly, fall instantly; squeeze speeds up recovery from small reductions
            if target < self.g {
                self.g = target;
            } else {
                let fast = if self.g > 0.7079 {
                    1.0 + 9.0 * self.sqz
                } else {
                    1.0
                };
                let c = (self.rel_c * fast).min(1.0);
                self.g += (target - self.g) * c;
                self.g = self.g.min(target);
            }
            let g = self.g;
            gr_max = gr_max.max(-20.0 * g.max(1e-9).log10());
            let (ol, or) = (self.xl.get(la), self.xr.get(la));
            left[i] = (ol * g * mk) as f32;
            right[i] = (or * g * mk) as f32;
        }
        self.gr_db = gr_max as f32;
    }

    fn reset(&mut self) {
        if self.dirty {
            self.update();
        }
        self.xl.fill(0.0);
        self.xr.fill(0.0);
        self.req.fill(1.0);
        self.avg.fill(1.0);
        self.hist = [[0.0; 4]; 2];
        self.t = T0;
        self.dq_head = 0;
        self.dq_len = 0;
        self.sum = self.attack as f64;
        self.sum_n = 0;
        self.g = 1.0;
        self.gr_db = 0.0;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < 7) else {
            return;
        };
        let (lo, hi) = RANGES[i];
        let x = match i {
            2..=4 => value.as_i32() as f32,
            _ => value.as_f32(),
        }
        .clamp(lo, hi);
        if x.is_finite() && x != self.v[i] {
            self.v[i] = x;
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.v.get(i).copied())
    }

    fn load_defaults(&mut self) {
        self.v = DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.gr_db,
            engaged: self.gr_db > 0.01,
        }
    }
}
