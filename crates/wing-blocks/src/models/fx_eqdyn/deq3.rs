//! `DEQ3` (TRIPLE DEQ): three channel dynamic-EQ bands ([`DeqBand`], T048) in series.
//!
//! Slots: per band `n = 1..3` at `9 (n-1) + 1 ..= 9 n`: `n-thr` -60..0 dB, `n-ratio` (1.2 1.3 1.5 2 3 5 10),
//! `n-att` 0..200 ms, `n-rel` 20..4000 ms, `n-filt` (OFF BP LP6 LP12 HP6 HP12), `n-g` -15..15 dB, `n-f`
//! 20..20k Hz, `n-q` 0.44..10, `n-mode` (low high) - the `DEQ` band layout three times. Defaults (schema):
//! -20 dB, 2, 50 ms, 301 ms, BP, 0 dB, 150 / 802 / 3990 Hz, Q 1, high.
//!
//! `fx_parchg_trideq` posts the same per-band block layout as the channel `dyneq` (band-pass "delta"
//! sections, `10^(g/20) - 1`, knee/ratio polynomials), so the channel band is reused unchanged (bands, key
//! filters, gain computer, ballistics: see [`crate::models::dyn_eq`]). Each band keys from the signal it
//! processes (the output of the previous band) [I]; the FX slot has no external side-chain.

use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::models::dyn_eq::{DeqBand, DeqBandParams, DeqFilter, DEQ_RATIOS};

/// Schema default band frequencies.
pub const FREQ_DEFAULTS: [f32; 3] = [150.0, 802.0, 3990.0];
/// Slots per band.
pub const BAND_SLOTS: usize = 9;

fn snap_ratio(v: f32) -> f32 {
    let mut best = DEQ_RATIOS[0];
    for &r in &DEQ_RATIOS {
        if (r - v).abs() < (best - v).abs() {
            best = r;
        }
    }
    best
}

fn values(p: &DeqBandParams) -> [f32; BAND_SLOTS] {
    [
        p.thr_db,
        p.ratio,
        p.att_ms,
        p.rel_ms,
        DeqFilter::ALL
            .iter()
            .position(|&f| f == p.filt)
            .unwrap_or(1) as f32,
        p.gain_db,
        p.freq,
        p.q,
        p.above as u32 as f32,
    ]
}

fn params(v: &[f32]) -> DeqBandParams {
    DeqBandParams {
        thr_db: v[0],
        ratio: v[1],
        att_ms: v[2],
        rel_ms: v[3],
        filt: DeqFilter::from_index(v[4] as usize),
        gain_db: v[5],
        freq: v[6],
        q: v[7],
        above: v[8] != 0.0,
    }
}

/// TRIPLE DEQ.
pub struct TripleDeq {
    v: [f32; 3 * BAND_SLOTS],
    dirty: bool,
    bands: [DeqBand; 3],
}

impl Default for TripleDeq {
    fn default() -> Self {
        Self::new()
    }
}

impl TripleDeq {
    pub fn new() -> Self {
        let mut m = Self {
            v: [0.0; 3 * BAND_SLOTS],
            dirty: true,
            bands: FREQ_DEFAULTS.map(|f| DeqBand::new(DeqBandParams::defaults(f))),
        };
        m.load_defaults();
        m.apply();
        m
    }

    /// Band `i` (0..3), staged parameters applied.
    pub fn band(&mut self, i: usize) -> &DeqBand {
        self.apply();
        &self.bands[i.min(2)]
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        for (i, b) in self.bands.iter_mut().enumerate() {
            b.set_params(params(&self.v[BAND_SLOTS * i..BAND_SLOTS * (i + 1)]));
        }
    }
}

impl Model for TripleDeq {
    fn id(&self) -> &'static str {
        "DEQ3"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.apply();
        for b in &mut self.bands {
            b.prepare(sample_rate);
        }
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        self.apply();
        for b in &mut self.bands {
            b.process(left, right, None);
        }
    }

    fn reset(&mut self) {
        self.apply();
        for b in &mut self.bands {
            b.reset();
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize)
            .checked_sub(1)
            .filter(|&i| i < 3 * BAND_SLOTS)
        else {
            return;
        };
        let x = match i % BAND_SLOTS {
            0 => value.as_f32().clamp(-60.0, 0.0),
            1 => match value {
                ParamValue::Enum(e) => match DEQ_RATIOS.get(e as usize) {
                    Some(&r) => r,
                    None => return,
                },
                other => snap_ratio(other.as_f32()),
            },
            2 => value.as_f32().clamp(0.0, 200.0),
            3 => value.as_f32().clamp(20.0, 4000.0),
            4 => value.as_i32().clamp(0, 5) as f32,
            5 => value.as_f32().clamp(-15.0, 15.0),
            6 => value.as_f32().clamp(20.0, 20_000.0),
            7 => value.as_f32().clamp(0.44, 10.0),
            _ => value.as_i32().clamp(0, 1) as f32,
        };
        if x.is_finite() {
            self.v[i] = x;
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.v.get(i).copied()
    }

    fn load_defaults(&mut self) {
        for (b, &f) in FREQ_DEFAULTS.iter().enumerate() {
            self.v[BAND_SLOTS * b..BAND_SLOTS * (b + 1)]
                .copy_from_slice(&values(&DeqBandParams::defaults(f)));
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        let g = self
            .bands
            .iter()
            .map(|b| b.dynamic_gain_db().abs())
            .fold(0.0, f32::max);
        ModelTelemetry {
            gain_reduction_db: g,
            engaged: g > 0.01,
        }
    }

    fn meter_bands(&self, out: &mut [f32]) -> usize {
        let n = self.bands.len().min(out.len());
        for (o, b) in out.iter_mut().zip(&self.bands[..n]) {
            *o = b.dynamic_gain_db();
        }
        n
    }
}
