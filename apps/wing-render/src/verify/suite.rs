//! The hardware-verification test signal ("suite"): one mono 48 kHz file played through channel 1 of every scene.
//!
//! ```text
//! silence | PILOT | SWEEP (log 20 Hz..20 kHz, 8 s) | STAIRS (1 kHz, -60..-4 dBFS, 4 dB steps)
//!         | STEPS (1 kHz, -40 / -10 dBFS, 3 cycles) | BURSTS (pink-ish noise at stepped levels)
//!         | TONE (1 kHz then 100 Hz, -12 dBFS) | END PILOT | silence
//! ```
//! The pilots (seeded noise bursts) align the recording with the render; the end pilot measures drift.
//! The layout is deterministic: both `gen` and `compare` rebuild it with [`build`].

use std::f64::consts::PI;

pub const FS: u32 = 48_000;
/// Bump when the layout changes (recordings made with another version cannot be compared).
pub const VERSION: u32 = 1;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    /// Frequency response (sweep).
    Fr,
    /// Static level staircase.
    Stairs,
    /// Level steps: attack/release behaviour.
    Steps,
    /// Noise bursts: detectors, gates.
    Bursts,
    /// Steady tones: harmonic distortion.
    Tone,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Fr => "fr",
            Kind::Stairs => "stairs",
            Kind::Steps => "steps",
            Kind::Bursts => "bursts",
            Kind::Tone => "tone",
        }
    }
    pub fn from_name(s: &str) -> Option<Kind> {
        [
            Kind::Fr,
            Kind::Stairs,
            Kind::Steps,
            Kind::Bursts,
            Kind::Tone,
        ]
        .into_iter()
        .find(|k| k.name() == s)
    }
}

/// One segment of the suite; `items` holds `(start, end, parameter)` triples (frames, parameter = level dBFS or
/// frequency Hz depending on the kind).
#[derive(Clone, Debug)]
pub struct Section {
    pub kind: Kind,
    pub start: usize,
    pub end: usize,
    pub items: Vec<(usize, usize, f64)>,
}

#[derive(Clone, Debug)]
pub struct Suite {
    pub samples: Vec<f32>,
    pub sections: Vec<Section>,
    /// Start frame and length of both pilots.
    pub pilot: (usize, usize),
    pub end_pilot: usize,
}

impl Suite {
    pub fn section(&self, k: Kind) -> &Section {
        self.sections.iter().find(|s| s.kind == k).expect("section")
    }
    pub fn seconds(&self) -> f64 {
        self.samples.len() as f64 / FS as f64
    }
}

struct Rng(u32);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0 as f64 / u32::MAX as f64 * 2.0 - 1.0
    }
    fn gauss(&mut self) -> f64 {
        (self.next() + self.next() + self.next() + self.next()) / 2.0
    }
}

fn s(sec: f64) -> usize {
    (sec * FS as f64).round() as usize
}

fn lin(db: f64) -> f64 {
    10f64.powf(db / 20.0)
}

fn ramp(i: usize, n: usize, r: usize) -> f64 {
    let edge = i.min(n - 1 - i.min(n - 1));
    if edge >= r {
        1.0
    } else {
        0.5 - 0.5 * (PI * edge as f64 / r as f64).cos()
    }
}

/// Build the suite.
pub fn build() -> Suite {
    let mut x: Vec<f32> = Vec::new();
    let mut sections = Vec::new();
    let silence = |x: &mut Vec<f32>, sec: f64| x.resize(x.len() + s(sec), 0.0);

    silence(&mut x, 0.5);
    let pilot = (x.len(), s(0.2));
    let mut rng = Rng(0x9e37_79b9);
    let push_pilot = |x: &mut Vec<f32>, rng: &mut Rng| {
        let n = s(0.2);
        for i in 0..n {
            x.push((rng.gauss() * 0.12 * ramp(i, n, s(0.005))) as f32);
        }
    };
    push_pilot(&mut x, &mut rng);
    silence(&mut x, 0.8);

    // SWEEP
    let start = x.len();
    let n = s(8.0);
    let (f1, f2) = (20.0f64, 20_000.0f64);
    let k = (f2 / f1).ln();
    for i in 0..n {
        let t = i as f64 / FS as f64;
        let ph = 2.0 * PI * f1 * 8.0 / k * ((t / 8.0 * k).exp() - 1.0);
        x.push((lin(-20.0) * ph.sin() * ramp(i, n, s(0.02))) as f32);
    }
    sections.push(Section {
        kind: Kind::Fr,
        start,
        end: x.len(),
        items: vec![],
    });
    silence(&mut x, 1.0);

    // STAIRS
    let start = x.len();
    let mut items = Vec::new();
    let mut ph = 0.0f64;
    for lvl in (-60..=-4).step_by(4) {
        silence(&mut x, 0.3);
        let (a, n) = (x.len(), s(0.5));
        for i in 0..n {
            x.push((lin(lvl as f64) * ph.sin() * ramp(i, n, s(0.01))) as f32);
            ph += 2.0 * PI * 1000.0 / FS as f64;
        }
        items.push((a, x.len(), lvl as f64));
    }
    sections.push(Section {
        kind: Kind::Stairs,
        start,
        end: x.len(),
        items,
    });
    silence(&mut x, 1.0);

    // STEPS: low, high, low, high, low, high, low (1.5 s each)
    let start = x.len();
    let mut items = Vec::new();
    for i in 0..7 {
        let lvl = if i % 2 == 0 { -40.0 } else { -10.0 };
        let a = x.len();
        for _ in 0..s(1.5) {
            x.push((lin(lvl) * ph.sin()) as f32);
            ph += 2.0 * PI * 1000.0 / FS as f64;
        }
        items.push((a, x.len(), lvl));
    }
    // fade the tail out to avoid a click at the end of the section
    let l = x.len();
    for i in 0..s(0.01) {
        x[l - 1 - i] *= (i as f32 / s(0.01) as f32).min(1.0);
    }
    sections.push(Section {
        kind: Kind::Steps,
        start,
        end: x.len(),
        items,
    });
    silence(&mut x, 1.0);

    // BURSTS: pink-ish noise (Voss-McCartney-style 3-pole filter), RMS = level
    let start = x.len();
    let mut items = Vec::new();
    let (mut b0, mut b1, mut b2) = (0.0f64, 0.0f64, 0.0f64);
    for lvl in [-50.0, -35.0, -25.0, -16.0, -25.0, -35.0, -50.0, -16.0] {
        let (a, n) = (x.len(), s(0.5));
        let mut buf = Vec::with_capacity(n);
        for _ in 0..n {
            let w = rng.gauss();
            b0 = 0.99765 * b0 + w * 0.0990460;
            b1 = 0.96300 * b1 + w * 0.2965164;
            b2 = 0.57000 * b2 + w * 1.0526913;
            buf.push(b0 + b1 + b2 + w * 0.1848);
        }
        let rms = (buf.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        for (i, v) in buf.iter().enumerate() {
            x.push((v / rms * lin(lvl) * ramp(i, n, s(0.005))) as f32);
        }
        items.push((a, x.len(), lvl));
        silence(&mut x, 0.5);
    }
    sections.push(Section {
        kind: Kind::Bursts,
        start,
        end: x.len(),
        items,
    });
    silence(&mut x, 0.5);

    // TONE
    let start = x.len();
    let mut items = Vec::new();
    for f in [1000.0f64, 100.0] {
        let (a, n) = (x.len(), s(2.5));
        for i in 0..n {
            x.push(
                (lin(-12.0) * (2.0 * PI * f * i as f64 / FS as f64).sin() * ramp(i, n, s(0.02)))
                    as f32,
            );
        }
        items.push((a, x.len(), f));
        silence(&mut x, 0.3);
    }
    sections.push(Section {
        kind: Kind::Tone,
        start,
        end: x.len(),
        items,
    });
    silence(&mut x, 0.7);

    // END PILOT: identical to the first
    let end_pilot = x.len();
    let first: Vec<f32> = x[pilot.0..pilot.0 + pilot.1].to_vec();
    x.extend_from_slice(&first);
    silence(&mut x, 1.0);

    Suite {
        samples: x,
        sections,
        pilot,
        end_pilot,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn suite_layout() {
        let s = build();
        assert!(s.seconds() > 40.0 && s.seconds() < 60.0, "{}", s.seconds());
        assert_eq!(s.sections.len(), 5);
        assert!(s.samples.iter().all(|v| v.abs() <= 0.95));
        let p = s.pilot;
        assert_eq!(
            s.samples[p.0..p.0 + p.1],
            s.samples[s.end_pilot..s.end_pilot + p.1]
        );
        // deterministic
        assert_eq!(build().samples, s.samples);
        let st = s.section(Kind::Stairs);
        assert_eq!(st.items.len(), 15);
        assert_eq!(s.section(Kind::Steps).items.len(), 7);
    }
}
