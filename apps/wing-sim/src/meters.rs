//! Meter sender side of T039: converts the engine's lock-free [`LevelBank`] (linear peaks since
//! the last read) into WING meter words in the protocol's [`MeterBank`], with ballistics, once
//! per meter packet tick (50 ms). Falls back to [`SyntheticMeters`] while no audio session is
//! attached.
//!
//! # Conversion (protocol.md §4.2)
//!
//! * Levels: `20 log10(peak)` dBFS in 1/256 dB, floored at [`FLOOR_DB`] (-128 dB, the i16
//!   minimum) for silence.
//! * Gain reduction: `256 * GR / full scale` ([`gr_to_meter`]), full scale 60 dB for the gate slot
//!   and 20 dB for the dynamics slot (protocol.md: "256 ≈ 20 dB GR for most models, the standard
//!   gate is 60 dB"). The value is the largest GR of the interval, no extra ballistics (the
//!   models have their own attack/release).
//! * V2 `gate LED` / `dyn state`: 1 while the slot reduces by more than [`ACTIVE_GR_DB`] [H].
//!   `automix gain`: 0 (automix is not modelled).
//!
//! # Ballistics (choice; the protocol docs do not specify any)
//!
//! Each tick takes the peak of the interval (so every transient reaches WING-Edit), then applies
//! instant attack and a linear-in-dB release of [`RELEASE_DB_PER_S`] (IEC 60268-18 digital peak
//! meter fall-back: 20 dB in 1.7 s). WING-Edit draws its own peak-hold markers on top. A steady
//! signal reads its true peak level.
//!
//! # What is real vs not yet (gaps)
//!
//! | WING meter | source |
//! |---|---|
//! | ch 1..40, aux 1..8 (V1/V2) | strip taps: in = tap IN, out = AFL (post mute/fader, pre pan), gate/dyn key + GR |
//! | bus 1..16, main 1..4, mtx 1..8 (V1/V2) | T036 mix graph ports: in = PFL (the sum; after the T037 hook once it exists), out = POST; gate/dyn words 0 until T037's hook writes its `StripMeter` |
//! | source groups (A7) | USB 1..48, OSC 1/2, $BUS 1..32, $MAIN 1..8, $MTX 1..16 live; LCL/AUX/AES50/SC/CRD/MOD/PLAY/AES carry silence in the simulator; USR (user-signal taps) not metered yet |
//! | output groups (A8) | USB 1..48 live; other groups are resolved by routing but not rendered: silent |
//! | monitor (A9) | T038 monitor section: solo bus L/R (louder of the two monitors' solo busses), mon 1/2 L/R outputs |
//! | dca, fx, rta | silent (DCA meters not derived yet; FX: T050; RTA) |

use std::sync::Arc;
use std::time::Instant;

use wing_engine::meters::{
    output_cell, solo_cell, source_cell, strip_cell, word, LevelBank, CELLS,
};
use wing_engine::meters::{SOURCE_BASE, STRIP_WORDS};
use wing_engine::routing::{OutAddr, OutGroup, SourceAddr, SrcGroup};
use wing_model::ParamStore;
use wing_proto::nrp::meter::{db_to_meter, gr_to_meter, GATE_GR_FULL_SCALE_DB, GR_FULL_SCALE_DB};
use wing_proto::nrp::MeterKind;
use wing_proto::server::{MeterBank, SyntheticMeters};

/// Silence (dB).
pub const FLOOR_DB: f32 = -128.0;
/// Level release (dB/s): IEC 60268-18 fall-back, 20 dB in 1.7 s.
pub const RELEASE_DB_PER_S: f32 = 20.0 / 1.7;
/// GR above which the V2 gate LED / dyn state reads 1 [H].
pub const ACTIVE_GR_DB: f32 = 0.5;

/// Strip ranges (first flat strip index, meter kinds).
const STRIP_KINDS: [(MeterKind, MeterKind, usize); 5] = [
    (MeterKind::Channel, MeterKind::ChannelV2, 0),
    (MeterKind::Aux, MeterKind::AuxV2, 40),
    (MeterKind::Bus, MeterKind::BusV2, 48),
    (MeterKind::Main, MeterKind::MainV2, 64),
    (MeterKind::Matrix, MeterKind::MatrixV2, 68),
];
const STRIPS_PER_KIND: [usize; 5] = [40, 8, 16, 4, 8];

fn is_gr_cell(cell: usize) -> bool {
    cell < SOURCE_BASE && matches!(cell % STRIP_WORDS, word::GATE_GR | word::DYN_GR)
}

/// Linear level to dB, floored.
#[inline]
pub fn lin_to_db(v: f32) -> f32 {
    if v > 0.0 {
        (20.0 * v.log10()).max(FLOOR_DB)
    } else {
        FLOOR_DB
    }
}

/// Converts a [`LevelBank`] into WING meter words (see the module docs).
pub struct RealMeters {
    levels: Arc<LevelBank>,
    /// Per cell: displayed level (dB) or, for GR cells, the interval's GR (dB).
    disp: Box<[f32]>,
    release_db_per_s: f32,
    last: Option<Instant>,
}

impl RealMeters {
    pub fn new(levels: Arc<LevelBank>) -> Self {
        let disp = (0..CELLS)
            .map(|c| if is_gr_cell(c) { 0.0 } else { FLOOR_DB })
            .collect();
        Self {
            levels,
            disp,
            release_db_per_s: RELEASE_DB_PER_S,
            last: None,
        }
    }

    pub fn levels(&self) -> &Arc<LevelBank> {
        &self.levels
    }

    /// Release rate (dB/s); `f32::INFINITY` = no ballistics (each tick shows its interval peak).
    pub fn set_release(&mut self, db_per_s: f32) {
        self.release_db_per_s = db_per_s;
    }

    /// Forget the ballistic state and any stale peaks.
    pub fn reset(&mut self) {
        for (c, d) in self.disp.iter_mut().enumerate() {
            *d = if is_gr_cell(c) { 0.0 } else { FLOOR_DB };
        }
        self.levels.clear();
        self.last = None;
    }

    /// Take the interval's peaks and advance the ballistics by `dt` seconds.
    pub fn update(&mut self, dt: f32) {
        let fall = self.release_db_per_s * dt.max(0.0);
        for (c, d) in self.disp.iter_mut().enumerate() {
            let v = self.levels.take(c);
            if is_gr_cell(c) {
                *d = v;
            } else {
                let p = lin_to_db(v);
                *d = if p >= *d { p } else { p.max(*d - fall) };
            }
        }
    }

    /// Displayed value of a cell (dB).
    pub fn value(&self, cell: usize) -> f32 {
        self.disp.get(cell).copied().unwrap_or(FLOOR_DB)
    }

    fn lvl(&self, cell: usize) -> i16 {
        db_to_meter(self.value(cell))
    }

    /// Write every meter word WING-Edit can request into `bank`.
    pub fn fill(&self, bank: &MeterBank) {
        let lay = bank.layout();
        let floor = db_to_meter(FLOOR_DB);
        for (k, &(v1, v2, first)) in STRIP_KINDS.iter().enumerate() {
            for i in 0..lay.count(v1).min(STRIPS_PER_KIND[k]) {
                let s = first + i;
                let c = |w| strip_cell(s, w);
                let (ggr, dgr) = (self.value(c(word::GATE_GR)), self.value(c(word::DYN_GR)));
                let (in_l, in_r) = (self.lvl(c(word::IN_L)), self.lvl(c(word::IN_R)));
                let (out_l, out_r) = (self.lvl(c(word::OUT_L)), self.lvl(c(word::OUT_R)));
                let (gkey, dkey) = (self.lvl(c(word::GATE_KEY)), self.lvl(c(word::DYN_KEY)));
                let gg = gr_to_meter(ggr, GATE_GR_FULL_SCALE_DB);
                let dg = gr_to_meter(dgr, GR_FULL_SCALE_DB);
                bank.set_block(v1, i, &[in_l, in_r, out_l, out_r, gkey, gg, dkey, dg]);
                let gled = (ggr > ACTIVE_GR_DB) as i16;
                let dstate = (dgr > ACTIVE_GR_DB) as i16;
                bank.set_block(
                    v2,
                    i,
                    &[
                        in_l, in_r, out_l, out_r, gkey, gg, gled, dkey, dg, dstate, 0,
                    ],
                );
            }
        }
        for g in 0..lay.count(MeterKind::Source) {
            let (_, w) = lay.block(MeterKind::Source, g).unwrap_or((0, 0));
            let grp = SrcGroup::ALL.get(g).copied();
            for k in 0..w {
                let v = grp
                    .and_then(|grp| SourceAddr::new(grp, k as u16 + 1))
                    .map_or(floor, |a| self.lvl(source_cell(a)));
                bank.set(MeterKind::Source, g, k, v);
            }
        }
        for g in 0..lay.count(MeterKind::Output) {
            let (_, w) = lay.block(MeterKind::Output, g).unwrap_or((0, 0));
            let grp = OutGroup::ALL.get(g).copied();
            for k in 0..w {
                let v = grp
                    .and_then(|grp| OutAddr::new(grp, k as u16 + 1))
                    .map_or(floor, |a| self.lvl(output_cell(a)));
                bank.set(MeterKind::Output, g, k, v);
            }
        }
        // monitor: solo L/R, mon1 L/R, mon2 L/R (T038 monitor node: solo busses, MON legs 1..4)
        let mon = |leg: u16| {
            SourceAddr::new(SrcGroup::Mon, leg).map_or(floor, |a| self.lvl(source_cell(a)))
        };
        bank.set_block(
            MeterKind::Monitor,
            0,
            &[
                self.lvl(solo_cell(0)),
                self.lvl(solo_cell(1)),
                mon(1),
                mon(2),
                mon(3),
                mon(4),
            ],
        );
        for i in 0..lay.count(MeterKind::Dca) {
            bank.set_block(MeterKind::Dca, i, &[floor; 4]);
        }
        for i in 0..lay.count(MeterKind::Fx) {
            bank.set_block(
                MeterKind::Fx,
                i,
                &[floor, floor, floor, floor, 0, 0, 0, 0, 0, 0],
            );
        }
        bank.set_block(MeterKind::Rta, 0, &[floor; 120]);
    }

    /// One sender tick at `now`: update (dt since the previous tick) and fill.
    pub fn tick(&mut self, bank: &MeterBank, now: Instant) {
        let dt = self
            .last
            .map_or(0.0, |t| now.saturating_duration_since(t).as_secs_f32());
        self.last = Some(now);
        self.update(dt);
        self.fill(bank);
    }
}

/// The meter streamer's per-tick source: real meters while an audio session is attached to
/// `levels`, synthetic ones (following `store`'s faders and mutes) otherwise.
pub fn meter_source(
    levels: Option<Arc<LevelBank>>,
    store: Option<Arc<ParamStore>>,
) -> impl FnMut(&MeterBank) + Send + 'static {
    let mut real = levels.map(RealMeters::new);
    let mut was_real = false;
    let t0 = Instant::now();
    move |bank| match &mut real {
        Some(r) if r.levels().is_attached() => {
            if !was_real {
                r.reset();
                was_real = true;
            }
            r.tick(bank, Instant::now());
        }
        _ => {
            was_real = false;
            SyntheticMeters::fill(bank, store.as_deref(), t0.elapsed().as_secs_f64());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_engine::meters::StripMeter;
    use wing_proto::nrp::meter::meter_to_db;

    #[test]
    fn levels_gr_and_ballistics() {
        let lv = LevelBank::shared();
        let mut rm = RealMeters::new(lv.clone());
        let bank = MeterBank::for_schema();
        let ch2 = StripMeter::at(lv.clone(), 1);
        ch2.level(word::IN_L, 0.5); // -6.02 dB
        ch2.level(word::DYN_GR, 10.0);
        ch2.level(word::GATE_GR, 30.0);
        lv.peak(source_cell(SourceAddr::new(SrcGroup::Usb, 3).unwrap()), 0.1);
        rm.update(0.05);
        rm.fill(&bank);
        let w = |k, i, j| bank.get(k, i, j).unwrap();
        assert!((meter_to_db(w(MeterKind::ChannelV2, 1, 0)) + 6.02).abs() < 0.01);
        assert_eq!(w(MeterKind::ChannelV2, 1, 1), i16::MIN);
        assert_eq!(w(MeterKind::ChannelV2, 1, 8), 128); // 10 dB of 20
        assert_eq!(w(MeterKind::ChannelV2, 1, 9), 1);
        assert_eq!(w(MeterKind::ChannelV2, 1, 5), 128); // 30 dB of 60
        assert_eq!(w(MeterKind::ChannelV2, 1, 6), 1);
        assert_eq!(w(MeterKind::Channel, 1, 7), 128);
        assert!((meter_to_db(w(MeterKind::Source, 6, 2)) + 20.0).abs() < 0.01);
        // silence: the level falls at the release rate, GR drops at once
        rm.update(0.5);
        rm.fill(&bank);
        let want = -6.02 - RELEASE_DB_PER_S * 0.5;
        assert!((meter_to_db(w(MeterKind::ChannelV2, 1, 0)) - want).abs() < 0.02);
        assert_eq!(w(MeterKind::ChannelV2, 1, 8), 0);
        for _ in 0..40 {
            rm.update(0.5);
        }
        rm.fill(&bank);
        assert_eq!(w(MeterKind::ChannelV2, 1, 0), i16::MIN);
    }
}
