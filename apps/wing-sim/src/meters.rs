//! Meter sender side of T039: converts the engine's lock-free [`LevelBank`] (linear peaks since
//! the last read) into WING meter words in the protocol's [`MeterBank`], with ballistics, once
//! per meter packet tick (50 ms). Falls back to [`SyntheticMeters`] while no audio session is
//! attached.
//!
//! # Conversion (protocol.md §4.2)
//!
//! * Levels: `20 log10(peak)` dBFS in 1/256 dB, floored at [`FLOOR_DB`] (-128 dB, the i16
//!   minimum) for silence.
//! * Gain reduction (T082): `256 * m`, `m` = the GR-meter position of the slot's **selected model**
//!   ([`DynMeterLaw`], WING-Edit's per-model `*_gain_*` law: 60 dB full scale for GATE/DUCK, 20 dB
//!   for most compressors, capped or negative for some, DEQ/DEQ2 from their band gains). The model is
//!   read from the parameter store (`ch/N/gate`, `ch/N/dyn`, ...), as WING-Edit decodes the word by
//!   its own `mdl`; without a store the gate slot uses the GATE law and the dynamics slot COMP's. The
//!   GR is the largest of the interval, no extra ballistics (the models have their own attack/release).
//! * V2 `gate LED` / `dyn state`: 1 while the slot reduces by more than [`ACTIVE_GR_DB`] [H] (DEQ2:
//!   band 2, see [`DynMeterLaw::words`]). `automix gain`: 0 (automix is not modelled).
//! * FX (`A6`, T082): in L/R = slot input, out L/R = slot output (levels of the owning insert's run,
//!   silence while no insert runs the slot), 6 state words from the model's GR and band gains through
//!   [`FxMeterLaw`] (see `wing_blocks::meter` for which models WING-Edit reads them for).
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
//! | fx 1..16 | FX slot in/out, model state words ([`FxMeterLaw`]) |
//! | dca 1..16 | pre L/R = loudest member's PFL, post L/R = loudest member's OUT (strip `tags` `#Dn`) [I] |
//! | rta (AA) | `crate::rta::RtaService` (T084): the analyser of the `cfg/rta` source/tap |

use std::sync::Arc;
use std::time::Instant;

use wing_engine::fx::FX_SLOTS;
use wing_engine::meters::{
    band, band_cell, fx_cell, fx_word, is_value_cell, output_cell, solo_cell, source_cell,
    strip_cell, word, LevelBank, CELLS,
};
use wing_engine::meters::{DynMeterLaw, FxMeterLaw, MAX_METER_BANDS};
use wing_engine::mix::logic::parse_tags;
use wing_engine::routing::{OutAddr, OutGroup, SourceAddr, SrcGroup, Strip, STRIP_COUNT};
use wing_model::{NodeId, ParamStore};
use wing_proto::nrp::meter::db_to_meter;
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

/// Model blocks whose selected model sets a meter law: per strip (gate, dyn) and per FX slot; and each
/// strip's `tags` (DCA membership for the DCA meters).
struct ModelBlocks {
    store: Arc<ParamStore>,
    gate: Vec<Option<NodeId>>,
    dyn_: Vec<Option<NodeId>>,
    fx: Vec<Option<NodeId>>,
    tags: Vec<Option<NodeId>>,
}

impl ModelBlocks {
    fn new(store: Arc<ParamStore>) -> Self {
        let path = |s: usize, blk: &str| {
            let p = match Strip::from_index(s)? {
                Strip::Ch(n) => format!("ch/{n}/{blk}"),
                Strip::Aux(n) => format!("aux/{n}/{blk}"),
                Strip::Bus(n) => format!("bus/{n}/{blk}"),
                Strip::Main(n) => format!("main/{n}/{blk}"),
                Strip::Mtx(n) => format!("mtx/{n}/{blk}"),
            };
            store.lookup(&p)
        };
        let gate = (0..STRIP_COUNT).map(|s| path(s, "gate")).collect();
        let tags = (0..STRIP_COUNT).map(|s| path(s, "tags")).collect();
        let dyn_ = (0..STRIP_COUNT).map(|s| path(s, "dyn")).collect();
        let fx = (1..=FX_SLOTS)
            .map(|n| store.lookup(&format!("fx/{n}")))
            .collect();
        Self {
            store,
            gate,
            dyn_,
            fx,
            tags,
        }
    }

    /// DCA membership mask of strip `s` (bit k = DCA k + 1, `#Dn` tags).
    fn dcas(&self, s: usize) -> u16 {
        self.tags
            .get(s)
            .copied()
            .flatten()
            .and_then(|id| self.store.with_str(id, |t| parse_tags(t).0))
            .unwrap_or(0)
    }

    fn model(&self, block: Option<NodeId>) -> Option<&'static str> {
        Some(self.store.active_model(block?)?.name)
    }
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
    models: Option<ModelBlocks>,
    /// Per cell: displayed level (dB) or, for GR cells, the interval's GR (dB).
    disp: Box<[f32]>,
    release_db_per_s: f32,
    last: Option<Instant>,
}

impl RealMeters {
    pub fn new(levels: Arc<LevelBank>) -> Self {
        let disp = (0..CELLS)
            .map(|c| if is_value_cell(c) { 0.0 } else { FLOOR_DB })
            .collect();
        Self {
            levels,
            models: None,
            disp,
            release_db_per_s: RELEASE_DB_PER_S,
            last: None,
        }
    }

    /// Resolve the gate/dynamics/FX meter laws from the models selected in `store` (see the module docs).
    pub fn with_store(mut self, store: Arc<ParamStore>) -> Self {
        self.models = Some(ModelBlocks::new(store));
        self
    }

    /// GR-meter law of strip `s`'s gate (`gate`) or dynamics slot.
    fn dyn_law(&self, s: usize, gate: bool) -> DynMeterLaw {
        let id = self
            .models
            .as_ref()
            .and_then(|m| m.model(*(if gate { &m.gate } else { &m.dyn_ }).get(s)?));
        DynMeterLaw::for_model(id.unwrap_or(if gate { "GATE" } else { "COMP" }))
    }

    /// State-word law of FX slot `k` (0-based).
    fn fx_law(&self, k: usize) -> FxMeterLaw {
        self.models
            .as_ref()
            .and_then(|m| m.model(*m.fx.get(k)?))
            .map_or(FxMeterLaw::None, FxMeterLaw::for_model)
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
            *d = if is_value_cell(c) { 0.0 } else { FLOOR_DB };
        }
        self.levels.clear();
        self.last = None;
    }

    /// Take the interval's peaks and advance the ballistics by `dt` seconds.
    pub fn update(&mut self, dt: f32) {
        let fall = self.release_db_per_s * dt.max(0.0);
        for (c, d) in self.disp.iter_mut().enumerate() {
            let v = self.levels.take(c);
            if is_value_cell(c) {
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
                let bands = |slot: usize| [0, 1].map(|k| self.value(band_cell(s, slot + k)));
                let (gg, gled) =
                    self.dyn_law(s, true)
                        .words(ggr, &bands(band::GATE), ggr > ACTIVE_GR_DB);
                let (dg, dstate) =
                    self.dyn_law(s, false)
                        .words(dgr, &bands(band::DYN), dgr > ACTIVE_GR_DB);
                bank.set_block(v1, i, &[in_l, in_r, out_l, out_r, gkey, gg, dkey, dg]);
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
        let dcas = self.dca_levels();
        for i in 0..lay.count(MeterKind::Dca) {
            let w = dcas.get(i).map_or([FLOOR_DB; 4], |d| *d).map(db_to_meter);
            bank.set_block(MeterKind::Dca, i, &w);
        }
        for i in 0..lay.count(MeterKind::Fx).min(FX_SLOTS) {
            let c = |w| fx_cell(i, w);
            let bands: [f32; MAX_METER_BANDS] =
                std::array::from_fn(|k| self.value(c(fx_word::BAND0 + k)));
            let st = self.fx_law(i).words(self.value(c(fx_word::GR)), &bands);
            let mut w = [floor; 10];
            for (k, wk) in [fx_word::IN_L, fx_word::IN_R, fx_word::OUT_L, fx_word::OUT_R]
                .into_iter()
                .enumerate()
            {
                w[k] = self.lvl(c(wk));
            }
            w[4..].copy_from_slice(&st);
            bank.set_block(MeterKind::Fx, i, &w);
        }
        // RTA (AA): written by the RTA service (T084, crate::rta), not here
    }

    /// DCA meters (pre L/R, post L/R in dB) of DCA 1..16: the loudest member strip's pre-fader (PFL) and
    /// post-fader (OUT, incl. its DCA offsets and mutes) levels [I]. Silent without a store (no membership).
    pub fn dca_levels(&self) -> [[f32; 4]; 16] {
        let mut d = [[FLOOR_DB; 4]; 16];
        let Some(m) = &self.models else {
            return d;
        };
        for s in 0..STRIP_COUNT {
            let mask = m.dcas(s);
            if mask == 0 {
                continue;
            }
            let c = |w| self.value(strip_cell(s, w));
            let lv = [
                c(word::PRE_L),
                c(word::PRE_R),
                c(word::OUT_L),
                c(word::OUT_R),
            ];
            for (k, dk) in d.iter_mut().enumerate() {
                if mask >> k & 1 != 0 {
                    for (a, b) in dk.iter_mut().zip(lv) {
                        *a = a.max(b);
                    }
                }
            }
        }
        d
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
    let mut real = levels.map(|l| {
        let r = RealMeters::new(l);
        match &store {
            Some(s) => r.with_store(s.clone()),
            None => r,
        }
    });
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

    /// DCA meters: the loudest member's PFL / OUT levels (user report: DCA meters were silent).
    #[test]
    fn dca_meters_follow_members() {
        let store = Arc::new(ParamStore::new());
        let tag = |p: &str, t: &str| {
            let id = store.lookup(p).unwrap();
            store.set_str(id, t, wing_model::ORIGIN_LOCAL).unwrap();
        };
        tag("ch/2/tags", "#D3,#M1");
        tag("bus/1/tags", "#D3");
        tag("ch/5/tags", "#D4");
        let lv = LevelBank::shared();
        let mut rm = RealMeters::new(lv.clone()).with_store(store);
        let bank = MeterBank::for_schema();
        let ch2 = StripMeter::at(lv.clone(), 1);
        ch2.level(word::PRE_L, 0.5);
        ch2.level(word::PRE_R, 0.5);
        ch2.level(word::OUT_L, 0.25);
        let bus1 = StripMeter::at(lv.clone(), 48);
        bus1.level(word::OUT_L, 0.1);
        bus1.level(word::OUT_R, 0.1);
        rm.update(0.05);
        rm.fill(&bank);
        let w = |i, j| meter_to_db(bank.get(MeterKind::Dca, i, j).unwrap());
        assert!((w(2, 0) + 6.02).abs() < 0.01 && (w(2, 1) + 6.02).abs() < 0.01);
        assert!((w(2, 2) + 12.04).abs() < 0.01, "post L = ch2 OUT");
        assert!((w(2, 3) + 20.0).abs() < 0.01, "post R = bus 1 OUT");
        // DCA 4's only member is silent, DCA 1 has none
        assert_eq!(bank.get(MeterKind::Dca, 3, 0).unwrap(), i16::MIN);
        assert_eq!(bank.get(MeterKind::Dca, 0, 2).unwrap(), i16::MIN);
    }

    /// T082: GR words follow the selected model's law; FX blocks carry slot levels and state words.
    #[test]
    fn per_model_gr_laws_and_fx_blocks() {
        use wing_engine::meters::FxMeter;
        let store = Arc::new(ParamStore::new());
        let set = |path: &str, name: &str| {
            let id = store.lookup(path).unwrap();
            store.set_enum_name(id, name, 0).unwrap();
        };
        set("ch/2/dyn/mdl", "76LA");
        set("ch/2/gate/mdl", "COMP");
        set("ch/3/dyn/mdl", "DEQ2");
        set("bus/1/dyn/mdl", "E88");
        set("fx/1/mdl", "*SOUL*");
        set("fx/2/mdl", "C5-CMB");
        let lv = LevelBank::shared();
        let mut rm = RealMeters::new(lv.clone()).with_store(store.clone());
        let bank = MeterBank::for_schema();
        let ch2 = StripMeter::at(lv.clone(), 1);
        ch2.level(word::DYN_GR, 10.0);
        ch2.level(word::GATE_GR, 10.0);
        let ch3 = StripMeter::at(lv.clone(), 2);
        ch3.bands(band::DYN, &[-6.0206, 0.0]);
        StripMeter::at(lv.clone(), 48).level(word::DYN_GR, 40.0);
        let fx1 = FxMeter::new(lv.clone(), 1);
        fx1.input(&[0.5], &[0.25]);
        lv.set(fx_cell(1, fx_word::BAND0), -3.0);
        rm.update(0.05);
        rm.fill(&bank);
        let w = |k, i, j| bank.get(k, i, j).unwrap();
        // 76LA: needle deflects the other way, -GR/20
        assert_eq!(w(MeterKind::ChannelV2, 1, 8), -128);
        assert_eq!(w(MeterKind::Channel, 1, 7), -128);
        // a compressor in the gate slot uses the compressor law (20 dB), not the gate's 60 dB
        assert_eq!(w(MeterKind::ChannelV2, 1, 5), 128);
        assert_eq!(w(MeterKind::ChannelV2, 1, 6), 1);
        // DEQ2: band 1 at -6 dB in the gain word, band 2 in the state word
        assert_eq!(w(MeterKind::ChannelV2, 2, 8), -128);
        assert_eq!(w(MeterKind::ChannelV2, 2, 9), 0);
        // bus 1 E88: GR/20 capped at 2.5
        assert_eq!(w(MeterKind::BusV2, 0, 8), 512);
        // FX 1 (*SOUL*): levels, LEDs off (g = 1 -> 2048)
        assert!((meter_to_db(w(MeterKind::Fx, 0, 0)) + 6.02).abs() < 0.01);
        assert!((meter_to_db(w(MeterKind::Fx, 0, 1)) + 12.04).abs() < 0.01);
        assert_eq!(w(MeterKind::Fx, 0, 2), i16::MIN);
        assert_eq!(w(MeterKind::Fx, 0, 4), 2048);
        assert_eq!(w(MeterKind::Fx, 0, 6), 2048);
        // FX 2 (C5-CMB): band 1 at -3 dB
        assert_eq!(w(MeterKind::Fx, 1, 4), -1024);
        assert_eq!(w(MeterKind::Fx, 1, 5), 0);
        // FX 3 (no model): silent levels, no state
        assert_eq!(w(MeterKind::Fx, 2, 0), i16::MIN);
        assert_eq!(w(MeterKind::Fx, 2, 4), 0);
    }
}
