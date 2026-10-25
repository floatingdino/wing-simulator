//! The resolved routing table and its builder (control thread, reads a [`ParamStore`]).

use std::sync::atomic::{AtomicBool, Ordering::*};
use std::sync::Arc;

use wing_model::{Change, Hook, NodeId as ParamId, ParamStore};

use super::addr::*;

/// Number of input strips that take a source (40 channels + 8 aux).
pub const INPUT_STRIPS: usize = 48;
/// Number of USB channels in each direction.
pub const USB_CHANNELS: usize = 48;
/// FX slots whose EXT model (`fx/N/mdl = EXT`) takes a return source (`egrp/ein/emode`, T05C).
pub const EXT_SLOTS: usize = 16;

/// What actually drives a resolved leg on the RT side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Feed {
    /// Nothing (OFF, out of range, or a group without hardware in the simulator).
    #[default]
    Silence,
    /// Driver channel `USB In k+1` (engine external input `k`).
    Usb(u8),
    /// Internal oscillator `k` (0 or 1).
    Osc(u8),
    /// An engine tap (mix output leg or strip tap).
    Tap(TapId),
    /// Mono sum (L + R) * 0.5 of a strip tap; the id is the L side, R is id + 1.
    TapMono(TapId),
}

/// A resolved mono leg: the addressed source, what feeds it and the linear gain to apply
/// (preamp gain, source mute and polarity folded in).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Leg {
    /// Addressed source (`None` = OFF / invalid).
    pub src: Option<SourceAddr>,
    pub feed: Feed,
    pub gain: f32,
}

impl Leg {
    pub const SILENT: Leg = Leg {
        src: None,
        feed: Feed::Silence,
        gain: 0.0,
    };
}

/// Channel width implied by the source's `mode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    /// Mono source: both sides carry the same leg.
    Mono,
    /// Stereo pair (legs n, n+1).
    Stereo,
    /// Mid/side pair (legs n = M, n+1 = S), decoded to L = M + S, R = M - S.
    MidSide,
}

/// Resolved source of one input strip (ch 1..40, aux 1..8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputRoute {
    /// Main source (`conn.grp/in`), `None` = OFF.
    pub main: Option<SourceAddr>,
    /// Alt source (`conn.altgrp/altin`), `None` = OFF.
    pub alt: Option<SourceAddr>,
    /// Whether the alt source is selected (`altsrc || (srcauto && io.altsw)`).
    pub use_alt: bool,
    pub width: Width,
    /// L and R legs (identical for mono).
    pub legs: [Leg; 2],
}

impl InputRoute {
    /// The selected source (first leg).
    pub fn source(&self) -> Option<SourceAddr> {
        if self.use_alt {
            self.alt
        } else {
            self.main
        }
    }

    const OFF: InputRoute = InputRoute {
        main: None,
        alt: None,
        use_alt: false,
        width: Width::Mono,
        legs: [Leg::SILENT; 2],
    };
}

/// Oscillator waveform (`io/in/OSC/n/osc/mode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OscMode {
    Sine,
    Pink,
    White,
}

/// Oscillator settings (`io/in/OSC/n/osc`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OscParams {
    pub mode: OscMode,
    /// RMS level, linear (from `lvl` dBFS) [I].
    pub level: f32,
    /// Sine frequency in Hz.
    pub freq: f32,
}

/// The complete, immutable routing state the RT side works from.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingTable {
    /// Global ALT INPUT switch (`io/altsw`).
    pub altsw: bool,
    inputs: Box<[InputRoute]>,
    /// EXT returns of FX slots 1..16 (`main` = the source, `None` = the slot is not an active EXT).
    ext: Box<[InputRoute]>,
    outputs: Box<[Leg]>,
    pub osc: [OscParams; 2],
}

impl RoutingTable {
    /// Input channel `n` (1..40).
    pub fn channel(&self, n: usize) -> &InputRoute {
        assert!((1..=40).contains(&n), "channel {n}");
        &self.inputs[n - 1]
    }

    /// Aux channel `n` (1..8).
    pub fn aux(&self, n: usize) -> &InputRoute {
        assert!((1..=8).contains(&n), "aux {n}");
        &self.inputs[40 + n - 1]
    }

    /// Input strip by flat index (0..40 = ch, 40..48 = aux).
    #[inline]
    pub fn input(&self, i: usize) -> &InputRoute {
        &self.inputs[i]
    }

    /// External return of FX slot `n` (1..16): legs L/R of the slot's `EXT` model; `source()` is `None`
    /// unless the slot runs `EXT` with a return group other than OFF. `Width::Mono` for `emode` M (both
    /// legs the same source), `Width::Stereo` for ST and M/S (the odd-aligned pair; the model decodes M/S).
    #[inline]
    pub fn ext(&self, n: usize) -> &InputRoute {
        &self.ext[n - 1]
    }

    /// What output `addr` carries.
    #[inline]
    pub fn output(&self, addr: OutAddr) -> &Leg {
        &self.outputs[addr.index()]
    }

    /// What USB out `n` (1..48, the driver's `USB Out n`) carries.
    #[inline]
    pub fn usb_out(&self, n: usize) -> &Leg {
        &self.outputs[OutGroup::Usb.count_before() + n - 1]
    }
}

impl OutGroup {
    fn count_before(self) -> usize {
        OutAddr { grp: self, n: 1 }.index()
    }
}

#[derive(Clone, Copy, Default)]
struct SrcNodes {
    mode: Option<ParamId>,
    g: Option<ParamId>,
    mute: Option<ParamId>,
    pol: Option<ParamId>,
    ugrp: Option<ParamId>,
    uin: Option<ParamId>,
    utap: Option<ParamId>,
    ulr: Option<ParamId>,
    olvl: Option<ParamId>,
    omode: Option<ParamId>,
    of: Option<ParamId>,
}

#[derive(Clone, Copy, Default)]
struct StripNodes {
    grp: Option<ParamId>,
    inn: Option<ParamId>,
    altgrp: Option<ParamId>,
    altin: Option<ParamId>,
    srcauto: Option<ParamId>,
    altsrc: Option<ParamId>,
}

#[derive(Clone, Copy, Default)]
struct ExtNodes {
    mdl: Option<ParamId>,
    egrp: Option<ParamId>,
    ein: Option<ParamId>,
    emode: Option<ParamId>,
    /// Selector enum index of `EXT` in the slot's model group.
    ext_index: Option<i64>,
}

#[derive(Clone, Copy, Default)]
struct OutNodes {
    grp: Option<ParamId>,
    inn: Option<ParamId>,
}

/// Builds [`RoutingTable`]s from a [`ParamStore`]. Construct once (resolves every parameter it
/// reads); [`RoutingBuilder::build`] is then cheap and may run on every routing change.
pub struct RoutingBuilder {
    altsw: Option<ParamId>,
    src: Box<[SrcNodes]>,
    strips: Box<[StripNodes]>,
    ext: Box<[ExtNodes]>,
    outs: Box<[OutNodes]>,
    watch: Arc<[bool]>,
}

fn num(store: &ParamStore, id: Option<ParamId>) -> Option<f64> {
    store.get_f64(id?)
}

fn flag(store: &ParamStore, id: Option<ParamId>) -> bool {
    num(store, id).is_some_and(|v| v != 0.0)
}

fn int(store: &ParamStore, id: Option<ParamId>, default: i64) -> i64 {
    num(store, id).map_or(default, |v| v as i64)
}

fn db_to_lin(db: f64) -> f32 {
    10f64.powf(db / 20.0) as f32
}

impl RoutingBuilder {
    pub fn new(store: &ParamStore) -> Self {
        let mut watched = Vec::new();
        let mut look = |path: String| {
            let id = store.lookup(&path);
            if let Some(id) = id {
                watched.push(id);
            }
            id
        };
        let altsw = look("io/altsw".into());
        let mut src = vec![SrcNodes::default(); SOURCE_COUNT].into_boxed_slice();
        for (i, s) in src.iter_mut().enumerate() {
            let a = SourceAddr::from_index(i).expect("source index");
            let p = format!("io/in/{}/{}", a.grp.io_name(), a.n);
            s.mode = look(format!("{p}/mode"));
            if a.grp.has_preamp() {
                s.g = look(format!("{p}/g"));
            }
            if a.grp.mix_group().is_none() {
                s.mute = look(format!("{p}/mute"));
                s.pol = look(format!("{p}/pol"));
            }
            if a.grp == SrcGroup::Usr {
                s.ugrp = look(format!("{p}/user/grp"));
                s.uin = look(format!("{p}/user/in"));
                s.utap = look(format!("{p}/user/tap"));
                s.ulr = look(format!("{p}/user/lr"));
            }
            if a.grp == SrcGroup::Osc {
                s.olvl = look(format!("{p}/osc/lvl"));
                s.omode = look(format!("{p}/osc/mode"));
                s.of = look(format!("{p}/osc/f"));
            }
        }
        let mut strips = vec![StripNodes::default(); INPUT_STRIPS].into_boxed_slice();
        for (i, s) in strips.iter_mut().enumerate() {
            let p = if i < 40 {
                format!("ch/{}/in", i + 1)
            } else {
                format!("aux/{}/in", i - 39)
            };
            s.grp = look(format!("{p}/conn/grp"));
            s.inn = look(format!("{p}/conn/in"));
            s.altgrp = look(format!("{p}/conn/altgrp"));
            s.altin = look(format!("{p}/conn/altin"));
            s.srcauto = look(format!("{p}/set/srcauto"));
            s.altsrc = look(format!("{p}/set/altsrc"));
        }
        let mut ext = vec![ExtNodes::default(); EXT_SLOTS].into_boxed_slice();
        for (i, e) in ext.iter_mut().enumerate() {
            let n = i + 1;
            e.mdl = look(format!("fx/{n}/mdl"));
            // EXT params: slot 1 egrp, 2 ein, 3 emode (models.md §6)
            e.egrp = look(format!("fx/{n}/1"));
            e.ein = look(format!("fx/{n}/2"));
            e.emode = look(format!("fx/{n}/3"));
            let key = if n < 9 { "fx/1" } else { "fx/9" };
            let schema = store.schema();
            e.ext_index = schema
                .groups()
                .iter()
                .find(|g| g.key == key)
                .and_then(|g| schema.models(g).iter().find(|m| m.name == "EXT"))
                .map(|m| m.enum_index as i64);
        }
        let mut outs = vec![OutNodes::default(); OUTPUT_COUNT].into_boxed_slice();
        for (i, o) in outs.iter_mut().enumerate() {
            let a = OutAddr::from_index(i).expect("output index");
            let p = format!("io/out/{}/{}", a.grp.token(), a.n);
            o.grp = look(format!("{p}/grp"));
            o.inn = look(format!("{p}/in"));
        }
        let mut watch = vec![false; store.schema().node_count()];
        for id in watched {
            watch[id.index()] = true;
        }
        Self {
            altsw,
            src,
            strips,
            ext,
            outs,
            watch: watch.into(),
        }
    }

    /// Whether a change to `id` can alter the routing table.
    pub fn watches(&self, id: ParamId) -> bool {
        self.watch.get(id.index()).copied().unwrap_or(false)
    }

    /// A [`ParamStore`] hook that raises `dirty` whenever a routing parameter changes.
    pub fn change_hook(&self, dirty: Arc<AtomicBool>) -> Hook {
        let watch = self.watch.clone();
        Box::new(move |c: &Change| {
            if watch.get(c.node().index()).copied().unwrap_or(false) {
                dirty.store(true, Release);
            }
        })
    }

    /// Resolve the complete routing state.
    pub fn build(&self, store: &ParamStore) -> RoutingTable {
        let altsw = flag(store, self.altsw);
        let inputs = self
            .strips
            .iter()
            .map(|s| self.input_route(store, s, altsw))
            .collect();
        let ext = self.ext.iter().map(|e| self.ext_route(store, e)).collect();
        let outputs = self
            .outs
            .iter()
            .map(|o| {
                let g = SrcGroup::from_index(int(store, o.grp, 0) as u32);
                let n = int(store, o.inn, 1);
                self.addr(g, n)
                    .map_or(Leg::SILENT, |a| self.leg(store, a, 0))
            })
            .collect();
        let osc = [0, 1].map(|k| {
            let s = &self.src[SourceAddr::new(SrcGroup::Osc, k + 1).unwrap().index()];
            OscParams {
                mode: match int(store, s.omode, 0) {
                    1 => OscMode::Pink,
                    2 => OscMode::White,
                    _ => OscMode::Sine,
                },
                level: db_to_lin(num(store, s.olvl).unwrap_or(-18.0)),
                freq: num(store, s.of).unwrap_or(1000.0) as f32,
            }
        });
        RoutingTable {
            altsw,
            inputs,
            ext,
            outputs,
            osc,
        }
    }

    fn addr(&self, g: Option<SrcGroup>, n: i64) -> Option<SourceAddr> {
        SourceAddr::new(g?, u16::try_from(n).ok()?)
    }

    /// Return route of an FX slot running `EXT` (`egrp` != OFF), else [`InputRoute::OFF`].
    fn ext_route(&self, store: &ParamStore, e: &ExtNodes) -> InputRoute {
        let active = e.ext_index.is_some_and(|x| int(store, e.mdl, 0) == x);
        let src = self.addr(
            SrcGroup::from_index(int(store, e.egrp, 0) as u32),
            int(store, e.ein, 1),
        );
        let (true, Some(src)) = (active, src.filter(|a| a.grp <= SrcGroup::Aes)) else {
            return InputRoute::OFF;
        };
        let width = if int(store, e.emode, 1) == 0 {
            Width::Mono
        } else {
            Width::Stereo
        };
        let (l, r) = match width {
            Width::Mono => {
                let l = self.leg(store, src, 0);
                (l, l)
            }
            _ => {
                // odd-aligned pair, as for channel sources
                let first = SourceAddr {
                    n: src.n - (1 - src.n % 2),
                    ..src
                };
                let l = self.leg(store, first, 0);
                let r = first.next().map_or(Leg::SILENT, |a| self.leg(store, a, 0));
                (l, r)
            }
        };
        InputRoute {
            main: Some(src),
            alt: None,
            use_alt: false,
            width,
            legs: [l, r],
        }
    }

    fn input_route(&self, store: &ParamStore, s: &StripNodes, altsw: bool) -> InputRoute {
        let main = self.addr(
            SrcGroup::from_index(int(store, s.grp, 0) as u32),
            int(store, s.inn, 1),
        );
        let alt = self.addr(
            SrcGroup::from_index(int(store, s.altgrp, 0) as u32),
            int(store, s.altin, 1),
        );
        let use_alt = flag(store, s.altsrc) || (flag(store, s.srcauto) && altsw);
        let Some(src) = (if use_alt { alt } else { main }) else {
            return InputRoute {
                main,
                alt,
                use_alt,
                ..InputRoute::OFF
            };
        };
        let width = if src.grp.mix_group().is_some() {
            Width::Stereo // mix outputs are stereo strips [I]
        } else {
            match int(store, self.src[src.index()].mode, 0) {
                1 => Width::Stereo,
                2 => Width::MidSide,
                _ => Width::Mono,
            }
        };
        let (l, r) = match width {
            Width::Mono => {
                let l = self.leg(store, src, 0);
                (l, l)
            }
            _ => {
                // pairs are odd-aligned (1/2, 3/4, ...): either leg selects the pair [I]
                let first = SourceAddr {
                    n: src.n - (1 - src.n % 2),
                    ..src
                };
                let l = self.leg(store, first, 0);
                let r = first.next().map_or(Leg::SILENT, |a| self.leg(store, a, 0));
                (l, r)
            }
        };
        InputRoute {
            main,
            alt,
            use_alt,
            width,
            legs: [l, r],
        }
    }

    /// Resolve one source leg.
    fn leg(&self, store: &ParamStore, a: SourceAddr, depth: u8) -> Leg {
        let s = &self.src[a.index()];
        let mut gain = 1.0f32;
        if let Some(g) = num(store, s.g) {
            gain *= db_to_lin(g); // preamp gain applies; phantom (vph) is ignored
        }
        if flag(store, s.mute) {
            gain = 0.0;
        }
        if flag(store, s.pol) {
            gain = -gain;
        }
        let feed = match a.grp {
            SrcGroup::Usb => Feed::Usb((a.n - 1) as u8),
            SrcGroup::Osc => Feed::Osc((a.n - 1) as u8),
            SrcGroup::Usr if a.n <= 24 => {
                // strip tap: grp OFF|CH|AUX|BUS|MAIN|MTX, in, tap PRE|POST, lr L+R|L|R
                let n = int(store, s.uin, 1).clamp(0, 255) as u8;
                let strip = match int(store, s.ugrp, 0) {
                    1 => Some(Strip::Ch(n)),
                    2 => Some(Strip::Aux(n)),
                    3 => Some(Strip::Bus(n)),
                    4 => Some(Strip::Main(n)),
                    5 => Some(Strip::Mtx(n)),
                    _ => None,
                };
                let point = if int(store, s.utap, 0) == 1 {
                    StripPoint::PostProc
                } else {
                    StripPoint::PreFader
                };
                let lr = int(store, s.ulr, 0);
                let side = if lr == 2 { Side::R } else { Side::L };
                match strip.and_then(|strip| Tap::Strip { strip, point, side }.id()) {
                    Some(id) if lr == 0 => Feed::TapMono(id),
                    Some(id) => Feed::Tap(id),
                    None => Feed::Silence,
                }
            }
            SrcGroup::Usr => {
                // alias of a physical source (grp OFF..AES, in)
                let g = SrcGroup::from_index(int(store, s.ugrp, 0) as u32)
                    .filter(|g| *g <= SrcGroup::Aes);
                match self.addr(g, int(store, s.uin, 1)) {
                    Some(inner) if depth == 0 => {
                        let l = self.leg(store, inner, depth + 1);
                        gain *= l.gain;
                        l.feed
                    }
                    _ => Feed::Silence,
                }
            }
            g => match g.mix_group() {
                Some(grp) => Tap::Mix { grp, leg: a.n }
                    .id()
                    .map_or(Feed::Silence, Feed::Tap),
                None => Feed::Silence,
            },
        };
        Leg {
            src: Some(a),
            feed,
            gain,
        }
    }
}

/// Control-thread glue: a builder, a dirty flag raised by a store hook, and the publisher.
pub struct RoutingControl {
    builder: RoutingBuilder,
    sender: super::swap::TableSender<RoutingTable>,
    dirty: Arc<AtomicBool>,
}

impl RoutingControl {
    /// Build the initial table from `store`.
    pub fn new(store: &ParamStore) -> Self {
        let builder = RoutingBuilder::new(store);
        let table = builder.build(store);
        Self {
            builder,
            sender: super::swap::TableSender::new(table),
            dirty: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn builder(&self) -> &RoutingBuilder {
        &self.builder
    }

    /// Hook to register on the store (`ParamStore::add_hook`) so routing changes mark the table
    /// dirty.
    pub fn change_hook(&self) -> Hook {
        self.builder.change_hook(self.dirty.clone())
    }

    /// Mark dirty if `id` is a routing parameter (for callers that see changes another way).
    pub fn notify(&self, id: ParamId) {
        if self.builder.watches(id) {
            self.dirty.store(true, Release);
        }
    }

    /// A new RT reader (control thread).
    pub fn subscribe(&mut self) -> super::RoutingReader {
        self.sender.subscribe()
    }

    /// The latest published table.
    pub fn table(&self) -> &RoutingTable {
        self.sender.latest()
    }

    /// Rebuild and publish if a routing parameter changed since the last refresh; always frees
    /// tables the RT side has retired. Returns whether a new table was published.
    pub fn refresh(&mut self, store: &ParamStore) -> bool {
        self.sender.collect();
        if !self.dirty.swap(false, AcqRel) {
            return false;
        }
        let t = self.builder.build(store);
        if &t == self.sender.latest() {
            return false;
        }
        self.sender.publish(t);
        true
    }

    /// Rebuild and publish unconditionally (e.g. after loading a snapshot).
    pub fn rebuild(&mut self, store: &ParamStore) {
        self.dirty.store(false, Release);
        let t = self.builder.build(store);
        self.sender.publish(t);
    }

    /// Free retired tables.
    pub fn collect(&self) {
        self.sender.collect();
    }

    /// Whether every reader runs the latest table.
    pub fn settled(&self) -> bool {
        self.sender.settled()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::Value;

    fn set(store: &ParamStore, path: &str, text: &str) {
        let id = store.lookup(path).unwrap_or_else(|| panic!("{path}"));
        store
            .set_text(id, text, 0)
            .unwrap_or_else(|e| panic!("{path}={text}: {e}"));
    }

    #[test]
    fn factory_defaults_resolve() {
        let store = ParamStore::new();
        let b = RoutingBuilder::new(&store);
        let t = b.build(&store);
        // every routing parameter of the schema was found
        assert!(b.altsw.is_some());
        assert!(b
            .strips
            .iter()
            .all(|s| s.grp.is_some() && s.altsrc.is_some()));
        assert!(b.outs.iter().all(|o| o.grp.is_some() && o.inn.is_some()));
        for (i, s) in b.src.iter().enumerate() {
            let a = SourceAddr::from_index(i).unwrap();
            assert!(s.mode.is_some(), "{a}");
        }
        assert!(!t.altsw);
        assert_eq!(t.osc[0].mode, OscMode::Sine);
    }

    #[test]
    fn user_signals_and_mix_outputs() {
        let store = ParamStore::new();
        set(&store, "io/in/USR/1/user/grp", "CH");
        set(&store, "io/in/USR/1/user/in", "5");
        set(&store, "io/in/USR/1/user/tap", "POST");
        set(&store, "io/in/USR/1/user/lr", "L+R");
        set(&store, "io/in/USR/2/user/grp", "BUS");
        set(&store, "io/in/USR/2/user/in", "3");
        set(&store, "io/in/USR/2/user/lr", "R");
        set(&store, "io/in/USR/30/user/grp", "USB");
        set(&store, "io/in/USR/30/user/in", "7");
        set(&store, "io/in/USB/7/pol", "1");
        set(&store, "io/out/USB/1/grp", "USR");
        set(&store, "io/out/USB/1/in", "1");
        set(&store, "io/out/USB/2/grp", "USR");
        set(&store, "io/out/USB/2/in", "2");
        set(&store, "io/out/USB/3/grp", "USR");
        set(&store, "io/out/USB/3/in", "30");
        set(&store, "io/out/USB/4/grp", "MAIN");
        set(&store, "io/out/USB/4/in", "2");
        set(&store, "io/out/USB/5/grp", "OFF");
        set(&store, "io/out/USB/6/grp", "LCL");
        set(&store, "io/out/USB/6/in", "3");
        set(&store, "io/in/LCL/3/g", "20");
        set(&store, "io/out/USB/7/grp", "OSC");
        set(&store, "io/out/USB/7/in", "2");
        set(&store, "io/in/OSC/2/mute", "1");
        let t = RoutingBuilder::new(&store).build(&store);
        let ch5_post_l = Tap::Strip {
            strip: Strip::Ch(5),
            point: StripPoint::PostProc,
            side: Side::L,
        };
        assert_eq!(t.usb_out(1).feed, Feed::TapMono(ch5_post_l.id().unwrap()));
        let bus3_pre_r = Tap::Strip {
            strip: Strip::Bus(3),
            point: StripPoint::PreFader,
            side: Side::R,
        };
        assert_eq!(t.usb_out(2).feed, Feed::Tap(bus3_pre_r.id().unwrap()));
        assert_eq!(t.usb_out(3).feed, Feed::Usb(6));
        assert_eq!(t.usb_out(3).gain, -1.0);
        let main1_r = Tap::Mix {
            grp: MixGroup::Main,
            leg: 2,
        };
        assert_eq!(t.usb_out(4).feed, Feed::Tap(main1_r.id().unwrap()));
        assert_eq!(*t.usb_out(5), Leg::SILENT);
        let l = t.usb_out(6);
        assert_eq!(l.feed, Feed::Silence);
        assert_eq!(l.src, SourceAddr::new(SrcGroup::Lcl, 3));
        assert!((l.gain - 10.0).abs() < 1e-4);
        assert_eq!(t.usb_out(7).feed, Feed::Osc(1));
        assert_eq!(t.usb_out(7).gain, 0.0);
    }

    #[test]
    fn stereo_midside_and_alt_logic() {
        let store = ParamStore::new();
        set(&store, "ch/1/in/conn/grp", "USB");
        set(&store, "ch/1/in/conn/in", "9");
        set(&store, "io/in/USB/9/mode", "ST");
        set(&store, "ch/2/in/conn/grp", "USB");
        set(&store, "ch/2/in/conn/in", "11");
        set(&store, "io/in/USB/11/mode", "M/S");
        set(&store, "ch/3/in/conn/grp", "USB");
        set(&store, "ch/3/in/conn/in", "48");
        set(&store, "io/in/USB/48/mode", "ST");
        set(&store, "ch/4/in/conn/grp", "BUS");
        set(&store, "ch/4/in/conn/in", "4");
        set(&store, "ch/5/in/conn/grp", "USB");
        set(&store, "ch/5/in/conn/in", "1");
        set(&store, "io/in/USB/1/mode", "M");
        set(&store, "ch/5/in/conn/altgrp", "OSC");
        set(&store, "ch/5/in/conn/altin", "1");
        let b = RoutingBuilder::new(&store);
        let t = b.build(&store);
        let c1 = t.channel(1);
        assert_eq!(c1.width, Width::Stereo);
        assert_eq!(
            (c1.legs[0].feed, c1.legs[1].feed),
            (Feed::Usb(8), Feed::Usb(9))
        );
        assert_eq!(t.channel(2).width, Width::MidSide);
        // even leg of a pair selects the pair
        assert_eq!(
            t.channel(3).legs.map(|l| l.feed),
            [Feed::Usb(46), Feed::Usb(47)]
        );
        assert_eq!(t.channel(3).main, SourceAddr::new(SrcGroup::Usb, 48));
        let c4 = t.channel(4);
        assert_eq!(c4.width, Width::Stereo);
        assert_eq!(
            c4.legs.map(|l| l.feed),
            [3, 4].map(|leg| Feed::Tap(
                Tap::Mix {
                    grp: MixGroup::Bus,
                    leg
                }
                .id()
                .unwrap()
            ))
        );
        let c5 = t.channel(5);
        assert_eq!(c5.width, Width::Mono);
        assert_eq!(c5.legs[0], c5.legs[1]);
        // factory srcauto: check both global and per-channel selection
        let id = |p: &str| store.lookup(p).unwrap();
        store.set(id("ch/5/in/set/srcauto"), Value::Int(1)).unwrap();
        store.set(id("io/altsw"), Value::Int(1)).unwrap();
        assert_eq!(b.build(&store).channel(5).legs[0].feed, Feed::Osc(0));
        store.set(id("ch/5/in/set/srcauto"), Value::Int(0)).unwrap();
        assert_eq!(b.build(&store).channel(5).legs[0].feed, Feed::Usb(0));
        store.set(id("ch/5/in/set/altsrc"), Value::Int(1)).unwrap();
        store.set(id("io/altsw"), Value::Int(0)).unwrap();
        assert_eq!(b.build(&store).channel(5).legs[0].feed, Feed::Osc(0));
    }

    #[test]
    fn control_refreshes_on_routing_changes_only() {
        let mut store = ParamStore::new();
        let ctl_store_probe = RoutingControl::new(&store);
        store.add_hook(ctl_store_probe.change_hook());
        let mut ctl = ctl_store_probe;
        let mut rx = ctl.subscribe();
        assert!(!ctl.refresh(&store));
        let fdr = store.lookup("ch/1/fdr").unwrap();
        assert!(!ctl.builder().watches(fdr));
        store.set(fdr, Value::Float(-10.0)).unwrap();
        assert!(!ctl.refresh(&store));
        let altsw = store.lookup("io/altsw").unwrap();
        assert!(ctl.builder().watches(altsw));
        store.set(altsw, Value::Int(1)).unwrap();
        assert!(ctl.refresh(&store));
        assert!(!rx.get().altsw);
        assert!(rx.update());
        assert!(rx.get().altsw);
        assert!(ctl.settled());
    }
}
