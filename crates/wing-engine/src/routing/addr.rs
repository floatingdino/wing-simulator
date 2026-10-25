//! Address spaces: source groups and source legs, output groups, strips and tap points.

/// Source groups, in the order of the WING `grp` enums. `io/out/*/grp` uses all of them;
/// `ch|aux/*/in/conn/grp|altgrp` is the prefix up to [`SrcGroup::Mtx`];
/// `io/in/USR/25..56/user/grp` is the prefix up to [`SrcGroup::Aes`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum SrcGroup {
    Off = 0,
    /// LOCAL IN (preamps).
    Lcl,
    /// AUX IN (local line inputs).
    Aux,
    /// AES50 port A.
    A,
    /// AES50 port B.
    B,
    /// AES50 port C.
    C,
    /// StageConnect.
    Sc,
    /// USB audio from the computer (the driver's `USB In 1..48`).
    Usb,
    /// Expansion card.
    Crd,
    /// Module.
    Mod,
    /// USB player.
    Play,
    /// AES/EBU in.
    Aes,
    /// User signals (1..24: strip taps, 25..56: aliases of other sources).
    Usr,
    /// Oscillators.
    Osc,
    /// Bus output legs (2k-1 / 2k = L / R of bus k).
    Bus,
    /// Main output legs.
    Main,
    /// Matrix output legs.
    Mtx,
    /// FX SEND legs (FX slot k feed).
    Send,
    /// Monitor output legs (1/2 = monitor A, 3/4 = monitor B).
    Mon,
}

/// How a source group is realised in the simulator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Liveness {
    /// Carries audio from the HAL driver (USB 1..48 = REAPER hardware outputs 1..48).
    Driver,
    /// Generated inside the engine (oscillators).
    Internal,
    /// Derived from engine signals (user signals and mix outputs); resolved to a [`Tap`].
    Derived,
    /// No hardware behind it: routable, carries silence.
    Silent,
}

impl SrcGroup {
    /// Every addressable group (without `Off`), in enum order.
    pub const ALL: [SrcGroup; 18] = [
        SrcGroup::Lcl,
        SrcGroup::Aux,
        SrcGroup::A,
        SrcGroup::B,
        SrcGroup::C,
        SrcGroup::Sc,
        SrcGroup::Usb,
        SrcGroup::Crd,
        SrcGroup::Mod,
        SrcGroup::Play,
        SrcGroup::Aes,
        SrcGroup::Usr,
        SrcGroup::Osc,
        SrcGroup::Bus,
        SrcGroup::Main,
        SrcGroup::Mtx,
        SrcGroup::Send,
        SrcGroup::Mon,
    ];

    /// Group by enum index (`grp` parameter raw value).
    pub fn from_index(i: u32) -> Option<Self> {
        match i {
            0 => Some(SrcGroup::Off),
            1..=18 => Some(Self::ALL[i as usize - 1]),
            _ => None,
        }
    }

    /// Snapshot/wire token.
    pub fn token(self) -> &'static str {
        match self {
            SrcGroup::Off => "OFF",
            SrcGroup::Lcl => "LCL",
            SrcGroup::Aux => "AUX",
            SrcGroup::A => "A",
            SrcGroup::B => "B",
            SrcGroup::C => "C",
            SrcGroup::Sc => "SC",
            SrcGroup::Usb => "USB",
            SrcGroup::Crd => "CRD",
            SrcGroup::Mod => "MOD",
            SrcGroup::Play => "PLAY",
            SrcGroup::Aes => "AES",
            SrcGroup::Usr => "USR",
            SrcGroup::Osc => "OSC",
            SrcGroup::Bus => "BUS",
            SrcGroup::Main => "MAIN",
            SrcGroup::Mtx => "MTX",
            SrcGroup::Send => "SEND",
            SrcGroup::Mon => "MON",
        }
    }

    /// Name of the group's node under `io/in` (`$BUS` for the read-only mix-output groups).
    pub fn io_name(self) -> &'static str {
        match self {
            SrcGroup::Bus => "$BUS",
            SrcGroup::Main => "$MAIN",
            SrcGroup::Mtx => "$MTX",
            SrcGroup::Send => "$SEND",
            SrcGroup::Mon => "$MON",
            g => g.token(),
        }
    }

    /// Number of addressable mono legs (WING-Edit 3.3.4 schema; fw 1.13 has LCL 8 / USR 24).
    pub fn count(self) -> u16 {
        match self {
            SrcGroup::Off => 0,
            SrcGroup::Lcl => 24,
            SrcGroup::Aux => 8,
            SrcGroup::A | SrcGroup::B | SrcGroup::C => 48,
            SrcGroup::Sc => 32,
            SrcGroup::Usb => 48,
            SrcGroup::Crd | SrcGroup::Mod => 64,
            SrcGroup::Play => 4,
            SrcGroup::Aes => 2,
            SrcGroup::Usr => 56,
            SrcGroup::Osc => 2,
            SrcGroup::Bus => 32,
            SrcGroup::Main => 8,
            SrcGroup::Mtx => 16,
            SrcGroup::Send => 32,
            SrcGroup::Mon => 4,
        }
    }

    /// What drives this group in the simulator.
    pub fn liveness(self) -> Liveness {
        match self {
            SrcGroup::Usb => Liveness::Driver,
            SrcGroup::Osc => Liveness::Internal,
            SrcGroup::Usr
            | SrcGroup::Bus
            | SrcGroup::Main
            | SrcGroup::Mtx
            | SrcGroup::Send
            | SrcGroup::Mon => Liveness::Derived,
            _ => Liveness::Silent,
        }
    }

    /// Mix-output group (BUS/MAIN/MTX/SEND/MON): stereo strips addressed as mono legs.
    pub fn mix_group(self) -> Option<MixGroup> {
        match self {
            SrcGroup::Bus => Some(MixGroup::Bus),
            SrcGroup::Main => Some(MixGroup::Main),
            SrcGroup::Mtx => Some(MixGroup::Mtx),
            SrcGroup::Send => Some(MixGroup::Send),
            SrcGroup::Mon => Some(MixGroup::Mon),
            _ => None,
        }
    }

    /// Whether the group carries a preamp gain `g` (and phantom `vph`) under `io/in`.
    pub fn has_preamp(self) -> bool {
        matches!(
            self,
            SrcGroup::Lcl | SrcGroup::A | SrcGroup::B | SrcGroup::C | SrcGroup::Sc
        )
    }

    fn base(self) -> u16 {
        let mut b = 0;
        for g in Self::ALL {
            if g == self {
                return b;
            }
            b += g.count();
        }
        b
    }
}

/// Total number of source legs over all groups (the flat source address space).
pub const SOURCE_COUNT: usize = 540;

/// One mono source leg: group + 1-based index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SourceAddr {
    pub grp: SrcGroup,
    /// 1-based index within the group.
    pub n: u16,
}

impl SourceAddr {
    /// `None` for `Off` or an index outside the group.
    pub fn new(grp: SrcGroup, n: u16) -> Option<Self> {
        (grp != SrcGroup::Off && n >= 1 && n <= grp.count()).then_some(Self { grp, n })
    }

    /// Flat index in `0..SOURCE_COUNT`.
    pub fn index(self) -> usize {
        (self.grp.base() + self.n - 1) as usize
    }

    /// Inverse of [`SourceAddr::index`].
    pub fn from_index(i: usize) -> Option<Self> {
        let mut b = 0usize;
        for g in SrcGroup::ALL {
            let c = g.count() as usize;
            if i < b + c {
                return Some(Self {
                    grp: g,
                    n: (i - b + 1) as u16,
                });
            }
            b += c;
        }
        None
    }

    /// The next leg of the group (right side of a stereo pair).
    pub fn next(self) -> Option<Self> {
        Self::new(self.grp, self.n + 1)
    }
}

impl std::fmt::Display for SourceAddr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.grp.token(), self.n)
    }
}

/// Output groups (`io/out/<grp>`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum OutGroup {
    Lcl,
    Aux,
    A,
    B,
    C,
    Sc,
    /// USB to the computer (the driver's `USB Out 1..48`, recorded by REAPER).
    Usb,
    Crd,
    Mod,
    /// USB/SD recorder.
    Rec,
    Aes,
}

/// Total number of output legs.
pub const OUTPUT_COUNT: usize = 374;

impl OutGroup {
    pub const ALL: [OutGroup; 11] = [
        OutGroup::Lcl,
        OutGroup::Aux,
        OutGroup::A,
        OutGroup::B,
        OutGroup::C,
        OutGroup::Sc,
        OutGroup::Usb,
        OutGroup::Crd,
        OutGroup::Mod,
        OutGroup::Rec,
        OutGroup::Aes,
    ];

    pub fn token(self) -> &'static str {
        match self {
            OutGroup::Lcl => "LCL",
            OutGroup::Aux => "AUX",
            OutGroup::A => "A",
            OutGroup::B => "B",
            OutGroup::C => "C",
            OutGroup::Sc => "SC",
            OutGroup::Usb => "USB",
            OutGroup::Crd => "CRD",
            OutGroup::Mod => "MOD",
            OutGroup::Rec => "REC",
            OutGroup::Aes => "AES",
        }
    }

    pub fn count(self) -> u16 {
        match self {
            OutGroup::Lcl | OutGroup::Aux => 8,
            OutGroup::A | OutGroup::B | OutGroup::C | OutGroup::Usb => 48,
            OutGroup::Sc => 32,
            OutGroup::Crd | OutGroup::Mod => 64,
            OutGroup::Rec => 4,
            OutGroup::Aes => 2,
        }
    }

    /// Only USB outputs leave the simulator (to the driver); the rest are resolved but unheard.
    pub fn is_live(self) -> bool {
        self == OutGroup::Usb
    }

    fn base(self) -> u16 {
        let mut b = 0;
        for g in Self::ALL {
            if g == self {
                return b;
            }
            b += g.count();
        }
        b
    }
}

/// One output leg: group + 1-based index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct OutAddr {
    pub grp: OutGroup,
    pub n: u16,
}

impl OutAddr {
    pub fn new(grp: OutGroup, n: u16) -> Option<Self> {
        (n >= 1 && n <= grp.count()).then_some(Self { grp, n })
    }

    /// Flat index in `0..OUTPUT_COUNT`.
    pub fn index(self) -> usize {
        (self.grp.base() + self.n - 1) as usize
    }

    pub fn from_index(i: usize) -> Option<Self> {
        let mut b = 0usize;
        for g in OutGroup::ALL {
            let c = g.count() as usize;
            if i < b + c {
                return Some(Self {
                    grp: g,
                    n: (i - b + 1) as u16,
                });
            }
            b += c;
        }
        None
    }
}

/// Mix-output groups that can be tapped as stereo legs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum MixGroup {
    /// Bus post-processing output (16 stereo).
    Bus,
    /// Main post-processing output (4 stereo).
    Main,
    /// Matrix post-processing output (8 stereo).
    Mtx,
    /// FX slot send feed ("FX SEND", 16 stereo) [I].
    Send,
    /// Monitor outputs (A/B stereo).
    Mon,
}

impl MixGroup {
    const ALL: [MixGroup; 5] = [
        MixGroup::Bus,
        MixGroup::Main,
        MixGroup::Mtx,
        MixGroup::Send,
        MixGroup::Mon,
    ];

    /// Mono legs (2 per stereo strip).
    pub fn legs(self) -> u16 {
        match self {
            MixGroup::Bus => 32,
            MixGroup::Main => 8,
            MixGroup::Mtx => 16,
            MixGroup::Send => 32,
            MixGroup::Mon => 4,
        }
    }

    fn base(self) -> u16 {
        let mut b = 0;
        for g in Self::ALL {
            if g == self {
                return b;
            }
            b += g.legs();
        }
        b
    }
}

/// A processing strip.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Strip {
    /// Input channel 1..40.
    Ch(u8),
    /// Aux channel 1..8.
    Aux(u8),
    /// Bus 1..16.
    Bus(u8),
    /// Main 1..4.
    Main(u8),
    /// Matrix 1..8.
    Mtx(u8),
}

/// Number of strips (40 ch + 8 aux + 16 bus + 4 main + 8 mtx).
pub const STRIP_COUNT: usize = 76;

impl Strip {
    /// Flat index in `0..STRIP_COUNT`, `None` when out of range.
    pub fn index(self) -> Option<usize> {
        let (base, n, max) = match self {
            Strip::Ch(n) => (0, n, 40),
            Strip::Aux(n) => (40, n, 8),
            Strip::Bus(n) => (48, n, 16),
            Strip::Main(n) => (64, n, 4),
            Strip::Mtx(n) => (68, n, 8),
        };
        (n >= 1 && n <= max).then_some(base + n as usize - 1)
    }

    pub fn from_index(i: usize) -> Option<Self> {
        let s = match i {
            0..=39 => Strip::Ch(i as u8 + 1),
            40..=47 => Strip::Aux((i - 40) as u8 + 1),
            48..=63 => Strip::Bus((i - 48) as u8 + 1),
            64..=67 => Strip::Main((i - 64) as u8 + 1),
            68..=75 => Strip::Mtx((i - 68) as u8 + 1),
            _ => return None,
        };
        Some(s)
    }
}

/// Where on a strip a user signal is taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum StripPoint {
    /// "PRE FADER".
    PreFader = 0,
    /// "POST PROCESSING" (post fader and pan) [I].
    PostProc = 1,
}

/// Side of a stereo signal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Side {
    L = 0,
    R = 1,
}

/// An engine signal that routing can read but that is produced by mixing/processing nodes
/// (T036). Every tap has a dense [`TapId`]; the engine supplies a `Port` per id once those
/// nodes exist ([`super::TapPorts`]); until then taps read as silence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Tap {
    /// Mono leg `leg` (1-based; 2k-1 = L, 2k = R of strip k) of a mix output group.
    Mix { grp: MixGroup, leg: u16 },
    /// One side of a strip's pre-fader / post-processing signal (user signals, i.e. direct
    /// outs of channels and pre/post taps of busses, mains and matrices).
    Strip {
        strip: Strip,
        point: StripPoint,
        side: Side,
    },
}

/// Dense tap index in `0..TAP_COUNT`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TapId(pub u16);

const MIX_LEGS: usize = 92;
/// Number of taps: 92 mix legs + 76 strips x 2 points x 2 sides.
pub const TAP_COUNT: usize = MIX_LEGS + STRIP_COUNT * 4;

impl Tap {
    /// Dense id; `None` when the leg/strip is out of range. For strip taps, the R side's id is
    /// the L side's id + 1.
    pub fn id(self) -> Option<TapId> {
        match self {
            Tap::Mix { grp, leg } => {
                (leg >= 1 && leg <= grp.legs()).then(|| TapId(grp.base() + leg - 1))
            }
            Tap::Strip { strip, point, side } => strip
                .index()
                .map(|s| TapId((MIX_LEGS + s * 4 + point as usize * 2 + side as usize) as u16)),
        }
    }

    pub fn from_id(id: TapId) -> Option<Self> {
        let i = id.0 as usize;
        if i < MIX_LEGS {
            let mut b = 0usize;
            for g in MixGroup::ALL {
                let c = g.legs() as usize;
                if i < b + c {
                    return Some(Tap::Mix {
                        grp: g,
                        leg: (i - b + 1) as u16,
                    });
                }
                b += c;
            }
            None
        } else {
            let k = i - MIX_LEGS;
            let strip = Strip::from_index(k / 4)?;
            let point = if k % 4 < 2 {
                StripPoint::PreFader
            } else {
                StripPoint::PostProc
            };
            let side = if k.is_multiple_of(2) {
                Side::L
            } else {
                Side::R
            };
            Some(Tap::Strip { strip, point, side })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_and_round_trips() {
        let total: usize = SrcGroup::ALL.iter().map(|g| g.count() as usize).sum();
        assert_eq!(total, SOURCE_COUNT);
        for i in 0..SOURCE_COUNT {
            let a = SourceAddr::from_index(i).unwrap();
            assert_eq!(a.index(), i);
        }
        assert!(SourceAddr::from_index(SOURCE_COUNT).is_none());
        assert_eq!(
            SourceAddr::new(SrcGroup::Usb, 1).unwrap().to_string(),
            "USB.1"
        );
        assert!(SourceAddr::new(SrcGroup::Usb, 49).is_none());
        assert!(SourceAddr::new(SrcGroup::Off, 1).is_none());
        for (i, g) in SrcGroup::ALL.iter().enumerate() {
            assert_eq!(SrcGroup::from_index(i as u32 + 1), Some(*g));
        }

        let total: usize = OutGroup::ALL.iter().map(|g| g.count() as usize).sum();
        assert_eq!(total, OUTPUT_COUNT);
        for i in 0..OUTPUT_COUNT {
            assert_eq!(OutAddr::from_index(i).unwrap().index(), i);
        }

        for i in 0..STRIP_COUNT {
            assert_eq!(Strip::from_index(i).unwrap().index(), Some(i));
        }
        assert_eq!(Strip::Ch(41).index(), None);

        for i in 0..TAP_COUNT {
            let t = Tap::from_id(TapId(i as u16)).unwrap();
            assert_eq!(t.id(), Some(TapId(i as u16)));
        }
        assert!(Tap::from_id(TapId(TAP_COUNT as u16)).is_none());
        let l = Tap::Strip {
            strip: Strip::Bus(3),
            point: StripPoint::PostProc,
            side: Side::L,
        };
        let r = Tap::Strip {
            strip: Strip::Bus(3),
            point: StripPoint::PostProc,
            side: Side::R,
        };
        assert_eq!(r.id().unwrap().0, l.id().unwrap().0 + 1);
    }
}
