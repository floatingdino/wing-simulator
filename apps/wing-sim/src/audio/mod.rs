//! Realtime audio path of the daemon (T026): HAL driver shm ↔ engine ↔ USB outs + host bridges.
//!
//! * [`console`]: the processing graph (routing → 40 ch + 8 aux strips → busses, mains, matrices (T036) →
//!   output patch) bound to the `ParamStore` through the T03A pipeline, and [`Console`], which
//!   runs one engine block per call.
//! * [`driver`]: [`AudioService`]: attach/reattach to the driver, the RT driver thread, rate
//!   changes, clean detach, status line.
//! * [`hostio`]: `--hostio` config / default route (system default output) and the bridge lifecycle.
//! * [`stats`]: lock-free cycle-time histogram (load %, p50/p99).
//! * [`workgroup`]: the daemon's audio work interval (driver thread + engine workers).
//!
//! Use: [`prepare`] before the store is shared (it hooks routing and DCA/solo changes), then
//! [`AudioService::spawn`].

pub mod console;
pub mod driver;
pub mod hostio;
pub mod stats;
pub mod workgroup;

use std::sync::atomic::{AtomicBool, Ordering::*};
use std::sync::Arc;

use wing_engine::mix::{MixLogic, MonitorLevels};
use wing_engine::routing::RoutingControl;
use wing_model::{NodeId, ParamStore};

pub use console::{Console, ConsoleConfig, ConsoleNodes};
pub use driver::{service, AudioOptions, AudioService};
pub use hostio::HostIoSetting;

/// Log sink (same shape as the daemon's).
pub type LogFn = Arc<dyn Fn(&str) + Send + Sync>;

/// The simulated monitor level knobs (T03F): PHONES (`cfg/mon/1/lvl`, monitor A) and SPEAKERS
/// (`cfg/mon/2/lvl`, monitor B).
///
/// On a WING these levels are physical knobs; WING-Edit has no knob, keeps them at -oo in its
/// scene and sends that with every PC->MIXER push (fixtures/protocol/pc-to-mixer-scene.json),
/// which used to silence the default host route (`mon/1`). Here the knob owns the level: the
/// monitor section plays the knob value ([`MonitorLevels`] in the engine), and
/// [`ConsoleControl::refresh`] writes it back over any client set of `lvl` (so clients see the
/// knob position, as they would read the pot). Set it through the control socket
/// (`monitor-level`), OSC (`/wingsim/mon/<1|2>/lvl`) or [`MonitorKnobs::set`]. Default 0 dB.
#[derive(Clone)]
pub struct MonitorKnobs {
    levels: Arc<MonitorLevels>,
    dirty: Arc<AtomicBool>,
}

impl Default for MonitorKnobs {
    fn default() -> Self {
        Self {
            levels: Arc::new(MonitorLevels::default()),
            dirty: Arc::new(AtomicBool::new(true)),
        }
    }
}

impl MonitorKnobs {
    /// The level of monitor `m` (1 = A / PHONES, 2 = B / SPEAKERS) in dB (-144 = -oo).
    pub fn get(&self, m: usize) -> f32 {
        self.levels.get(m)
    }

    /// Both levels (A, B) in dB.
    pub fn both(&self) -> [f32; 2] {
        [self.get(1), self.get(2)]
    }

    /// Turn knob `m` (1 or 2) to `db` (clamped to -144..=10); the audio control thread mirrors
    /// it into `cfg/mon/<m>/lvl`. Returns the stored value.
    pub fn set(&self, m: usize, db: f32) -> Result<f32, String> {
        if !(1..=2).contains(&m) {
            return Err(format!("no monitor {m} (1 = A, 2 = B)"));
        }
        let v = self.levels.set(m, db);
        self.dirty.store(true, Release);
        Ok(v)
    }

    /// The engine-side levels (for [`wing_engine::mix::MixConfig::monitor_levels`]).
    pub fn levels(&self) -> Arc<MonitorLevels> {
        self.levels.clone()
    }

    /// One-line description (`mon/1 0.0 dB, mon/2 -oo`).
    pub fn describe(&self) -> String {
        let f = |v: f32| {
            if v <= -144.0 {
                "-oo".to_string()
            } else {
                format!("{v:.1} dB")
            }
        };
        format!("mon/1 {}, mon/2 {}", f(self.get(1)), f(self.get(2)))
    }

    /// Store hook: a change of either `lvl` (by anyone) schedules a mirror.
    fn hook(&self, ids: [NodeId; 2]) -> wing_model::Hook {
        let dirty = self.dirty.clone();
        Box::new(move |c| {
            if ids.iter().any(|i| i.0 == c.id) {
                dirty.store(true, Release);
            }
        })
    }

    /// Write the knob levels into `cfg/mon/1|2/lvl` where they differ (control thread).
    fn mirror(&self, store: &ParamStore, ids: [NodeId; 2]) {
        if !self.dirty.swap(false, AcqRel) {
            return;
        }
        for (k, &id) in ids.iter().enumerate() {
            let v = self.get(k + 1) as f64;
            if store.get_f64(id).is_none_or(|cur| (cur - v).abs() > 1e-3) {
                let _ = store.set(id, wing_model::Value::Float(v as f32));
            }
        }
        // our own sets re-marked dirty through the hook: one more (no-op) pass settles it
    }
}

/// OSC `/wingsim/mon/<m>/lvl` (`m` = 1|2|a|b): with a numeric (or `-inf`) argument turn the
/// knob, without one query it. Replies `/wingsim/mon/<1|2>/lvl ,f <dB>` (or `/* ,s ERROR ...`).
/// `None`: not a monitor address.
pub fn monitor_osc(
    knobs: Option<&MonitorKnobs>,
    addr: &str,
    args: &[wing_proto::osc::Arg],
) -> Option<Option<(String, Vec<wing_proto::osc::Arg>)>> {
    use wing_proto::osc::Arg;
    let m = addr
        .strip_prefix("/wingsim/mon/")?
        .strip_suffix("/lvl")
        .and_then(crate::control::parse_monitor)?;
    let err = |e: &str| {
        Some(Some((
            "/*".to_string(),
            vec![Arg::Str(format!("ERROR {e}"))],
        )))
    };
    let Some(k) = knobs else {
        return err("audio is off (--no-audio)");
    };
    if let Some(a) = args.first() {
        let db = match a {
            Arg::Float(v) if v.is_finite() => Some(*v),
            Arg::Float(v) if *v < 0.0 => Some(-144.0),
            Arg::Int(v) => Some(*v as f32),
            Arg::Str(s) => crate::control::parse_db(s),
            _ => None,
        };
        let Some(db) = db else {
            return err("usage: /wingsim/mon/<1|2>/lvl [dB]");
        };
        if let Err(e) = k.set(m, db) {
            return err(&e);
        }
    }
    Some(Some((
        format!("/wingsim/mon/{m}/lvl"),
        vec![Arg::Float(k.get(m))],
    )))
}

/// Control-side state the console derives from the store: the routing tables (T024) and the
/// DCA / mute-group / solo logic (T038). Both are refreshed by the audio control thread.
pub struct ConsoleControl {
    pub routing: RoutingControl,
    pub logic: MixLogic,
    /// Monitor level knobs (T03F).
    pub monitor: MonitorKnobs,
    monitor_ids: [NodeId; 2],
}

impl ConsoleControl {
    /// Rebuild/publish whatever changed (control thread). Returns whether anything was published.
    pub fn refresh(&mut self, store: &ParamStore) -> bool {
        self.monitor.mirror(store, self.monitor_ids);
        let a = self.routing.refresh(store);
        let b = self.logic.refresh(store);
        a || b
    }
}

/// Build the routing control and the mix logic from `store` and hook them to parameter changes.
/// Call before the store is wrapped in an `Arc`.
///
/// Also installs the monitor level knobs ([`MonitorKnobs`], 0 dB): `cfg/mon/1|2/lvl` are the
/// console's PHONES / SPEAKERS knobs, absent from snapshots and -oo in the factory tree and in
/// WING-Edit's scene pushes; the knobs own them.
pub fn prepare(store: &mut ParamStore) -> ConsoleControl {
    let routing = RoutingControl::new(store);
    store.add_hook(routing.change_hook());
    let logic = MixLogic::new(store);
    store.add_hook(logic.change_hook());
    let monitor_ids = [1, 2].map(|m| {
        store
            .lookup(&format!("cfg/mon/{m}/lvl"))
            .expect("schema has cfg/mon/*/lvl")
    });
    let monitor = MonitorKnobs::default();
    store.add_hook(monitor.hook(monitor_ids));
    monitor.mirror(store, monitor_ids);
    ConsoleControl {
        routing,
        logic,
        monitor,
        monitor_ids,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_proto::osc::Arg;

    fn lvl(s: &ParamStore, m: u8) -> Option<f64> {
        s.get_f64(s.lookup(&format!("cfg/mon/{m}/lvl")).unwrap())
    }

    #[test]
    fn knobs_mirror_into_the_store_over_client_sets() {
        let mut store = ParamStore::new();
        let mut ctl = prepare(&mut store);
        // factory -oo replaced by the knob's 0 dB at prepare
        assert_eq!((lvl(&store, 1), lvl(&store, 2)), (Some(0.0), Some(0.0)));
        // a client (WING-Edit push) sets -oo: the next refresh puts the knob back
        let id = store.lookup("cfg/mon/1/lvl").unwrap();
        store.set_text(id, "-144", 77).unwrap();
        ctl.refresh(&store);
        assert_eq!(lvl(&store, 1), Some(0.0));
        ctl.monitor.set(2, -20.0).unwrap();
        ctl.refresh(&store);
        assert!((lvl(&store, 2).unwrap() + 20.0).abs() < 0.1);
        // settles: the mirror's own write leaves at most one no-op pass
        ctl.refresh(&store);
        assert!(!ctl.monitor.dirty.load(Acquire));
    }

    #[test]
    fn osc_monitor_level() {
        let k = MonitorKnobs::default();
        let r = |addr: &str, args: &[Arg]| monitor_osc(Some(&k), addr, args);
        assert_eq!(
            r("/wingsim/mon/1/lvl", &[]),
            Some(Some(("/wingsim/mon/1/lvl".into(), vec![Arg::Float(0.0)])))
        );
        assert_eq!(
            r("/wingsim/mon/b/lvl", &[Arg::Float(-6.5)]),
            Some(Some(("/wingsim/mon/2/lvl".into(), vec![Arg::Float(-6.5)])))
        );
        assert_eq!(k.get(2), -6.5);
        r("/wingsim/mon/a/lvl", &[Arg::Str("-inf".into())]);
        assert_eq!(k.get(1), -144.0);
        r("/wingsim/mon/1/lvl", &[Arg::Int(20)]);
        assert_eq!(k.get(1), 10.0, "clamped");
        assert!(matches!(
            r("/wingsim/mon/1/lvl", &[Arg::Str("loud".into())]),
            Some(Some((a, _))) if a == "/*"
        ));
        assert_eq!(r("/wingsim/mon/3/lvl", &[]), None);
        assert_eq!(r("/wingsim/record", &[]), None);
        assert!(matches!(
            monitor_osc(None, "/wingsim/mon/1/lvl", &[]),
            Some(Some((a, _))) if a == "/*"
        ));
    }
}
