//! Realtime audio path of the daemon (T026): HAL driver shm ↔ engine ↔ USB outs + host bridges.
//!
//! * [`console`]: the processing graph (routing → 40 ch + 8 aux strips → busses, mains, matrices (T036) →
//!   output patch) bound to the `ParamStore` through the T03A pipeline, and [`Console`], which
//!   runs one engine block per call.
//! * [`driver`]: [`AudioService`]: attach/reattach to the driver, the RT driver thread, rate
//!   changes, clean detach, status line.
//! * [`hostio`]: `--hostio` config / default Scarlett route and the bridge lifecycle.
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

use std::sync::Arc;

use wing_engine::mix::MixLogic;
use wing_engine::routing::RoutingControl;
use wing_model::ParamStore;

pub use console::{Console, ConsoleConfig, ConsoleNodes};
pub use driver::{service, AudioOptions, AudioService};
pub use hostio::HostIoSetting;

/// Log sink (same shape as the daemon's).
pub type LogFn = Arc<dyn Fn(&str) + Send + Sync>;

/// Set `cfg/mon/1|2/lvl` to 0 dB where they are -oo (see [`prepare`]).
pub fn default_monitor_levels(store: &ParamStore) {
    for m in 1..=2 {
        if let Some(id) = store.lookup(&format!("cfg/mon/{m}/lvl")) {
            if store.get_f64(id).is_some_and(|v| v <= -144.0) {
                let _ = store.set(id, wing_model::Value::Float(0.0));
            }
        }
    }
}

/// Control-side state the console derives from the store: the routing tables (T024) and the
/// DCA / mute-group / solo logic (T038). Both are refreshed by the audio control thread.
pub struct ConsoleControl {
    pub routing: RoutingControl,
    pub logic: MixLogic,
}

impl ConsoleControl {
    /// Rebuild/publish whatever changed (control thread). Returns whether anything was published.
    pub fn refresh(&mut self, store: &ParamStore) -> bool {
        let a = self.routing.refresh(store);
        let b = self.logic.refresh(store);
        a || b
    }
}

/// Build the routing control and the mix logic from `store` and hook them to parameter changes.
/// Call before the store is wrapped in an `Arc`.
///
/// Also gives the monitor outputs a usable level: `cfg/mon/1|2/lvl` are the console's PHONES /
/// SPEAKERS knobs, absent from snapshots and -oo in the factory tree, which would leave the
/// default host route (`mon/1`) silent; a -oo level is set to 0 dB here.
pub fn prepare(store: &mut ParamStore) -> ConsoleControl {
    default_monitor_levels(store);
    let routing = RoutingControl::new(store);
    store.add_hook(routing.change_hook());
    let logic = MixLogic::new(store);
    store.add_hook(logic.change_hook());
    ConsoleControl { routing, logic }
}
