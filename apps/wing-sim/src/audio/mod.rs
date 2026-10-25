//! Realtime audio path of the daemon (T026): HAL driver shm ↔ engine ↔ USB outs + host bridges.
//!
//! * [`console`]: the processing graph (routing → 40 ch + 8 aux strips → provisional main 1 →
//!   output patch) bound to the `ParamStore` through the T03A pipeline, and [`Console`], which
//!   runs one engine block per call.
//! * [`driver`]: [`AudioService`]: attach/reattach to the driver, the RT driver thread, rate
//!   changes, clean detach, status line.
//! * [`hostio`]: `--hostio` config / default Scarlett route and the bridge lifecycle.
//! * [`stats`]: lock-free cycle-time histogram (load %, p50/p99).
//! * [`workgroup`]: the daemon's audio work interval (driver thread + engine workers).
//!
//! Use: [`prepare`] before the store is shared (it hooks routing changes), then
//! [`AudioService::spawn`].

pub mod console;
pub mod driver;
pub mod hostio;
pub mod stats;
pub mod workgroup;

use std::sync::Arc;

use wing_engine::routing::RoutingControl;
use wing_model::ParamStore;

pub use console::{Console, ConsoleConfig, ConsoleNodes};
pub use driver::{service, AudioOptions, AudioService};
pub use hostio::HostIoSetting;

/// Log sink (same shape as the daemon's).
pub type LogFn = Arc<dyn Fn(&str) + Send + Sync>;

/// Build the routing control from `store` and hook it to routing parameter changes. Call before
/// the store is wrapped in an `Arc`.
pub fn prepare(store: &mut ParamStore) -> RoutingControl {
    let routing = RoutingControl::new(store);
    store.add_hook(routing.change_hook());
    routing
}
