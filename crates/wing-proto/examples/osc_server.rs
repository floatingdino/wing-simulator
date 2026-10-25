//! Run the OSC server standalone on a default-initialised ParamStore.
//!
//! cargo run -p wing-proto --example osc_server -- [--bind A.B.C.D:PORT] [--seconds N]
//!
//! Then e.g. `oscsend 127.0.0.1 2223 /ch/1/fdr f -6.0` or python-osc.

use std::sync::Arc;
use std::time::Duration;
use wing_model::ParamStore;
use wing_proto::osc::{OscAttachment, OscConfig, OscServer};

fn main() {
    let mut cfg = OscConfig::default();
    let mut seconds: Option<u64> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let v = args
            .next()
            .unwrap_or_else(|| panic!("missing value for {a}"));
        match a.as_str() {
            "--bind" => cfg.bind = v.parse().expect("bind"),
            "--seconds" => seconds = Some(v.parse().expect("seconds")),
            _ => panic!("unknown option {a}"),
        }
    }
    let mut store = ParamStore::new();
    let att = OscAttachment::attach(&mut store);
    let server = OscServer::bind(Arc::new(store), att, cfg).expect("bind");
    eprintln!("OSC server on {}", server.local_addr().unwrap());
    let h = server.spawn().expect("spawn");
    match seconds {
        Some(s) => std::thread::sleep(Duration::from_secs(s)),
        None => loop {
            std::thread::sleep(Duration::from_secs(3600));
        },
    }
    eprintln!("{} datagrams", h.requests());
    h.stop();
}
