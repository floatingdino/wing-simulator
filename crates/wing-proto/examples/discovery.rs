//! Run the discovery responder standalone (manual check against WING-Edit).
//!
//! cargo run -p wing-proto --example discovery -- [--seconds N] [--name S] [--model S]
//!     [--serial S] [--fw S] [--ip A.B.C.D] [--bind A.B.C.D:PORT]

use std::time::{Duration, Instant};
use wing_proto::discovery::{Responder, ResponderConfig};

fn main() {
    let mut cfg = ResponderConfig::default();
    let mut seconds: Option<u64> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let v = args
            .next()
            .unwrap_or_else(|| panic!("missing value for {a}"));
        match a.as_str() {
            "--seconds" => seconds = Some(v.parse().expect("seconds")),
            "--name" => cfg.identity.name = v,
            "--model" => cfg.identity.model = v,
            "--serial" => cfg.identity.serial = v,
            "--fw" => cfg.identity.firmware = v,
            "--ip" => cfg.advertise_ip = Some(v.parse().expect("ip")),
            "--bind" => cfg.bind = v.parse().expect("bind"),
            _ => panic!("unknown option {a}"),
        }
    }
    let r = Responder::bind(cfg).expect("bind");
    eprintln!("discovery responder on {}", r.local_addr().unwrap());
    let start = Instant::now();
    let h = r
        .spawn(move |from, reply| {
            println!("[{:7.3}s] {from} <- {reply}", start.elapsed().as_secs_f32());
        })
        .expect("spawn");
    match seconds {
        Some(s) => std::thread::sleep(Duration::from_secs(s)),
        None => loop {
            std::thread::sleep(Duration::from_secs(3600));
        },
    }
    eprintln!("{} replies", h.replies());
    h.stop();
}
