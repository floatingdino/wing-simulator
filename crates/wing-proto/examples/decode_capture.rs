//! Decode a `tools/protocol-probe/fake_wing.py` TCP capture log with the wing-proto NRP codec (T00B).
//!
//! The log contains hexdumps of every TCP chunk received from a client (`TCP2222 <- (ip, port)
//! len=N` followed by `    OFFSET  hex...  |ascii|` lines) and the fake console's replies
//! (`TCP2222 -> (ip, port) <hex>`). This tool reassembles the per-connection byte streams, feeds
//! them through [`NrpDecoder`] and prints/exports what it finds.
//!
//! ```text
//! gzip -dc tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz |
//!   cargo run -p wing-proto --example decode_capture -- [--tree data/schema/wing-tree.json]
//!     [--out-dir fixtures/protocol [--prefix NAME] [--server-bins]] [--tokens]
//!     [--scene-conn N --scene-out FILE] [--source TEXT]
//! ```
//!
//! - `--out-dir DIR [--prefix NAME] [--server-bins]`: writes `<NAME>-<n>.bin` (raw client wire
//!   bytes; NAME defaults to `wing-edit-connect`) and, with `--server-bins`, `<NAME>-<n>.server.bin`
//!   (raw fake-console wire bytes) per connection.
//! - `--tokens`: prints every decoded token (timestamp of the chunk it completed in).
//! - `--scene-out`: writes the `D7 <hash> <value>` sets of connection `--scene-conn` as JSON
//!   (`path -> value`, unmapped hashes listed separately).

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;
use std::io::Read;
use wing_proto::nrp::{MeterCommand, MeterRequestParser, NrpDecoder, Token};

#[derive(Default)]
struct Conn {
    port: u16,
    accepted: f64,
    closed: Option<f64>,
    /// (timestamp, bytes) per received chunk.
    rx: Vec<(f64, Vec<u8>)>,
    /// (timestamp, bytes) per sent reply.
    tx: Vec<(f64, Vec<u8>)>,
}

fn parse_ts(line: &str) -> Option<(f64, &str)> {
    let rest = line.strip_prefix('[')?;
    let end = rest.find("s]")?;
    let t = rest[..end].trim().parse().ok()?;
    Some((t, rest[end + 2..].trim_start()))
}

fn parse_port(s: &str) -> Option<u16> {
    // "... ('192.168.20.130', 60819) ..."
    let a = s.find("', ")? + 3;
    let b = a + s[a..].find(')')?;
    s[a..b].parse().ok()
}

fn hex_bytes(s: &str) -> Vec<u8> {
    s.split_whitespace()
        .flat_map(|w| {
            (0..w.len())
                .step_by(2)
                .map(move |i| u8::from_str_radix(&w[i..i + 2], 16).expect("hex"))
        })
        .collect()
}

fn parse_capture(text: &str) -> Vec<Conn> {
    let mut conns: Vec<Conn> = Vec::new();
    let mut by_port: HashMap<u16, usize> = HashMap::new();
    let mut cur_rx: Option<usize> = None; // conn index while reading hexdump lines
    for line in text.lines() {
        if let Some(h) = line.strip_prefix("    ") {
            // Hexdump line: "OFFSET(8)  hex(47)  |ascii|"; "decoded ..." lines are ignored.
            if let Some(ci) = cur_rx {
                if h.len() >= 10
                    && h.as_bytes()[8] == b' '
                    && h[..8].bytes().all(|c| c.is_ascii_hexdigit())
                {
                    let hex = &h[10..(10 + 47).min(h.len())];
                    let bytes = hex_bytes(hex);
                    conns[ci].rx.last_mut().unwrap().1.extend(bytes);
                }
            }
            continue;
        }
        cur_rx = None;
        let Some((t, rest)) = parse_ts(line) else {
            continue;
        };
        if rest.starts_with("TCP2222 accept from") {
            let port = parse_port(rest).expect("port");
            by_port.insert(port, conns.len());
            conns.push(Conn {
                port,
                accepted: t,
                ..Default::default()
            });
        } else if rest.starts_with("TCP2222 <- ") {
            let ci = by_port[&parse_port(rest).expect("port")];
            conns[ci].rx.push((t, Vec::new()));
            cur_rx = Some(ci);
        } else if rest.starts_with("TCP2222 -> ") {
            let ci = by_port[&parse_port(rest).expect("port")];
            let hex = rest.rsplit(' ').next().unwrap();
            conns[ci].tx.push((t, hex_bytes(hex)));
        } else if rest.contains("closed by peer") {
            if let Some(&ci) = by_port.get(&parse_port(rest).unwrap_or(0)) {
                conns[ci].closed = Some(t);
            }
        }
    }
    // Sanity check against the logged chunk lengths is done by the caller via totals.
    conns
}

fn load_hashes(path: &str) -> HashMap<u32, String> {
    let text = std::fs::read_to_string(path).expect("read wing-tree.json");
    let mut map = HashMap::new();
    let mut inside = false;
    for line in text.lines() {
        if !inside {
            inside = line.trim_start().starts_with("\"hashes\": {");
            continue;
        }
        let l = line.trim();
        if l.starts_with('}') {
            break;
        }
        // "path": "hex",
        let mut parts = l.trim_end_matches(',').splitn(2, "\": \"");
        let (Some(p), Some(h)) = (parts.next(), parts.next()) else {
            continue;
        };
        let p = p.trim_start_matches('"');
        let h = h.trim_end_matches('"');
        map.insert(u32::from_str_radix(h, 16).expect("hash"), p.to_string());
    }
    map
}

#[derive(Clone, Debug)]
enum Val {
    Int(i32, &'static str),
    F(f32, &'static str),
    S(String),
}

impl Val {
    fn from(t: &Token) -> Option<Val> {
        Some(match *t {
            Token::SmallInt(v) => Val::Int(v as i32, "small"),
            Token::Int16(v) => Val::Int(v as i32, "D3"),
            Token::Int32(v) => Val::Int(v, "D4"),
            Token::Float(v) => Val::F(v, "D5"),
            Token::RawFloat(v) => Val::F(v, "D6"),
            Token::Str(s) => Val::S(String::from_utf8_lossy(s).into_owned()),
            _ => return None,
        })
    }
    fn enc(&self) -> &'static str {
        match self {
            Val::Int(_, e) | Val::F(_, e) => e,
            Val::S(s) if s.is_empty() => "D0",
            Val::S(s) if s.len() <= 64 => "str80",
            Val::S(_) => "D1",
        }
    }
    fn json(&self) -> String {
        match self {
            Val::Int(v, _) => v.to_string(),
            Val::F(v, _) if v.is_finite() => {
                let s = format!("{v:?}");
                s
            }
            Val::F(v, _) => format!("\"{v}\""),
            Val::S(s) => json_str(s),
        }
    }
}

fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn fmt_token(t: &Token, names: &HashMap<u32, String>) -> String {
    match *t {
        Token::NavHash(h) => format!(
            "D7 {h:08x} ({})",
            names.get(&h).map(String::as_str).unwrap_or("?UNMAPPED")
        ),
        Token::Str(s) => format!("Str {:?}", String::from_utf8_lossy(s)),
        Token::NodeDef(b) => format!("NodeDef len={}", b.len()),
        ref o => format!("{o:?}"),
    }
}

struct Set {
    t: f64,
    hash: u32,
    val: Val,
}

struct Report {
    sets: Vec<Set>,
    gets: Vec<(f64, u32)>,
    other: Vec<(f64, String)>,
    errors: Vec<(f64, String)>,
    meter: Vec<(f64, MeterCommand)>,
    pending_ctrl: Option<u32>,
}

fn analyse(c: &Conn, names: &HashMap<u32, String>, print_tokens: bool) -> Report {
    let mut dec = NrpDecoder::new();
    let mut mp = MeterRequestParser::new();
    let mut rep = Report {
        sets: vec![],
        gets: vec![],
        other: vec![],
        errors: vec![],
        meter: vec![],
        pending_ctrl: None,
    };
    // Cursor state: last navigated hash and whether a value/request has consumed it.
    let mut cursor: Option<u32> = None;
    let mut cursor_used = true;
    for (t, chunk) in &c.rx {
        dec.feed(chunk);
        while let Some(r) = dec.next_meter() {
            match r {
                Ok(tok) => {
                    if print_tokens {
                        println!("  [{t:8.3}] ch3 {tok:?}");
                    }
                    if let Some(cmd) = mp.feed(tok) {
                        rep.meter.push((*t, cmd));
                    }
                }
                Err(e) => rep.errors.push((*t, format!("ch3 {e}"))),
            }
        }
        while let Some(r) = dec.next_control() {
            match r {
                Ok(tok) => {
                    if print_tokens {
                        println!("  [{t:8.3}] ch1 {}", fmt_token(&tok, names));
                    }
                    if let Token::NavHash(h) = tok {
                        if !cursor_used {
                            rep.other.push((*t, format!("bare D7 {h:08x}")));
                        }
                        cursor = Some(h);
                        cursor_used = false;
                    } else if let Some(v) = Val::from(&tok) {
                        match cursor {
                            Some(h) if !cursor_used => rep.sets.push(Set {
                                t: *t,
                                hash: h,
                                val: v,
                            }),
                            _ => rep.other.push((*t, format!("value without D7: {v:?}"))),
                        }
                        cursor_used = true;
                    } else if tok == Token::DataRequest && !cursor_used {
                        rep.gets.push((*t, cursor.unwrap()));
                        cursor_used = true;
                    } else {
                        let what = match cursor {
                            Some(h) if !cursor_used => {
                                format!(
                                    "{} + {}",
                                    fmt_token(&Token::NavHash(h), names),
                                    fmt_token(&tok, names)
                                )
                            }
                            _ => fmt_token(&tok, names),
                        };
                        rep.other.push((*t, what));
                        cursor_used = true;
                        if tok == Token::Root {
                            cursor = None;
                            cursor_used = false;
                        }
                    }
                }
                Err(e) => rep.errors.push((*t, format!("ch1 {e}"))),
            }
        }
    }
    if !cursor_used {
        rep.pending_ctrl = cursor;
    }
    for ch in 0..14u8 {
        if ch != 1 && ch != 3 && dec.other_channel_bytes(ch) > 0 {
            rep.errors.push((
                0.0,
                format!("{} bytes on channel {ch}", dec.other_channel_bytes(ch)),
            ));
        }
    }
    if dec.unselected_bytes() > 0 {
        rep.errors.push((
            0.0,
            format!("{} bytes before any channel select", dec.unselected_bytes()),
        ));
    }
    rep
}

fn main() {
    let mut tree = "data/schema/wing-tree.json".to_string();
    let mut out_dir: Option<String> = None;
    let mut prefix = "wing-edit-connect".to_string();
    let mut server_bins = false;
    let mut tokens = false;
    let mut scene_conn: usize = 1;
    let mut source = "capture".to_string();
    let mut scene_out: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--tree" => tree = args.next().expect("--tree PATH"),
            "--out-dir" => out_dir = args.next(),
            "--prefix" => prefix = args.next().expect("--prefix NAME"),
            "--server-bins" => server_bins = true,
            "--tokens" => tokens = true,
            "--scene-conn" => scene_conn = args.next().expect("N").parse().expect("N"),
            "--scene-out" => scene_out = args.next(),
            "--source" => source = args.next().expect("--source TEXT"),
            o => panic!("unknown arg {o}"),
        }
    }
    let mut text = String::new();
    std::io::stdin().read_to_string(&mut text).expect("stdin");
    let names = load_hashes(&tree);
    eprintln!("schema hashes loaded: {}", names.len());
    let conns = parse_capture(&text);

    for (i, c) in conns.iter().enumerate() {
        let n = i + 1;
        let rx_total: usize = c.rx.iter().map(|x| x.1.len()).sum();
        let tx_total: usize = c.tx.iter().map(|x| x.1.len()).sum();
        println!(
            "== connection {n}: client port {} accepted {:.3}s closed {} ; {} chunks / {rx_total} B in, {} replies / {tx_total} B out",
            c.port,
            c.accepted,
            c.closed.map(|t| format!("{t:.3}s (by peer)")).unwrap_or("never (log ends)".into()),
            c.rx.len(),
            c.tx.len()
        );
        if let Some(d) = &out_dir {
            let rx: Vec<u8> = c.rx.iter().flat_map(|x| x.1.iter().copied()).collect();
            let tx: Vec<u8> = c.tx.iter().flat_map(|x| x.1.iter().copied()).collect();
            std::fs::write(format!("{d}/{prefix}-{n}.bin"), rx).expect("write");
            if server_bins {
                std::fs::write(format!("{d}/{prefix}-{n}.server.bin"), tx).expect("write");
            }
        }
        let rep = analyse(c, &names, tokens);
        for (t, cmd) in &rep.meter {
            println!("  [{t:8.3}] meter {cmd:?}");
        }
        let mapped = rep
            .sets
            .iter()
            .filter(|s| names.contains_key(&s.hash))
            .count();
        let mut uniq: BTreeMap<u32, usize> = BTreeMap::new();
        for s in &rep.sets {
            *uniq.entry(s.hash).or_default() += 1;
        }
        let uniq_mapped = uniq.keys().filter(|h| names.contains_key(h)).count();
        let mut encs: BTreeMap<&str, usize> = BTreeMap::new();
        for s in &rep.sets {
            *encs.entry(s.val.enc()).or_default() += 1;
        }
        if let (Some(f), Some(l)) = (rep.sets.first(), rep.sets.last()) {
            println!(
                "  sets: {} ({} unique hashes, {} mapped = {:.3}%; {} unique mapped) from {:.3}s to {:.3}s; encodings {encs:?}",
                rep.sets.len(),
                uniq.len(),
                mapped,
                100.0 * mapped as f64 / rep.sets.len() as f64,
                uniq_mapped,
                f.t,
                l.t
            );
            let dups: Vec<_> = uniq.iter().filter(|(_, &n)| n > 1).take(10).collect();
            if !dups.is_empty() {
                println!("  repeated hashes (first 10): {dups:x?}");
            }
            println!(
                "  first set: {} ; last set: {}",
                names
                    .get(&f.hash)
                    .cloned()
                    .unwrap_or(format!("{:08x}", f.hash)),
                names
                    .get(&l.hash)
                    .cloned()
                    .unwrap_or(format!("{:08x}", l.hash))
            );
        }
        let unm: Vec<String> = uniq
            .keys()
            .filter(|h| !names.contains_key(h))
            .map(|h| format!("{h:08x}"))
            .collect();
        println!("  unmapped set hashes ({}): {}", unm.len(), unm.join(" "));
        println!("  gets (D7 h DC): {}", rep.gets.len());
        for (t, h) in rep.gets.iter().take(400) {
            println!(
                "    [{t:8.3}] get {h:08x} {}",
                names.get(h).map(String::as_str).unwrap_or("?UNMAPPED")
            );
        }
        println!("  other control tokens: {}", rep.other.len());
        for (t, s) in &rep.other {
            println!("    [{t:8.3}] {s}");
        }
        if let Some(h) = rep.pending_ctrl {
            println!("  trailing D7 {h:08x} with no value/request");
        }
        for (t, e) in &rep.errors {
            println!("  ERROR [{t:8.3}] {e}");
        }

        if let Some(path) = scene_out.as_ref().filter(|_| n == scene_conn) {
            let mut params: BTreeMap<String, String> = BTreeMap::new();
            let mut unmapped: BTreeMap<String, String> = BTreeMap::new();
            // Sets addressed to hash 0 (nodes WING-Edit never hashed), grouped by the preceding
            // mapped path: (after, values).
            let mut zero_runs: Vec<(String, Vec<String>)> = Vec::new();
            let mut prev = String::new();
            let mut prev_zero = false;
            for s in &rep.sets {
                match names.get(&s.hash) {
                    Some(p) => {
                        params.insert(p.clone(), s.val.json());
                        prev = p.clone();
                        prev_zero = false;
                    }
                    None if s.hash == 0 => {
                        if !prev_zero {
                            zero_runs.push((prev.clone(), Vec::new()));
                        }
                        zero_runs.last_mut().unwrap().1.push(s.val.json());
                        prev_zero = true;
                    }
                    None => {
                        unmapped.insert(format!("{:08x}", s.hash), s.val.json());
                        prev_zero = false;
                    }
                }
            }
            let zero_json = zero_runs
                .iter()
                .map(|(a, v)| {
                    format!(
                        "\n    {{\"after\": {}, \"values\": [{}]}}",
                        json_str(a),
                        v.join(", ")
                    )
                })
                .collect::<Vec<_>>()
                .join(",");
            let mut o = String::new();
            let _ = writeln!(o, "{{");
            let _ = writeln!(
                o,
                "  \"source\": {},",
                json_str(&format!(
                    "{source}, connection {n} (WING-Edit 3.3.4 PC->MIXER push, D7 <hash> <value>)"
                ))
            );
            let _ = writeln!(
                o,
                "  \"generator\": \"cargo run -p wing-proto --example decode_capture\","
            );
            let _ = writeln!(o, "  \"set_count\": {},", rep.sets.len());
            let _ = writeln!(
                o,
                "  \"encodings\": {{{}}},",
                encs.iter()
                    .map(|(k, v)| format!("\"{k}\": {v}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            let _ = writeln!(
                o,
                "  \"unmapped\": {{{}}},",
                unmapped
                    .iter()
                    .map(|(k, v)| format!("\"{k}\": {v}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            let _ = writeln!(
                o,
                "  \"hash0_set_count\": {},",
                zero_runs.iter().map(|r| r.1.len()).sum::<usize>()
            );
            let _ = writeln!(o, "  \"hash0_runs\": [{zero_json}\n  ],");
            let _ = writeln!(o, "  \"params\": {{");
            let last = params.len();
            for (i, (k, v)) in params.iter().enumerate() {
                let _ = writeln!(
                    o,
                    "    {}: {v}{}",
                    json_str(k),
                    if i + 1 < last { "," } else { "" }
                );
            }
            let _ = writeln!(o, "  }}");
            let _ = writeln!(o, "}}");
            std::fs::write(path, o).expect("write scene");
            eprintln!(
                "wrote {path}: {} params, {} unmapped, {} hash-0 runs",
                params.len(),
                unmapped.len(),
                zero_runs.len()
            );
        }
    }
}
