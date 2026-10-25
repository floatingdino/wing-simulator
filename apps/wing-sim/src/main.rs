//! Wing Simulator daemon: CLI, config, control server and RT engine host.

fn main() {
    println!("wing-sim {}", env!("CARGO_PKG_VERSION"));
}

#[cfg(test)]
mod tests {
    #[test]
    fn scaffold() {
        assert_eq!(1 + 1, 2);
    }
}
