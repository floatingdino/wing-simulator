//! Offline renderer: multitrack WAVs plus .snap to stems/mix.

fn main() {
    println!("wing-render {}", env!("CARGO_PKG_VERSION"));
}

#[cfg(test)]
mod tests {
    #[test]
    fn scaffold() {
        assert_eq!(1 + 1, 2);
    }
}
