.PHONY: check fmt build bench

check:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace

fmt:
	cargo fmt --all

build:
	cargo build --release --workspace

bench:
	cargo bench -p wing-dsp
