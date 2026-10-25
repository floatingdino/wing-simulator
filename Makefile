.PHONY: check check-full fmt build bench app

check:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace

# Everything: fast set plus #[ignore = "slow: ..."] tests, in release (T074). Run before finishing a DSP task.
check-full:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace --release -- --include-ignored

fmt:
	cargo fmt --all

build:
	cargo build --release --workspace

bench:
	cargo bench -p wing-dsp

# Menu-bar app (T064; needs Xcode / Swift 6.2+): release daemon + diff tool, the .app
# (apps/WingSimMenu/build/WingSimMenu.app), Swift unit tests and a launch/quit smoke test.
app:
	cargo build --release -p wing-sim -p wing-diff
	apps/WingSimMenu/build.sh
	cd apps/WingSimMenu && swift test
	apps/WingSimMenu/smoke.sh
