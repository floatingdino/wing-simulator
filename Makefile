.PHONY: check check-full fidelity-report fmt build bench app dist

check:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace

# Everything: fast set plus #[ignore = "slow: ..."] tests, in release (T074). Run before finishing a DSP task.
check-full:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace --release -- --include-ignored

# Regenerate docs/fidelity-report.md from the full reference grids (T072).
fidelity-report:
	WING_FIDELITY_REPORT=$(CURDIR)/docs/fidelity-report.md cargo test -p wing-blocks --release --test fidelity_all -- --ignored fidelity_all_full

fmt:
	cargo fmt --all

build:
	cargo build --release --workspace

bench:
	cargo bench -p wing-dsp

# Menu-bar app (T064; needs Xcode / Swift 6.2+): release daemon + diff tool, the .app
# (apps/WingSimMenu/build/WingSimMenu.app), Swift unit tests and a launch/quit smoke test.
app:
	cargo build --release -p wing-sim -p wing-render -p wing-diff
	apps/WingSimMenu/build.sh
	cd apps/WingSimMenu && swift test
	apps/WingSimMenu/smoke.sh

# Release bundle dist/WingSimulator-<version>/ (T073): binaries, driver, app, install/uninstall scripts. No sudo.
dist:
	tools/dist/dist.sh
