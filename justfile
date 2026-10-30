# Default local PR gate: format, lint (std + no_std), and test (std + no_std + doctests).
default: pr-check

# Full local verification gate matching copper-rs workflow.
pr-check: fmt lint test

# Format all Rust sources in the workspace.
fmt:
    cargo fmt --all

# Check formatting and run clippy with warnings denied on both std and no_std surfaces.
lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets --features std -- --deny warnings
    cargo clippy --workspace --lib --no-default-features -- --deny warnings

# Run unit tests (std + no_std) with nextest and verify doctests.
test:
    cargo nextest run --workspace --features std
    cargo nextest run --workspace --no-default-features
    cargo test --workspace --doc --features std

# Print the current public API for review (needs cargo-public-api + pinned nightly; see copper-workflow).
# Pre-1.0 this is advisory; at 1.0, diff its output against the previous release before tagging.
api-check:
    cargo public-api -p cu-kinematics
    cargo public-api -p cu-collision
    cargo public-api -p cu-robot-description
    cargo public-api -p cu-trajectory
    cargo public-api -p cu-controller
    cargo public-api -p cu-io
    cargo public-api -p cu-motion-planning
    cargo public-api -p cu-joint-rrt-star
    cargo public-api -p cu-mujoco-sim

# Dry-run packaging for every crate without publishing (publish = false must be flipped first at 1.0).
release-dry-run:
    cargo publish --dry-run -p cu-kinematics
    cargo publish --dry-run -p cu-collision
    cargo publish --dry-run -p cu-robot-description
    cargo publish --dry-run -p cu-trajectory
    cargo publish --dry-run -p cu-controller
    cargo publish --dry-run -p cu-io
    cargo publish --dry-run -p cu-motion-planning
    cargo publish --dry-run -p cu-joint-rrt-star
    cargo publish --dry-run -p cu-mujoco-sim

# Verify strict no_std compatibility across shared crates.
nostd-ci:
    cargo check --workspace --no-default-features
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo nextest run --workspace --no-default-features
