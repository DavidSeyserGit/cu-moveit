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

# Verify strict no_std compatibility across shared crates.
nostd-ci:
    cargo check --workspace --no-default-features
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo nextest run --workspace --no-default-features
