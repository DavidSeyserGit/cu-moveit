# Default local PR gate: format, lint (std + no_std), and test (std + no_std + doctests).
default: pr-check

# Full local verification gate matching copper-rs workflow.
pr-check: fmt lint test semver-check

# Format all Rust sources in the workspace.
fmt:
    cargo fmt --all

# Check formatting and run clippy with warnings denied on both std and no_std surfaces.
lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets --features std -- --deny warnings
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo clippy --workspace --all-targets --features std,cu29/reflect -- --deny warnings

# Run unit tests (std + no_std) with nextest and verify doctests.
test:
    cargo nextest run --workspace --features std
    cargo nextest run --workspace --no-default-features
    cargo test --workspace --doc --features std
    cargo nextest run --workspace --features std,cu29/reflect

# Print the current public API for review (needs cargo-public-api + pinned nightly; see copper-workflow).
# Pre-1.0 this is advisory; at 1.0, diff its output against the previous release before tagging.
api-check:
    cargo public-api -p repose-kinematics
    cargo public-api -p repose-collision
    cargo public-api -p repose-robot-description
    cargo public-api -p repose-trajectory
    cargo public-api -p repose-controller
    cargo public-api -p repose-io
    cargo public-api -p repose-motion-planning
    cargo public-api -p repose-joint-rrt-star
    cargo public-api -p repose-mujoco-sim
    cargo public-api -p repose

# Check every library crate for SemVer breaks against the latest release tag (needs cargo-semver-checks).
# Part of pr-check. A deliberate breaking change on 0.x needs a minor release: run
# `SEMVER_RELEASE_TYPE=minor just pr-check`. Crates absent from the baseline (new or renamed) are skipped.
semver-check baseline=`git describe --tags --abbrev=0 --match 'v*'`:
    #!/usr/bin/env bash
    set -euo pipefail
    release_type="${SEMVER_RELEASE_TYPE:-}"
    args=()
    for pkg in $(cargo metadata --no-deps --format-version 1 | jq -r '.packages[].name'); do
        if git grep -q "^name = \"$pkg\"" "{{baseline}}" -- '*Cargo.toml'; then
            args+=(-p "$pkg")
        else
            echo "semver-check: skipping $pkg (not in {{baseline}})"
        fi
    done
    if [ ${#args[@]} -eq 0 ]; then exit 0; fi
    # One invocation so the baseline is checked out and built once for all crates.
    cargo semver-checks check-release "${args[@]}" --baseline-rev "{{baseline}}" ${release_type:+--release-type "$release_type"}

# Bump the shared workspace version, commit and tag vX.Y.Z (needs cargo-release; see docs/versioning.md).
# Dry run by default; pass --execute to apply. Never pushes or publishes.
release level *args:
    cargo release {{level}} --workspace {{args}}

# Dry-run packaging for every crate without publishing (publish = false must be flipped first at 1.0).
release-dry-run:
    cargo publish --dry-run -p repose-kinematics
    cargo publish --dry-run -p repose-collision
    cargo publish --dry-run -p repose-robot-description
    cargo publish --dry-run -p repose-trajectory
    cargo publish --dry-run -p repose-controller
    cargo publish --dry-run -p repose-io
    cargo publish --dry-run -p repose-motion-planning
    cargo publish --dry-run -p repose-joint-rrt-star
    cargo publish --dry-run -p repose-mujoco-sim
    cargo publish --dry-run -p repose

# Verify strict no_std compatibility across shared crates.
nostd-ci:
    cargo check --workspace --no-default-features
    cargo clippy --workspace --lib --no-default-features -- --deny warnings
    cargo nextest run --workspace --no-default-features
