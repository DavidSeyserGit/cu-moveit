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
# Part of pr-check. Breaks are allowed once CHANGELOG.md has a `### Breaking` section under Unreleased
# (the next release is then a minor); SEMVER_RELEASE_TYPE overrides. Crates absent from the baseline are skipped.
semver-check baseline=`git describe --tags --abbrev=0 --match 'v*'`:
    #!/usr/bin/env bash
    set -euo pipefail
    release_type="${SEMVER_RELEASE_TYPE:-}"
    # A `### Breaking` section under Unreleased means the next release is a minor (pre-1.0).
    if [ -z "$release_type" ] && awk '/^## \[Unreleased\]/{u=1;next} /^## \[/{u=0} u && /^### Breaking/{f=1} END{exit !f}' CHANGELOG.md; then
        release_type=minor
        echo "semver-check: CHANGELOG has unreleased breaking changes; checking as a minor release"
    fi
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

# Bump the shared workspace version and commit, on a release branch (needs cargo-release; see docs/versioning.md).
# Dry run by default; pass --execute to apply. Never tags, pushes or publishes.
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

# Create a worktree for a new branch off origin/master at <main checkout>/../repose-wt/<last path segment>.
# Works from any worktree. The branch has no upstream until its first `git push -u origin <branch>`.
wt branch:
    #!/usr/bin/env bash
    set -euo pipefail
    main=$(dirname "$(git rev-parse --path-format=absolute --git-common-dir)")
    dir="$(dirname "$main")/repose-wt/{{file_name(branch)}}"
    git fetch origin master
    git worktree add --no-track -b "{{branch}}" "$dir" origin/master
    echo "cd $dir"

# Remove clean worktrees (and their local branches) whose work is on master: the branch is an ancestor of
# origin/master, or its PR was merged with exactly the local tip (squash merges). Anything else is kept.
wt-clean:
    #!/usr/bin/env bash
    set -euo pipefail
    git fetch --prune origin
    git worktree list --porcelain -z | while IFS= read -r -d '' line; do
        case "$line" in
            "worktree "*) path="${line#worktree }" ;;
            "branch refs/heads/"*) branch="${line#branch refs/heads/}" ;;
            "")
                if [ -n "${branch:-}" ] && [ "$branch" != master ]; then
                    tip=$(git rev-parse "refs/heads/$branch")
                    merged=$(gh pr list --state merged --head "$branch" --json headRefOid --jq ".[] | select(.headRefOid == \"$tip\") | .headRefOid" 2>/dev/null || true)
                    if git merge-base --is-ancestor "$tip" origin/master || [ -n "$merged" ]; then
                        echo "removing $path ($branch)"
                        git worktree remove "$path" && git branch -D "$branch" || echo "  kept: worktree has local changes"
                    fi
                fi
                path=""; branch="" ;;
        esac
    done
    git worktree prune

# Tag origin/master vX.Y.Z from the workspace version and push the tag (after the release PR is merged).
tag:
    #!/usr/bin/env bash
    set -euo pipefail
    [ "$(git branch --show-current)" = master ] || { echo "run on master"; exit 1; }
    [ -z "$(git status --porcelain --untracked-files=no)" ] || { echo "working tree is dirty"; exit 1; }
    git pull --ff-only origin master
    [ "$(git rev-parse HEAD)" = "$(git rev-parse origin/master)" ] || { echo "HEAD is not origin/master"; exit 1; }
    version=$(cargo metadata --no-deps --format-version 1 | jq -r '.packages[] | select(.name == "repose") | .version')
    git tag -a "v$version" -m "Repose v$version"
    git push origin "v$version"
