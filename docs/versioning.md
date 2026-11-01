# Versioning

Repose follows [Semantic Versioning](https://semver.org/) with the pre-1.0
rules below. This document is the source of truth; the README links here.

## One version for the whole workspace

- Every library crate inherits `version` from `[workspace.package]`. Crates are
  never versioned or tagged individually.
- `repose` (the umbrella crate) carries the **product version**: "Repose 0.7"
  means every `cu-*` crate at `0.7.x`.
- Releases are tagged `vX.Y.Z` on `master`.

We will reconsider per-crate versions only once a single crate (e.g.
`repose-kinematics`) has significant outside users who do not depend on the rest.

## What counts as breaking

Pre-1.0, a **minor** bump (`0.6 → 0.7`) is a breaking release and a **patch**
bump (`0.6.0 → 0.6.1`) is never breaking. A change is breaking if it changes
any of these:

1. **Rust public API**: removed or renamed items, changed signatures, new trait
   requirements, new required fields on public structs, changed
   const-generic parameters.
2. **Payload encoding**: any change to the field layout or order of a payload
   that derives `Encode`/`Decode`. This breaks existing Copper logs and
   replay even when the Rust API looks unchanged.
3. **RON configuration**: renamed, removed, or reinterpreted task config keys,
   or changed units or defaults that alter behavior.
4. **Copper (`cu29`) minor version**: moving to a new `cu29` minor is always
   a Repose minor bump.
5. **MSRV**: raising `rust-version`.

Patch releases may fix bugs, add new items (new functions, tasks, presets)
and improve docs or performance, as long as none of the above change.

Breaking releases ship a migration note under `docs/` (see
`docs/migration-0.5.md`) when users have to change code.

## Copper compatibility

The README has a compatibility table mapping Repose minors to `cu29` and the
MSRV. Update it with every release that changes either one.

## 1.0

1.0 happens when the motion-planning (`CuJointPlanner`), servo and igus CRI
bridge APIs are stable. Crates are named `repose` / `repose-*` since 0.7 (the
`cu-*` names were retired in the rebrand); crates.io publishing starts at 1.0. From 1.0 on, normal SemVer applies: breaking
changes require a major bump.

## Applications

Studio lives in its own repository (`ReposeRobotics/studio`, a submodule here)
and has its own SemVer and `studio-vX.Y.Z` tags. Its release notes state which
Repose version it targets.

Examples and the local playground are not versioned (`version = "0.0.0"`,
`publish = false`); they always track the workspace they live in.

## Release process

1. Update `CHANGELOG.md`: move `Unreleased` entries under the new version
   with the date, and update the README compatibility table if needed.
2. `just pr-check`, which includes `just semver-check`: every library crate is
   checked with `cargo-semver-checks` against the latest `v*` tag. If a branch
   breaks the API on purpose, the next release must be a minor; confirm with
   `SEMVER_RELEASE_TYPE=minor just pr-check`. Crates missing from the baseline
   tag (new or renamed) are skipped.
3. `just release <patch|minor>`: dry run; then `just release <level> --execute`.
   This runs `cargo release`, which bumps the shared version, commits and
   creates the `vX.Y.Z` tag. It does not push or publish.
4. Push the commit and tag, and publish the GitHub release using the
   changelog section as the notes.
