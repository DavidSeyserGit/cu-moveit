# Changelog

All notable changes to Repose. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versioning
follows [docs/versioning.md](docs/versioning.md). Pre-1.0, every minor release
may be breaking.

## [Unreleased]

### Changed

- `master` is protected; changes land through squash-merged PRs from
  per-branch worktrees (`just wt`, `just wt-clean`). Releases bump the version
  in a PR and are tagged afterwards with `just tag`. CI runs the SemVer check
  on PRs (label `breaking` to allow a minor break).

## [0.7.0] - 2026-11-01

### Breaking

- Crates renamed for the Repose rebrand: `cu-repose` → `repose`, and every
  `cu-<name>` crate → `repose-<name>` (Rust paths `cu_<name>` →
  `repose_<name>`, including type paths in RON configs). Copper's own crates
  (`cu29`, `cu-spatial-payloads`, …) are unchanged. Update dependencies and
  `use` paths; nothing else changed.

### Changed

- Versioning policy documented in `docs/versioning.md`; release notes
  consolidated into this changelog (older notes under `docs/releases/`).
- `just pr-check` runs `cargo-semver-checks`; `just release` wraps
  `cargo-release` with one shared version and `vX.Y.Z` tags.

## [0.6.0] - 2026-11-01

Runtime reflection works again, and the MuJoCo example is a RON-only graph.
Full notes: [docs/releases/0.6.0.md](docs/releases/0.6.0.md).

## [0.5.0] - 2026-10-31

Breaking API simplification across the workspace; see
[docs/migration-0.5.md](docs/migration-0.5.md).
Full notes: [docs/releases/0.5.0.md](docs/releases/0.5.0.md).

## [0.4.1] - 2026-10-31

## [0.4.0] - 2026-10-31

Planner-agnostic motion planning: sampling-based and non-sampling planners
share one contract and one Copper task.
Full notes: [docs/releases/0.4.0.md](docs/releases/0.4.0.md).

## [0.3.0] - 2026-10-30

## [0.2.3] - 2026-10-30

## [0.2.2] - 2026-10-30

## [0.2.1] - 2026-10-30

## [0.2.0] - 2026-10-30

## [0.1.0] - 2026-10-29

Initial release.

[Unreleased]: https://github.com/ReposeRobotics/repose/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/ReposeRobotics/repose/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/ReposeRobotics/repose/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/ReposeRobotics/repose/compare/v0.4.1...v0.5.0
[0.4.1]: https://github.com/ReposeRobotics/repose/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/ReposeRobotics/repose/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/ReposeRobotics/repose/compare/v0.2.3...v0.3.0
[0.2.3]: https://github.com/ReposeRobotics/repose/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/ReposeRobotics/repose/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/ReposeRobotics/repose/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/ReposeRobotics/repose/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/ReposeRobotics/repose/releases/tag/v0.1.0
