# Contributing to Repose

This repository follows the exact contribution workflow and design standards of [`copper-project/copper-rs`](https://github.com/copper-project/copper-rs/blob/master/CONTRIBUTING.md) so crates developed here can be upstreamed directly into `copper-rs`.

## Contribution Workflow

1. Create a branch from `master` using `user/kind/description`, for example `lucas/feat/kinematics-ik`.
2. Make one focused change. Add tests and update documentation or examples where behavior requires them.
3. Run `just` (`just pr-check`) from the repository root. It checks formatting, runs `clippy --deny warnings` on `std` and `no_std`, runs `cargo nextest` on `std` and `no_std`, and runs doctests.
4. Review the complete diff, then commit with a clear message.

## Design Expectations

- **Static over dynamic:** Prefer types, const generics (`const N: usize`), and compile-time wiring to runtime string lookup or heap-allocated vectors.
- **Realtime paths stay lean:** Do not add allocations, copies, serialization passes, or latency to the hot path. Write results in place via `&mut` parameters.
- **`no_std` is a real target:** Shared libraries, payloads, and tasks must compile and pass tests with `--no-default-features`.
- **Determinism and replay are product features:** Stateful tasks must implement `Freezable` (`freeze`/`thaw`) and propagate `input.tov` to `output.tov`.
