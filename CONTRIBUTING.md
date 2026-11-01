# Contributing to Repose

This repository follows the exact contribution workflow and design standards of [`copper-project/copper-rs`](https://github.com/copper-project/copper-rs/blob/master/CONTRIBUTING.md) so crates developed here can be upstreamed directly into `copper-rs`.

## Contribution Workflow

AI agents and tools follow the same rules, spelled out in `AGENTS.md`.

`master` is protected: every change lands through a pull request with green CI,
and nobody pushes to it directly.

### Branches

- Branch from the latest `origin/master` and name it `user/kind/description`,
  e.g. `lucas/feat/kinematics-ik`. `kind` is one of `feat`, `fix`, `refactor`,
  `docs`, `chore`, `perf`, `test`.
- Agent-created branches may use the tool's prefix (e.g. `t3/...`).
- One branch, one focused change. Keep branches short-lived; rebase on
  `origin/master` instead of merging it in.

### Worktrees

Keep the main checkout on a clean `master`: use it only to pull, review and
tag. Do the work in one worktree per branch, next to the repository:

```bash
just wt lucas/feat/kinematics-ik      # ../repose-wt/kinematics-ik on a new branch from origin/master
cd ../repose-wt/kinematics-ik
# ...work, commit...
git push -u origin lucas/feat/kinematics-ik
gh pr create --fill
just wt-clean                         # from the main checkout: remove worktrees whose branch is merged
```

Worktrees share one `.git`, so branches and fetched refs are visible
everywhere, but each has its own `target/`. Don't check out the same branch in
two worktrees.

### Pull requests

1. Run `just` (`just pr-check`): formatting, clippy (`std`, `no_std`,
   reflect), nextest, doctests and the SemVer check.
2. Add an entry under `Unreleased` in `CHANGELOG.md` for any user-visible
   change.
3. Push (`git push -u origin <branch>` the first time) and open a PR against
   `master` (`gh pr create --fill`). The title is
   the eventual commit message; the body says what changed, why, and how it
   was tested.
4. If the change is intentionally breaking (see `docs/versioning.md`), list
   it under `### Breaking` in the `Unreleased` changelog section. The SemVer
   check then treats the next release as a minor and allows the break, for this
   PR and for later ones until the release.
5. PRs are **squash-merged**, so `master` gets one commit per PR and stays
   linear. Delete the branch after merging.

### Releases

Releases go through a PR too; see `docs/versioning.md`.

## Design Expectations

- **Static over dynamic:** Prefer types, const generics (`const N: usize`), and compile-time wiring to runtime string lookup or heap-allocated vectors.
- **Realtime paths stay lean:** Do not add allocations, copies, serialization passes, or latency to the hot path. Write results in place via `&mut` parameters.
- **`no_std` is a real target:** Shared libraries, payloads, and tasks must compile and pass tests with `--no-default-features`.
- **Determinism and replay are product features:** Stateful tasks must implement `Freezable` (`freeze`/`thaw`) and propagate `input.tov` to `output.tov`.
