# cu-moveit Roadmap

## Goal

Build a general, MoveIt-equivalent manipulation stack for
[copper-rs (`cu29`)](https://github.com/copper-project/copper-rs) — Copper-native,
not ported from ROS. It covers kinematics, collision checking, motion planning,
trajectory generation, execution, servoing, and robot bridges for Copper apps,
in that order.

## Success Criteria

- Every phase below is closed only when all of its done-criteria hold
  (see Validation Plan).
- The stack runs end-to-end in a local demonstration: robot description in,
  planned and timed trajectory out, executed under collision monitoring.
- Public APIs are documented, semver-stable in intent, and crates.io-ready;
  no placeholder surfaces ship as "done".

## Current State (done)

- `cu-kinematics`: `CuKinematicChain<N>` FK, space/body Jacobians,
  manipulability, DLS and differential IK with singularity/joint-limit damping,
  FK/IK/servo `CuTask`s, igus ReBeL preset.
- `cu-collision`: fixed-capacity sphere/capsule model, signed-distance queries,
  self/environment checking, `CuCollisionCheckTask<N>`.
- `cu-robot-description`: startup URDF serial-chain and SRDF group/exclusion/
  state import into fixed-size kinematic and collision models.
- `cu-joint-rrt-star`: joint-space `CuAnytimeTask` RRT* with deterministic
  `CuRng` seeding, informed pruning, rewiring, and shortcutting.
- All four crates pass `just pr-check` and are published at
  https://github.com/DavidSeyserGit/cu-moveit.

## Approach

Phases are sequential; each phase builds only on the crates and tasks of the
phases before it. Serial chains are the only mechanism in Phases 1–4; general
mechanisms are a deliberate later phase, never a drive-by addition. Copper
house rules apply throughout: `const N: usize` static topology, zero hot-path
allocation, in-place `&mut` outputs, `no_std`-compatible shared crates,
`cu29::units` physical units, `CuResult<T>` errors, `Reflect` + `Freezable`
tasks, deterministic replay-compatible behaviour.

## Steps

1. **Trajectory generation and execution (next).** Time-parameterization of
   geometric paths under velocity/acceleration/jerk limits (TOPP-style) into
   fixed-capacity trajectory payloads; `CuFollowTrajectoryTask<N>` with
   interpolation, progress/failure reporting, preemption, and abort; execution
   monitoring against the collision model. A local-only demonstration (in the
   git-excluded `playground/`, never committed) proves URDF → RRT* → timed
   trajectory → simulated execution end-to-end.
2. **Planning completeness.** Versioned, fixed-capacity planning-scene API
   (attached objects, obstacle add/remove) shared by collision and planner
   tasks; Cartesian paths and pose-goal planning on the existing IK; added
   planners (e.g. PRM/BIT*-class) only where seeded benchmark results justify
   them; deterministic benchmark harness (success rate, path cost, time).
3. **Real-time servoing.** Harden `CuServoTask<N>` into a production servo
   loop: rate limiting, watchdog timeouts, singularity/limit behaviour under
   load, latency/allocation budgets enforced by tests.
4. **Robot bridges.** Define a generic `CuBridge` joint command/state contract
   so execution stays robot-agnostic; prove it with `cu-igus-cri` for the igus
   ReBeL CRI protocol (`192.168.3.11:3920`); hardware-in-the-loop example plans
   and executes on the real ReBeL.
5. **Generality (deliberately last).** Tree/branched kinematic structures,
   then closed-chain mechanisms, then dynamics and torque-level control. No
   Phase 1–4 API may be designed so these become impossible; none of this
   starts before Step 4 is done.
6. **1.0 stabilization and publishing.** Rustdoc and doctests on every public
   item; semver commitment; a stated compatibility policy for RON
   configuration schemas. Publishing is pre-wired so this step is a flip, not
   a restructure: the workspace already carries one shared version, versioned
   path dependencies between crates, correct `repository` metadata, and
   `just api-check` / `just release-dry-run`. To publish: flip the single
   `publish = false` switch in the workspace `Cargo.toml` to `true`, publish
   all crates to crates.io in lockstep (dependency order) at the shared
   version, add the thin `cu-moveit` facade crate re-exporting them behind
   features, and tag the release `v1.0.0`. Until then, releases are
   git tags (`vX.Y.Z`, workspace-wide, never per-crate) with GitHub Releases,
   and consumers use git dependencies.

## Validation Plan

A phase counts as done only when all four hold:

1. `just pr-check` passes: fmt, `std` and `no_std` clippy with denied
   warnings, nextest, and doctests.
2. The phase's public API is documented and semver-stable in intent.
3. Packaging metadata (repository, license, keywords, categories) is correct
   and crates.io-ready.
4. A runnable local demonstration (in `playground/`, never committed)
   demonstrates the phase end-to-end.

Passing tests alone does not close a phase.

## Non-Goals — What We Will Not Do

- **No ROS / ROS 2 interop.** No ROS nodes, messages, actions, `ros2_control`
  adapters, or MoveIt plugin/config compatibility. URDF/SRDF *file* import at
  startup (already in `cu-robot-description`) is the sole exception — a file
  format, not a ROS runtime dependency.
- **No generic perception.** No point-cloud pipelines, camera drivers, object
  recognition, or grasp synthesis. External perception may publish Copper
  payloads for the planning scene to consume; building it is not this project.
- **No GUI product.** No planning-scene editor or RViz equivalent as a
  deliverable. The local playground is an unpublished development aid and does
  not set product direction.
- **No dynamic runtime topology.** No runtime-loaded task graphs, plugin
  registries, or stringly-typed hot-path configuration; Copper's compile-time
  graph and RON policies are the model.
- **No early generality.** Branched/closed chains and dynamics appear only in
  Step 5, on that step's terms — not as additions to earlier phases.

## Risks / Open Questions

- Highest risk: Step 1 execution semantics (preemption, abort, monitoring)
  fixing trajectory payload shapes that later steps must live with; validate
  with the end-to-end example before freezing them.
- Undecided: additional robot presets beyond the igus ReBeL (added only when a
  bridge or user needs one); the second bridge target after `cu-igus-cri`
  (decided by hardware access); whether benchmark results ship as a report
  crate or as local demonstrations.
