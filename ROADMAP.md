# Repose Roadmap

## Goal

Build Repose, a general manipulation stack comparable in scope to MoveIt, for
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
- `cu-motion-planning`: planner-agnostic `CuJointPlanner` trait, request/path
  payloads with explicit `CuPlanStatus`, shared scene validity checks, the
  generic anytime `CuJointPlannerTask<P, N>`, and built-in planners:
  RRT-Connect and PRM (sampling), A* lattice (search), gradient path
  optimizer and STOMP (optimization), direct joint line.
- `cu-joint-rrt-star`: sampling-based RRT* implementing `CuJointPlanner`, with
  deterministic `CuRng` seeding, informed pruning, rewiring, and shortcutting.
- `cu-trajectory`: fixed-capacity trajectory payloads, profile timing, per-waypoint
  Cartesian TCP zones, C² spline geometry, quintic interpolation, and S-curve
  joint jerk bounds; follower preemption and empty-trajectory cancellation.
- `cu-mujoco-sim`: `CuBridge` to a MuJoCo sidecar over TCP/JSON carrying
  `CuJointPositionsPayload<6>` both ways (released in `v0.2.1`); the local
  playground closes the plan → follow → sim → feedback loop.
- All crates pass `just pr-check` and are published at
  https://github.com/ReposeRobotics/repose.

## Approach

Phases are sequential; each phase builds only on the crates and tasks of the
phases before it. Serial chains are the only mechanism in Phases 1–4; general
mechanisms are a deliberate later phase, never a drive-by addition. Copper
house rules apply throughout: `const N: usize` static topology, zero hot-path
allocation, in-place `&mut` outputs, `no_std`-compatible shared crates,
`cu29::units` physical units, `CuResult<T>` errors, `Reflect` + `Freezable`
tasks, deterministic replay-compatible behaviour.

## Release 0.3 (released in `v0.3.0`)

Controller-interface completion, building on `v0.2.2`:

- **Effort/torque commands (landed in `v0.2.3`).** `CuJointCommandPayload<N>`
  carries position, velocity, and effort modes, with typed torque/force per
  joint kind. `CuControllerTask<N>` computes PD effort; bridges apply raw
  torque/force. Position-mode velocity feedforward landed with it.
- **Gripper and digital IO (implemented).** `cu-io` defines the
  robot-agnostic `digital_in`/`digital_out` payloads
  (`CuDigitalInputPayload<N>` with device status, `CuDigitalOutputPayload<N>`
  with a drive mask). A gripper is IO, not a payload of its own:
  `CuGripperTask<IN, OUT>` maps open/close commands onto RON-configured
  output bits and optional sensor inputs, reporting open/closed/moving/fault.
  Bridge support (sim and hardware) follows with each bridge.
- **Robot-side status/fault reporting (implemented).** `CuJointStatePayload<N>` gains a
  bridge-reported status (ok, fault with code, estop, not-ready), distinct
  from the controller's own status; the controller surfaces it and faults
  instead of commanding a robot that reports a fault.
- **Rate limiting and watchdog (implemented).** `CuControllerTask<N>` gains configurable
  command rate limiting (max joint velocity/acceleration of emitted
  commands) and a time-based watchdog: stale measured state (by `Tov`)
  beyond a configured age faults and holds, replacing today's purely
  per-cycle checks. (Pulled forward from Step 3.)

## Steps

1. **Trajectory generation and simulated execution (implemented; closeout pending).**
   Fixed-capacity trajectory payloads, profile timing, per-waypoint Cartesian
   TCP-zone blending, and `CuFollowTrajectoryTask<N>` interpolation, status,
   preemption, and cancellation are implemented. Cartesian S-curve joint jerk
   bounds require quintic execution. The git-excluded `playground/` demonstrates
   local planning, timing, and simulated playback. Closeout reviews the public
   API, packaging, and cancellation semantics against the Validation Plan.
   Collision monitoring and collision-triggered abort during execution are
   deferred to Step 3; they do not block Step 1.
2. **Planning and simulated robot execution (next).** Build in this order:
   - A versioned, fixed-capacity planning scene shared by collision checking and
     planner tasks: stable object IDs, obstacle add/update/remove, attached
     objects with link-relative transforms, and explicit collision exclusions.
   - Consistent scene snapshots for each planning request, with scene revisions
     carried into results and deterministic restart/invalidation when the scene
     changes during anytime planning.
   - Connect a physics simulator through a Copper joint-state/command interface.
     Read measured joint states, plan from the current state, time the path,
     send follower setpoints to the simulator, and observe returned joint states
     and execution status. Use a local, git-excluded demonstration to exercise
     the complete loop; kinematic animation alone does not satisfy this step.
     Define the joint command/state contract here so it can also support the
     hardware bridge in Step 4. (Implemented: `cu-mujoco-sim` `CuBridge` over
     `CuJointPositionsPayload<6>`, released in `v0.2.1`; the local playground
     closes the loop and the Trajectory Studio executes in the sim.)
   - **Unified robot controller (implemented in `v0.2.2`).** Upstream `cu29` has no robot
     controller abstraction — only the generic `CuBridge` trait and task
     roles — so this contract is defined here, robot-agnostically:
     a `CuControllerTask<N>` between the trajectory follower and a `CuBridge`,
     consuming the follower setpoint plus measured joint state and emitting one
     unified joint command (positions, optional velocities) with an explicit
     status (idle/running/holding/fault) and tracking error. Shared
     `CuJointStatePayload<N>` / `CuJointCommandPayload<N>` payloads (extending
     today's `CuJointPositionsPayload<N>`) are the only interface a bridge
     sees: simulators (`cu-mujoco-sim`) and hardware (`cu-igus-cri`, Step 4)
     implement the same `joint_command`/`joint_state` channels, so swapping
     sim for a real robot is a RON graph change, not an app change. Generalize
     `cu-mujoco-sim` from fixed `N = 6` as the first consumer.
     Shape (per Copper component design): a new crate
     `components/tasks/cu_controller` holding the payloads and a
     `CuControllerTask<N>` — a multi-input `CuTask` (follower setpoint +
     measured state in, unified command out), `Reflect` + `Freezable`
     (last command, status, watchdog state round-trip for replay), zero
     hot-path allocation, `Tov` propagated from the state input. Bridges stay
     pure transports: `CuBridge` with `joint_command`/`joint_state` channels,
     polling in `preprocess`, dispatch in `send`/`receive`. Done: the
     playground sim demo runs through the controller unchanged
     (`v0.2.2`).
   - Pose-goal planning using the existing IK and joint-space RRT*, with explicit
     no-IK-solution, invalid-start/goal, capacity, and planning-failure outcomes.
   - Integrate existing Cartesian paths and TCP zones with the same scene and
     request/result contracts. Recheck the reconstructed execution curve for
     joint limits, collisions, and zone containment before accepting a plan.
   - A seeded benchmark harness reporting success rate, geometric path cost,
     planning effort, and elapsed time. Add planners such as PRM/BIT* only where
     measured results justify them.
   Completion requires a local physics-simulator demonstration that reads joint
   state, plans and times a trajectory, commands the simulated robot, and
   observes state feedback and execution status. It also updates obstacles,
   attaches an object, and plans joint, pose-goal, and Cartesian requests
   consistently against the selected scene revision, plus the Validation Plan
   checks.
3. **Real-time servoing.** Harden `CuServoTask<N>` into a production servo
   loop (controller rate limiting and watchdog land earlier, in 0.3):
   singularity/limit behaviour under load, latency/allocation budgets
   enforced by tests. Add execution collision
   monitoring and collision-triggered abort with defined stop/hold reporting;
   distinguish geometric validation before execution from monitoring during it.
4. **Robot bridges.** Extend the joint command/state contract established in
   Step 2 to hardware through a generic `CuBridge`; prove it with `cu-igus-cri` for the igus
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
   version, publish the `cu-repose` umbrella crate that re-exports them, and tag the release `v1.0.0`. Until then, releases are
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

- Highest risk: scene revision and invalidation semantics across anytime planning
  and later execution monitoring. Define ownership and stale-result handling
  before freezing the scene/request/result contracts.
- Undecided: additional robot presets beyond the igus ReBeL (added only when a
  bridge or user needs one); the second bridge target after `cu-igus-cri`
  (decided by hardware access); whether benchmark results ship as a report
  crate or as local demonstrations.
