# cu-moveit

Deterministic, zero-allocation robot manipulation, kinematics, collision checking, and motion planning components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs), designed from the ground up to follow Copper's `std`/`no_std`, compile-time static graph, and replay-deterministic architecture.

## Crates

| Crate | Path | Kind | Description |
|---|---|---|---|
| `cu-kinematics` | `components/libs/cu_kinematics` | `lib` / `task` | Const-generic `CuKinematicChain<N>` forward kinematics, space/body geometric Jacobians, DLS inverse kinematics, differential IK with singularity/limit damping, `CuForwardKinematicsTask<N>`, `CuInverseKinematicsTask<N>`, `CuServoTask<N>`, and 6-DOF igus ReBeL model |
| `cu-collision` | `components/libs/cu_collision` | `lib` / `task` | Fixed-capacity sphere/capsule primitives, exact signed-distance queries, `CuRobotCollisionModel<N>`, `CuCollisionWorld<MAX_OBSTACLES>`, `CuCollisionCheckTask<N>`, and igus ReBeL collision geometry |
| `cu-robot-description` | `components/libs/cu_robot_description` | `startup lib` | URDF serial-chain extraction, primitive collision conversion, and SRDF serial groups, collision exclusions, and named states |
| `cu-trajectory` | `components/libs/cu_trajectory` | `lib` / `task` | Fixed-capacity joint trajectories, interpolation, timing profiles, and sampled Cartesian TCP-zone blending |
| `cu-motion-planning` | `components/tasks/cu_motion_planning` | `lib` / `task` | Planner-agnostic contract: `CuJointPlanner` trait, `CuJointPlanRequest<N>` / `CuJointPlanPath<N>` with explicit `CuPlanStatus`, shared scene validity checks, the generic anytime `CuJointPlannerTask<P, N>`, and built-in planners: RRT-Connect and PRM (sampling), A* lattice (search), gradient path optimizer and STOMP (optimization), direct joint line |
| `cu-joint-rrt-star` | `components/tasks/cu_joint_rrt_star` | `task` | Sampling-based RRT* implementing `CuJointPlanner`, with deterministic `CuRng` seeding, informed pruning, rewiring, and path shortcutting; `CuJointRrtStarTask` is the generic planner task running it |
| `cu-controller` | `components/tasks/cu_controller` | `task` | Robot-agnostic joint controller with position/velocity/effort modes, robot status, state watchdog, and command rate limiting |
| `cu-io` | `components/tasks/cu_io` | `task` | Robot-agnostic digital IO payloads and an IO-driven gripper task |

See [ROADMAP.md](ROADMAP.md) for where this is going and what is explicitly out
of scope.

## Consuming (pre-1.0)

Nothing is published to crates.io yet (`publish = false` workspace-wide; the API
is 0.1 and breaking). Depend on the git repository instead:

```toml
cu-kinematics = { git = "https://github.com/DavidSeyserGit/cu-moveit", tag = "v0.1.0" }
```

The workspace is versioned and tagged as a whole (`vX.Y.Z`), never per crate.
At 1.0 the crates will be published to crates.io in lockstep behind a single
`cu-moveit` facade crate; see the publishing step in [ROADMAP.md](ROADMAP.md).

## Quick Start

```bash
just
```

## Robot configuration

Only serial kinematic chains are supported. The joint count `N` is part of the task
and payload types; branched and closed-chain mechanisms are outside the current scope.

Task initialization never selects a robot preset. FK, IK, and servo tasks require
an explicit `chain` configuration containing exactly `N` joint specifications.
An optional `tool_offset` overrides the chain's flange-to-tool transform.

Collision and planner tasks also require an explicit collision model and typed
`safety_margin`. `CuRobotCollisionModel<N, M>` stores up to `M` shapes attached to
`N` movable frames or the base. The default `M = N` preserves one-shape-per-joint
construction; imported robots can have several shapes per link, missing geometry,
and shapes on fixed links. Contacts identify shape indices.

Use `new_with_chain` or `new_with_model` to supply imported models directly.
`CuJointPosition` carries `Revolute(Angle)`, `Prismatic(Length)`, or `Fixed`;
`CuJointVelocity` similarly distinguishes angular and linear velocity. Incorrect
coordinate units return an error. Planner distances and costs are dimensionless,
scaled by `CuJointMetric` (default: one radian and one meter).

The igus ReBeL helpers remain optional presets for examples and applications;
no task initializer uses them automatically.

## URDF and SRDF

```rust,ignore
let robot = CuRobotDescription::<6, 24>::from_urdf_srdf(
    &urdf_xml,
    &srdf_xml,
    "manipulator",
    Length::new::<meter>(0.005),
)?;
let fk = CuForwardKinematicsTask::new_with_chain(robot.chain);
```

Parsing requires the `cu-robot-description` crate's `std` feature and runs at
startup. The resulting kinematic and collision models remain fixed-size and
usable by the `no_std` runtime crates. `N` counts movable joints, not fixed joints;
`M` is a collision-shape capacity. Import errors report insufficient capacity or
a joint-count mismatch. No XML parser or dynamic name lookup runs in a task cycle.

URDF import selects one base-to-tip serial path, folds fixed transforms into
joint origins and the tool offset, and supports revolute, continuous, and
prismatic joints. Other source-tree branches are outside the selected model.
Continuous joints accept unbounded angles and sample a canonical `[-pi, pi]`
interval; planning uses unwrapped coordinates, without shortest-angle wrapping.

Collision spheres are exact. Cylinders become conservative capsules whose
endpoints are at the cylinder's end-face centers; the rounded ends extend beyond
the original cylinder. Meshes and boxes require explicit preprocessing and are
rejected. All collision elements on the selected links, including base and fixed
links, are retained. Links without collision elements contribute no shapes.
Imported models check all distinct rigid-body pairs unless SRDF disables them.

The SRDF subset supports a group containing exactly one `<chain>`,
`<disable_collisions>`, and complete `<group_state>` joint values. State order is
mapped by joint name. Selected-chain mimic/passive joints and composed groups
are rejected. End-effectors, virtual-joint world transforms, and other SRDF
semantics are not imported. Supply expanded URDF XML; Xacro expansion is external.
The format follows the [MoveIt URDF/SRDF guide](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html).

Run the complete mixed revolute/prismatic example:

```bash
cargo run -p cu-robot-description --example serial_robot
```

It loads URDF/SRDF, resolves two named states, computes FK and IK, checks the IK
solution for collisions, and runs the anytime planner. It exits successfully when
all stages pass. Edge checks use discrete angular/linear interpolation; they are
not swept-volume continuous collision detection.

Existing callers must convert angular coordinates with `angle.into()` and match
joint-velocity variants. Joint-limit RON values now use `revolute`, `prismatic`,
`continuous`, or `fixed` variants. Planner `step_size`, `goal_threshold`, and
`rewire_radius` replace their former `_rad` keys and use the configured metric.

## Continuous motion and TCP zones

`cu_trajectory::cartesian_blend::blend_cartesian_path` takes a serial chain,
collision model, joint waypoints, and one typed `Length` zone radius per
waypoint. A zero interior radius is an exact stop; a positive radius replaces
that TCP corner with a sampled quadratic fly-by path. The first and last radii
must be zero. Tool orientation follows shortest-arc interpolation, and IK is
seeded from the previous solution. Input/output capacity is fixed at compile
time; invalid geometry, IK failure, collision, or capacity exhaustion clears the
output and returns a status.

Geometry and timing are separate: the zone is a property of each waypoint,
the profile a property of the motion. `path_timing::parameterize_cartesian_path`
times the path with any `CuTrajectoryProfile` from `CuPathTimingConfig::profile`.
Every profile flies through every positive zone and stops exactly at zero
zones. The knots are joined by a C² natural cubic spline over a speed-weighted
joint-space arc length; exact stops split the path into rest-to-rest pieces,
each zone is crossed at a constant corner speed bounded by its curvature, and
the moves between stops and corners follow the profile from one boundary speed
to the next. Joint velocity and acceleration stay within the configured limits;
S-curve joint jerk is checked on the quintic reconstruction, including curvature
contributions. Every profile is certified on the reconstruction its follower
executes — quintic Hermite for the S-curve, cubic Hermite for the others — and
uniform time stretching brings any excess within the limits. The geometric spline is
rechecked for joint limits, collisions, and zone containment, and a path that
reverses inside a zone is rejected with `NoContinuousMotion` naming the
waypoint. Read the result with
`CuTrajectoryPayload::sample_quintic` for S-curve motion, or configure a
`CuFollowTrajectoryTask` with `config: {"interpolation": "quintic"}`. Other
profiles use `sample_cubic` and the `cubic` follower mode. Match the follower's `MAX_POINTS`
type parameter to the generated payload capacity. The default follower
interpolation remains linear. Interpolation mode is included in newly frozen follower state;
earlier frozen snapshots need migration.

Timing is conservative and automatic. Finite joint jerk bounds apply to the
S-curve profile with quintic execution. Collision and zone checks apply to the geometric spline; the reconstructed
execution curve still needs a separate collision and zone recheck. Checks are discrete and depend on their configured
resolution; they are not swept-volume collision guarantees. The zone radius is a
TCP-position tolerance, not a separate orientation tolerance or an exact
reproduction of a vendor controller's zone algorithm.

## Motion planners

All planners implement `cu_motion_planning::CuJointPlanner` and run in one
Copper task, `CuJointPlannerTask<P, N>`: `begin` starts a job from a
`CuJointPlanRequest<N>`, `step(budget)` does a bounded quantum of work, and
`write_best` fills a `CuJointPlanPath<N>` in place. The task owns the anytime
contract (base and refine budgets, monotone publishing, seeding, replay
state). Failures are published, not dropped: the path then carries no
waypoints and its `status` names the reason (`invalid_start`,
`invalid_goal`, `invalid_request`, `exhausted`, `capacity_exceeded`).

| Planner | Kind | Behaviour |
|---|---|---|
| `CuJointRrtStar` (`cu-joint-rrt-star`) | sampling | Seeded RRT* with rewiring and shortcutting; keeps shortening its path. |
| `CuJointRrtConnect` | sampling | Bidirectional trees; fastest to a first path, converges on connection without optimizing. |
| `CuJointPrm` | sampling | Probabilistic roadmap (up to 1024 nodes) plus Dijkstra; improves as the roadmap densifies. Rebuilt per job. |
| `CuJointAStar` | search | Deterministic A* on a joint-space lattice (`resolution`); complete at that resolution within its node capacity. |
| `CuJointPathOptimizer` | optimization | Deterministic CHOMP-style gradient descent on smoothness and clearance; improves each quantum. |
| `CuJointStomp` | optimization | Seeded stochastic rollouts weighted by cost; no gradients, copes with flat or discontinuous costs. |
| `CuJointDirectPlanner` | direct | The straight joint line or a definite `exhausted`; a cheap first attempt. |

The optimizers are local: they can stay stuck behind large obstacles where
sampling or search planners still succeed.

Sampling planners derive all randomness from the job seed; deterministic
planners ignore it. New planners only implement the trait and share
`CuSceneValidity` for collision and clearance queries.

## Planning scene snapshots

`CuPlanningScene<O, A>` stores fixed-capacity obstacle and attached-shape
geometry with stable object IDs and a scene revision. The collision-check task
accepts joint positions, a scene snapshot, and stable-ID collision exclusions.
Every planner captures the scene and resolved exclusions once per planning job (`CuSceneValidity`), checks
obstacles and attached shapes throughout planning, and returns `scene_revision` and the captured exclusion set
with the path. Consumers must compare both with their current inputs before
accepting a plan. Refinement uses the captured snapshot until the next
base call starts a new job.

Attached shapes use link-local geometry and zero-based FK frame indices.
`CuSceneExclusions` permits explicit attached-object contact with a robot shape,
another attached object, or an obstacle. Scene objects are referenced by stable
IDs; robot shapes use their model construction indices. Invalid exclusion
references reject the query or planning job. Scene and exclusions must be published consistently.

Scene decoding validates identity, counts, and geometry. Mutations fail before
changing state if the revision cannot advance. Segment collision checks remain
discrete and depend on the configured joint resolution.
