# cu-moveit

Deterministic, zero-allocation robot manipulation, kinematics, collision checking, and motion planning components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs), designed from the ground up to follow Copper's `std`/`no_std`, compile-time static graph, and replay-deterministic architecture.

## Crates

| Crate | Path | Kind | Description |
|---|---|---|---|
| `cu-kinematics` | `components/libs/cu_kinematics` | `lib` / `task` | Const-generic `CuKinematicChain<N>` forward kinematics, space/body geometric Jacobians, DLS inverse kinematics, differential IK with singularity/limit damping, `CuForwardKinematicsTask<N>`, `CuInverseKinematicsTask<N>`, `CuServoTask<N>`, and 6-DOF igus ReBeL model |
| `cu-collision` | `components/libs/cu_collision` | `lib` / `task` | Fixed-capacity sphere/capsule primitives, exact signed-distance queries, `CuRobotCollisionModel<N>`, `CuCollisionWorld<MAX_OBSTACLES>`, `CuCollisionCheckTask<N>`, and igus ReBeL collision geometry |
| `cu-robot-description` | `components/libs/cu_robot_description` | `startup lib` | URDF serial-chain extraction, primitive collision conversion, and SRDF serial groups, collision exclusions, and named states |
| `cu-joint-rrt-star` | `components/tasks/cu_joint_rrt_star` | `task` | Joint-space $N$-DOF RRT* anytime motion planner (`CuAnytimeTask`) with deterministic `CuRng` seeding, informed pruning, rewiring, and path shortcutting |

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
