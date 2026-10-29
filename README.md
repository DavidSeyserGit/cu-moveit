# cu-moveit

Deterministic, zero-allocation robot manipulation, kinematics, collision checking, and motion planning components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs), designed from the ground up to follow Copper's `std`/`no_std`, compile-time static graph, and replay-deterministic architecture.

## Crates

| Crate | Path | Kind | Description |
|---|---|---|---|
| `cu-kinematics` | `components/libs/cu_kinematics` | `lib` / `task` | Const-generic `CuKinematicChain<N>` forward kinematics, space/body geometric Jacobians, DLS inverse kinematics, differential IK with singularity/limit damping, `CuForwardKinematicsTask<N>`, `CuInverseKinematicsTask<N>`, `CuServoTask<N>`, and 6-DOF igus ReBeL model |
| `cu-collision` | `components/libs/cu_collision` | `lib` / `task` | Fixed-capacity sphere/capsule primitives, exact signed-distance queries, `CuRobotCollisionModel<N>`, `CuCollisionWorld<MAX_OBSTACLES>`, `CuCollisionCheckTask<N>`, and igus ReBeL collision geometry |
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

Collision and planner tasks also require `collision_model` with exactly `N`
`link_shapes` (one local sphere or capsule per serial link) and a typed
`safety_margin`. Non-adjacent link pairs are checked by default. Missing models or
mismatched array lengths fail initialization. When creating tasks directly in Rust,
use `new_with_chain` or `new_with_model` to supply the models explicitly.

The igus ReBeL helpers remain optional presets for examples and applications;
no task initializer uses them automatically.
