# cu-moveit

Deterministic, zero-allocation robot manipulation, kinematics, and motion planning components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs), designed from the ground up to follow Copper's `std`/`no_std`, compile-time static graph, and replay-deterministic architecture.

## Crates

| Crate | Path | Kind | Description |
|---|---|---|---|
| `cu-kinematics` | `components/libs/cu_kinematics` | `lib` / `task` | Const-generic `CuKinematicChain<N>` forward kinematics, space/body geometric Jacobians, damped least-squares inverse kinematics, `CuForwardKinematicsTask<N>`, and 6-DOF igus ReBeL model |

## Quick Start

```bash
just
```
