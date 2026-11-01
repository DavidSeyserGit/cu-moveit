# Repose Agent Guidance

**Repose** (this repository, umbrella crate `repose`) is a Copper-idiomatic (`cu29`) manipulation, kinematics, collision checking, and motion planning workspace built in strict alignment with `copper-project/copper-rs`.

## Skills

Before writing or reviewing code, read the applicable `SKILL.md` under `.agents/skills/`:

| Work | Skill |
|---|---|
| Architecture / reading apps | `.agents/skills/copper-arch/SKILL.md` |
| Rust / documentation / console or UI style | `.agents/skills/copper-coding-style/SKILL.md` |
| Core / runtime constraints | `.agents/skills/copper-core-dev/SKILL.md` |
| Public traits, adapters, config policies | `.agents/skills/copper-api-flavor/SKILL.md` |
| Sources, tasks, sinks, bridges | `.agents/skills/copper-component-design/SKILL.md` |
| RON config | `.agents/skills/copper-ron-config/SKILL.md` |
| Build, checks, justfiles, releases | `.agents/skills/copper-workflow/SKILL.md` |

Public API shape (params structs, constructors, units, visibility) follows `docs/api-conventions.md`.
Versioning, breaking-change rules, and releases follow `docs/versioning.md`; record user-visible changes under `Unreleased` in `CHANGELOG.md`.

## Non-Negotiable Copper House Rules

1. **Static over dynamic:** Encode degrees of freedom (`const N: usize`) and frame/chain topology at compile time; zero heap allocations on the real-time path.
2. **In-place `&mut` outputs (`copper-api-flavor`):** Hot-path methods write into caller-provided `&mut` buffers or `&mut Self::Output<'o>`. Never return payloads by value on the hot path.
3. **`std` / `no_std` discipline (`copper-coding-style`):** Shared crates start with `#![cfg_attr(not(feature = "std"), no_std)]`. Never use `std::` outside `#[cfg(feature = "std")]`.
4. **Physical units (`cu29::units`):** Public APIs and payloads use `cu29::units::si::f64::{Angle, AngularVelocity, Length}` rather than bare floats whose unit exists only in comments.
5. **Error handling:** Fallible operations return `CuResult<T>`. Never `unwrap()`, `expect()`, or `panic!` in library or task code.
6. **Payload & Task contracts:**
   - Payloads derive `#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]`.
   - `Tov` stays on the `CuMsg` envelope, never inside payload structs.
   - Tasks derive `Reflect` and implement `Freezable` + `CuTask` (or `CuAnytimeTask`).
7. **Workflow:** Remove generated `Cargo.lock` before a work pass if needed, run `just fmt` after edits, and run `just pr-check` before committing.

## Roadmap

1. `components/libs/repose_kinematics` (`repose-kinematics`): Serial chain kinematics (`CuKinematicChain<N>`), Forward Kinematics, Geometric Jacobians, Damped Least-Squares Inverse Kinematics, `CuForwardKinematicsTask<N>`, and 6-DOF igus ReBeL preset.
2. `components/libs/repose_collision` (`repose-collision`): Fixed-capacity capsule/sphere self-collision and environment collision checking (`no_std`).
3. `components/tasks/cu_servo` (`cu-servo`): Real-time Jacobian velocity/twist servoing (`CuTask`) with joint-limit and singularity damping.
4. `components/tasks/repose_motion_planning` (`repose-motion-planning`): planner-agnostic `CuJointPlanner` contract and generic anytime planner task, with non-sampling planners (direct, path optimizer); `components/tasks/repose_joint_rrt_star` (`repose-joint-rrt-star`) is the sampling-based RRT* implementation with deterministic `CuRng`.
5. `components/bridges/cu_igus_cri` (`cu-igus-cri`): Native `CuBridge` for the igus ReBeL CRI Ethernet protocol (`192.168.3.11:3920`).
