# cu-kinematics

Deterministic, zero-allocation serial-chain robot kinematics library and `CuTask` components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs).

## Features

- **`no_std` compatible:** Compiles and runs identically on Linux (`std`) and bare-metal targets (`--no-default-features`).
- **Compile-time fixed DOF (`CuKinematicChain<N>`):** Zero heap allocations on the real-time path; all outputs are written in place via `&mut` references.
- **Unit-typed API (`cu29::units`):** Joint positions distinguish `Angle` and `Length`; joint velocities distinguish `AngularVelocity` and `Velocity`. Bounds carry matching units, and solvers reject mismatched coordinates.
- **Forward Kinematics & Space Jacobian:** Computes end-effector `Transform3D<f64>`, intermediate link frames, and $6 \times N$ geometric space Jacobians in place.
- **Damped Least-Squares Inverse Kinematics:** Bounded, allocation-free iterative solver (`inverse_kinematics_dls`) with joint-limit clamping and explicit `CuIkStatus` outcomes.
- **Built-in Robot Models:** Includes the 6-DOF **igus ReBeL** cobot (`igus_rebel_chain()`, or `"chain": "igus_rebel"` in RON).

URDF and SRDF startup import is provided by the sibling `cu-robot-description` crate. Imported fixed joints are folded into transforms and do not consume movable joint slots.
