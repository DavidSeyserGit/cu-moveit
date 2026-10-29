# cu-kinematics

Deterministic, zero-allocation serial-chain robot kinematics library and `CuTask` components for [`copper-rs` (`cu29`)](https://github.com/copper-project/copper-rs).

## Features

- **`no_std` compatible:** Compiles and runs identically on Linux (`std`) and bare-metal targets (`--no-default-features`).
- **Compile-time fixed DOF (`CuKinematicChain<N>`):** Zero heap allocations on the real-time path; all outputs are written in place via `&mut` references.
- **Unit-typed API (`cu29::units`):** Joint angles (`Angle`), velocities (`AngularVelocity`), and link offsets (`Length`) carry SI physical units.
- **Forward Kinematics & Space Jacobian:** Computes end-effector `Transform3D<f64>`, intermediate link frames, and $6 \times N$ geometric space Jacobians in place.
- **Damped Least-Squares Inverse Kinematics:** Bounded, allocation-free iterative solver (`inverse_kinematics_dls`) with joint-limit clamping and explicit `CuIkStatus` outcomes.
- **Built-in Robot Models:** Includes the 6-DOF **igus ReBeL** cobot (`robots::igus_rebel_chain`).
