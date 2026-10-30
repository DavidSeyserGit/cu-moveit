# cu-moveit 0.2.2

Unified robot-agnostic controller, on top of the MuJoCo sim bridge from 0.2.1.

## Added

- New crate `cu-controller` (`components/tasks/cu_controller`):
  - `CuJointStatePayload<N>` — measured joint positions + velocities; the
    only state payload a bridge has to produce.
  - `CuJointCommandPayload<N>` — unified joint command (positions,
    velocities, `CuControllerStatus`, max tracking error); the only command
    payload a bridge has to carry.
  - `CuControllerTask<N>` — multi-input `CuTask` between the trajectory
    follower and a bridge: follower setpoint + measured state in, unified
    command out. Idle holds the measured state; finished setpoints hold;
    tracking error above `tracking_error_limit_rad` (default 0.5 rad) faults
    and holds the measured state instead of chasing the setpoint. Last
    command/status round-trip through `Freezable` for deterministic replay.

## Changed

- `cu-mujoco-sim` is the first consumer of the unified contract: its
  `joint_command` channel carries `CuJointCommandPayload<6>` and its
  `joint_state` channel emits `CuJointStatePayload<6>` (now including
  measured velocities). Swapping the sim for a hardware bridge is a RON
  graph change only.
- The local (git-excluded) playground sim demo now runs
  planner → follower → controller → MuJoCo → state feedback, unchanged
  oracle: `{"success":true,"final_error_rad":<0.02,"cycles":56}`.

Workspace version 0.2.2, one shared version as usual; consume via git
dependency on the `v0.2.2` tag.
