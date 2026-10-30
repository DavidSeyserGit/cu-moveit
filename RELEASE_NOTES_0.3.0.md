# cu-moveit 0.3.0

Controller-interface completion: robot status, watchdog, rate limiting,
and robot-agnostic digital IO with an IO-driven gripper.

## Added

- `cu-io` (new crate):
  - `CuDigitalInputPayload<N>` (input levels + `CuIoStatus`) and
    `CuDigitalOutputPayload<N>` (output levels + drive mask) are the only IO
    types a bridge carries on its `digital_in` / `digital_out` channels.
    Unmasked outputs keep their level, so several IO users can share one
    output channel.
  - `CuGripperTask<IN, OUT>` treats a gripper as IO: open/close commands map
    onto RON-configured output bits (`close_output`, optional
    `open_output`), optional sensor inputs (`closed_input`, `open_input`)
    report the state, and `settle_ns` / `motion_timeout_ns` bound the
    motion. States: unknown, opening, open, closing, closed, fault. Outputs
    stay undriven until the first command and hold on fault.
- `cu-controller`:
  - `CuJointStatePayload<N>` carries the bridge-reported `CuRobotStatus`
    (not ready, ready, fault code, emergency stop); the controller faults
    instead of commanding a robot that is not ready.
  - Opt-in state watchdog (`state_timeout_ns`): missing, future, or expired
    state timestamps fault and hold.
  - Opt-in command rate limiting (`max_joint_speed_si`,
    `max_joint_acceleration_si`, position mode only) while running and
    holding; holding is reported once the limited command has arrived.
- `cu-trajectory`: Cartesian path timing with per-waypoint TCP zones for
  every profile, C² spline geometry, quintic execution
  (`sample_quintic`, `CuTrajectoryInterpolation::Quintic`), and S-curve
  joint jerk bounds verified on the executed motion.

## Changed

- `cu-mujoco-sim` stamps received state with the context time and reports
  `Ready` once state is decoded.

Workspace version 0.3.0, one shared version as usual; consume via git
dependency on the `v0.3.0` tag.
