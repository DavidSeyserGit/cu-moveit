# cu-moveit 0.3.0

Velocity and effort control for the unified robot-agnostic controller.

## Added

- `cu-controller` extends the unified command contract:
  - `CuControllerMode` — `Position` (default), `Velocity`, and `Effort`
    modes on `CuJointCommandPayload<N>`.
  - `CuJointEffort` — typed torque/force per joint kind in SI units,
    carried by the command payload's effort fields.
  - `CuControllerTask<N>` computes its own PD effort in effort mode;
    bridges apply the command as raw torque/force. Configure `mode`,
    `effort_kp`, and `effort_kv` through RON, or change them at runtime
    through `set_mode` and `set_effort_gains`. Faults output zero effort.
    The crate has 9 unit tests.

## Changed

- `cu-mujoco-sim` sends `{"type":"command","q","qd","effort","mode"}`
  per cycle over the bridge wire.
- Position mode now carries command velocities for bridge velocity
  feedforward.
- The local (git-excluded) playground MuJoCo sidecar applies position
  (with velocity feedforward), velocity, and effort torque modes. The
  Studio adds a Control mode selector; the exec server exposes
  `POST /controller`.

## Still open from the 0.3 list

Gripper and digital IO, robot-side status/fault reporting, and controller
rate limiting + watchdog remain open.

Workspace version 0.3.0, one shared version as usual; consume via git
dependency on the `v0.3.0` tag.
