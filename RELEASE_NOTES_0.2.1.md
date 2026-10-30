# cu-moveit 0.2.1

This pre-1.0 feature release adds simulated robot execution: Copper can now
drive an external MuJoCo simulator through the new `cu-mujoco-sim` bridge.

- New crate `cu-mujoco-sim` (`components/bridges/cu_mujoco_sim`): a `CuBridge`
  speaking line-delimited JSON over TCP to a MuJoCo sidecar process. Tx
  `joint_command` and Rx `joint_state` channels carry
  `CuJointPositionsPayload<6>`; config keys `host`, `port`, and
  `read_timeout_ms` default to `127.0.0.1`, `8765`, and `250`.
- This establishes the joint command/state contract that the hardware bridge
  in roadmap Step 4 will extend.
- The bridge is host-only (`std`); the shared libraries remain `no_std`.
- Verified end to end in a local, git-excluded demonstration: the app reads
  measured joint state from MuJoCo, plans from that state with RRT*, times
  the path, streams follower setpoints back, and confirms the simulated robot
  reaches the goal. Kinematic animation alone does not satisfy this.

Crates.io publishing remains disabled; this release is distributed as a Git tag.
