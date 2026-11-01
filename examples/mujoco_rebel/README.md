# MuJoCo ReBeL example

This headless Copper application drives a simulated igus ReBeL to a joint
goal. The whole graph is library components configured in
[`copperconfig.ron`](copperconfig.ron):

```text
mujoco/joint_state ─► goal ─► planner ─► timing ─► follower ─► controller ─► mujoco/joint_command
         └──────────────────────────────────────────────────────────▲
```

| Node | Component | Role |
|---|---|---|
| `goal` | `repose_motion_planning::CuJointGoalTask<6>` | First ready state + configured goal → one plan request |
| `planner` | `repose_joint_rrt_star::CuJointRrtStarTask<6>` | Anytime RRT* in the background, seeded by `cu_rng::CuRngBundle` |
| `timing` | `repose_motion_planning::CuPlanToTrajectoryTask<6, 1024>` | Final path → cubic-timed trajectory within joint limits |
| `follower` | `repose_trajectory::CuFollowTrajectoryTask<6, 1024>` | Samples the trajectory on the message clock |
| `controller` | `repose_controller::CuControllerTask<6>` | Tracking-error watchdog and rate-limited commands |
| `observer` | `observer::GoalObserver` (this crate) | Test oracle: writes `result.json` and exits |

The planner uses the `igus_rebel` chain and self-collision presets; the scene
is empty. The sidecar removes visual and collision
geometry from the simulation and uses position control with gravity
compensation.

## Requirements

- Rust 1.95 or newer and the Copper dependencies in `Cargo.toml`.
- Python 3 with `mujoco` and `numpy` (`python -m pip install mujoco numpy`).
- An expanded igus ReBeL URDF whose movable joints are named `joint1` through
  `joint6`, ordered to match `repose_kinematics::igus_rebel_chain()`. The URDF supplies MuJoCo's
  inertial model; verify that it describes the same robot as the Copper preset.

From this directory, start the sidecar in one terminal:

```bash
python sidecar.py --urdf /path/to/igus_rebel.urdf
```

In another terminal:

```bash
cargo run --manifest-path Cargo.toml
```

The bridge connects to `127.0.0.1:55091`; change both `--port` and the port in
`copperconfig.ron` if needed. The observer writes `result.json` next to the
example and exits nonzero on timeout or controller fault. The runtime also
writes a `sim_demo.copper` log there. Planning failures are published as a
`CuPlanStatus` on the planner output (visible in the log); bridge errors are
reported by the runtime. Stop the sidecar with Ctrl-C after the app exits.

The goal and motion limits are in `copperconfig.ron`. All joint values in
this example are radians. This demonstration is not a hardware interface:
its collision checks are discrete, and it does not monitor collisions during
execution.
