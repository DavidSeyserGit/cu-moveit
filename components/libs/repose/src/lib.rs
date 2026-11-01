//! **Repose**: deterministic, real-time robot arm manipulation for
//! [`copper-rs`](https://github.com/copper-project/copper-rs).
//!
//! This umbrella crate re-exports every Repose component under one
//! dependency, so an application needs a single line in `Cargo.toml`:
//!
//! ```toml
//! repose = { git = "https://github.com/ReposeRobotics/repose", tag = "v0.7.0" }
//! ```
//!
//! | Module | Crate | Provides |
//! |---|---|---|
//! | [`kinematics`] | `repose-kinematics` | Serial chains, FK, Jacobians, IK, servoing, joint-state payloads |
//! | [`collision`] | `repose-collision` | Capsule/sphere robot models, planning scenes, collision checks |
//! | [`robot_description`] | `repose-robot-description` | URDF/SRDF import into fixed-size models (`std`) |
//! | [`trajectory`] | `repose-trajectory` | Trajectory timing, interpolation, following |
//! | [`motion_planning`] | `repose-motion-planning` | Planner contract, anytime planner task, built-in planners |
//! | [`rrt_star`] | `repose-joint-rrt-star` | Joint-space RRT* planner |
//! | [`controller`] | `repose-controller` | Joint controller with watchdog and rate limiting |
//! | [`io`] | `repose-io` | Digital IO payloads and an IO-driven gripper |
//! | `mujoco_sim` | `repose-mujoco-sim` | MuJoCo simulator bridge (feature `mujoco-sim`) |
//!
//! Copper RON configs keep naming the component crates directly
//! (`type: "repose_trajectory::CuFollowTrajectoryTask<6, 1024>"`); depend on
//! them individually if you only need part of the stack.

#![cfg_attr(not(feature = "std"), no_std)]

pub use repose_collision as collision;
pub use repose_controller as controller;
pub use repose_io as io;
pub use repose_joint_rrt_star as rrt_star;
pub use repose_kinematics as kinematics;
pub use repose_motion_planning as motion_planning;
#[cfg(feature = "mujoco-sim")]
pub use repose_mujoco_sim as mujoco_sim;
pub use repose_robot_description as robot_description;
pub use repose_trajectory as trajectory;
