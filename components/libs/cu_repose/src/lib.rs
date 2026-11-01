//! **Repose**: deterministic, real-time robot arm manipulation for
//! [`copper-rs`](https://github.com/copper-project/copper-rs).
//!
//! This umbrella crate re-exports every Repose component under one
//! dependency, so an application needs a single line in `Cargo.toml`:
//!
//! ```toml
//! cu-repose = { git = "https://github.com/DavidSeyserGit/cu-moveit", tag = "v0.7.0" }
//! ```
//!
//! | Module | Crate | Provides |
//! |---|---|---|
//! | [`kinematics`] | `cu-kinematics` | Serial chains, FK, Jacobians, IK, servoing, joint-state payloads |
//! | [`collision`] | `cu-collision` | Capsule/sphere robot models, planning scenes, collision checks |
//! | [`robot_description`] | `cu-robot-description` | URDF/SRDF import into fixed-size models (`std`) |
//! | [`trajectory`] | `cu-trajectory` | Trajectory timing, interpolation, following |
//! | [`motion_planning`] | `cu-motion-planning` | Planner contract, anytime planner task, built-in planners |
//! | [`rrt_star`] | `cu-joint-rrt-star` | Joint-space RRT* planner |
//! | [`controller`] | `cu-controller` | Joint controller with watchdog and rate limiting |
//! | [`io`] | `cu-io` | Digital IO payloads and an IO-driven gripper |
//! | `mujoco_sim` | `cu-mujoco-sim` | MuJoCo simulator bridge (feature `mujoco-sim`) |
//!
//! Copper RON configs keep naming the component crates directly
//! (`type: "cu_trajectory::CuFollowTrajectoryTask<6, 1024>"`); depend on
//! them individually if you only need part of the stack.

#![cfg_attr(not(feature = "std"), no_std)]

pub use cu_collision as collision;
pub use cu_controller as controller;
pub use cu_io as io;
pub use cu_joint_rrt_star as rrt_star;
pub use cu_kinematics as kinematics;
pub use cu_motion_planning as motion_planning;
#[cfg(feature = "mujoco-sim")]
pub use cu_mujoco_sim as mujoco_sim;
pub use cu_robot_description as robot_description;
pub use cu_trajectory as trajectory;
