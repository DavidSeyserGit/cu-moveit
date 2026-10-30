//! Planner-agnostic joint-space motion planning for `copper-rs`.
//!
//! Every planner — sampling-based (RRT*, PRM, ...), optimization-based
//! (CHOMP/TrajOpt style), search-based, or direct — implements one trait,
//! [`CuJointPlanner`], and runs inside one Copper task,
//! [`CuJointPlannerTask`]. The task owns the anytime base/refine contract,
//! publishing, seeding, and replay state, so a planner only has to answer:
//! start a job ([`CuJointPlanner::begin`]), do a bounded quantum of work
//! ([`CuJointPlanner::step`]), and write its best path in place
//! ([`CuJointPlanner::write_best`]).
//!
//! Requests and results are planner-neutral payloads:
//! [`CuJointPlanRequest`] carries the planning scene, exclusions, start and
//! goal; [`CuJointPlanPath`] carries the path, its cost, the scene revision it
//! was planned against, and an explicit [`CuPlanStatus`] — failures such as an
//! invalid start or an exhausted search are published, not silently dropped.
//!
//! Built-in planners:
//! - Sampling: [`rrt_connect::CuJointRrtConnect`] (bidirectional,
//!   feasibility-first) and [`prm::CuJointPrm`] (probabilistic roadmap,
//!   improving).
//! - Search: [`astar::CuJointAStar`] (A* on a joint-space lattice,
//!   resolution-complete).
//! - Optimization: [`optimizer::CuJointPathOptimizer`] (deterministic
//!   gradient, CHOMP-style) and [`stomp::CuJointStomp`] (seeded stochastic
//!   rollouts).
//! - Direct: [`direct::CuJointDirectPlanner`] (straight joint line, one shot).
//!
//! RRT* lives in `cu-joint-rrt-star`. Every planner shares
//! [`validity::CuSceneValidity`] for collision and clearance queries.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod astar;
mod common;
pub mod direct;
pub mod optimizer;
pub mod prm;
pub mod rrt_connect;
pub mod stomp;
pub mod task;
pub mod validity;

pub use astar::CuJointAStar;
pub use astar::CuJointAStarParams;
pub use direct::CuJointDirectPlanner;
pub use optimizer::CuJointOptimizerParams;
pub use optimizer::CuJointPathOptimizer;
pub use prm::CuJointPrm;
pub use prm::CuJointPrmParams;
pub use rrt_connect::CuJointRrtConnect;
pub use rrt_connect::CuJointRrtConnectParams;
pub use stomp::CuJointStomp;
pub use stomp::CuJointStompParams;
pub use task::CuJointPlannerDebugState;
pub use task::CuJointPlannerTask;
pub use validity::CuSceneValidity;
pub use validity::joint_distance;

use bincode::Decode;
use bincode::Encode;
use cu_collision::CuPlanningScene;
use cu_collision::CuSceneExclusions;
use cu_collision::CuSceneRevision;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Fixed capacity of waypoints in a published [`CuJointPlanPath`].
pub const MAX_WAYPOINTS: usize = 32;
/// Fixed capacity of scene obstacles in a [`CuJointPlanRequest`].
pub const MAX_OBSTACLES: usize = 16;
/// Fixed capacity of attached shapes in a [`CuJointPlanRequest`].
pub const MAX_ATTACHED_SHAPES: usize = 8;
/// Fixed capacity of stable-ID exclusions in a [`CuJointPlanRequest`].
pub const MAX_EXCLUSIONS: usize = 16;

/// Joint-space planning request (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlanRequest<const N: usize> {
    pub scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    pub exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub start: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub goal: [CuJointPosition; N],
}

impl<const N: usize> Default for CuJointPlanRequest<N> {
    fn default() -> Self {
        Self {
            scene: CuPlanningScene::new(),
            exclusions: CuSceneExclusions::new(),
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
        }
    }
}

/// Outcome of a planning job, carried by [`CuJointPlanPath`].
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuPlanStatus {
    /// The job is running and has no collision-free path yet.
    #[default]
    Searching,
    /// A collision-free path exists; more work may improve it.
    Improving,
    /// A collision-free path exists and further work cannot improve it.
    Converged,
    /// The planner gave up without a path (search budget or capacity spent,
    /// optimizer stuck in collision, or no direct line).
    Exhausted,
    /// Parameters, scene, attached shapes, or exclusions are invalid.
    InvalidRequest,
    /// The start configuration is out of limits or in collision.
    InvalidStart,
    /// The goal configuration is out of limits or in collision.
    InvalidGoal,
    /// A path exists but does not fit [`MAX_WAYPOINTS`].
    CapacityExceeded,
}

impl CuPlanStatus {
    /// `true` when a collision-free path is available.
    pub const fn has_path(self) -> bool {
        matches!(self, Self::Improving | Self::Converged)
    }

    /// `true` when no further [`CuJointPlanner::step`] can change the outcome.
    pub const fn is_final(self) -> bool {
        !matches!(self, Self::Searching | Self::Improving)
    }
}

/// Planned joint-space path (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlanPath<const N: usize> {
    pub waypoints: [CuJointPositionsPayload<N>; MAX_WAYPOINTS],
    /// Number of valid waypoints in `waypoints[..len]`; `0` unless the status
    /// has a path.
    pub len: u32,
    /// Dimensionless joint-space L2 path length under the planner's metric.
    pub cost: f64,
    /// Planner work spent on the job (samples, optimizer iterations, ...).
    pub effort: u32,
    pub status: CuPlanStatus,
    /// Scene revision the path was planned against.
    pub scene_revision: CuSceneRevision,
    /// Collision exclusions captured for this plan.
    pub exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
}

impl<const N: usize> Default for CuJointPlanPath<N> {
    fn default() -> Self {
        Self {
            waypoints: core::array::from_fn(|_| CuJointPositionsPayload::default()),
            len: 0,
            cost: f64::INFINITY,
            effort: 0,
            status: CuPlanStatus::default(),
            scene_revision: CuSceneRevision::INITIAL,
            exclusions: CuSceneExclusions::new(),
        }
    }
}

impl<const N: usize> CuJointPlanPath<N> {
    /// Valid waypoints.
    pub fn waypoints(&self) -> &[CuJointPositionsPayload<N>] {
        &self.waypoints[..(self.len as usize).min(MAX_WAYPOINTS)]
    }
}

/// How a planner explores the configuration space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuPlannerKind {
    /// Random sampling (RRT*, PRM, ...); results depend on the job seed.
    Sampling,
    /// Deterministic trajectory optimization (CHOMP, TrajOpt, ...).
    Optimization,
    /// Deterministic graph or lattice search (A*, ...).
    Search,
    /// A single deterministic construction (straight line, ...).
    Direct,
}

/// A joint-space motion planner usable by [`CuJointPlannerTask`].
///
/// Jobs are bounded and allocation-free: `begin` resets in place,
/// `step(budget)` does at most `budget` units of work, and `write_best`
/// copies the best path into a caller-owned payload. Deterministic planners
/// ignore the seed; sampling planners must derive all randomness from it so
/// replay reproduces every job.
pub trait CuJointPlanner<const N: usize>: Sized {
    /// Exploration family, for diagnostics and documentation.
    const KIND: CuPlannerKind;

    /// Builds the planner from task RON configuration (robot models,
    /// parameters).
    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self>;

    /// Starts a new job for `request`, discarding the previous one.
    fn begin(&mut self, request: &CuJointPlanRequest<N>, seed: u64) -> CuResult<CuPlanStatus>;

    /// Runs one bounded quantum of at most `budget` work units.
    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus>;

    /// Writes the best path so far into `out` (waypoints, length, cost,
    /// effort, scene revision, exclusions) and returns the job status. `out`
    /// holds no waypoints unless the status has a path.
    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus;

    /// Quality of the best path in `0.0..=1.0` (`0.0` without a path).
    fn quality(&self) -> Quality;
}
