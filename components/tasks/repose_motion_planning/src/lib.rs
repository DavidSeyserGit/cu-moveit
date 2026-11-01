//! Planner-agnostic joint-space motion planning for `copper-rs`.
//!
//! Every planner — sampling-based (RRT*, PRM, ...), optimization-based
//! (CHOMP/TrajOpt style), search-based, or direct — implements one trait,
//! [`CuJointPlanner`], and runs inside one Copper task,
//! [`CuJointPlannerTask`]. The task owns the anytime base/refine contract,
//! publishing, seeding, and replay state, so a planner only has to answer:
//! build itself from validated parameters ([`CuJointPlanner::new`]), start a
//! job ([`CuJointPlanner::begin`]), do a bounded quantum of work
//! ([`CuJointPlanner::step`]), and write its best path in place
//! ([`CuJointPlanner::write_best`]).
//!
//! Requests and results are planner-neutral payloads:
//! [`CuJointPlanRequest`] carries the planning scene, exclusions, start and
//! goal; [`CuJointPlanPath`] carries the path, its cost, the scene revision it
//! was planned against, and an explicit [`CuPlanStatus`] — failures such as an
//! invalid start or an exhausted search are published, not silently dropped.
//!
//! Built-in planners, each with a `Cu<Planner>Params` struct and a task alias:
//! - Sampling: [`CuJointRrtConnect`] ([`CuJointRrtConnectTask`];
//!   bidirectional, feasibility-first) and [`CuJointPrm`] ([`CuJointPrmTask`];
//!   probabilistic roadmap, improving).
//! - Search: [`CuJointAStar`] ([`CuJointAStarTask`]; A* on a joint-space
//!   lattice, resolution-complete).
//! - Optimization: [`CuJointPathOptimizer`] ([`CuJointPathOptimizerTask`];
//!   deterministic gradient, CHOMP-style) and [`CuJointStomp`]
//!   ([`CuJointStompTask`]; seeded stochastic rollouts).
//! - Direct: [`CuJointDirect`] ([`CuJointDirectTask`]; straight joint line,
//!   one shot).
//!
//! Every planner task has the same node config, [`CuJointPlannerParams`];
//! switching planners means changing the task type and the `params` block:
//!
//! ```ron
//! (
//!     id: "planner",
//!     type: "repose_motion_planning::CuJointPrmTask<6>",
//!     config: {
//!         "chain": "igus_rebel",
//!         "collision_model": "igus_rebel",
//!         "params": {"connection_radius": 1.5, "max_nodes": 512},
//!         "refine_budget": 16, // optional; default per planner
//!     },
//! )
//! ```
//!
//! A plan-and-execute graph needs no application code:
//!
//! ```text
//! joint state ─► CuJointGoalTask ─► CuJoint<Planner>Task ─► CuPlanToTrajectoryTask ─► follower
//! ```
//!
//! [`CuJointGoalTask`] turns measured state and a configured goal into a
//! request, and [`CuPlanToTrajectoryTask`] times each final path for
//! `repose_trajectory::CuFollowTrajectoryTask`.
//!
//! RRT* lives in `repose-joint-rrt-star`. Every planner shares
//! [`CuSceneValidity`] for collision and clearance queries.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

/// Implements [`TypePath`] for a const-generic planner `$ty<N, M>` with the
/// fixed path `$path` (const arguments are not part of the path).
#[macro_export]
macro_rules! impl_planner_type_path {
    ($ty:ident, $path:literal) => {
        impl<const N: usize, const M: usize> ::cu29::prelude::TypePath for $ty<N, M> {
            fn type_path() -> &'static str {
                $path
            }
            fn short_type_path() -> &'static str {
                ::core::stringify!($ty)
            }
            fn type_ident() -> Option<&'static str> {
                Some(::core::stringify!($ty))
            }
            fn crate_name() -> Option<&'static str> {
                $path.split("::").next()
            }
            fn module_path() -> Option<&'static str> {
                $path.rsplit_once("::").map(|(module, _)| module)
            }
        }
    };
}

mod astar;
mod common;
mod direct;
mod goal_task;
mod optimizer;
mod prm;
mod rrt_connect;
mod stomp;
mod task;
mod trajectory_task;
mod validity;

pub use astar::ASTAR_MAX_NODES;
pub use astar::CuJointAStar;
pub use astar::CuJointAStarParams;
pub use astar::CuJointAStarTask;
pub use direct::CuJointDirect;
pub use direct::CuJointDirectParams;
pub use direct::CuJointDirectTask;
pub use goal_task::CuJointGoalParams;
pub use goal_task::CuJointGoalTask;
pub use optimizer::CuJointPathOptimizer;
pub use optimizer::CuJointPathOptimizerParams;
pub use optimizer::CuJointPathOptimizerTask;
pub use prm::CuJointPrm;
pub use prm::CuJointPrmParams;
pub use prm::CuJointPrmTask;
pub use prm::PRM_MAX_NEIGHBORS;
pub use prm::PRM_MAX_NODES;
pub use rrt_connect::CuJointRrtConnect;
pub use rrt_connect::CuJointRrtConnectParams;
pub use rrt_connect::CuJointRrtConnectTask;
pub use rrt_connect::RRT_CONNECT_MAX_NODES;
pub use stomp::CuJointStomp;
pub use stomp::CuJointStompParams;
pub use stomp::CuJointStompTask;
pub use stomp::STOMP_MAX_ROLLOUTS;
pub use task::CuJointPlannerDebugState;
pub use task::CuJointPlannerParams;
pub use task::CuJointPlannerTask;
pub use task::planner_resources;
pub use trajectory_task::CuPlanToTrajectoryTask;
pub use validity::CuSceneValidity;
pub use validity::joint_distance;

use bincode::Decode;
use bincode::Encode;
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use repose_collision::CuPlanningScene;
use repose_collision::CuRobotCollisionModel;
use repose_collision::CuSceneExclusions;
use repose_collision::CuSceneRevision;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointPositionsPayload;
use repose_kinematics::CuKinematicChain;
use serde::Deserialize;
use serde::Serialize;
use serde::de::DeserializeOwned;

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
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointPlanRequest<const N: usize> {
    pub scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    pub exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
    #[serde(with = "repose_kinematics::serde_arrays::position_array")]
    pub start: [CuJointPosition; N],
    #[serde(with = "repose_kinematics::serde_arrays::position_array")]
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
#[reflect(no_field_bounds, from_reflect = false)]
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
/// A planner is built once from robot models and its validated
/// [`Params`](Self::Params) with [`new`](Self::new); invalid parameters fail
/// there, not at the first job. Jobs are bounded and allocation-free: `begin`
/// resets in place, `step(budget)` does at most `budget` units of work, and
/// `write_best` copies the best path into a caller-owned payload.
/// Deterministic planners ignore the seed; sampling planners must derive all
/// randomness from it so replay reproduces every job.
///
/// `M` is the number of collision-model shapes (default `N`).
///
/// Planners implement [`TypePath`] so the generic planner task stays
/// nameable and inspectable under Copper's runtime reflection; use
/// [`impl_planner_type_path!`] for a const-generic planner.
pub trait CuJointPlanner<const N: usize, const M: usize = N>: Sized + TypePath {
    /// Exploration family, for diagnostics and documentation.
    const KIND: CuPlannerKind;

    /// Default `(base_budget, refine_budget)` work units of
    /// [`CuJointPlannerTask`] when the node config leaves them out.
    const DEFAULT_BUDGETS: (u32, u32) = (64, 32);

    /// The planner's parameter struct: the `params` block of the planner
    /// task's node config (`serde(default, deny_unknown_fields)`).
    type Params: Clone + Default + DeserializeOwned;

    /// Builds the planner for explicit robot models, validating `params`.
    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: Self::Params,
    ) -> CuResult<Self>;

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
