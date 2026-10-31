//! Direct joint-line planner: non-sampling, deterministic, one shot.
//!
//! The straight joint-space line from start to goal is either collision-free
//! against the captured scene (`Converged`) or it is not (`Exhausted`). It is
//! the cheapest possible planner and a useful first attempt before a sampling
//! or optimizing planner.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuJointPlannerTask;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::common::check_endpoints;
use crate::common::write_header;
use crate::validity::CuSceneValidity;
use crate::validity::joint_distance;
use bincode::Decode;
use bincode::Encode;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Parameters of [`CuJointDirect`].
#[derive(
    Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointDirectParams {
    /// Resolution of the segment collision check.
    pub collision_check_step: CuJointResolution,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl CuJointDirectParams {
    /// Checks the metric and collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()
    }
}

/// Straight joint-space line planner.
///
/// Built with [`CuJointPlanner::new`] from robot models and
/// [`CuJointDirectParams`]; runs in [`CuJointDirectTask`].
#[derive(Clone)]
pub struct CuJointDirect<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    metric: CuJointMetric,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    status: CuPlanStatus,
    effort: u32,
}

/// [`CuJointPlannerTask`] running [`CuJointDirect`].
pub type CuJointDirectTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointDirect<N, M>, N, M>;

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointDirect<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Direct;

    type Params = CuJointDirectParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointDirectParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            validity: CuSceneValidity::new(chain, collision_model, params.collision_check_step),
            metric: params.metric,
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
            status: CuPlanStatus::InvalidRequest,
            effort: 0,
        })
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, _seed: u64) -> CuResult<CuPlanStatus> {
        self.start = request.start;
        self.goal = request.goal;
        self.effort = 0;
        self.status = check_endpoints(&mut self.validity, request, self.metric);
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        if self.status == CuPlanStatus::Searching && budget > 0 {
            self.effort = 1;
            self.status = if self.validity.is_segment_free(&self.start, &self.goal) {
                CuPlanStatus::Converged
            } else {
                CuPlanStatus::Exhausted
            };
        }
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        write_header(out, &self.validity, self.status, self.effort);
        if self.status.has_path() {
            out.waypoints[0] = CuJointPositionsPayload {
                positions: self.start,
            };
            out.waypoints[1] = CuJointPositionsPayload {
                positions: self.goal,
            };
            out.len = 2;
            out.cost = joint_distance(&self.start, &self.goal, self.metric);
        }
        self.status
    }

    fn quality(&self) -> Quality {
        quality_from_f32(if self.status.has_path() { 1.0 } else { 0.0 })
    }
}
