//! Direct joint-line planner: non-sampling, deterministic, one shot.
//!
//! The straight joint-space line from start to goal is either collision-free
//! against the captured scene (`Converged`) or it is not (`Exhausted`). It is
//! the cheapest possible planner and a useful first attempt before a sampling
//! or optimizing planner.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::common::check_endpoints;
use crate::common::write_header;
use crate::validity::CuSceneValidity;
use crate::validity::joint_distance;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;

/// Straight joint-space line planner.
///
/// Config keys: `chain`, `collision_model` (robot models), optional
/// `collision_check_step` (`CuJointResolution`) and `metric`
/// (`CuJointMetric`).
#[derive(Clone)]
pub struct CuJointDirectPlanner<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    metric: CuJointMetric,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    status: CuPlanStatus,
    effort: u32,
}

impl<const N: usize, const M: usize> CuJointDirectPlanner<N, M> {
    /// Creates the planner for explicit robot models.
    pub fn new(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        resolution: CuJointResolution,
        metric: CuJointMetric,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, resolution),
            metric,
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
            status: CuPlanStatus::InvalidRequest,
            effort: 0,
        }
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N> for CuJointDirectPlanner<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Direct;

    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let chain = CuKinematicChain::from_config(config)?;
        let model = CuRobotCollisionModel::from_config(config)?;
        let mut resolution = CuJointResolution::default();
        let mut metric = CuJointMetric::default();
        if let Some(cfg) = config {
            if let Some(v) = cfg.get_value::<CuJointResolution>("collision_check_step")? {
                resolution = v;
            }
            if let Some(v) = cfg.get_value::<CuJointMetric>("metric")? {
                metric = v;
            }
        }
        Ok(Self::new(chain, model, resolution, metric))
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
