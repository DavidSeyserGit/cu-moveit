//! Helpers shared by the built-in planners.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuPlanStatus;
use crate::MAX_WAYPOINTS;
use crate::validity::CuSceneValidity;
use crate::validity::joint_distance;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuKinematicChain;
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;

/// Validates the scene and endpoints shared by every planner's `begin`.
pub(crate) fn check_endpoints<const N: usize, const M: usize>(
    validity: &mut CuSceneValidity<N, M>,
    request: &CuJointPlanRequest<N>,
    metric: CuJointMetric,
) -> CuPlanStatus {
    if metric.validate().is_err() || !validity.capture(&request.scene, &request.exclusions) {
        CuPlanStatus::InvalidRequest
    } else if !validity.is_configuration_free(&request.start) {
        CuPlanStatus::InvalidStart
    } else if !validity.is_configuration_free(&request.goal) {
        CuPlanStatus::InvalidGoal
    } else {
        CuPlanStatus::Searching
    }
}

/// Fills the job header of `out` and clears its waypoints.
pub(crate) fn write_header<const N: usize, const M: usize>(
    out: &mut CuJointPlanPath<N>,
    validity: &CuSceneValidity<N, M>,
    status: CuPlanStatus,
    effort: u32,
) {
    out.len = 0;
    out.cost = f64::INFINITY;
    out.effort = effort;
    out.status = status;
    out.scene_revision = validity.scene_revision();
    out.exclusions = validity.exclusions().clone();
}

/// Uniform sample inside the joint limits.
pub(crate) fn sample_uniform<const N: usize>(
    chain: &CuKinematicChain<N>,
    rng: &mut CuRng,
) -> [CuJointPosition; N] {
    core::array::from_fn(|j| {
        let limits = chain.joints()[j].limits;
        let lo = limits.lower().si_value();
        let hi = limits.upper().si_value();
        let u: f64 = rng.random();
        limits.lower().with_si_value(lo + u * (hi - lo))
    })
}

/// Moves from `from` toward `to` by at most `max_step` (normalized metric).
pub(crate) fn steer<const N: usize>(
    from: &[CuJointPosition; N],
    to: &[CuJointPosition; N],
    max_step: f64,
    metric: CuJointMetric,
) -> [CuJointPosition; N] {
    let dist = joint_distance(from, to, metric);
    if dist <= max_step || dist <= 1e-12 {
        return *to;
    }
    let scale = max_step / dist;
    core::array::from_fn(|j| {
        let a = from[j].si_value();
        from[j].with_si_value(a + scale * (to[j].si_value() - a))
    })
}

/// Length of a configuration sequence under `metric`.
pub(crate) fn path_length<const N: usize>(
    path: &[[CuJointPosition; N]],
    metric: CuJointMetric,
) -> f64 {
    path.windows(2)
        .map(|w| joint_distance(&w[0], &w[1], metric))
        .sum()
}

/// Greedy line-of-sight shortcut of `raw` (start to goal) into `out`: from
/// each anchor jump to the farthest later configuration with a free segment.
/// Sets `len` and `cost`; returns `false` when the result still exceeds
/// [`MAX_WAYPOINTS`].
pub(crate) fn write_shortcut<const N: usize, const M: usize>(
    validity: &CuSceneValidity<N, M>,
    raw: &[[CuJointPosition; N]],
    metric: CuJointMetric,
    out: &mut CuJointPlanPath<N>,
) -> bool {
    if raw.is_empty() {
        return false;
    }
    let mut len = 0usize;
    let mut cost = 0.0;
    let mut anchor = 0usize;
    out.waypoints[0] = CuJointPositionsPayload { positions: raw[0] };
    len += 1;
    while anchor + 1 < raw.len() {
        let mut next = anchor + 1;
        for candidate in (anchor + 2..raw.len()).rev() {
            if validity.is_segment_free(&raw[anchor], &raw[candidate]) {
                next = candidate;
                break;
            }
        }
        if len >= MAX_WAYPOINTS {
            return false;
        }
        out.waypoints[len] = CuJointPositionsPayload {
            positions: raw[next],
        };
        cost += joint_distance(&raw[anchor], &raw[next], metric);
        len += 1;
        anchor = next;
    }
    out.len = len as u32;
    out.cost = cost;
    true
}

/// Quality as straight-line length over path length, in `0.0..=1.0`.
pub(crate) fn length_quality(straight: f64, cost: f64) -> Quality {
    if !cost.is_finite() {
        return quality_from_f32(0.0);
    }
    if cost <= 1e-9 || straight <= 1e-9 {
        return quality_from_f32(1.0);
    }
    quality_from_f32((straight / cost).clamp(0.0, 1.0) as f32)
}
