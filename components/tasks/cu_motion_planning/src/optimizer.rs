//! Joint-path optimizer: non-sampling, deterministic, anytime.
//!
//! A CHOMP-style planner. The path is a fixed number of waypoints, seeded on
//! the straight joint-space line, and every iteration takes one gradient step
//! on
//!
//! ```text
//! cost = w_smooth · ½ Σ |x[i+1] − x[i]|²  +  w_obstacle · Σ max(0, ε − c(x[i]))² / ε
//! ```
//!
//! in metric-normalized joint coordinates, where `c` is the signed clearance
//! of the robot and attached objects against the captured scene and `ε` the
//! target clearance. The obstacle gradient is a forward finite difference of
//! the clearance, so any [`CuSceneValidity`] scene works without analytic
//! distance gradients. After each step the whole path is checked segment by
//! segment; the shortest collision-free path seen so far is the published
//! result. No randomness is involved: the same request always yields the
//! same path.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuJointPlannerTask;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::MAX_WAYPOINTS;
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
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Parameters of [`CuJointPathOptimizer`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointPathOptimizerParams {
    /// Path waypoints including start and goal (`3..=MAX_WAYPOINTS`).
    pub waypoints: u32,
    /// Gradient step per unit of cost gradient (dimensionless, metric-scaled).
    pub learning_rate: f64,
    /// Clearance below which the obstacle term pushes the path away.
    pub clearance: Length,
    /// Weight of the obstacle term (dimensionless, `>= 0`).
    pub obstacle_weight: f64,
    /// Weight of the smoothness term (dimensionless, `>= 0`).
    pub smoothness_weight: f64,
    /// Finite-difference step for the clearance gradient (dimensionless,
    /// metric-scaled).
    pub gradient_step: f64,
    /// Iterations after which the job ends.
    pub max_iterations: u32,
    /// Resolution of the segment collision checks.
    pub collision_check_step: CuJointResolution,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointPathOptimizerParams {
    fn default() -> Self {
        Self {
            waypoints: 12,
            learning_rate: 0.05,
            clearance: Length::new::<meter>(0.05),
            obstacle_weight: 40.0,
            smoothness_weight: 1.0,
            gradient_step: 1e-3,
            max_iterations: 300,
            collision_check_step: CuJointResolution::default(),
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointPathOptimizerParams {
    /// Checks every parameter, the metric, and the collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()?;
        let valid = {
            let positive = |v: f64| v.is_finite() && v > 0.0;
            (3..=MAX_WAYPOINTS as u32).contains(&self.waypoints)
                && positive(self.learning_rate)
                && positive(self.clearance.get::<meter>())
                && self.obstacle_weight.is_finite()
                && self.obstacle_weight >= 0.0
                && self.smoothness_weight.is_finite()
                && self.smoothness_weight >= 0.0
                && positive(self.gradient_step)
                && self.max_iterations > 0
        };
        if valid {
            Ok(())
        } else {
            Err(CuError::from(
                "cu-motion-planning: invalid path optimizer parameters",
            ))
        }
    }
}

/// Deterministic anytime joint-path optimizer.
///
/// Built with [`CuJointPlanner::new`] from robot models and [`CuJointPathOptimizerParams`];
/// runs in [`CuJointPathOptimizerTask`].
#[derive(Clone)]
pub struct CuJointPathOptimizer<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointPathOptimizerParams,
    path: [[CuJointPosition; N]; MAX_WAYPOINTS],
    best: [[CuJointPosition; N]; MAX_WAYPOINTS],
    count: usize,
    best_cost: f64,
    straight_cost: f64,
    iterations: u32,
    status: CuPlanStatus,
}

/// [`CuJointPlannerTask`] running [`CuJointPathOptimizer`].
pub type CuJointPathOptimizerTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointPathOptimizer<N, M>, N, M>;

impl<const N: usize, const M: usize> CuJointPathOptimizer<N, M> {
    /// The validated planner parameters.
    pub const fn params(&self) -> &CuJointPathOptimizerParams {
        &self.params
    }

    fn build(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        params: CuJointPathOptimizerParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, params.collision_check_step),
            params,
            path: [[CuJointPosition::default(); N]; MAX_WAYPOINTS],
            best: [[CuJointPosition::default(); N]; MAX_WAYPOINTS],
            count: 0,
            best_cost: f64::INFINITY,
            straight_cost: 0.0,
            iterations: 0,
            status: CuPlanStatus::InvalidRequest,
        }
    }

    /// Iterations run on the current job.
    pub const fn iterations(&self) -> u32 {
        self.iterations
    }

    fn penalty(&self, q: &[CuJointPosition; N]) -> f64 {
        self.penalty_of(self.validity.clearance(q))
    }

    fn penalty_of(&self, clearance: Option<f64>) -> f64 {
        let eps = self.params.clearance.get::<meter>();
        match clearance {
            Some(c) if c >= eps => 0.0,
            Some(c) => self.params.obstacle_weight * (eps - c) * (eps - c) / eps,
            // Unevaluable configurations are as bad as deep penetration.
            None => self.params.obstacle_weight * eps,
        }
    }

    fn path_cost(&self, path: &[[CuJointPosition; N]]) -> f64 {
        path.windows(2)
            .map(|w| joint_distance(&w[0], &w[1], self.params.metric))
            .sum()
    }

    fn path_is_free(&self) -> bool {
        (1..self.count - 1).all(|i| self.validity.is_configuration_free(&self.path[i]))
            && (0..self.count - 1).all(|i| {
                self.validity
                    .is_segment_free(&self.path[i], &self.path[i + 1])
            })
    }

    fn record_if_better(&mut self) {
        if !self.path_is_free() {
            return;
        }
        let cost = self.path_cost(&self.path[..self.count]);
        if cost < self.best_cost {
            self.best_cost = cost;
            self.best[..self.count].copy_from_slice(&self.path[..self.count]);
        }
    }

    /// One gradient step on every interior waypoint; returns the largest
    /// normalized joint update.
    fn descend(&mut self) -> f64 {
        let metric = self.params.metric;
        let h = self.params.gradient_step;
        let eps = self.params.clearance.get::<meter>();
        let mut largest = 0.0_f64;
        let mut next = self.path;
        for (i, slot) in next.iter_mut().enumerate().take(self.count - 1).skip(1) {
            let q = self.path[i];
            let clearance = self.validity.clearance(&q);
            let base = self.penalty_of(clearance);
            // Well clear of every obstacle a finite-difference probe stays in
            // the flat zero-penalty region, so the clearance queries are skipped.
            let flat = clearance.is_some_and(|c| c > 2.0 * eps);
            for j in 0..N {
                if matches!(q[j], CuJointPosition::Fixed) {
                    continue;
                }
                let scale = metric.scale(q[j]);
                let x = q[j].si_value() / scale;
                let prev = self.path[i - 1][j].si_value() / scale;
                let after = self.path[i + 1][j].si_value() / scale;
                let mut gradient = self.params.smoothness_weight * (2.0 * x - prev - after);
                if !flat {
                    let mut probe = q;
                    probe[j] = q[j].with_si_value((x + h) * scale);
                    gradient += (self.penalty(&probe) - base) / h;
                }
                let limits = self.validity.chain().joints()[j].limits;
                let lo = limits.lower().si_value() / scale;
                let hi = limits.upper().si_value() / scale;
                let updated = (x - self.params.learning_rate * gradient).clamp(lo, hi);
                largest = largest.max(libm::fabs(updated - x));
                slot[j] = q[j].with_si_value(updated * scale);
            }
        }
        self.path = next;
        largest
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointPathOptimizer<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Optimization;

    type Params = CuJointPathOptimizerParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointPathOptimizerParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self::build(chain, collision_model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, _seed: u64) -> CuResult<CuPlanStatus> {
        self.iterations = 0;
        self.best_cost = f64::INFINITY;
        self.count = self.params.waypoints as usize;
        self.straight_cost = joint_distance(&request.start, &request.goal, self.params.metric);
        self.status = check_endpoints(&mut self.validity, request, self.params.metric);
        if self.status == CuPlanStatus::Searching {
            let last = (self.count - 1) as f64;
            for i in 0..self.count {
                for j in 0..N {
                    self.path[i][j] = request.start[j]
                        .interpolate(request.goal[j], i as f64 / last)
                        .map_err(|_| CuError::from("cu-motion-planning: joint kinds disagree"))?;
                }
            }
            // The straight line is the cheapest possible path; when it is
            // free there is nothing to optimize.
            self.record_if_better();
            if self.best_cost.is_finite() {
                self.status = CuPlanStatus::Converged;
            }
        }
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        for _ in 0..budget {
            if self.status.is_final() {
                break;
            }
            let moved = self.descend();
            self.iterations += 1;
            self.record_if_better();
            let done = moved <= 1e-7 || self.iterations >= self.params.max_iterations;
            self.status = match (self.best_cost.is_finite(), done) {
                (true, true) => CuPlanStatus::Converged,
                (true, false) => CuPlanStatus::Improving,
                (false, true) => CuPlanStatus::Exhausted,
                (false, false) => CuPlanStatus::Searching,
            };
        }
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        write_header(out, &self.validity, self.status, self.iterations);
        if self.status.has_path() {
            for (slot, q) in out.waypoints.iter_mut().zip(&self.best[..self.count]) {
                *slot = CuJointPositionsPayload { positions: *q };
            }
            out.len = self.count as u32;
            out.cost = self.best_cost;
        }
        self.status
    }

    fn quality(&self) -> Quality {
        if !self.best_cost.is_finite() {
            return quality_from_f32(0.0);
        }
        if self.best_cost <= 1e-9 {
            return quality_from_f32(1.0);
        }
        quality_from_f32((self.straight_cost / self.best_cost).clamp(0.0, 1.0) as f32)
    }
}
