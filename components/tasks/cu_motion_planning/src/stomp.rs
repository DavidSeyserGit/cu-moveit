//! STOMP: stochastic trajectory optimization, anytime and seeded.
//!
//! Like [`CuJointPathOptimizer`](crate::CuJointPathOptimizer) the
//! path is a fixed number of waypoints seeded on the straight joint line, but
//! updates need no gradients. Each iteration draws `rollouts` smooth noisy
//! copies of the path, scores every waypoint of every rollout by its clearance
//! penalty plus a local acceleration (smoothness) cost, weights the noise per
//! waypoint by `exp(-sensitivity · normalized cost)`, and adds the smoothed
//! weighted noise to the path. That copes with costs that are flat or
//! discontinuous where a gradient would vanish. The shortest collision-free
//! path seen so far is the result. All randomness comes from the job seed.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuJointPlannerTask;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::MAX_WAYPOINTS;
use crate::common::check_endpoints;
use crate::common::length_quality;
use crate::common::path_length;
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
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Rollout capacity per iteration.
pub const STOMP_MAX_ROLLOUTS: usize = 16;

/// Parameters of [`CuJointStomp`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointStompParams {
    /// Path waypoints including start and goal (`3..=MAX_WAYPOINTS`).
    pub waypoints: u32,
    /// Noisy rollouts per iteration (`2..=STOMP_MAX_ROLLOUTS`).
    pub rollouts: u32,
    /// Standard deviation of the exploration noise (dimensionless,
    /// metric-scaled).
    pub noise: f64,
    /// Exponent scale turning normalized rollout costs into weights
    /// (dimensionless).
    pub sensitivity: f64,
    /// Clearance below which the obstacle cost applies.
    pub clearance: Length,
    /// Weight of the obstacle cost (dimensionless, `>= 0`).
    pub obstacle_weight: f64,
    /// Weight of the smoothness cost (dimensionless, `>= 0`).
    pub smoothness_weight: f64,
    /// Iterations after which the job ends.
    pub max_iterations: u32,
    /// Resolution of the segment collision checks.
    pub collision_check_step: CuJointResolution,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointStompParams {
    fn default() -> Self {
        Self {
            waypoints: 12,
            rollouts: 8,
            noise: 0.15,
            sensitivity: 10.0,
            clearance: Length::new::<meter>(0.05),
            obstacle_weight: 40.0,
            smoothness_weight: 1.0,
            max_iterations: 200,
            collision_check_step: CuJointResolution::default(),
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointStompParams {
    /// Checks every parameter, the metric, and the collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()?;
        let valid = {
            let positive = |v: f64| v.is_finite() && v > 0.0;
            (3..=MAX_WAYPOINTS as u32).contains(&self.waypoints)
                && (2..=STOMP_MAX_ROLLOUTS as u32).contains(&self.rollouts)
                && positive(self.noise)
                && positive(self.sensitivity)
                && positive(self.clearance.get::<meter>())
                && self.obstacle_weight.is_finite()
                && self.obstacle_weight >= 0.0
                && self.smoothness_weight.is_finite()
                && self.smoothness_weight >= 0.0
                && self.max_iterations > 0
        };
        if valid {
            Ok(())
        } else {
            Err(CuError::from(
                "cu-motion-planning: invalid STOMP parameters",
            ))
        }
    }
}

/// Stochastic trajectory optimizer.
///
/// Built with [`CuJointPlanner::new`] from robot models and [`CuJointStompParams`];
/// runs in [`CuJointStompTask`].
pub struct CuJointStomp<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointStompParams,
    path: [[f64; N]; MAX_WAYPOINTS],
    kinds: [CuJointPosition; N],
    best: [[CuJointPosition; N]; MAX_WAYPOINTS],
    count: usize,
    best_cost: f64,
    straight_cost: f64,
    rng: CuRng,
    iterations: u32,
    status: CuPlanStatus,
}

/// Approximately standard normal sample (Irwin–Hall with four uniforms).
fn normal(rng: &mut CuRng) -> f64 {
    let sum: f64 = (0..4).map(|_| rng.random::<f64>()).sum();
    (sum - 2.0) * libm::sqrt(3.0)
}

/// In-place `[1/4, 1/2, 1/4]` smoothing along the path, endpoints fixed at zero.
fn smooth<const N: usize>(values: &mut [[f64; N]], count: usize) {
    let copy: [[f64; N]; MAX_WAYPOINTS] =
        core::array::from_fn(|i| if i < count { values[i] } else { [0.0; N] });
    for i in 1..count - 1 {
        for j in 0..N {
            values[i][j] = 0.25 * copy[i - 1][j] + 0.5 * copy[i][j] + 0.25 * copy[i + 1][j];
        }
    }
    values[0] = [0.0; N];
    values[count - 1] = [0.0; N];
}

/// [`CuJointPlannerTask`] running [`CuJointStomp`].
pub type CuJointStompTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointStomp<N, M>, N, M>;

impl<const N: usize, const M: usize> CuJointStomp<N, M> {
    /// The validated planner parameters.
    pub const fn params(&self) -> &CuJointStompParams {
        &self.params
    }

    fn build(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        params: CuJointStompParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, params.collision_check_step),
            params,
            path: [[0.0; N]; MAX_WAYPOINTS],
            kinds: [CuJointPosition::default(); N],
            best: [[CuJointPosition::default(); N]; MAX_WAYPOINTS],
            count: 0,
            best_cost: f64::INFINITY,
            straight_cost: 0.0,
            rng: CuRng::from_seed(0),
            iterations: 0,
            status: CuPlanStatus::InvalidRequest,
        }
    }

    /// Iterations run on the current job.
    pub const fn iterations(&self) -> u32 {
        self.iterations
    }

    fn scale(&self, j: usize) -> f64 {
        self.params.metric.scale(self.kinds[j])
    }

    fn configuration(&self, x: &[f64; N]) -> [CuJointPosition; N] {
        core::array::from_fn(|j| self.kinds[j].with_si_value(x[j] * self.scale(j)))
    }

    fn clamp(&self, x: &mut [f64; N]) {
        for (j, value) in x.iter_mut().enumerate() {
            if matches!(self.kinds[j], CuJointPosition::Fixed) {
                *value = 0.0;
                continue;
            }
            let limits = self.validity.chain().joints()[j].limits;
            let lo = limits.lower().si_value() / self.scale(j);
            let hi = limits.upper().si_value() / self.scale(j);
            *value = value.clamp(lo, hi);
        }
    }

    fn obstacle_cost(&self, x: &[f64; N]) -> f64 {
        let eps = self.params.clearance.get::<meter>();
        match self.validity.clearance(&self.configuration(x)) {
            Some(c) if c >= eps => 0.0,
            Some(c) => self.params.obstacle_weight * (eps - c) * (eps - c) / eps,
            None => self.params.obstacle_weight * eps,
        }
    }

    fn record_if_better(&mut self) {
        let mut q = [[CuJointPosition::default(); N]; MAX_WAYPOINTS];
        for (slot, x) in q.iter_mut().zip(&self.path[..self.count]) {
            *slot = self.configuration(x);
        }
        let free = (1..self.count - 1).all(|i| self.validity.is_configuration_free(&q[i]))
            && (0..self.count - 1).all(|i| self.validity.is_segment_free(&q[i], &q[i + 1]));
        if !free {
            return;
        }
        let cost = path_length(&q[..self.count], self.params.metric);
        if cost < self.best_cost {
            self.best_cost = cost;
            self.best = q;
        }
    }

    // Rollout, waypoint and joint indices address several parallel arrays;
    // explicit indices read clearer than zipped iterators here.
    #[allow(clippy::needless_range_loop)]
    fn iterate(&mut self) {
        let k = self.params.rollouts as usize;
        let count = self.count;
        let mut noise = [[[0.0_f64; N]; MAX_WAYPOINTS]; STOMP_MAX_ROLLOUTS];
        let mut costs = [[0.0_f64; MAX_WAYPOINTS]; STOMP_MAX_ROLLOUTS];
        for r in 0..k {
            for i in 1..count - 1 {
                for j in 0..N {
                    noise[r][i][j] = self.params.noise * normal(&mut self.rng);
                }
            }
            smooth(&mut noise[r], count);
            for i in 1..count - 1 {
                let mut x = self.path[i];
                for j in 0..N {
                    x[j] += noise[r][i][j];
                }
                self.clamp(&mut x);
                let mut acceleration = 0.0;
                for j in 0..N {
                    let a = (self.path[i - 1][j] + noise[r][i - 1][j]) - 2.0 * x[j]
                        + (self.path[i + 1][j] + noise[r][i + 1][j]);
                    acceleration += a * a;
                }
                costs[r][i] = self.obstacle_cost(&x) + self.params.smoothness_weight * acceleration;
            }
        }
        let mut update = [[0.0_f64; N]; MAX_WAYPOINTS];
        for i in 1..count - 1 {
            let (lo, hi) = (0..k).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| {
                (lo.min(costs[r][i]), hi.max(costs[r][i]))
            });
            let span = (hi - lo).max(1e-12);
            let mut weights = [0.0_f64; STOMP_MAX_ROLLOUTS];
            let mut total = 0.0;
            for r in 0..k {
                weights[r] = libm::exp(-self.params.sensitivity * (costs[r][i] - lo) / span);
                total += weights[r];
            }
            for r in 0..k {
                for j in 0..N {
                    update[i][j] += weights[r] / total * noise[r][i][j];
                }
            }
        }
        smooth(&mut update, count);
        for i in 1..count - 1 {
            let mut x = self.path[i];
            for j in 0..N {
                x[j] += update[i][j];
            }
            self.clamp(&mut x);
            self.path[i] = x;
        }
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointStomp<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Optimization;

    type Params = CuJointStompParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointStompParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self::build(chain, collision_model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, seed: u64) -> CuResult<CuPlanStatus> {
        self.rng.reseed(seed);
        self.iterations = 0;
        self.best_cost = f64::INFINITY;
        self.count = self.params.waypoints as usize;
        self.kinds = request.start;
        self.straight_cost = joint_distance(&request.start, &request.goal, self.params.metric);
        self.status = check_endpoints(&mut self.validity, request, self.params.metric);
        if self.status != CuPlanStatus::Searching {
            return Ok(self.status);
        }
        let last = (self.count - 1) as f64;
        for i in 0..self.count {
            let t = i as f64 / last;
            for j in 0..N {
                let a = self.params.metric.normalized(request.start[j]);
                let b = self.params.metric.normalized(request.goal[j]);
                self.path[i][j] = a + t * (b - a);
            }
        }
        // A free straight line is already the shortest path.
        self.record_if_better();
        if self.best_cost.is_finite() {
            self.status = CuPlanStatus::Converged;
        }
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        for _ in 0..budget {
            if self.status.is_final() {
                break;
            }
            self.iterate();
            self.iterations += 1;
            self.record_if_better();
            let done = self.iterations >= self.params.max_iterations;
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
        length_quality(self.straight_cost, self.best_cost)
    }
}
