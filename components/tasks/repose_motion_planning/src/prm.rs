//! PRM: sampling-based probabilistic roadmap.
//!
//! The roadmap starts with the start and goal configurations. Each work unit
//! adds one uniform sample; free samples connect to their nearest roadmap
//! neighbours within `connection_radius` through free segments. After every
//! quantum that changed the roadmap, Dijkstra finds the shortest roadmap
//! path, which is shortcut by line of sight; the job improves as the roadmap
//! densifies and converges when it is full. The roadmap is rebuilt for every
//! job from the job seed, so replay reproduces it.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuJointPlannerTask;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::common::check_endpoints;
use crate::common::length_quality;
use crate::common::sample_uniform;
use crate::common::write_header;
use crate::common::write_shortcut;
use crate::validity::CuSceneValidity;
use crate::validity::joint_distance;
use bincode::Decode;
use bincode::Encode;
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use repose_collision::CuRobotCollisionModel;
use repose_kinematics::CuJointMetric;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointResolution;
use repose_kinematics::CuKinematicChain;
use serde::Deserialize;
use serde::Serialize;

/// Roadmap node capacity, including start and goal.
pub const PRM_MAX_NODES: usize = 1024;
/// Longest roadmap path (in nodes) that can be reconstructed.
const PRM_MAX_PATH: usize = 256;
/// Edge capacity per roadmap node.
pub const PRM_MAX_NEIGHBORS: usize = 10;

/// Parameters of [`CuJointPrm`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointPrmParams {
    /// Joint-space radius for roadmap edges (dimensionless, metric-scaled).
    pub connection_radius: f64,
    /// Neighbours tried per new node (`1..=PRM_MAX_NEIGHBORS`).
    pub neighbors: u32,
    /// Roadmap capacity (`3..=PRM_MAX_NODES`).
    pub max_nodes: u32,
    /// Resolution of the segment collision checks.
    pub collision_check_step: CuJointResolution,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointPrmParams {
    fn default() -> Self {
        Self {
            connection_radius: 2.0,
            neighbors: 8,
            max_nodes: PRM_MAX_NODES as u32,
            collision_check_step: CuJointResolution::default(),
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointPrmParams {
    /// Checks every parameter, the metric, and the collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()?;
        let valid = {
            self.connection_radius.is_finite()
                && self.connection_radius > 0.0
                && (1..=PRM_MAX_NEIGHBORS as u32).contains(&self.neighbors)
                && (3..=PRM_MAX_NODES as u32).contains(&self.max_nodes)
        };
        if valid {
            Ok(())
        } else {
            Err(CuError::from(
                "repose-motion-planning: invalid PRM parameters",
            ))
        }
    }
}

/// Probabilistic roadmap planner (single query per job).
///
/// Built with [`CuJointPlanner::new`] from robot models and [`CuJointPrmParams`];
/// runs in [`CuJointPrmTask`].
pub struct CuJointPrm<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointPrmParams,
    nodes: [[CuJointPosition; N]; PRM_MAX_NODES],
    edges: [[u16; PRM_MAX_NEIGHBORS]; PRM_MAX_NODES],
    degree: [u8; PRM_MAX_NODES],
    len: usize,
    rng: CuRng,
    straight_cost: f64,
    samples: u32,
    status: CuPlanStatus,
    best: CuJointPlanPath<N>,
}

const START: usize = 0;
const GOAL: usize = 1;

/// [`CuJointPlannerTask`] running [`CuJointPrm`].
pub type CuJointPrmTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointPrm<N, M>, N, M>;

impl<const N: usize, const M: usize> CuJointPrm<N, M> {
    /// The validated planner parameters.
    pub const fn params(&self) -> &CuJointPrmParams {
        &self.params
    }

    fn build(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        params: CuJointPrmParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, params.collision_check_step),
            params,
            nodes: [[CuJointPosition::default(); N]; PRM_MAX_NODES],
            edges: [[0; PRM_MAX_NEIGHBORS]; PRM_MAX_NODES],
            degree: [0; PRM_MAX_NODES],
            len: 0,
            rng: CuRng::from_seed(0),
            straight_cost: 0.0,
            samples: 0,
            status: CuPlanStatus::InvalidRequest,
            best: CuJointPlanPath::default(),
        }
    }

    /// Roadmap size of the current job.
    pub const fn roadmap_size(&self) -> usize {
        self.len
    }

    fn link(&mut self, a: usize, b: usize) {
        let cap = PRM_MAX_NEIGHBORS.min(self.params.neighbors as usize);
        if (self.degree[a] as usize) < cap && (self.degree[b] as usize) < cap {
            self.edges[a][self.degree[a] as usize] = b as u16;
            self.degree[a] += 1;
            self.edges[b][self.degree[b] as usize] = a as u16;
            self.degree[b] += 1;
        }
    }

    /// Connects node `idx` to its nearest free neighbours; returns whether
    /// any edge was added.
    fn connect(&mut self, idx: usize) -> bool {
        let k = (self.params.neighbors as usize).min(PRM_MAX_NEIGHBORS);
        let mut nearest = [(usize::MAX, f64::INFINITY); PRM_MAX_NEIGHBORS];
        for other in 0..self.len {
            if other == idx {
                continue;
            }
            let d = joint_distance(&self.nodes[idx], &self.nodes[other], self.params.metric);
            if d > self.params.connection_radius || d >= nearest[k - 1].1 {
                continue;
            }
            let mut slot = k - 1;
            while slot > 0 && nearest[slot - 1].1 > d {
                nearest[slot] = nearest[slot - 1];
                slot -= 1;
            }
            nearest[slot] = (other, d);
        }
        let mut added = false;
        for &(other, _) in nearest[..k].iter().take_while(|n| n.0 != usize::MAX) {
            if self
                .validity
                .is_segment_free(&self.nodes[idx], &self.nodes[other])
            {
                let before = self.degree[idx];
                self.link(idx, other);
                added |= self.degree[idx] != before;
            }
        }
        added
    }

    /// Dijkstra from start to goal; on a shorter path, shortcuts it into `best`.
    fn search(&mut self) {
        let mut dist = [f64::INFINITY; PRM_MAX_NODES];
        let mut prev = [u16::MAX; PRM_MAX_NODES];
        let mut done = [false; PRM_MAX_NODES];
        dist[START] = 0.0;
        loop {
            let mut current = usize::MAX;
            for i in 0..self.len {
                if !done[i]
                    && dist[i].is_finite()
                    && (current == usize::MAX || dist[i] < dist[current])
                {
                    current = i;
                }
            }
            if current == usize::MAX || current == GOAL {
                break;
            }
            done[current] = true;
            for e in 0..self.degree[current] as usize {
                let next = self.edges[current][e] as usize;
                let d = dist[current]
                    + joint_distance(&self.nodes[current], &self.nodes[next], self.params.metric);
                if d < dist[next] {
                    dist[next] = d;
                    prev[next] = current as u16;
                }
            }
        }
        if !dist[GOAL].is_finite() || dist[GOAL] >= self.best.cost {
            return;
        }
        let mut raw = [[CuJointPosition::default(); N]; PRM_MAX_PATH];
        let mut len = 0;
        let mut at = GOAL;
        loop {
            if len >= PRM_MAX_PATH {
                if !self.best.cost.is_finite() {
                    self.status = CuPlanStatus::CapacityExceeded;
                }
                return;
            }
            raw[len] = self.nodes[at];
            len += 1;
            if at == START {
                break;
            }
            at = prev[at] as usize;
        }
        raw[..len].reverse();
        let mut candidate = CuJointPlanPath::default();
        if write_shortcut(
            &self.validity,
            &raw[..len],
            self.params.metric,
            &mut candidate,
        ) {
            if candidate.cost < self.best.cost {
                self.best = candidate;
            }
        } else if !self.best.cost.is_finite() {
            self.status = CuPlanStatus::CapacityExceeded;
        }
    }

    fn refresh_status(&mut self) {
        if self.status.is_final() {
            return;
        }
        let full = self.len >= self.params.max_nodes as usize;
        let solved = self.best.cost.is_finite();
        let optimal = solved && self.best.cost <= self.straight_cost + 1e-9;
        self.status = match (solved, full || optimal) {
            (true, true) => CuPlanStatus::Converged,
            (true, false) => CuPlanStatus::Improving,
            (false, true) => CuPlanStatus::Exhausted,
            (false, false) => CuPlanStatus::Searching,
        };
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointPrm<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Sampling;

    type Params = CuJointPrmParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointPrmParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self::build(chain, collision_model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, seed: u64) -> CuResult<CuPlanStatus> {
        self.rng.reseed(seed);
        self.samples = 0;
        self.len = 0;
        self.degree = [0; PRM_MAX_NODES];
        self.best = CuJointPlanPath::default();
        self.straight_cost = joint_distance(&request.start, &request.goal, self.params.metric);
        self.status = check_endpoints(&mut self.validity, request, self.params.metric);
        if self.status != CuPlanStatus::Searching {
            return Ok(self.status);
        }
        self.nodes[START] = request.start;
        self.nodes[GOAL] = request.goal;
        self.len = 2;
        if self.validity.is_segment_free(&request.start, &request.goal) {
            self.link(START, GOAL);
            self.search();
        }
        self.refresh_status();
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        let mut changed = false;
        for _ in 0..budget {
            if self.status.is_final() || self.len >= self.params.max_nodes as usize {
                break;
            }
            self.samples += 1;
            let q = sample_uniform(self.validity.chain(), &mut self.rng);
            if !self.validity.is_configuration_free(&q) {
                continue;
            }
            let idx = self.len;
            self.nodes[idx] = q;
            self.degree[idx] = 0;
            self.len += 1;
            changed |= self.connect(idx);
        }
        if changed {
            self.search();
        }
        self.refresh_status();
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        write_header(out, &self.validity, self.status, self.samples);
        if self.status.has_path() {
            out.waypoints = self.best.waypoints.clone();
            out.len = self.best.len;
            out.cost = self.best.cost;
        }
        self.status
    }

    fn quality(&self) -> Quality {
        length_quality(
            self.straight_cost,
            if self.status.has_path() {
                self.best.cost
            } else {
                f64::INFINITY
            },
        )
    }
}

crate::impl_planner_type_path!(CuJointPrm, "repose_motion_planning::CuJointPrm");
