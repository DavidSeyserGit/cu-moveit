//! RRT-Connect: sampling-based, bidirectional, feasibility-first.
//!
//! Two trees grow from start and goal. Each iteration extends one tree one
//! step toward a uniform sample, then greedily connects the other tree to the
//! new node; the trees swap roles every iteration. The first connection is
//! shortcut by line of sight and the job converges: RRT-Connect finds a path
//! quickly but does not optimize it (use RRT*, PRM, or an optimizer for that).
//! All randomness comes from the job seed.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::common::check_endpoints;
use crate::common::length_quality;
use crate::common::sample_uniform;
use crate::common::steer;
use crate::common::write_header;
use crate::common::write_shortcut;
use crate::validity::CuSceneValidity;
use crate::validity::joint_distance;
use bincode::Decode;
use bincode::Encode;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Node capacity of each of the two trees.
pub const RRT_CONNECT_MAX_NODES: usize = 256;

/// Parameters of [`CuJointRrtConnect`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct CuJointRrtConnectParams {
    /// Maximum normalized joint-space step per extension.
    pub step_size: f64,
    /// Node capacity per tree (`2..=RRT_CONNECT_MAX_NODES`).
    pub max_nodes: u32,
    pub collision_check_step: CuJointResolution,
    pub metric: CuJointMetric,
}

impl Default for CuJointRrtConnectParams {
    fn default() -> Self {
        Self {
            step_size: 0.3,
            max_nodes: RRT_CONNECT_MAX_NODES as u32,
            collision_check_step: CuJointResolution::default(),
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointRrtConnectParams {
    fn is_valid(&self) -> bool {
        self.step_size.is_finite()
            && self.step_size > 0.0
            && (2..=RRT_CONNECT_MAX_NODES as u32).contains(&self.max_nodes)
    }
}

#[derive(Clone)]
struct Tree<const N: usize> {
    nodes: [[CuJointPosition; N]; RRT_CONNECT_MAX_NODES],
    parents: [u16; RRT_CONNECT_MAX_NODES],
    len: usize,
}

enum Extend {
    Trapped,
    Advanced(usize),
    Reached(usize),
}

impl<const N: usize> Tree<N> {
    fn new() -> Self {
        Self {
            nodes: [[CuJointPosition::default(); N]; RRT_CONNECT_MAX_NODES],
            parents: [u16::MAX; RRT_CONNECT_MAX_NODES],
            len: 0,
        }
    }

    fn reset(&mut self, root: [CuJointPosition; N]) {
        self.nodes[0] = root;
        self.parents[0] = u16::MAX;
        self.len = 1;
    }

    fn nearest(&self, q: &[CuJointPosition; N], metric: CuJointMetric) -> usize {
        let mut best = (0, f64::INFINITY);
        for i in 0..self.len {
            let d = joint_distance(&self.nodes[i], q, metric);
            if d < best.1 {
                best = (i, d);
            }
        }
        best.0
    }

    /// One step from the nearest node toward `target`.
    fn extend<const M: usize>(
        &mut self,
        validity: &CuSceneValidity<N, M>,
        target: &[CuJointPosition; N],
        params: &CuJointRrtConnectParams,
    ) -> Extend {
        if self.len >= params.max_nodes as usize {
            return Extend::Trapped;
        }
        let near = self.nearest(target, params.metric);
        let q_new = steer(&self.nodes[near], target, params.step_size, params.metric);
        if !validity.is_segment_free(&self.nodes[near], &q_new) {
            return Extend::Trapped;
        }
        let idx = self.len;
        self.nodes[idx] = q_new;
        self.parents[idx] = near as u16;
        self.len += 1;
        if joint_distance(&q_new, target, params.metric) <= 1e-9 {
            Extend::Reached(idx)
        } else {
            Extend::Advanced(idx)
        }
    }

    /// Repeated extension toward `target` until reached or blocked.
    fn connect<const M: usize>(
        &mut self,
        validity: &CuSceneValidity<N, M>,
        target: &[CuJointPosition; N],
        params: &CuJointRrtConnectParams,
    ) -> Option<usize> {
        loop {
            match self.extend(validity, target, params) {
                Extend::Advanced(_) => {}
                Extend::Reached(idx) => return Some(idx),
                Extend::Trapped => return None,
            }
        }
    }

    /// Writes the chain from node `idx` to the root into `out`; returns its length.
    fn chain_to_root(&self, mut idx: usize, out: &mut [[CuJointPosition; N]]) -> usize {
        let mut len = 0;
        loop {
            out[len] = self.nodes[idx];
            len += 1;
            let parent = self.parents[idx];
            if parent == u16::MAX || len >= out.len() {
                return len;
            }
            idx = parent as usize;
        }
    }
}

/// Bidirectional RRT-Connect planner.
///
/// Config keys: `chain`, `collision_model`, and optional `step_size`,
/// `max_nodes`, `collision_check_step`, `metric`.
pub struct CuJointRrtConnect<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointRrtConnectParams,
    /// `trees[0]` grows from the start, `trees[1]` from the goal.
    trees: [Tree<N>; 2],
    active: usize,
    rng: CuRng,
    straight_cost: f64,
    iterations: u32,
    status: CuPlanStatus,
    best: CuJointPlanPath<N>,
}

impl<const N: usize, const M: usize> CuJointRrtConnect<N, M> {
    /// Creates the planner for explicit robot models.
    pub fn new(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        params: CuJointRrtConnectParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, params.collision_check_step),
            params,
            trees: [Tree::new(), Tree::new()],
            active: 0,
            rng: CuRng::from_seed(0),
            straight_cost: 0.0,
            iterations: 0,
            status: CuPlanStatus::InvalidRequest,
            best: CuJointPlanPath::default(),
        }
    }

    /// Joins the trees at `start_idx` (start tree) and `goal_idx` (goal tree).
    fn finish(&mut self, start_idx: usize, goal_idx: usize) {
        let mut raw = [[CuJointPosition::default(); N]; 2 * RRT_CONNECT_MAX_NODES];
        let head = self.trees[0].chain_to_root(start_idx, &mut raw);
        raw[..head].reverse();
        let tail = self.trees[1].chain_to_root(goal_idx, &mut raw[head..]);
        let len = head + tail;
        self.status = if write_shortcut(
            &self.validity,
            &raw[..len],
            self.params.metric,
            &mut self.best,
        ) {
            CuPlanStatus::Converged
        } else {
            CuPlanStatus::CapacityExceeded
        };
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N> for CuJointRrtConnect<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Sampling;

    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let chain = CuKinematicChain::from_config(config)?;
        let model = CuRobotCollisionModel::from_config(config)?;
        let mut params = CuJointRrtConnectParams::default();
        if let Some(cfg) = config {
            if let Some(v) = cfg.get::<f64>("step_size")? {
                params.step_size = v;
            }
            if let Some(v) = cfg.get::<u32>("max_nodes")? {
                params.max_nodes = v;
            }
            if let Some(v) = cfg.get_value::<CuJointResolution>("collision_check_step")? {
                params.collision_check_step = v;
            }
            if let Some(v) = cfg.get_value::<CuJointMetric>("metric")? {
                params.metric = v;
            }
        }
        if !params.is_valid() {
            return Err(CuError::from(
                "cu-motion-planning: invalid RRT-Connect parameters",
            ));
        }
        Ok(Self::new(chain, model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, seed: u64) -> CuResult<CuPlanStatus> {
        self.rng.reseed(seed);
        self.iterations = 0;
        self.active = 0;
        self.best = CuJointPlanPath::default();
        self.straight_cost = joint_distance(&request.start, &request.goal, self.params.metric);
        self.status = if self.params.is_valid() {
            check_endpoints(&mut self.validity, request, self.params.metric)
        } else {
            CuPlanStatus::InvalidRequest
        };
        if self.status != CuPlanStatus::Searching {
            return Ok(self.status);
        }
        self.trees[0].reset(request.start);
        self.trees[1].reset(request.goal);
        if self.validity.is_segment_free(&request.start, &request.goal) {
            self.trees[1].parents[0] = u16::MAX;
            self.finish(0, 0);
        }
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        for _ in 0..budget {
            if self.status.is_final() {
                break;
            }
            self.iterations += 1;
            let (a, b) = (self.active, 1 - self.active);
            self.active = b;
            let sample = sample_uniform(self.validity.chain(), &mut self.rng);
            let [first, second] = &mut self.trees;
            let (grow, other) = if a == 0 {
                (first, second)
            } else {
                (second, first)
            };
            let added = match grow.extend(&self.validity, &sample, &self.params) {
                Extend::Advanced(idx) | Extend::Reached(idx) => idx,
                Extend::Trapped => {
                    let full = |t: &Tree<N>| t.len >= self.params.max_nodes as usize;
                    if full(grow) && full(other) {
                        self.status = CuPlanStatus::Exhausted;
                    }
                    continue;
                }
            };
            let target = grow.nodes[added];
            if let Some(joined) = other.connect(&self.validity, &target, &self.params) {
                if a == 0 {
                    self.finish(added, joined);
                } else {
                    self.finish(joined, added);
                }
            }
        }
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        write_header(out, &self.validity, self.status, self.iterations);
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
