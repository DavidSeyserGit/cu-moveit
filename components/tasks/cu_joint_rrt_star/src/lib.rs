//! Deterministic, allocation-free $N$-DOF joint-space RRT* for `copper-rs`.
//!
//! [`CuJointRrtStar`] is the sampling-based implementation of
//! [`cu_motion_planning::CuJointPlanner`]: `begin` validates the request and
//! seeds the tree (connecting start and goal directly when the line is free),
//! and each `step` runs a bounded number of RRT* iterations with informed
//! pruning, neighbourhood rewiring, and line-of-sight shortcutting on output.
//! All randomness comes from the job seed, so replay reproduces every plan.
//! [`CuJointRrtStarTask`] is the generic planner task running RRT*.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

use bincode::Decode;
use bincode::Encode;
use cu_collision::CuRobotCollisionModel;
use cu_collision::CuSceneExclusions;
use cu_collision::CuSceneRevision;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu_motion_planning::CuJointPlanner;
use cu_motion_planning::CuJointPlannerTask;
use cu_motion_planning::CuPlannerKind;
use cu_motion_planning::CuSceneValidity;
use cu_motion_planning::joint_distance;
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

pub use cu_motion_planning::CuJointPlanPath;
pub use cu_motion_planning::CuJointPlanRequest;
pub use cu_motion_planning::CuJointPlanner as _;
pub use cu_motion_planning::CuPlanStatus;
pub use cu_motion_planning::MAX_ATTACHED_SHAPES;
pub use cu_motion_planning::MAX_EXCLUSIONS;
pub use cu_motion_planning::MAX_OBSTACLES;
pub use cu_motion_planning::MAX_WAYPOINTS;
pub use cu_motion_planning::task::planner_resources;

/// The generic planner task running RRT*.
pub type CuJointRrtStarTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointRrtStar<N, M>, N>;

/// Fixed capacity of tree nodes in `CuJointRrtStar`.
pub const MAX_NODES: usize = 256;
const CONVERGED_QUALITY: f32 = 0.999;

fn is_converged_quality(q: Quality) -> bool {
    q.raw() >= CONVERGED_QUALITY
}

/// Configuration parameters for joint-space RRT* planning.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct CuJointRrtParams {
    /// Maximum dimensionless joint-space L2 step per extension, using `metric`.
    pub step_size: f64,
    /// Probability in `[0.0, 1.0]` of sampling the exact goal configuration.
    pub goal_bias: f64,
    /// Joint-space $L_2$ distance within which a node can connect directly to the goal.
    pub goal_threshold: f64,
    /// RRT* neighborhood rewiring radius in joint space.
    pub rewire_radius: f64,
    /// Angular/linear resolution for discrete edge collision sampling.
    pub collision_check_step: CuJointResolution,
    /// Maximum active tree nodes (`<= MAX_NODES`).
    pub max_nodes: u32,
    /// Scaling for dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointRrtParams {
    fn default() -> Self {
        Self {
            step_size: 0.35,
            goal_bias: 0.15,
            goal_threshold: 0.40,
            rewire_radius: 0.75,
            collision_check_step: CuJointResolution::default(),
            max_nodes: MAX_NODES as u32,
            metric: CuJointMetric::default(),
        }
    }
}

#[derive(Clone, Copy)]
struct RrtNode<const N: usize> {
    q: [CuJointPosition; N],
    parent: u16,
    cost: f64,
}

impl<const N: usize> Default for RrtNode<N> {
    fn default() -> Self {
        Self {
            q: [CuJointPosition::default(); N],
            parent: u16::MAX,
            cost: 0.0,
        }
    }
}

/// Allocation-free fixed-capacity joint-space RRT* solver.
pub struct CuJointRrtStar<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointRrtParams,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    straight_line_cost: f64,
    nodes: [RrtNode<N>; MAX_NODES],
    node_count: usize,
    goal_node_idx: Option<usize>,
    best_cost: f64,
    iterations: u32,
    rng: CuRng,
    status: CuPlanStatus,
}

impl<const N: usize, const M: usize> CuJointRrtStar<N, M> {
    /// Creates an idle RRT* solver for explicit robot models; start a job
    /// with [`CuJointPlanner::begin`].
    pub fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointRrtParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, collision_model, params.collision_check_step),
            params,
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
            straight_line_cost: 0.0,
            nodes: [RrtNode::default(); MAX_NODES],
            node_count: 0,
            goal_node_idx: None,
            best_cost: f64::INFINITY,
            iterations: 0,
            rng: CuRng::from_seed(0),
            status: CuPlanStatus::InvalidRequest,
        }
    }

    fn params_valid(&self) -> bool {
        let p = &self.params;
        p.metric.validate().is_ok()
            && p.step_size.is_finite()
            && p.step_size > 0.0
            && p.rewire_radius.is_finite()
            && p.rewire_radius > 0.0
            && p.goal_threshold.is_finite()
            && p.goal_threshold > 0.0
            && p.goal_bias.is_finite()
            && (0.0..=1.0).contains(&p.goal_bias)
            && p.max_nodes >= 2
    }

    /// Updates the job status after tree changes.
    fn refresh_status(&mut self) {
        if self.status.is_final() && !self.status.has_path() {
            return;
        }
        let cap = (self.params.max_nodes as usize).min(MAX_NODES);
        let done = self.node_count >= cap || is_converged_quality(self.quality());
        self.status = match (self.has_solution(), done) {
            (true, true) => CuPlanStatus::Converged,
            (true, false) => CuPlanStatus::Improving,
            (false, true) => CuPlanStatus::Exhausted,
            (false, false) => CuPlanStatus::Searching,
        };
    }

    /// Runs `budget` bounded RRT* iterations in place.
    pub fn grow(&mut self, budget: u32) {
        if self.status.is_final() {
            return;
        }
        let cap = (self.params.max_nodes as usize).min(MAX_NODES);
        let step = self.params.step_size;
        let goal_threshold = self.params.goal_threshold;
        let rewire_radius = self.params.rewire_radius;

        let mut it = 0;
        while it < budget {
            if self.node_count >= cap {
                break;
            }
            self.iterations += 1;
            it += 1;

            let q_sample = self.sample_config();
            let nearest_idx = self.nearest_node(&q_sample);
            let q_new = steer(
                &self.nodes[nearest_idx].q,
                &q_sample,
                step,
                self.params.metric,
            );

            // Informed pruning: skip configurations that cannot possibly improve best_cost
            let min_possible = joint_distance(&self.start, &q_new, self.params.metric)
                + joint_distance(&q_new, &self.goal, self.params.metric);
            if min_possible >= self.best_cost {
                continue;
            }

            if !self.validity.is_configuration_free(&q_new) {
                continue;
            }

            // Choose best parent inside `rewire_radius` neighborhood
            let mut best_parent = nearest_idx;
            let mut best_new_cost = self.nodes[nearest_idx].cost
                + joint_distance(&self.nodes[nearest_idx].q, &q_new, self.params.metric);

            if !self
                .validity
                .is_segment_free(&self.nodes[nearest_idx].q, &q_new)
            {
                continue;
            }

            let mut idx = 0;
            while idx < self.node_count {
                if idx != nearest_idx {
                    let d = joint_distance(&self.nodes[idx].q, &q_new, self.params.metric);
                    if d <= rewire_radius {
                        let cand_cost = self.nodes[idx].cost + d;
                        if cand_cost < best_new_cost
                            && self.validity.is_segment_free(&self.nodes[idx].q, &q_new)
                        {
                            best_parent = idx;
                            best_new_cost = cand_cost;
                        }
                    }
                }
                idx += 1;
            }

            let new_idx = self.node_count;
            self.nodes[new_idx] = RrtNode {
                q: q_new,
                parent: best_parent as u16,
                cost: best_new_cost,
            };
            self.node_count += 1;

            // Rewire neighbors through `new_idx` if cheaper. Skipping ancestors of
            // `new_idx` keeps the parent links acyclic.
            let mut rewired = false;
            let mut nbr = 0;
            while nbr < new_idx {
                if nbr != best_parent {
                    let d = joint_distance(&q_new, &self.nodes[nbr].q, self.params.metric);
                    if d <= rewire_radius
                        && best_new_cost + d < self.nodes[nbr].cost
                        && !self.is_ancestor_of(nbr, new_idx)
                        && self.validity.is_segment_free(&q_new, &self.nodes[nbr].q)
                    {
                        self.nodes[nbr].parent = new_idx as u16;
                        self.nodes[nbr].cost = best_new_cost + d;
                        rewired = true;
                    }
                }
                nbr += 1;
            }

            // Propagate rewired costs to descendants so tree costs stay consistent.
            if rewired {
                let mut pass = 0;
                loop {
                    let mut changed = false;
                    let mut i = 0;
                    while i < self.node_count {
                        let p = self.nodes[i].parent;
                        if p != u16::MAX {
                            let pu = p as usize;
                            let c = self.nodes[pu].cost
                                + joint_distance(
                                    &self.nodes[pu].q,
                                    &self.nodes[i].q,
                                    self.params.metric,
                                );
                            if c < self.nodes[i].cost - 1e-12 {
                                self.nodes[i].cost = c;
                                changed = true;
                            }
                        }
                        i += 1;
                    }
                    pass += 1;
                    if !changed || pass >= self.node_count {
                        break;
                    }
                }
                if let Some(g) = self.goal_node_idx
                    && self.nodes[g].cost < self.best_cost
                {
                    self.best_cost = self.nodes[g].cost;
                }
            }

            // Try connecting `new_idx` to `goal` if within `goal_threshold` and capacity allows
            let dist_to_goal = joint_distance(&q_new, &self.goal, self.params.metric);
            if dist_to_goal <= goal_threshold
                && best_new_cost + dist_to_goal < self.best_cost
                && self.validity.is_segment_free(&q_new, &self.goal)
            {
                if let Some(g_idx) = self.goal_node_idx {
                    self.nodes[g_idx].parent = new_idx as u16;
                    self.nodes[g_idx].cost = best_new_cost + dist_to_goal;
                    self.best_cost = best_new_cost + dist_to_goal;
                } else if self.node_count < cap {
                    let g_idx = self.node_count;
                    self.nodes[g_idx] = RrtNode {
                        q: self.goal,
                        parent: new_idx as u16,
                        cost: best_new_cost + dist_to_goal,
                    };
                    self.node_count += 1;
                    self.goal_node_idx = Some(g_idx);
                    self.best_cost = best_new_cost + dist_to_goal;
                }
            }
        }
        self.refresh_status();
    }

    /// Returns `true` if a collision-free path from `start` to `goal` has been found.
    pub const fn has_solution(&self) -> bool {
        self.goal_node_idx.is_some()
    }

    /// Returns `true` if the node buffer is full or the path is already straight-line optimal.
    pub fn is_exhausted(&self) -> bool {
        let cap = (self.params.max_nodes as usize).min(MAX_NODES);
        self.status.is_final() || self.node_count >= cap || is_converged_quality(self.quality())
    }

    /// Returns the best joint-space path cost found so far.
    pub fn best_cost(&self) -> f64 {
        self.best_cost
    }

    /// Returns the number of RRT* iterations executed for the current job.
    pub const fn iterations(&self) -> u32 {
        self.iterations
    }

    /// Returns the number of nodes currently in the tree.
    pub const fn tree_size(&self) -> u32 {
        self.node_count as u32
    }

    /// Returns the scene revision captured for the current job.
    pub const fn scene_revision(&self) -> CuSceneRevision {
        self.validity.scene_revision()
    }

    /// Exclusions captured for the current job.
    pub const fn exclusions(&self) -> &CuSceneExclusions<MAX_EXCLUSIONS> {
        self.validity.exclusions()
    }

    /// Computes the normalized quality in `[0.0, 1.0]` (`straight_line_cost / best_cost`).
    pub fn quality(&self) -> Quality {
        self.quality_of_cost(self.best_cost)
    }

    /// Computes the normalized quality in `[0.0, 1.0]` of an actually emitted path
    /// (`straight_line_cost / cost`).
    pub fn quality_of_cost(&self, cost: f64) -> Quality {
        if !self.has_solution() || !cost.is_finite() {
            return quality_from_f32(0.0);
        }
        if self.straight_line_cost <= 1e-9 || cost <= 1e-9 {
            return quality_from_f32(1.0);
        }
        let ratio = (self.straight_line_cost / cost).clamp(0.0, 1.0);
        quality_from_f32(ratio as f32)
    }

    /// Dimensionless joint-space L2 cost of an emitted waypoint sequence
    /// (`waypoints[..len]`) using the configured metric.
    pub fn path_cost(
        &self,
        waypoints: &[CuJointPositionsPayload<N>; MAX_WAYPOINTS],
        len: u32,
    ) -> f64 {
        let mut total = 0.0_f64;
        let mut i = 0usize;
        while i + 1 < len as usize {
            total += joint_distance(
                &waypoints[i].positions,
                &waypoints[i + 1].positions,
                self.params.metric,
            );
            i += 1;
        }
        total
    }

    /// Extracts and shortcuts the best path in place into `out_waypoints`, returning the
    /// number of waypoints written (`Some(len)`), or `None` if no solution exists or the
    /// shortcut path still exceeds `MAX_WAYPOINTS`.
    pub fn write_path(
        &self,
        out_waypoints: &mut [CuJointPositionsPayload<N>; MAX_WAYPOINTS],
    ) -> Option<u32> {
        let mut anchor = self.goal_node_idx?;

        // Greedy line-of-sight shortcutting while walking the parent chain from the goal
        // back to the start, so raw paths longer than MAX_WAYPOINTS are still shortcut
        // before any capacity rejection. `shortcut[..count]` runs from goal back to start.
        let mut shortcut = [[CuJointPosition::default(); N]; MAX_WAYPOINTS];
        let mut count = 0usize;
        shortcut[count] = self.nodes[anchor].q;
        count += 1;

        let mut steps = 0;
        loop {
            let parent = self.nodes[anchor].parent;
            if parent == u16::MAX {
                break; // `anchor` is the start node
            }
            steps += 1;
            if steps > self.node_count {
                return None; // defensive: parent chain never reached the root
            }
            // Farthest ancestor toward the start reachable from `anchor` in a straight line.
            let mut chosen = parent as usize;
            let mut curr = parent as usize;
            let mut hops = 0;
            while hops <= self.node_count {
                hops += 1;
                let up = self.nodes[curr].parent;
                if up == u16::MAX {
                    break;
                }
                if self
                    .validity
                    .is_segment_free(&self.nodes[anchor].q, &self.nodes[up as usize].q)
                {
                    chosen = up as usize;
                }
                curr = up as usize;
            }
            if count >= MAX_WAYPOINTS {
                return None;
            }
            shortcut[count] = self.nodes[chosen].q;
            count += 1;
            anchor = chosen;
        }

        // Reverse goal->start into start->goal.
        let mut i = 0;
        while i < count {
            out_waypoints[i] = CuJointPositionsPayload {
                positions: shortcut[count - 1 - i],
            };
            i += 1;
        }
        Some(count as u32)
    }

    fn sample_config(&mut self) -> [CuJointPosition; N] {
        let coin: f64 = self.rng.random();
        if coin < self.params.goal_bias {
            return self.goal;
        }
        let joints = self.validity.chain().joints();
        let mut q = [CuJointPosition::default(); N];
        let mut j = 0;
        while j < N {
            let lo = joints[j].limits.lower().si_value();
            let hi = joints[j].limits.upper().si_value();
            let u: f64 = self.rng.random();
            q[j] = joints[j].limits.lower().with_si_value(lo + u * (hi - lo));
            j += 1;
        }
        q
    }

    /// Returns `true` when `maybe_ancestor` lies on the parent chain of `node`.
    fn is_ancestor_of(&self, maybe_ancestor: usize, node: usize) -> bool {
        let mut curr = node;
        let mut guard = 0;
        while curr != u16::MAX as usize && guard <= self.node_count {
            if curr == maybe_ancestor {
                return true;
            }
            curr = self.nodes[curr].parent as usize;
            guard += 1;
        }
        false
    }

    fn nearest_node(&self, q_target: &[CuJointPosition; N]) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::INFINITY;
        let mut i = 0;
        while i < self.node_count {
            // Do not expand out of the goal node itself
            if Some(i) != self.goal_node_idx {
                let d = joint_distance(&self.nodes[i].q, q_target, self.params.metric);
                if d < best_dist {
                    best_dist = d;
                    best_idx = i;
                }
            }
            i += 1;
        }
        best_idx
    }
}

fn steer<const N: usize>(
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
    let mut out = [CuJointPosition::default(); N];
    let mut i = 0;
    while i < N {
        let a = from[i].si_value();
        let b = to[i].si_value();
        out[i] = from[i].with_si_value(a + scale * (b - a));
        i += 1;
    }
    out
}

impl<const N: usize, const M: usize> CuJointPlanner<N> for CuJointRrtStar<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Sampling;

    /// Config keys: `chain`, `collision_model`, and optional `metric`,
    /// `collision_check_step`, `step_size`, `goal_bias`, `goal_threshold`,
    /// `rewire_radius`, `max_nodes`.
    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let chain = CuKinematicChain::from_config(config)?;
        let collision_model = CuRobotCollisionModel::from_config(config)?;
        let mut params = CuJointRrtParams::default();
        if let Some(cfg) = config {
            if let Some(v) = cfg.get_value::<CuJointMetric>("metric")? {
                params.metric = v;
            }
            if let Some(v) = cfg.get_value::<CuJointResolution>("collision_check_step")? {
                params.collision_check_step = v;
            }
            if let Some(v) = cfg.get::<f64>("step_size")? {
                params.step_size = v;
            }
            if let Some(v) = cfg.get::<f64>("goal_bias")? {
                params.goal_bias = v;
            }
            if let Some(v) = cfg.get::<f64>("goal_threshold")? {
                params.goal_threshold = v;
            }
            if let Some(v) = cfg.get::<f64>("rewire_radius")? {
                params.rewire_radius = v;
            }
            if let Some(v) = cfg.get::<u32>("max_nodes")? {
                params.max_nodes = v;
            }
        }
        Ok(Self::new(chain, collision_model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, seed: u64) -> CuResult<CuPlanStatus> {
        self.start = request.start;
        self.goal = request.goal;
        self.straight_line_cost = joint_distance(&self.start, &self.goal, self.params.metric);
        self.node_count = 0;
        self.goal_node_idx = None;
        self.best_cost = f64::INFINITY;
        self.iterations = 0;
        self.rng.reseed(seed);
        self.status = if !self.params_valid()
            || !self.validity.capture(&request.scene, &request.exclusions)
        {
            CuPlanStatus::InvalidRequest
        } else if !self.validity.is_configuration_free(&self.start) {
            CuPlanStatus::InvalidStart
        } else if !self.validity.is_configuration_free(&self.goal) {
            CuPlanStatus::InvalidGoal
        } else {
            CuPlanStatus::Searching
        };
        if self.status != CuPlanStatus::Searching {
            return Ok(self.status);
        }
        self.nodes[0] = RrtNode {
            q: self.start,
            parent: u16::MAX,
            cost: 0.0,
        };
        self.node_count = 1;
        if self.straight_line_cost <= 1e-9 {
            self.goal_node_idx = Some(0);
            self.best_cost = 0.0;
        } else if self.validity.is_segment_free(&self.start, &self.goal) {
            // Direct line of sight: the straight line is optimal.
            self.nodes[1] = RrtNode {
                q: self.goal,
                parent: 0,
                cost: self.straight_line_cost,
            };
            self.node_count = 2;
            self.goal_node_idx = Some(1);
            self.best_cost = self.straight_line_cost;
        }
        self.refresh_status();
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        self.grow(budget);
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        out.len = 0;
        out.cost = f64::INFINITY;
        out.effort = self.iterations;
        out.status = self.status;
        out.scene_revision = self.scene_revision();
        out.exclusions = self.exclusions().clone();
        if !self.status.has_path() {
            return self.status;
        }
        match self.write_path(&mut out.waypoints) {
            Some(len) => {
                out.len = len;
                out.cost = self.path_cost(&out.waypoints, len);
            }
            None => out.status = CuPlanStatus::CapacityExceeded,
        }
        out.status
    }

    fn quality(&self) -> Quality {
        CuJointRrtStar::quality(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_collision::CuCollisionShape;
    use cu_collision::CuCollisionWorld;
    use cu_collision::CuPlanningScene;
    use cu_collision::CuSphere;
    use cu_collision::igus_rebel_collision_model;
    use cu_kinematics::igus_rebel_chain;
    use cu_spatial_payloads::Transform3D;
    use cu29::cutask_anytime::AnytimeStatus;
    use cu29::cutask_anytime::CuAnytimeTask;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle as Angle64;

    #[test]
    fn planner_initialization_requires_explicit_serial_models() {
        let resources = || planner_resources::Resources {
            rng: Owned(CuRng::from_seed(1)),
        };
        assert!(CuJointRrtStarTask::<1>::new(None, resources()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]},"collision_model": {"link_shapes": [{"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}], "safety_margin": 0.01}}"#).unwrap();
        let mut task = CuJointRrtStarTask::<1>::new(Some(&cfg), resources()).unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        let request = CuMsg::new(Some(CuJointPlanRequest {
            scene: CuPlanningScene::new(),
            exclusions: CuSceneExclusions::new(),
            start: [Angle64::new::<radian>(0.0).into()],
            goal: [Angle64::new::<radian>(0.5).into()],
        }));
        let mut output = CuMsg::new(None);
        task.base(&ctx, &request, &mut output).unwrap();
        assert_eq!(output.payload().unwrap().len, 2);
    }

    fn test_task(seed: u64) -> CuJointRrtStarTask<6> {
        CuJointRrtStarTask::new_with_planner(
            CuJointRrtStar::new(
                igus_rebel_chain(),
                igus_rebel_collision_model(),
                CuJointRrtParams::default(),
            ),
            64,
            32,
            seed,
        )
    }

    #[test]
    fn test_unobstructed_joint_plan_converges_in_base() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = test_task(42);

        let start = [Angle64::new::<radian>(0.0).into(); 6];
        let goal = [
            Angle64::new::<radian>(0.5).into(),
            Angle64::new::<radian>(0.3).into(),
            Angle64::new::<radian>(-0.3).into(),
            Angle64::new::<radian>(0.2).into(),
            Angle64::new::<radian>(-0.2).into(),
            Angle64::new::<radian>(0.4).into(),
        ];

        let req = CuMsg::new(Some(CuJointPlanRequest {
            scene: CuPlanningScene::new(),
            exclusions: CuSceneExclusions::new(),
            start,
            goal,
        }));
        let mut out = CuMsg::new(None);

        let status = task.base(&ctx, &req, &mut out).unwrap();
        assert!(matches!(status, AnytimeStatus::Converged(_)));
        let path = out.payload().unwrap();
        assert_eq!(path.len, 2);
        assert_eq!(path.waypoints[0].positions, start);
        assert_eq!(path.waypoints[1].positions, goal);
    }

    #[test]
    fn test_obstructed_joint_plan_refines_around_obstacle() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = test_task(7);

        let start = [
            Angle64::new::<radian>(-0.6).into(),
            Angle64::new::<radian>(0.7).into(),
            Angle64::new::<radian>(-0.2).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
        ];
        let goal = [
            Angle64::new::<radian>(0.6).into(),
            Angle64::new::<radian>(0.7).into(),
            Angle64::new::<radian>(-0.2).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
        ];

        // Midpoint q_mid = [0.0, 0.7, -0.2, 0, 0, 0]: compute its end-effector position and place
        // an obstacle sphere there so the straight-line joint path collides, forcing RRT* to route around!
        let q_mid = [
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.7).into(),
            Angle64::new::<radian>(-0.2).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
        ];
        let mut mid_ee = Transform3D::<f64>::identity();
        igus_rebel_chain()
            .forward_kinematics(&q_mid, &mut mid_ee)
            .unwrap();
        let m = mid_ee.to_matrix();

        let mut scene = CuPlanningScene::<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>::new();
        scene
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [m[0][3], m[1][3], m[2][3]],
                0.08,
            )))
            .unwrap();

        let req = CuMsg::new(Some(CuJointPlanRequest {
            scene,
            exclusions: CuSceneExclusions::new(),
            start,
            goal,
        }));
        let mut out = CuMsg::new(None);

        let status = task.base(&ctx, &req, &mut out).unwrap();
        assert!(!matches!(status, AnytimeStatus::Aborted));

        let mut best_cost = f64::INFINITY;
        for _ in 0..8 {
            let st = task.refine(&ctx, &mut out).unwrap();
            assert!(!matches!(st, AnytimeStatus::Aborted));
            if let Some(p) = out.payload() {
                let c = p.cost;
                assert!(c <= best_cost + 1e-9);
                best_cost = c;
            }
        }

        let path = out
            .payload()
            .expect("RRT* should find a collision-free path around the obstacle");
        assert!(path.len >= 3);

        // Verify every segment of the published path is collision-free
        let model = igus_rebel_collision_model();
        let chain = igus_rebel_chain();
        let mut verify_world = CuCollisionWorld::<MAX_OBSTACLES>::new();
        req.payload()
            .unwrap()
            .scene
            .write_collision_world(&mut verify_world)
            .unwrap();
        for w in 0..(path.len as usize - 1) {
            assert!(
                model
                    .is_segment_collision_free(
                        &chain,
                        &path.waypoints[w].positions,
                        &path.waypoints[w + 1].positions,
                        Angle64::new::<radian>(0.05),
                        Some(&verify_world),
                    )
                    .unwrap()
            );
        }
    }
}
