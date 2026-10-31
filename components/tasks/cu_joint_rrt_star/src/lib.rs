//! Deterministic, allocation-free $N$-DOF joint-space RRT* for `copper-rs`.
//!
//! [`CuJointRrtStar`] is the sampling-based implementation of
//! [`cu_motion_planning::CuJointPlanner`]: `begin` validates the request and
//! seeds the tree (connecting start and goal directly when the line is free),
//! and each `step` runs a bounded number of RRT* iterations with informed
//! pruning, neighbourhood rewiring, and line-of-sight shortcutting on output.
//! All randomness comes from the job seed, so replay reproduces every plan.
//! [`CuJointRrtStarTask`] is the generic planner task running RRT*; its node
//! config is [`cu_motion_planning::CuJointPlannerParams`] with a
//! [`CuJointRrtStarParams`] `params` block:
//!
//! ```ron
//! (
//!     id: "planner",
//!     type: "cu_joint_rrt_star::CuJointRrtStarTask<6>",
//!     config: {
//!         "chain": "igus_rebel",
//!         "collision_model": "igus_rebel",
//!         "params": {"step_size": 0.3, "goal_bias": 0.2, "max_nodes": 200},
//!         "base_budget": 64,
//!         "refine_budget": 32,
//!     },
//! )
//! ```
//!
//! Request and path payloads ([`cu_motion_planning::CuJointPlanRequest`],
//! [`cu_motion_planning::CuJointPlanPath`]) come from `cu-motion-planning`.

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
use cu_motion_planning::CuJointPlanPath;
use cu_motion_planning::CuJointPlanRequest;
use cu_motion_planning::CuJointPlanner;
use cu_motion_planning::CuJointPlannerTask;
use cu_motion_planning::CuPlanStatus;
use cu_motion_planning::CuPlannerKind;
use cu_motion_planning::CuSceneValidity;
use cu_motion_planning::MAX_EXCLUSIONS;
use cu_motion_planning::MAX_WAYPOINTS;
use cu_motion_planning::joint_distance;
use cu_rng::prelude::*;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// [`CuJointPlannerTask`] running [`CuJointRrtStar`].
pub type CuJointRrtStarTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointRrtStar<N, M>, N, M>;

/// Fixed capacity of tree nodes in [`CuJointRrtStar`].
pub const RRT_STAR_MAX_NODES: usize = 256;
const CONVERGED_QUALITY: f32 = 0.999;

fn is_converged_quality(q: Quality) -> bool {
    q.raw() >= CONVERGED_QUALITY
}

/// Parameters of [`CuJointRrtStar`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointRrtStarParams {
    /// Maximum joint-space step per extension (dimensionless, metric-scaled).
    pub step_size: f64,
    /// Probability in `[0.0, 1.0]` of sampling the exact goal configuration
    /// (dimensionless).
    pub goal_bias: f64,
    /// Joint-space distance within which a node can connect directly to the
    /// goal (dimensionless, metric-scaled).
    pub goal_threshold: f64,
    /// Neighbourhood rewiring radius (dimensionless, metric-scaled).
    pub rewire_radius: f64,
    /// Angular/linear resolution for discrete edge collision sampling.
    pub collision_check_step: CuJointResolution,
    /// Maximum active tree nodes (`>= 2`; capped at [`RRT_STAR_MAX_NODES`]).
    pub max_nodes: u32,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointRrtStarParams {
    fn default() -> Self {
        Self {
            step_size: 0.35,
            goal_bias: 0.15,
            goal_threshold: 0.40,
            rewire_radius: 0.75,
            collision_check_step: CuJointResolution::default(),
            max_nodes: RRT_STAR_MAX_NODES as u32,
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointRrtStarParams {
    /// Checks every parameter, the metric, and the collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()?;
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(self.step_size)
            || !positive(self.rewire_radius)
            || !positive(self.goal_threshold)
        {
            return Err(CuError::from(
                "cu-joint-rrt-star: 'step_size', 'rewire_radius', and 'goal_threshold' must be positive and finite",
            ));
        }
        if !(self.goal_bias.is_finite() && (0.0..=1.0).contains(&self.goal_bias)) {
            return Err(CuError::from(
                "cu-joint-rrt-star: 'goal_bias' must be in [0, 1]",
            ));
        }
        if self.max_nodes < 2 {
            return Err(CuError::from("cu-joint-rrt-star: 'max_nodes' must be >= 2"));
        }
        Ok(())
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
///
/// Built with [`CuJointPlanner::new`] from robot models and
/// [`CuJointRrtStarParams`] (invalid parameters fail there); runs in
/// [`CuJointRrtStarTask`].
pub struct CuJointRrtStar<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointRrtStarParams,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    straight_line_cost: f64,
    nodes: [RrtNode<N>; RRT_STAR_MAX_NODES],
    node_count: usize,
    goal_node_idx: Option<usize>,
    best_cost: f64,
    iterations: u32,
    rng: CuRng,
    status: CuPlanStatus,
}

impl<const N: usize, const M: usize> CuJointRrtStar<N, M> {
    /// The validated planner parameters.
    pub const fn params(&self) -> &CuJointRrtStarParams {
        &self.params
    }

    /// Updates the job status after tree changes.
    fn refresh_status(&mut self) {
        if self.status.is_final() && !self.status.has_path() {
            return;
        }
        let cap = (self.params.max_nodes as usize).min(RRT_STAR_MAX_NODES);
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
        let cap = (self.params.max_nodes as usize).min(RRT_STAR_MAX_NODES);
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
        let cap = (self.params.max_nodes as usize).min(RRT_STAR_MAX_NODES);
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

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointRrtStar<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Sampling;

    type Params = CuJointRrtStarParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointRrtStarParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            validity: CuSceneValidity::new(chain, collision_model, params.collision_check_step),
            params,
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
            straight_line_cost: 0.0,
            nodes: [RrtNode::default(); RRT_STAR_MAX_NODES],
            node_count: 0,
            goal_node_idx: None,
            best_cost: f64::INFINITY,
            iterations: 0,
            rng: CuRng::from_seed(0),
            status: CuPlanStatus::InvalidRequest,
        })
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
        self.status = if !self.validity.capture(&request.scene, &request.exclusions) {
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
    use cu_collision::CuPlanningScene;
    use cu_collision::CuSphere;
    use cu_collision::igus_rebel_collision_model;
    use cu_kinematics::igus_rebel_chain;
    use cu_motion_planning::MAX_ATTACHED_SHAPES;
    use cu_motion_planning::MAX_OBSTACLES;
    use cu_motion_planning::planner_resources;
    use cu_spatial_payloads::Transform3D;
    use cu29::cutask_anytime::AnytimeStatus;
    use cu29::cutask_anytime::CuAnytimeTask;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle as Angle64;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;

    #[test]
    fn planner_initialization_requires_explicit_serial_models() {
        let resources = || planner_resources::Resources {
            rng: Owned(CuRng::from_seed(1)),
        };
        assert!(CuJointRrtStarTask::<1>::new(None, resources()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]},"collision_model": {"shapes": [{"shape": {"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}, "frame": 0}], "safety_margin": 0.01}}"#).unwrap();
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
        CuJointRrtStarTask::from_planner(
            CuJointRrtStar::new(
                igus_rebel_chain(),
                igus_rebel_collision_model(),
                CuJointRrtStarParams::default(),
            )
            .unwrap(),
            64,
            32,
            seed,
        )
        .unwrap()
    }

    #[test]
    fn preset_models_and_nested_params_configure_the_task() {
        let resources = || planner_resources::Resources {
            rng: Owned(CuRng::from_seed(1)),
        };
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"chain": "igus_rebel", "collision_model": "igus_rebel",
                "params": {"step_size": 0.3, "goal_bias": 0.2}, "refine_budget": 8}"#,
        )
        .unwrap();
        let task = CuJointRrtStarTask::<6>::new(Some(&cfg), resources()).unwrap();
        assert_eq!(task.planner().params().step_size, 0.3);
        assert_eq!(task.planner().params().goal_bias, 0.2);
        // Invalid parameters fail at construction, not at the first job.
        for bad in [
            r#"{"chain": "igus_rebel", "collision_model": "igus_rebel", "params": {"goal_bias": 1.5}}"#,
            r#"{"chain": "igus_rebel", "collision_model": "igus_rebel", "params": {"step_size": 0.0}}"#,
            r#"{"chain": "igus_rebel", "collision_model": "igus_rebel", "params": {"stepsize": 0.3}}"#,
            r#"{"chain": "igus_rebel", "collision_model": "igus_rebel", "step_size": 0.3}"#,
            r#"{"chain": "igus_rebel", "collision_model": "igus_rebel", "base_budget": 0}"#,
            r#"{"chain": "igus_rebel"}"#,
        ] {
            let cfg: ComponentConfig =
                ron::from_str(&format!("#![enable(unwrap_newtypes)] {bad}")).unwrap();
            assert!(
                CuJointRrtStarTask::<6>::new(Some(&cfg), resources()).is_err(),
                "{bad}"
            );
        }
        let invalid = CuJointRrtStarParams {
            goal_bias: -0.1,
            ..Default::default()
        };
        assert!(invalid.validate().is_err());
        assert!(
            CuJointRrtStar::<6>::new(igus_rebel_chain(), igus_rebel_collision_model(), invalid)
                .is_err()
        );
    }

    #[test]
    fn identical_seeds_reproduce_identical_paths() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let run = |seed| {
            let mut task = test_task(seed);
            let req = CuMsg::new(Some(obstructed_request()));
            let mut out = CuMsg::new(None);
            task.base(&ctx, &req, &mut out).unwrap();
            for _ in 0..4 {
                task.refine(&ctx, &mut out).unwrap();
            }
            out.payload().cloned()
        };
        assert_eq!(run(11), run(11));
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

    fn obstructed_request() -> CuJointPlanRequest<6> {
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
            .add_obstacle(
                CuCollisionShape::Sphere(
                    CuSphere::new(
                        [m[0][3], m[1][3], m[2][3]].map(Length::new::<meter>),
                        Length::new::<meter>(0.08),
                    )
                    .unwrap(),
                ),
                None,
            )
            .unwrap();

        CuJointPlanRequest {
            scene,
            exclusions: CuSceneExclusions::new(),
            start,
            goal,
        }
    }

    #[test]
    fn test_obstructed_joint_plan_refines_around_obstacle() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = test_task(7);
        let req = CuMsg::new(Some(obstructed_request()));
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
        let verify_env = req.payload().unwrap().scene.collision_env();
        for w in 0..(path.len as usize - 1) {
            assert!(
                model
                    .is_segment_collision_free(
                        &chain,
                        &path.waypoints[w].positions,
                        &path.waypoints[w + 1].positions,
                        Angle64::new::<radian>(0.05),
                        &verify_env,
                    )
                    .unwrap()
            );
        }
    }
}
