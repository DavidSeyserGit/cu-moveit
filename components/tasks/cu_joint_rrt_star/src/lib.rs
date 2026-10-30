//! Deterministic, allocation-free $N$-DOF joint-space RRT* anytime motion planner for `copper-rs`.
//!
//! [`CuJointRrtStarTask`] consumes a [`CuJointPlanRequest`] and publishes a [`CuJointPlanPath`].
//! `base()` runs the initial iteration budget and commits the first valid collision-free path;
//! each `refine()` runs one bounded quantum of RRT* iterations and updates the output only when
//! the joint-space path cost strictly improves.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_collision::CuCollisionReport;
use cu_collision::CuCollisionShape;
use cu_collision::CuCollisionWorld;
use cu_collision::CuPlanningScene;
use cu_collision::CuRobotCollisionModel;
use cu_collision::CuSceneExclusions;
use cu_collision::CuSceneRevision;
use cu_collision::scene_query::CuSceneQueryExclusions;
use cu_collision::scene_query::check_attached_configuration;
use cu_kinematics::CuJointMetric;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu_rng::prelude::*;
use cu29::cutask_anytime::AnytimeStatus;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use cu29::resources;
use serde::Deserialize;
use serde::Serialize;

/// Fixed capacity of tree nodes in `CuJointRrtStar`.
pub const MAX_NODES: usize = 256;
/// Fixed capacity of waypoints in a published `CuJointPlanPath`.
pub const MAX_WAYPOINTS: usize = 32;
/// Fixed capacity of scene obstacles in `CuJointPlanRequest`.
pub const MAX_OBSTACLES: usize = 16;
/// Fixed capacity of attached shapes in `CuJointPlanRequest`.
pub const MAX_ATTACHED_SHAPES: usize = 8;
/// Fixed capacity of stable-ID exclusions in `CuJointPlanRequest`.
pub const MAX_EXCLUSIONS: usize = 16;

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

/// Joint-space motion planning request payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlanRequest<const N: usize> {
    pub scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    pub exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub start: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
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

/// Planned collision-free joint-space trajectory payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlanPath<const N: usize> {
    pub waypoints: [CuJointPositionsPayload<N>; MAX_WAYPOINTS],
    /// Number of valid waypoints in `waypoints[..len]`.
    pub len: u32,
    /// Dimensionless joint-space L2 cost using the configured metric.
    pub cost: f64,
    /// Total RRT* iterations executed on this job.
    pub iterations: u32,
    /// Scene revision the published path was planned against.
    pub scene_revision: CuSceneRevision,
    /// Collision exclusions captured for this plan; compare with the current set.
    pub exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
}

impl<const N: usize> Default for CuJointPlanPath<N> {
    fn default() -> Self {
        Self {
            waypoints: core::array::from_fn(|_| CuJointPositionsPayload::default()),
            len: 0,
            cost: f64::INFINITY,
            iterations: 0,
            scene_revision: CuSceneRevision::INITIAL,
            exclusions: CuSceneExclusions::new(),
        }
    }
}

/// Compact debug state exposed by `CuJointRrtStarTask` for remote inspection without serializing the full tree.
#[derive(Default, Debug, Reflect)]
pub struct CuJointPlannerDebugState {
    pub iterations: u32,
    pub tree_size: u32,
    pub best_cost: f64,
    pub published_cost: f64,
    pub published_quality: Quality,
    pub scene_revision: CuSceneRevision,
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
    chain: CuKinematicChain<N>,
    collision_model: CuRobotCollisionModel<N, M>,
    params: CuJointRrtParams,
    scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    world: CuCollisionWorld<MAX_OBSTACLES>,
    attached_shapes: [CuCollisionShape; MAX_ATTACHED_SHAPES],
    attached_frames: [Option<usize>; MAX_ATTACHED_SHAPES],
    attached_len: usize,
    query_exclusions: CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    straight_line_cost: f64,
    nodes: [RrtNode<N>; MAX_NODES],
    node_count: usize,
    goal_node_idx: Option<usize>,
    best_cost: f64,
    iterations: u32,
    rng: CuRng,
    valid_endpoints: bool,
}

impl<const N: usize, const M: usize> CuJointRrtStar<N, M> {
    /// Creates and initializes a new joint-space RRT* solver for `(start, goal)`.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointRrtParams,
        scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
        exclusions: &CuSceneExclusions<MAX_EXCLUSIONS>,
        start: [CuJointPosition; N],
        goal: [CuJointPosition; N],
        seed: u64,
    ) -> Self {
        let mut solver = Self {
            chain,
            collision_model,
            params,
            scene: CuPlanningScene::new(),
            world: CuCollisionWorld::new(),
            attached_shapes: [CuCollisionShape::default(); MAX_ATTACHED_SHAPES],
            attached_frames: [None; MAX_ATTACHED_SHAPES],
            attached_len: 0,
            query_exclusions: CuSceneQueryExclusions::new(),
            exclusions: CuSceneExclusions::new(),
            start,
            goal,
            straight_line_cost: 0.0,
            nodes: [RrtNode::default(); MAX_NODES],
            node_count: 0,
            goal_node_idx: None,
            best_cost: f64::INFINITY,
            iterations: 0,
            rng: CuRng::from_seed(seed),
            valid_endpoints: false,
        };
        solver.reset(scene, exclusions, start, goal, seed);
        solver
    }

    /// Resets the solver in place for a new job without allocating.
    pub fn reset(
        &mut self,
        scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
        exclusions: &CuSceneExclusions<MAX_EXCLUSIONS>,
        start: [CuJointPosition; N],
        goal: [CuJointPosition; N],
        seed: u64,
    ) {
        self.scene = scene;
        self.exclusions = exclusions.clone();
        self.start = start;
        self.goal = goal;
        self.straight_line_cost = joint_distance(&start, &goal, self.params.metric);
        self.node_count = 0;
        self.goal_node_idx = None;
        self.best_cost = f64::INFINITY;
        self.iterations = 0;
        self.rng.reseed(seed);

        let scene_ok = self.capture_scene(exclusions);
        let start_ok = scene_ok && self.is_scene_configuration_free(&self.start);
        let goal_ok = scene_ok && self.is_scene_configuration_free(&self.goal);
        self.valid_endpoints = scene_ok
            && start_ok
            && goal_ok
            && self.params.metric.validate().is_ok()
            && self.params.step_size.is_finite()
            && self.params.step_size > 0.0
            && self.params.rewire_radius.is_finite()
            && self.params.rewire_radius > 0.0
            && self.params.goal_threshold.is_finite()
            && self.params.goal_threshold > 0.0
            && self.params.goal_bias.is_finite()
            && (0.0..=1.0).contains(&self.params.goal_bias)
            && self.params.max_nodes >= 2
            && self.params.collision_check_step.metric().validate().is_ok();

        if self.valid_endpoints {
            self.nodes[0] = RrtNode {
                q: self.start,
                parent: u16::MAX,
                cost: 0.0,
            };
            self.node_count = 1;

            if self.straight_line_cost <= 1e-9 {
                self.goal_node_idx = Some(0);
                self.best_cost = 0.0;
            } else if self.is_scene_segment_free(&self.start, &self.goal) {
                // Direct line-of-sight connection from start to goal!
                self.nodes[1] = RrtNode {
                    q: self.goal,
                    parent: 0,
                    cost: self.straight_line_cost,
                };
                self.node_count = 2;
                self.goal_node_idx = Some(1);
                self.best_cost = self.straight_line_cost;
            }
        }
    }

    /// Runs `budget` bounded RRT* iterations in place.
    pub fn grow(&mut self, budget: u32) {
        if !self.valid_endpoints {
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

            if !self.is_scene_configuration_free(&q_new) {
                continue;
            }

            // Choose best parent inside `rewire_radius` neighborhood
            let mut best_parent = nearest_idx;
            let mut best_new_cost = self.nodes[nearest_idx].cost
                + joint_distance(&self.nodes[nearest_idx].q, &q_new, self.params.metric);

            if !self.is_scene_segment_free(&self.nodes[nearest_idx].q, &q_new) {
                continue;
            }

            let mut idx = 0;
            while idx < self.node_count {
                if idx != nearest_idx {
                    let d = joint_distance(&self.nodes[idx].q, &q_new, self.params.metric);
                    if d <= rewire_radius {
                        let cand_cost = self.nodes[idx].cost + d;
                        if cand_cost < best_new_cost
                            && self.is_scene_segment_free(&self.nodes[idx].q, &q_new)
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
                        && self.is_scene_segment_free(&q_new, &self.nodes[nbr].q)
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
                && self.is_scene_segment_free(&q_new, &self.goal)
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
    }

    /// Returns `true` if a collision-free path from `start` to `goal` has been found.
    pub const fn has_solution(&self) -> bool {
        self.goal_node_idx.is_some()
    }

    /// Returns `true` if the node buffer is full or the path is already straight-line optimal.
    pub fn is_exhausted(&self) -> bool {
        let cap = (self.params.max_nodes as usize).min(MAX_NODES);
        !self.valid_endpoints || self.node_count >= cap || is_converged_quality(self.quality())
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
        self.scene.revision()
    }

    /// Captures the scene snapshot into query caches, returning `false` when invalid.
    ///
    /// Stable-ID `exclusions` resolve against the captured snapshot; invalid
    /// references invalidate the whole job.
    fn capture_scene(&mut self, exclusions: &CuSceneExclusions<MAX_EXCLUSIONS>) -> bool {
        if self.scene.write_collision_world(&mut self.world).is_err() {
            self.attached_len = 0;
            return false;
        }
        let attached = self.scene.attached_shapes();
        if attached.len() > MAX_ATTACHED_SHAPES {
            self.attached_len = 0;
            return false;
        }
        let mut i = 0;
        while i < attached.len() {
            if attached[i].link_index >= N {
                self.attached_len = 0;
                return false;
            }
            self.attached_shapes[i] = attached[i].shape;
            self.attached_frames[i] = Some(attached[i].link_index);
            i += 1;
        }
        while i < MAX_ATTACHED_SHAPES {
            self.attached_shapes[i] = CuCollisionShape::default();
            self.attached_frames[i] = None;
            i += 1;
        }
        self.attached_len = attached.len();
        let mut resolved = CuSceneQueryExclusions::new();
        if exclusions
            .resolve(
                &self.scene,
                self.collision_model.shape_count(),
                &mut resolved,
            )
            .is_err()
        {
            self.attached_len = 0;
            return false;
        }
        self.query_exclusions = resolved;
        true
    }

    /// Returns `true` when `q` is within limits and free of self, environment,
    /// and attached-shape collisions against the captured scene snapshot.
    fn is_scene_configuration_free(&self, q: &[CuJointPosition; N]) -> bool {
        if !self
            .collision_model
            .is_configuration_collision_free(&self.chain, q, Some(&self.world))
            .unwrap_or(false)
        {
            return false;
        }
        if self.attached_len == 0 {
            return true;
        }
        let mut attached_world_shapes = [CuCollisionShape::default(); MAX_ATTACHED_SHAPES];
        let mut robot_world_shapes = [CuCollisionShape::default(); M];
        let mut report = CuCollisionReport::<0>::default();
        check_attached_configuration(
            &self.collision_model,
            &self.chain,
            q,
            &self.attached_shapes,
            &self.attached_frames,
            self.attached_len,
            Some(&self.world),
            &self.query_exclusions,
            &mut attached_world_shapes,
            &mut robot_world_shapes,
            &mut report,
        )
        .is_ok()
            && report.is_collision_free()
    }

    /// Returns `true` when the interpolated segment from `a` to `b` is collision-free
    /// against the captured scene snapshot at the configured resolution.
    fn is_scene_segment_free(&self, a: &[CuJointPosition; N], b: &[CuJointPosition; N]) -> bool {
        let metric = self.params.collision_check_step.metric();
        if metric.validate().is_err() {
            return false;
        }
        let mut max_delta = 0.0_f64;
        let mut j = 0;
        while j < N {
            if self.chain.joints()[j].validate_coordinate(a[j]).is_err()
                || self.chain.joints()[j].validate_coordinate(b[j]).is_err()
            {
                return false;
            }
            let delta = (metric.normalized(b[j]) - metric.normalized(a[j])).abs();
            if delta > max_delta {
                max_delta = delta;
            }
            j += 1;
        }
        if max_delta > 1_000_000.0 {
            return false;
        }
        let steps = (libm::ceil(max_delta) as usize).max(1);
        let mut s = 0;
        while s <= steps {
            let alpha = (s as f64) / (steps as f64);
            let mut q_interp = [CuJointPosition::default(); N];
            let mut k = 0;
            while k < N {
                match a[k].interpolate(b[k], alpha) {
                    Ok(q) => q_interp[k] = q,
                    Err(_) => return false,
                }
                k += 1;
            }
            if !self.is_scene_configuration_free(&q_interp) {
                return false;
            }
            s += 1;
        }
        true
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
                if self.is_scene_segment_free(&self.nodes[anchor].q, &self.nodes[up as usize].q) {
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
        let joints = self.chain.joints();
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

fn joint_distance<const N: usize>(
    a: &[CuJointPosition; N],
    b: &[CuJointPosition; N],
    metric: CuJointMetric,
) -> f64 {
    let mut sum_sq = 0.0_f64;
    let mut i = 0;
    while i < N {
        let d = metric.normalized(b[i]) - metric.normalized(a[i]);
        sum_sq += d * d;
        i += 1;
    }
    libm::sqrt(sum_sq)
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

pub mod planner_resources {
    use super::*;
    resources!({ rng => Owned<CuRng> });
}

/// Anytime joint-space RRT* motion planner task (`CuAnytimeTask`).
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointRrtStarTask<const N: usize, const M: usize = N> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    #[reflect(ignore)]
    collision_model: CuRobotCollisionModel<N, M>,
    params: CuJointRrtParams,
    base_iterations: u32,
    block_iterations: u32,
    base_seed: u64,
    job_counter: u64,
    #[reflect(ignore)]
    planner: Option<CuJointRrtStar<N, M>>,
    published_cost: f64,
    published_quality: Quality,
}

impl<const N: usize, const M: usize> Freezable for CuJointRrtStarTask<N, M> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.base_seed, encoder)?;
        Encode::encode(&self.job_counter, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.base_seed = Decode::decode(decoder)?;
        self.job_counter = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<const N: usize, const M: usize> CuJointRrtStarTask<N, M> {
    /// Creates a joint-space RRT* anytime task with explicit kinematic chain, collision model, and seed.
    pub fn new_with_model(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointRrtParams,
        base_iterations: u32,
        block_iterations: u32,
        base_seed: u64,
    ) -> Self {
        Self {
            chain,
            collision_model,
            params,
            base_iterations,
            block_iterations,
            base_seed,
            job_counter: 0,
            planner: None,
            published_cost: f64::INFINITY,
            published_quality: quality_from_f32(0.0),
        }
    }

    /// Returns a compact snapshot of the current job's progress.
    pub fn debug_state(&self) -> CuJointPlannerDebugState {
        let p = self.planner.as_ref();
        CuJointPlannerDebugState {
            iterations: p.map_or(0, CuJointRrtStar::iterations),
            tree_size: p.map_or(0, CuJointRrtStar::tree_size),
            best_cost: p.map_or(f64::INFINITY, CuJointRrtStar::best_cost),
            published_cost: self.published_cost,
            published_quality: self.published_quality,
            scene_revision: p.map_or(CuSceneRevision::INITIAL, CuJointRrtStar::scene_revision),
        }
    }

    fn publish(&mut self, output: &mut CuMsg<CuJointPlanPath<N>>) -> Quality {
        let Some(planner) = self.planner.as_ref() else {
            return self.published_quality;
        };
        if !planner.has_solution() {
            return self.published_quality;
        }
        let mut waypoints = core::array::from_fn(|_| CuJointPositionsPayload::default());
        let Some(len) = planner.write_path(&mut waypoints) else {
            // The best path does not fit the waypoint capacity even after shortcutting:
            // signal that no accurate path can be published.
            output.clear_payload();
            self.published_quality = quality_from_f32(0.0);
            return self.published_quality;
        };
        // Publish the cost and quality of the actually emitted (shortcut) path.
        let cost = planner.path_cost(&waypoints, len);
        if cost < self.published_cost || self.published_quality.raw() <= 0.0 {
            output.set_payload(CuJointPlanPath {
                waypoints,
                len,
                cost,
                iterations: planner.iterations(),
                scene_revision: planner.scene_revision(),
                exclusions: planner.exclusions.clone(),
            });
            self.published_cost = cost;
            self.published_quality = planner.quality_of_cost(cost);
        }
        self.published_quality
    }
}

impl<const N: usize, const M: usize> CuAnytimeTask for CuJointRrtStarTask<N, M> {
    type Input<'m> = input_msg!(CuJointPlanRequest<N>);
    type Output<'m> = output_msg!(CuJointPlanPath<N>);
    type Resources<'r> = planner_resources::Resources;
    type Quality = Quality;

    fn register_debug_state_types(registry: &mut TypeRegistry) {
        registry.register::<CuJointPlannerDebugState>();
    }

    fn debug_state_type_path() -> &'static str {
        CuJointPlannerDebugState::type_path()
    }

    fn with_debug_state<R>(&self, f: impl FnOnce(&dyn bevy_reflect::Reflect) -> R) -> R {
        f(&self.debug_state())
    }

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let chain = CuKinematicChain::from_config(config)?;
        let collision_model = CuRobotCollisionModel::from_config(config)?;
        let mut params = CuJointRrtParams::default();
        let mut base_iterations = 64u32;
        let mut block_iterations = 32u32;

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
            if let Some(v) = cfg.get::<u32>("base_iterations")? {
                base_iterations = v;
            }
            if let Some(v) = cfg.get::<u32>("block_iterations")? {
                block_iterations = v;
            }
        }

        let Owned(mut rng) = resources.rng;
        Ok(Self::new_with_model(
            chain,
            collision_model,
            params,
            base_iterations,
            block_iterations,
            rng.random::<u64>(),
        ))
    }

    fn base<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        output.tov = input.tov;
        let Some(request) = input.payload() else {
            output.clear_payload();
            self.published_cost = f64::INFINITY;
            self.published_quality = quality_from_f32(0.0);
            return Ok(AnytimeStatus::Aborted);
        };

        self.job_counter = self.job_counter.wrapping_add(1);
        let seed = self.base_seed.wrapping_add(self.job_counter);

        let planner = match self.planner.as_mut() {
            Some(planner) => {
                planner.reset(
                    request.scene.clone(),
                    &request.exclusions,
                    request.start,
                    request.goal,
                    seed,
                );
                planner
            }
            None => self.planner.insert(CuJointRrtStar::new(
                self.chain,
                self.collision_model,
                self.params,
                request.scene.clone(),
                &request.exclusions,
                request.start,
                request.goal,
                seed,
            )),
        };

        if !planner.valid_endpoints {
            output.clear_payload();
            self.published_cost = f64::INFINITY;
            self.published_quality = quality_from_f32(0.0);
            return Ok(AnytimeStatus::Aborted);
        }

        if !planner.has_solution() {
            planner.grow(self.base_iterations);
        }

        self.published_cost = f64::INFINITY;
        self.published_quality = quality_from_f32(0.0);
        output.clear_payload();

        let quality = self.publish(output);
        if is_converged_quality(self.published_quality) {
            return Ok(AnytimeStatus::Converged(quality));
        }
        Ok(AnytimeStatus::Improved(quality))
    }

    fn refine<'o>(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        let planner = self
            .planner
            .as_mut()
            .ok_or_else(|| CuError::from("cu_joint_rrt_star: refine() called before base()"))?;

        if planner.is_exhausted() {
            return Ok(AnytimeStatus::Converged(self.published_quality));
        }

        planner.grow(self.block_iterations);
        let quality = self.publish(output);
        if is_converged_quality(self.published_quality) {
            return Ok(AnytimeStatus::Converged(quality));
        }
        Ok(AnytimeStatus::Improved(quality))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_collision::CuCollisionShape;
    use cu_collision::CuSphere;
    use cu_collision::igus_rebel_collision_model;
    use cu_kinematics::igus_rebel_chain;
    use cu_spatial_payloads::Transform3D;
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
        CuJointRrtStarTask::new_with_model(
            igus_rebel_chain(),
            igus_rebel_collision_model(),
            CuJointRrtParams::default(),
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
