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
use cu_collision::CuCollisionWorld;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuKinematicChain;
use cu_rng::prelude::*;
use cu29::cutask_anytime::AnytimeStatus;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use cu29::resources;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle as Angle64;
use serde::Deserialize;
use serde::Serialize;

/// Fixed capacity of tree nodes in `CuJointRrtStar`.
pub const MAX_NODES: usize = 256;
/// Fixed capacity of waypoints in a published `CuJointPlanPath`.
pub const MAX_WAYPOINTS: usize = 32;
/// Fixed capacity of obstacles in `CuJointPlanRequest`.
pub const MAX_OBSTACLES: usize = 16;

const CONVERGED_QUALITY: f32 = 0.999;

fn is_converged_quality(q: Quality) -> bool {
    q.raw() >= CONVERGED_QUALITY
}

/// Configuration parameters for joint-space RRT* planning.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct CuJointRrtParams {
    /// Maximum joint-space $L_2$ step size per tree extension.
    pub step_size: Angle64,
    /// Probability in `[0.0, 1.0]` of sampling the exact goal configuration.
    pub goal_bias: f64,
    /// Joint-space $L_2$ distance within which a node can connect directly to the goal.
    pub goal_threshold: Angle64,
    /// RRT* neighborhood rewiring radius in joint space.
    pub rewire_radius: Angle64,
    /// Maximum per-joint interpolation step for continuous edge collision checking.
    pub collision_check_step: Angle64,
    /// Maximum active tree nodes (`<= MAX_NODES`).
    pub max_nodes: u32,
}

impl Default for CuJointRrtParams {
    fn default() -> Self {
        Self {
            step_size: Angle64::new::<radian>(0.35),
            goal_bias: 0.15,
            goal_threshold: Angle64::new::<radian>(0.40),
            rewire_radius: Angle64::new::<radian>(0.75),
            collision_check_step: Angle64::new::<radian>(0.08),
            max_nodes: MAX_NODES as u32,
        }
    }
}

/// Joint-space motion planning request payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlanRequest<const N: usize> {
    pub world: CuCollisionWorld<MAX_OBSTACLES>,
    #[serde(with = "cu_kinematics::serde_arrays::angle_array")]
    pub start: [Angle64; N],
    #[serde(with = "cu_kinematics::serde_arrays::angle_array")]
    pub goal: [Angle64; N],
}

impl<const N: usize> Default for CuJointPlanRequest<N> {
    fn default() -> Self {
        Self {
            world: CuCollisionWorld::new(),
            start: [Angle64::default(); N],
            goal: [Angle64::default(); N],
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
    /// Joint-space $L_2$ cost of the RRT* tree solution.
    pub cost: Angle64,
    /// Total RRT* iterations executed on this job.
    pub iterations: u32,
}

impl<const N: usize> Default for CuJointPlanPath<N> {
    fn default() -> Self {
        Self {
            waypoints: core::array::from_fn(|_| CuJointPositionsPayload::default()),
            len: 0,
            cost: Angle64::new::<radian>(f64::INFINITY),
            iterations: 0,
        }
    }
}

/// Compact debug state exposed by `CuJointRrtStarTask` for remote inspection without serializing the full tree.
#[derive(Default, Debug, Reflect)]
pub struct CuJointPlannerDebugState {
    pub iterations: u32,
    pub tree_size: u32,
    pub best_cost: Angle64,
    pub published_cost: Angle64,
    pub published_quality: Quality,
}

#[derive(Clone, Copy)]
struct RrtNode<const N: usize> {
    q: [Angle64; N],
    parent: u16,
    cost_rad: f64,
}

impl<const N: usize> Default for RrtNode<N> {
    fn default() -> Self {
        Self {
            q: [Angle64::default(); N],
            parent: u16::MAX,
            cost_rad: 0.0,
        }
    }
}

/// Allocation-free fixed-capacity joint-space RRT* solver.
pub struct CuJointRrtStar<const N: usize> {
    chain: CuKinematicChain<N>,
    collision_model: CuRobotCollisionModel<N>,
    params: CuJointRrtParams,
    world: CuCollisionWorld<MAX_OBSTACLES>,
    start: [Angle64; N],
    goal: [Angle64; N],
    straight_line_cost_rad: f64,
    nodes: [RrtNode<N>; MAX_NODES],
    node_count: usize,
    goal_node_idx: Option<usize>,
    best_cost_rad: f64,
    iterations: u32,
    rng: CuRng,
    valid_endpoints: bool,
}

impl<const N: usize> CuJointRrtStar<N> {
    /// Creates and initializes a new joint-space RRT* solver for `(start, goal)`.
    pub fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N>,
        params: CuJointRrtParams,
        world: CuCollisionWorld<MAX_OBSTACLES>,
        start: [Angle64; N],
        goal: [Angle64; N],
        seed: u64,
    ) -> Self {
        let mut solver = Self {
            chain,
            collision_model,
            params,
            world: CuCollisionWorld::new(),
            start,
            goal,
            straight_line_cost_rad: 0.0,
            nodes: [RrtNode::default(); MAX_NODES],
            node_count: 0,
            goal_node_idx: None,
            best_cost_rad: f64::INFINITY,
            iterations: 0,
            rng: CuRng::from_seed(seed),
            valid_endpoints: false,
        };
        solver.reset(world, start, goal, seed);
        solver
    }

    /// Resets the solver in place for a new job without allocating.
    pub fn reset(
        &mut self,
        world: CuCollisionWorld<MAX_OBSTACLES>,
        start: [Angle64; N],
        goal: [Angle64; N],
        seed: u64,
    ) {
        self.world = world;
        self.start = start;
        self.goal = goal;
        self.straight_line_cost_rad = joint_distance_rad(&start, &goal);
        self.node_count = 0;
        self.goal_node_idx = None;
        self.best_cost_rad = f64::INFINITY;
        self.iterations = 0;
        self.rng.reseed(seed);

        let start_ok = self
            .collision_model
            .is_configuration_collision_free(&self.chain, &self.start, Some(&self.world))
            .unwrap_or(false);
        let goal_ok = self
            .collision_model
            .is_configuration_collision_free(&self.chain, &self.goal, Some(&self.world))
            .unwrap_or(false);
        self.valid_endpoints = start_ok && goal_ok;

        if self.valid_endpoints {
            self.nodes[0] = RrtNode {
                q: self.start,
                parent: u16::MAX,
                cost_rad: 0.0,
            };
            self.node_count = 1;

            if self.straight_line_cost_rad <= 1e-9 {
                self.goal_node_idx = Some(0);
                self.best_cost_rad = 0.0;
            } else if self
                .collision_model
                .is_segment_collision_free(
                    &self.chain,
                    &self.start,
                    &self.goal,
                    self.params.collision_check_step,
                    Some(&self.world),
                )
                .unwrap_or(false)
            {
                // Direct line-of-sight connection from start to goal!
                self.nodes[1] = RrtNode {
                    q: self.goal,
                    parent: 0,
                    cost_rad: self.straight_line_cost_rad,
                };
                self.node_count = 2;
                self.goal_node_idx = Some(1);
                self.best_cost_rad = self.straight_line_cost_rad;
            }
        }
    }

    /// Runs `budget` bounded RRT* iterations in place.
    pub fn grow(&mut self, budget: u32) {
        if !self.valid_endpoints {
            return;
        }
        let cap = (self.params.max_nodes as usize).min(MAX_NODES);
        let step_rad = self.params.step_size.get::<radian>();
        let goal_thresh_rad = self.params.goal_threshold.get::<radian>();
        let rewire_rad = self.params.rewire_radius.get::<radian>();

        let mut it = 0;
        while it < budget {
            if self.node_count >= cap {
                break;
            }
            self.iterations += 1;
            it += 1;

            let q_sample = self.sample_config();
            let nearest_idx = self.nearest_node(&q_sample);
            let q_new = steer(&self.nodes[nearest_idx].q, &q_sample, step_rad);

            // Informed pruning: skip configurations that cannot possibly improve best_cost_rad
            let min_possible =
                joint_distance_rad(&self.start, &q_new) + joint_distance_rad(&q_new, &self.goal);
            if min_possible >= self.best_cost_rad {
                continue;
            }

            if !self
                .collision_model
                .is_configuration_collision_free(&self.chain, &q_new, Some(&self.world))
                .unwrap_or(false)
            {
                continue;
            }

            // Choose best parent inside `rewire_rad` neighborhood
            let mut best_parent = nearest_idx;
            let mut best_new_cost = self.nodes[nearest_idx].cost_rad
                + joint_distance_rad(&self.nodes[nearest_idx].q, &q_new);

            if !self
                .collision_model
                .is_segment_collision_free(
                    &self.chain,
                    &self.nodes[nearest_idx].q,
                    &q_new,
                    self.params.collision_check_step,
                    Some(&self.world),
                )
                .unwrap_or(false)
            {
                continue;
            }

            let mut idx = 0;
            while idx < self.node_count {
                if idx != nearest_idx {
                    let d = joint_distance_rad(&self.nodes[idx].q, &q_new);
                    if d <= rewire_rad {
                        let cand_cost = self.nodes[idx].cost_rad + d;
                        if cand_cost < best_new_cost
                            && self
                                .collision_model
                                .is_segment_collision_free(
                                    &self.chain,
                                    &self.nodes[idx].q,
                                    &q_new,
                                    self.params.collision_check_step,
                                    Some(&self.world),
                                )
                                .unwrap_or(false)
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
                cost_rad: best_new_cost,
            };
            self.node_count += 1;

            // Rewire neighbors through `new_idx` if cheaper
            let mut nbr = 0;
            while nbr < new_idx {
                if nbr != best_parent {
                    let d = joint_distance_rad(&q_new, &self.nodes[nbr].q);
                    if d <= rewire_rad
                        && best_new_cost + d < self.nodes[nbr].cost_rad
                        && self
                            .collision_model
                            .is_segment_collision_free(
                                &self.chain,
                                &q_new,
                                &self.nodes[nbr].q,
                                self.params.collision_check_step,
                                Some(&self.world),
                            )
                            .unwrap_or(false)
                    {
                        self.nodes[nbr].parent = new_idx as u16;
                        self.nodes[nbr].cost_rad = best_new_cost + d;
                    }
                }
                nbr += 1;
            }

            // Try connecting `new_idx` to `goal` if within `goal_threshold` and capacity allows
            let dist_to_goal = joint_distance_rad(&q_new, &self.goal);
            if dist_to_goal <= goal_thresh_rad
                && best_new_cost + dist_to_goal < self.best_cost_rad
                && self
                    .collision_model
                    .is_segment_collision_free(
                        &self.chain,
                        &q_new,
                        &self.goal,
                        self.params.collision_check_step,
                        Some(&self.world),
                    )
                    .unwrap_or(false)
            {
                if let Some(g_idx) = self.goal_node_idx {
                    self.nodes[g_idx].parent = new_idx as u16;
                    self.nodes[g_idx].cost_rad = best_new_cost + dist_to_goal;
                    self.best_cost_rad = best_new_cost + dist_to_goal;
                } else if self.node_count < cap {
                    let g_idx = self.node_count;
                    self.nodes[g_idx] = RrtNode {
                        q: self.goal,
                        parent: new_idx as u16,
                        cost_rad: best_new_cost + dist_to_goal,
                    };
                    self.node_count += 1;
                    self.goal_node_idx = Some(g_idx);
                    self.best_cost_rad = best_new_cost + dist_to_goal;
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
    pub fn best_cost(&self) -> Angle64 {
        Angle64::new::<radian>(self.best_cost_rad)
    }

    /// Returns the number of RRT* iterations executed for the current job.
    pub const fn iterations(&self) -> u32 {
        self.iterations
    }

    /// Returns the number of nodes currently in the tree.
    pub const fn tree_size(&self) -> u32 {
        self.node_count as u32
    }

    /// Computes the normalized quality in `[0.0, 1.0]` (`straight_line_cost / best_cost`).
    pub fn quality(&self) -> Quality {
        if !self.has_solution() || !self.best_cost_rad.is_finite() {
            return quality_from_f32(0.0);
        }
        if self.straight_line_cost_rad <= 1e-9 || self.best_cost_rad <= 1e-9 {
            return quality_from_f32(1.0);
        }
        let ratio = (self.straight_line_cost_rad / self.best_cost_rad).clamp(0.0, 1.0);
        quality_from_f32(ratio as f32)
    }

    /// Extracts and shortcuts the best path in place into `out_waypoints`, returning the
    /// number of waypoints written (`Some(len)`), or `None` if no solution exists or it exceeds `MAX_WAYPOINTS`.
    pub fn write_path(
        &self,
        out_waypoints: &mut [CuJointPositionsPayload<N>; MAX_WAYPOINTS],
    ) -> Option<u32> {
        let mut curr = self.goal_node_idx?;
        let mut raw_chain = [[Angle64::default(); N]; MAX_WAYPOINTS];
        let mut count = 0usize;

        while curr != u16::MAX as usize {
            if count >= MAX_WAYPOINTS {
                return None;
            }
            raw_chain[count] = self.nodes[curr].q;
            count += 1;
            let parent = self.nodes[curr].parent;
            if parent == u16::MAX {
                break;
            }
            curr = parent as usize;
        }

        // Reverse `raw_chain[..count]` so index 0 is `start` and `count - 1` is `goal`
        let mut ordered = [[Angle64::default(); N]; MAX_WAYPOINTS];
        let mut i = 0;
        while i < count {
            ordered[i] = raw_chain[count - 1 - i];
            i += 1;
        }

        // Greedy line-of-sight shortcutting
        let mut out_len = 0usize;
        let mut anchor = 0usize;
        out_waypoints[0] = CuJointPositionsPayload {
            positions: ordered[0],
        };
        out_len += 1;

        while anchor + 1 < count {
            let mut farthest = count - 1;
            while farthest > anchor + 1 {
                if self
                    .collision_model
                    .is_segment_collision_free(
                        &self.chain,
                        &ordered[anchor],
                        &ordered[farthest],
                        self.params.collision_check_step,
                        Some(&self.world),
                    )
                    .unwrap_or(false)
                {
                    break;
                }
                farthest -= 1;
            }
            out_waypoints[out_len] = CuJointPositionsPayload {
                positions: ordered[farthest],
            };
            out_len += 1;
            anchor = farthest;
        }

        Some(out_len as u32)
    }

    fn sample_config(&mut self) -> [Angle64; N] {
        let coin: f64 = self.rng.random();
        if coin < self.params.goal_bias {
            return self.goal;
        }
        let joints = self.chain.joints();
        let mut q = [Angle64::default(); N];
        let mut j = 0;
        while j < N {
            let lo = joints[j].limits.lower.get::<radian>();
            let hi = joints[j].limits.upper.get::<radian>();
            let u: f64 = self.rng.random();
            q[j] = Angle64::new::<radian>(lo + u * (hi - lo));
            j += 1;
        }
        q
    }

    fn nearest_node(&self, q_target: &[Angle64; N]) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::INFINITY;
        let mut i = 0;
        while i < self.node_count {
            // Do not expand out of the goal node itself
            if Some(i) != self.goal_node_idx {
                let d = joint_distance_rad(&self.nodes[i].q, q_target);
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

fn joint_distance_rad<const N: usize>(a: &[Angle64; N], b: &[Angle64; N]) -> f64 {
    let mut sum_sq = 0.0_f64;
    let mut i = 0;
    while i < N {
        let d = b[i].get::<radian>() - a[i].get::<radian>();
        sum_sq += d * d;
        i += 1;
    }
    libm::sqrt(sum_sq)
}

fn steer<const N: usize>(
    from: &[Angle64; N],
    to: &[Angle64; N],
    max_step_rad: f64,
) -> [Angle64; N] {
    let dist = joint_distance_rad(from, to);
    if dist <= max_step_rad || dist <= 1e-12 {
        return *to;
    }
    let scale = max_step_rad / dist;
    let mut out = [Angle64::default(); N];
    let mut i = 0;
    while i < N {
        let a = from[i].get::<radian>();
        let b = to[i].get::<radian>();
        out[i] = Angle64::new::<radian>(a + scale * (b - a));
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
pub struct CuJointRrtStarTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    #[reflect(ignore)]
    collision_model: CuRobotCollisionModel<N>,
    params: CuJointRrtParams,
    base_iterations: u32,
    block_iterations: u32,
    base_seed: u64,
    job_counter: u64,
    #[reflect(ignore)]
    planner: Option<CuJointRrtStar<N>>,
    published_cost: Angle64,
    published_quality: Quality,
}

impl<const N: usize> Freezable for CuJointRrtStarTask<N> {
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

impl<const N: usize> CuJointRrtStarTask<N> {
    /// Creates a joint-space RRT* anytime task with explicit kinematic chain, collision model, and seed.
    pub fn new_with_model(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N>,
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
            published_cost: Angle64::new::<radian>(f64::INFINITY),
            published_quality: quality_from_f32(0.0),
        }
    }

    /// Returns a compact snapshot of the current job's progress.
    pub fn debug_state(&self) -> CuJointPlannerDebugState {
        let p = self.planner.as_ref();
        CuJointPlannerDebugState {
            iterations: p.map_or(0, CuJointRrtStar::iterations),
            tree_size: p.map_or(0, CuJointRrtStar::tree_size),
            best_cost: p.map_or(
                Angle64::new::<radian>(f64::INFINITY),
                CuJointRrtStar::best_cost,
            ),
            published_cost: self.published_cost,
            published_quality: self.published_quality,
        }
    }

    fn publish(&mut self, output: &mut CuMsg<CuJointPlanPath<N>>) -> Quality {
        if let Some(planner) = self.planner.as_ref()
            && planner.has_solution()
            && planner.best_cost().get::<radian>() < self.published_cost.get::<radian>()
        {
            let mut waypoints = core::array::from_fn(|_| CuJointPositionsPayload::default());
            if let Some(len) = planner.write_path(&mut waypoints) {
                output.set_payload(CuJointPlanPath {
                    waypoints,
                    len,
                    cost: planner.best_cost(),
                    iterations: planner.iterations(),
                });
                self.published_cost = planner.best_cost();
                self.published_quality = planner.quality();
            }
        }
        self.published_quality
    }
}

impl<const N: usize> CuAnytimeTask for CuJointRrtStarTask<N> {
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
            if let Some(v) = cfg.get::<f64>("step_size_rad")? {
                params.step_size = Angle64::new::<radian>(v);
            }
            if let Some(v) = cfg.get::<f64>("goal_bias")? {
                params.goal_bias = v;
            }
            if let Some(v) = cfg.get::<f64>("goal_threshold_rad")? {
                params.goal_threshold = Angle64::new::<radian>(v);
            }
            if let Some(v) = cfg.get::<f64>("rewire_radius_rad")? {
                params.rewire_radius = Angle64::new::<radian>(v);
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
            self.published_cost = Angle64::new::<radian>(f64::INFINITY);
            self.published_quality = quality_from_f32(0.0);
            return Ok(AnytimeStatus::Aborted);
        };

        self.job_counter = self.job_counter.wrapping_add(1);
        let seed = self.base_seed.wrapping_add(self.job_counter);

        let planner = match self.planner.as_mut() {
            Some(planner) => {
                planner.reset(request.world.clone(), request.start, request.goal, seed);
                planner
            }
            None => self.planner.insert(CuJointRrtStar::new(
                self.chain,
                self.collision_model,
                self.params,
                request.world.clone(),
                request.start,
                request.goal,
                seed,
            )),
        };

        if !planner.valid_endpoints {
            output.clear_payload();
            self.published_cost = Angle64::new::<radian>(f64::INFINITY);
            self.published_quality = quality_from_f32(0.0);
            return Ok(AnytimeStatus::Aborted);
        }

        if !planner.has_solution() {
            planner.grow(self.base_iterations);
        }

        self.published_cost = Angle64::new::<radian>(f64::INFINITY);
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

    #[test]
    fn planner_initialization_requires_explicit_serial_models() {
        let resources = || planner_resources::Resources {
            rng: Owned(CuRng::from_seed(1)),
        };
        assert!(CuJointRrtStarTask::<1>::new(None, resources()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}]},"collision_model": {"link_shapes": [{"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}], "safety_margin": 0.01}}"#).unwrap();
        let mut task = CuJointRrtStarTask::<1>::new(Some(&cfg), resources()).unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        let request = CuMsg::new(Some(CuJointPlanRequest {
            world: CuCollisionWorld::new(),
            start: [Angle64::new::<radian>(0.0)],
            goal: [Angle64::new::<radian>(0.5)],
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

        let start = [Angle64::new::<radian>(0.0); 6];
        let goal = [
            Angle64::new::<radian>(0.5),
            Angle64::new::<radian>(0.3),
            Angle64::new::<radian>(-0.3),
            Angle64::new::<radian>(0.2),
            Angle64::new::<radian>(-0.2),
            Angle64::new::<radian>(0.4),
        ];

        let req = CuMsg::new(Some(CuJointPlanRequest {
            world: CuCollisionWorld::new(),
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
            Angle64::new::<radian>(-0.6),
            Angle64::new::<radian>(0.7),
            Angle64::new::<radian>(-0.2),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
        ];
        let goal = [
            Angle64::new::<radian>(0.6),
            Angle64::new::<radian>(0.7),
            Angle64::new::<radian>(-0.2),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
        ];

        // Midpoint q_mid = [0.0, 0.7, -0.2, 0, 0, 0]: compute its end-effector position and place
        // an obstacle sphere there so the straight-line joint path collides, forcing RRT* to route around!
        let q_mid = [
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.7),
            Angle64::new::<radian>(-0.2),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
        ];
        let mut mid_ee = Transform3D::<f64>::identity();
        igus_rebel_chain()
            .forward_kinematics(&q_mid, &mut mid_ee)
            .unwrap();
        let m = mid_ee.to_matrix();

        let mut world = CuCollisionWorld::new();
        world
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [m[0][3], m[1][3], m[2][3]],
                0.08,
            )))
            .unwrap();

        let req = CuMsg::new(Some(CuJointPlanRequest { world, start, goal }));
        let mut out = CuMsg::new(None);

        let status = task.base(&ctx, &req, &mut out).unwrap();
        assert!(!matches!(status, AnytimeStatus::Aborted));

        let mut best_cost = f64::INFINITY;
        for _ in 0..8 {
            let st = task.refine(&ctx, &mut out).unwrap();
            assert!(!matches!(st, AnytimeStatus::Aborted));
            if let Some(p) = out.payload() {
                let c = p.cost.get::<radian>();
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
        for w in 0..(path.len as usize - 1) {
            assert!(
                model
                    .is_segment_collision_free(
                        &chain,
                        &path.waypoints[w].positions,
                        &path.waypoints[w + 1].positions,
                        Angle64::new::<radian>(0.05),
                        Some(&req.payload().unwrap().world),
                    )
                    .unwrap()
            );
        }
    }
}
