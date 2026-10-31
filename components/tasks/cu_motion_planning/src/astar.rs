//! A* lattice search: search-based, deterministic, resolution-complete.
//!
//! Configurations live on a grid anchored at the start with spacing
//! `resolution` (normalized joint units). Each expansion moves one joint by
//! one grid step in either direction through a free segment, at cost
//! `resolution`; the heuristic is the straight-line distance to the goal,
//! which is admissible and consistent for these moves. When an expanded node
//! lies within one grid cell diagonal of the goal and sees it through a free
//! segment, the lattice path plus that last segment is shortcut by line of
//! sight and the job converges. Exhausting the open set or the node capacity
//! ends the job without a path. No randomness is involved.

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuJointPlannerTask;
use crate::CuPlanStatus;
use crate::CuPlannerKind;
use crate::common::check_endpoints;
use crate::common::length_quality;
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
use cu29::cutask_anytime::Quality;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Lattice node capacity.
pub const ASTAR_MAX_NODES: usize = 2048;
/// Longest lattice path (in grid steps) that can be reconstructed.
const ASTAR_MAX_PATH: usize = 512;
const HASH_SLOTS: usize = 2 * ASTAR_MAX_NODES;
const EMPTY: u16 = u16::MAX;

/// Parameters of [`CuJointAStar`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointAStarParams {
    /// Grid spacing (dimensionless, metric-scaled joint units).
    pub resolution: f64,
    /// Node capacity (`2..=ASTAR_MAX_NODES`).
    pub max_nodes: u32,
    /// Resolution of the segment collision checks.
    pub collision_check_step: CuJointResolution,
    /// Scaling of dimensionless joint-space distances and costs.
    pub metric: CuJointMetric,
}

impl Default for CuJointAStarParams {
    fn default() -> Self {
        Self {
            resolution: 0.25,
            max_nodes: ASTAR_MAX_NODES as u32,
            collision_check_step: CuJointResolution::default(),
            metric: CuJointMetric::default(),
        }
    }
}

impl CuJointAStarParams {
    /// Checks every parameter, the metric, and the collision-check resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.metric.validate()?;
        self.collision_check_step.metric().validate()?;
        let valid = {
            self.resolution.is_finite()
                && self.resolution > 0.0
                && (2..=ASTAR_MAX_NODES as u32).contains(&self.max_nodes)
        };
        if valid {
            Ok(())
        } else {
            Err(CuError::from("cu-motion-planning: invalid A* parameters"))
        }
    }
}

/// A* search over a joint-space lattice.
///
/// Built with [`CuJointPlanner::new`] from robot models and [`CuJointAStarParams`];
/// runs in [`CuJointAStarTask`].
#[derive(Clone)]
pub struct CuJointAStar<const N: usize, const M: usize = N> {
    validity: CuSceneValidity<N, M>,
    params: CuJointAStarParams,
    start: [CuJointPosition; N],
    goal: [CuJointPosition; N],
    cells: [[i16; N]; ASTAR_MAX_NODES],
    cost: [f64; ASTAR_MAX_NODES],
    estimate: [f64; ASTAR_MAX_NODES],
    parent: [u16; ASTAR_MAX_NODES],
    closed: [bool; ASTAR_MAX_NODES],
    table: [u16; HASH_SLOTS],
    len: usize,
    open: usize,
    goal_reach: f64,
    straight_cost: f64,
    expansions: u32,
    status: CuPlanStatus,
    best: CuJointPlanPath<N>,
}

/// [`CuJointPlannerTask`] running [`CuJointAStar`].
pub type CuJointAStarTask<const N: usize, const M: usize = N> =
    CuJointPlannerTask<CuJointAStar<N, M>, N, M>;

impl<const N: usize, const M: usize> CuJointAStar<N, M> {
    /// The validated planner parameters.
    pub const fn params(&self) -> &CuJointAStarParams {
        &self.params
    }

    fn build(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        params: CuJointAStarParams,
    ) -> Self {
        Self {
            validity: CuSceneValidity::new(chain, model, params.collision_check_step),
            params,
            start: [CuJointPosition::default(); N],
            goal: [CuJointPosition::default(); N],
            cells: [[0; N]; ASTAR_MAX_NODES],
            cost: [0.0; ASTAR_MAX_NODES],
            estimate: [0.0; ASTAR_MAX_NODES],
            parent: [EMPTY; ASTAR_MAX_NODES],
            closed: [false; ASTAR_MAX_NODES],
            table: [EMPTY; HASH_SLOTS],
            len: 0,
            open: 0,
            goal_reach: 0.0,
            straight_cost: 0.0,
            expansions: 0,
            status: CuPlanStatus::InvalidRequest,
            best: CuJointPlanPath::default(),
        }
    }

    /// Nodes created on the current job.
    pub const fn lattice_size(&self) -> usize {
        self.len
    }

    fn configuration(&self, cell: &[i16; N]) -> [CuJointPosition; N] {
        let metric = self.params.metric;
        core::array::from_fn(|j| {
            let q = self.start[j];
            let offset = cell[j] as f64 * self.params.resolution * metric.scale(q);
            q.with_si_value(q.si_value() + offset)
        })
    }

    fn hash(cell: &[i16; N]) -> usize {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for &c in cell {
            h ^= c as u16 as u64;
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
        (h as usize) % HASH_SLOTS
    }

    fn find(&self, cell: &[i16; N]) -> Option<usize> {
        let mut slot = Self::hash(cell);
        loop {
            match self.table[slot] {
                EMPTY => return None,
                idx if self.cells[idx as usize] == *cell => return Some(idx as usize),
                _ => slot = (slot + 1) % HASH_SLOTS,
            }
        }
    }

    fn insert(&mut self, cell: [i16; N], cost: f64, parent: u16) -> usize {
        let idx = self.len;
        self.cells[idx] = cell;
        self.cost[idx] = cost;
        self.estimate[idx] =
            cost + joint_distance(&self.configuration(&cell), &self.goal, self.params.metric);
        self.parent[idx] = parent;
        self.closed[idx] = false;
        let mut slot = Self::hash(&cell);
        while self.table[slot] != EMPTY {
            slot = (slot + 1) % HASH_SLOTS;
        }
        self.table[slot] = idx as u16;
        self.len += 1;
        self.open += 1;
        idx
    }

    fn finish(&mut self, last: usize) {
        let mut raw = [[CuJointPosition::default(); N]; ASTAR_MAX_PATH];
        let mut len = 0;
        raw[len] = self.goal;
        len += 1;
        let mut at = last;
        loop {
            if len >= ASTAR_MAX_PATH {
                self.status = CuPlanStatus::CapacityExceeded;
                return;
            }
            raw[len] = self.configuration(&self.cells[at]);
            len += 1;
            if self.parent[at] == EMPTY {
                break;
            }
            at = self.parent[at] as usize;
        }
        raw[..len].reverse();
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

    fn expand(&mut self) {
        let mut current = usize::MAX;
        for i in 0..self.len {
            if !self.closed[i]
                && (current == usize::MAX || self.estimate[i] < self.estimate[current])
            {
                current = i;
            }
        }
        if current == usize::MAX {
            self.status = CuPlanStatus::Exhausted;
            return;
        }
        self.closed[current] = true;
        self.open -= 1;
        self.expansions += 1;
        let cell = self.cells[current];
        let q = self.configuration(&cell);
        if joint_distance(&q, &self.goal, self.params.metric) <= self.goal_reach
            && self.validity.is_segment_free(&q, &self.goal)
        {
            self.finish(current);
            return;
        }
        for j in 0..N {
            if matches!(self.start[j], CuJointPosition::Fixed) {
                continue;
            }
            for direction in [-1i16, 1] {
                let mut next = cell;
                next[j] = match next[j].checked_add(direction) {
                    Some(v) => v,
                    None => continue,
                };
                let cost = self.cost[current] + self.params.resolution;
                match self.find(&next) {
                    Some(idx) if self.closed[idx] || cost >= self.cost[idx] => {}
                    Some(idx) => {
                        let q_next = self.configuration(&next);
                        if self.validity.is_segment_free(&q, &q_next) {
                            self.estimate[idx] += cost - self.cost[idx];
                            self.cost[idx] = cost;
                            self.parent[idx] = current as u16;
                        }
                    }
                    None => {
                        if self.len >= self.params.max_nodes as usize {
                            continue;
                        }
                        let q_next = self.configuration(&next);
                        if self.validity.is_segment_free(&q, &q_next) {
                            self.insert(next, cost, current as u16);
                        }
                    }
                }
            }
        }
        if self.open == 0 {
            self.status = CuPlanStatus::Exhausted;
        }
    }
}

impl<const N: usize, const M: usize> CuJointPlanner<N, M> for CuJointAStar<N, M> {
    const KIND: CuPlannerKind = CuPlannerKind::Search;

    type Params = CuJointAStarParams;

    fn new(
        chain: CuKinematicChain<N>,
        collision_model: CuRobotCollisionModel<N, M>,
        params: CuJointAStarParams,
    ) -> CuResult<Self> {
        params.validate()?;
        Ok(Self::build(chain, collision_model, params))
    }

    fn begin(&mut self, request: &CuJointPlanRequest<N>, _seed: u64) -> CuResult<CuPlanStatus> {
        self.start = request.start;
        self.goal = request.goal;
        self.expansions = 0;
        self.len = 0;
        self.open = 0;
        self.table = [EMPTY; HASH_SLOTS];
        self.best = CuJointPlanPath::default();
        self.straight_cost = joint_distance(&request.start, &request.goal, self.params.metric);
        let active = self
            .start
            .iter()
            .filter(|q| !matches!(q, CuJointPosition::Fixed))
            .count()
            .max(1);
        // Every point is within half a cell diagonal of the lattice; one full
        // diagonal leaves room for the free-segment check to succeed.
        self.goal_reach = self.params.resolution * libm::sqrt(active as f64);
        self.status = check_endpoints(&mut self.validity, request, self.params.metric);
        if self.status == CuPlanStatus::Searching {
            self.insert([0; N], 0.0, EMPTY);
        }
        Ok(self.status)
    }

    fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        for _ in 0..budget {
            if self.status.is_final() {
                break;
            }
            self.expand();
        }
        Ok(self.status)
    }

    fn write_best(&self, out: &mut CuJointPlanPath<N>) -> CuPlanStatus {
        write_header(out, &self.validity, self.status, self.expansions);
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
