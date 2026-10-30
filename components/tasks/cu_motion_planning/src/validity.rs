//! Scene-aware validity checking shared by every planner.
//!
//! [`CuSceneValidity`] captures one [`CuPlanningScene`] snapshot per planning
//! job and answers the three questions any planner asks, sampling-based or
//! not: is a configuration free, is a joint-space segment free, and how much
//! clearance does a configuration have.

use crate::MAX_ATTACHED_SHAPES;
use crate::MAX_EXCLUSIONS;
use crate::MAX_OBSTACLES;
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
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu29::units::si::length::meter;

/// Collision queries against one captured planning-scene snapshot.
#[derive(Clone)]
pub struct CuSceneValidity<const N: usize, const M: usize = N> {
    chain: CuKinematicChain<N>,
    model: CuRobotCollisionModel<N, M>,
    resolution: CuJointResolution,
    scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    world: CuCollisionWorld<MAX_OBSTACLES>,
    attached_shapes: [CuCollisionShape; MAX_ATTACHED_SHAPES],
    attached_frames: [Option<usize>; MAX_ATTACHED_SHAPES],
    attached_len: usize,
    query_exclusions: CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
}

impl<const N: usize, const M: usize> CuSceneValidity<N, M> {
    /// Creates a checker with an empty scene; `resolution` sets the segment
    /// sampling step.
    pub fn new(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
        resolution: CuJointResolution,
    ) -> Self {
        Self {
            chain,
            model,
            resolution,
            scene: CuPlanningScene::new(),
            world: CuCollisionWorld::new(),
            attached_shapes: [CuCollisionShape::default(); MAX_ATTACHED_SHAPES],
            attached_frames: [None; MAX_ATTACHED_SHAPES],
            attached_len: 0,
            query_exclusions: CuSceneQueryExclusions::new(),
            exclusions: CuSceneExclusions::new(),
        }
    }

    /// The kinematic chain the checker evaluates.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Segment sampling resolution.
    pub const fn resolution(&self) -> CuJointResolution {
        self.resolution
    }

    /// Revision of the captured scene.
    pub const fn scene_revision(&self) -> CuSceneRevision {
        self.scene.revision()
    }

    /// Exclusions captured with the scene.
    pub const fn exclusions(&self) -> &CuSceneExclusions<MAX_EXCLUSIONS> {
        &self.exclusions
    }

    /// Captures a scene snapshot for the next job. Returns `false` (and keeps
    /// no attached shapes) when the scene, its attached shapes, or the
    /// stable-ID exclusions are invalid for this robot.
    pub fn capture(
        &mut self,
        scene: &CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
        exclusions: &CuSceneExclusions<MAX_EXCLUSIONS>,
    ) -> bool {
        self.scene = scene.clone();
        self.exclusions = exclusions.clone();
        self.attached_len = 0;
        if self.resolution.metric().validate().is_err()
            || self.scene.write_collision_world(&mut self.world).is_err()
        {
            return false;
        }
        let attached = self.scene.attached_shapes();
        if attached.len() > MAX_ATTACHED_SHAPES || attached.iter().any(|a| a.link_index >= N) {
            return false;
        }
        for i in 0..MAX_ATTACHED_SHAPES {
            match attached.get(i) {
                Some(shape) => {
                    self.attached_shapes[i] = shape.shape;
                    self.attached_frames[i] = Some(shape.link_index);
                }
                None => {
                    self.attached_shapes[i] = CuCollisionShape::default();
                    self.attached_frames[i] = None;
                }
            }
        }
        let mut resolved = CuSceneQueryExclusions::new();
        if exclusions
            .resolve(&self.scene, self.model.shape_count(), &mut resolved)
            .is_err()
        {
            return false;
        }
        self.query_exclusions = resolved;
        self.attached_len = attached.len();
        true
    }

    /// `true` when `q` is within joint limits and free of self, environment,
    /// and attached-object collisions.
    pub fn is_configuration_free(&self, q: &[CuJointPosition; N]) -> bool {
        if !self
            .model
            .is_configuration_collision_free(&self.chain, q, Some(&self.world))
            .unwrap_or(false)
        {
            return false;
        }
        if self.attached_len == 0 {
            return true;
        }
        let mut report = CuCollisionReport::<0>::default();
        self.check_attached(q, &mut report) && report.is_collision_free()
    }

    /// `true` when the joint-space line from `a` to `b` is free at the
    /// configured resolution (discrete sampling, not swept volumes).
    pub fn is_segment_free(&self, a: &[CuJointPosition; N], b: &[CuJointPosition; N]) -> bool {
        let metric = self.resolution.metric();
        let mut max_delta = 0.0_f64;
        for j in 0..N {
            let joint = &self.chain.joints()[j];
            if joint.validate_coordinate(a[j]).is_err() || joint.validate_coordinate(b[j]).is_err()
            {
                return false;
            }
            max_delta = max_delta.max(libm::fabs(
                metric.normalized(b[j]) - metric.normalized(a[j]),
            ));
        }
        if !max_delta.is_finite() || max_delta > 1_000_000.0 {
            return false;
        }
        let steps = (libm::ceil(max_delta) as usize).max(1);
        for s in 0..=steps {
            let alpha = s as f64 / steps as f64;
            let mut q = [CuJointPosition::default(); N];
            for j in 0..N {
                match a[j].interpolate(b[j], alpha) {
                    Ok(value) => q[j] = value,
                    Err(_) => return false,
                }
            }
            if !self.is_configuration_free(&q) {
                return false;
            }
        }
        true
    }

    /// Smallest signed surface distance in meters between the robot (and
    /// attached objects) and itself or the scene; negative means penetration.
    /// `None` when `q` cannot be evaluated.
    pub fn clearance(&self, q: &[CuJointPosition; N]) -> Option<f64> {
        let mut report = CuCollisionReport::<0>::default();
        self.model
            .check_configuration(&self.chain, q, Some(&self.world), &mut report)
            .ok()?;
        let mut clearance = report.min_signed_distance.get::<meter>();
        if self.attached_len > 0 {
            let mut attached = CuCollisionReport::<0>::default();
            if !self.check_attached(q, &mut attached) {
                return None;
            }
            clearance = clearance.min(attached.min_signed_distance.get::<meter>());
        }
        Some(clearance)
    }

    fn check_attached(&self, q: &[CuJointPosition; N], report: &mut CuCollisionReport<0>) -> bool {
        let mut attached_world = [CuCollisionShape::default(); MAX_ATTACHED_SHAPES];
        let mut robot_world = [CuCollisionShape::default(); M];
        check_attached_configuration(
            &self.model,
            &self.chain,
            q,
            &self.attached_shapes,
            &self.attached_frames,
            self.attached_len,
            Some(&self.world),
            &self.query_exclusions,
            &mut attached_world,
            &mut robot_world,
            report,
        )
        .is_ok()
    }
}

/// Dimensionless joint-space L2 distance under `metric`.
pub fn joint_distance<const N: usize>(
    a: &[CuJointPosition; N],
    b: &[CuJointPosition; N],
    metric: CuJointMetric,
) -> f64 {
    let mut sum = 0.0_f64;
    for j in 0..N {
        let d = metric.normalized(b[j]) - metric.normalized(a[j]);
        sum += d * d;
    }
    libm::sqrt(sum)
}
