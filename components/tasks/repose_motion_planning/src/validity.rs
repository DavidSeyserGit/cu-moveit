//! Scene-aware validity checking shared by every planner.
//!
//! [`CuSceneValidity`] captures one [`CuPlanningScene`] snapshot per planning
//! job and answers the three questions any planner asks, sampling-based or
//! not: is a configuration free, is a joint-space segment free, and how much
//! clearance does a configuration have.

use crate::MAX_ATTACHED_SHAPES;
use crate::MAX_EXCLUSIONS;
use crate::MAX_OBSTACLES;
use cu29::units::si::length::meter;
use repose_collision::CuCollisionEnv;
use repose_collision::CuCollisionReport;
use repose_collision::CuPlanningScene;
use repose_collision::CuRobotCollisionModel;
use repose_collision::CuSceneExclusions;
use repose_collision::CuSceneRevision;
use repose_kinematics::CuJointMetric;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointResolution;
use repose_kinematics::CuKinematicChain;

/// Collision queries against one captured planning-scene snapshot.
///
/// Every query checks the robot against itself, the scene obstacles, and the
/// scene's attached shapes in one pass, honouring the captured stable-ID
/// exclusions.
#[derive(Clone)]
pub struct CuSceneValidity<const N: usize, const M: usize = N> {
    chain: CuKinematicChain<N>,
    model: CuRobotCollisionModel<N, M>,
    resolution: CuJointResolution,
    scene: CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
    exclusions: CuSceneExclusions<MAX_EXCLUSIONS>,
    /// `false` until a valid scene has been captured.
    captured: bool,
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
            exclusions: CuSceneExclusions::new(),
            captured: true,
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

    /// Captures a scene snapshot for the next job. Returns `false` (and
    /// treats every configuration as blocked) when the resolution, the
    /// scene's attached shapes, or the stable-ID exclusions are invalid for
    /// this robot.
    pub fn capture(
        &mut self,
        scene: &CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED_SHAPES>,
        exclusions: &CuSceneExclusions<MAX_EXCLUSIONS>,
    ) -> bool {
        self.scene = scene.clone();
        self.exclusions = exclusions.clone();
        self.captured = self.resolution.metric().validate().is_ok()
            && self
                .scene
                .attached_shapes()
                .iter()
                .all(|a| a.frame.is_none_or(|frame| frame < N))
            && exclusions
                .validate(&self.scene, self.model.shape_count())
                .is_ok();
        self.captured
    }

    /// Obstacles, attached shapes, and exclusions of the captured scene.
    fn env(&self) -> CuCollisionEnv<'_> {
        self.scene.collision_env().with_exclusions(&self.exclusions)
    }

    /// `true` when `q` is within joint limits and free of self, environment,
    /// and attached-object collisions.
    pub fn is_configuration_free(&self, q: &[CuJointPosition; N]) -> bool {
        self.captured
            && self
                .model
                .is_configuration_collision_free(&self.chain, q, &self.env())
                .unwrap_or(false)
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
        if !self.captured {
            return None;
        }
        let mut report = CuCollisionReport::<0>::default();
        self.model
            .check_configuration(&self.chain, q, &self.env(), &mut report)
            .ok()?;
        Some(report.min_signed_distance().get::<meter>())
    }
}

/// Joint-space L2 distance under `metric` (dimensionless, metric-scaled).
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
