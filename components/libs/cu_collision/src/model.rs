//! Robot link collision models, fixed-capacity obstacle worlds, and in-place collision reports.

use crate::shapes::CuCollisionShape;
use crate::shapes::shape_signed_distance;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle as Angle64;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Category of a detected collision or safety-margin violation.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuContactKind {
    #[default]
    SelfCollision,
    EnvironmentCollision,
}

/// Single contact or clearance violation between a robot link and another link or obstacle.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCollisionContact {
    pub kind: CuContactKind,
    /// Robot link index (`0..N`).
    pub link_index: u16,
    /// Partner link index (`0..N`) for `SelfCollision` or obstacle index (`0..len`) for `EnvironmentCollision`.
    pub partner_index: u16,
    /// Surface-to-surface signed distance (negative indicates penetration).
    pub signed_distance: Length64,
}

/// Fixed-capacity collision report populated in place (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuCollisionReport<const MAX_CONTACTS: usize> {
    #[serde(with = "crate::serde_arrays::collision_contact_array")]
    pub contacts: [CuCollisionContact; MAX_CONTACTS],
    /// Number of valid entries in `contacts`.
    pub len: u32,
    /// Minimum surface-to-surface signed distance observed across all checked pairs.
    pub min_signed_distance: Length64,
}

impl<const MAX_CONTACTS: usize> Default for CuCollisionReport<MAX_CONTACTS> {
    fn default() -> Self {
        Self {
            contacts: [CuCollisionContact::default(); MAX_CONTACTS],
            len: 0,
            min_signed_distance: Length64::new::<meter>(f64::INFINITY),
        }
    }
}

impl<const MAX_CONTACTS: usize> CuCollisionReport<MAX_CONTACTS> {
    /// Resets the collision report in place for reuse.
    pub fn clear(&mut self) {
        self.len = 0;
        self.min_signed_distance = Length64::new::<meter>(f64::INFINITY);
    }

    /// Returns `true` if no self-collisions or environment collisions were recorded.
    pub const fn is_collision_free(&self) -> bool {
        self.len == 0
    }

    /// Returns the recorded contacts slice `&contacts[..len]`.
    pub fn as_slice(&self) -> &[CuCollisionContact] {
        let count = (self.len as usize).min(MAX_CONTACTS);
        &self.contacts[..count]
    }

    fn push_contact(&mut self, contact: CuCollisionContact) {
        let idx = self.len as usize;
        if idx < MAX_CONTACTS {
            self.contacts[idx] = contact;
            self.len += 1;
        }
    }
}

/// Fixed-capacity environment obstacle world (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuCollisionWorld<const MAX_OBSTACLES: usize> {
    #[serde(with = "crate::serde_arrays::collision_shape_array")]
    pub obstacles: [CuCollisionShape; MAX_OBSTACLES],
    /// Number of active obstacles in `obstacles[..len]`.
    pub len: u32,
}

impl<const MAX_OBSTACLES: usize> Default for CuCollisionWorld<MAX_OBSTACLES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_OBSTACLES: usize> CuCollisionWorld<MAX_OBSTACLES> {
    /// Creates an empty obstacle world.
    pub fn new() -> Self {
        Self {
            obstacles: [CuCollisionShape::default(); MAX_OBSTACLES],
            len: 0,
        }
    }

    /// Clears all obstacles in place.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Appends an obstacle to the world if capacity permits.
    pub fn add_obstacle(&mut self, shape: CuCollisionShape) -> CuResult<u16> {
        let idx = self.len as usize;
        if idx >= MAX_OBSTACLES {
            return Err(CuError::from(
                "cu_collision: obstacle world capacity exceeded",
            ));
        }
        self.obstacles[idx] = shape;
        self.len += 1;
        Ok(idx as u16)
    }

    /// Borrows the active obstacles slice.
    pub fn as_slice(&self) -> &[CuCollisionShape] {
        let count = (self.len as usize).min(MAX_OBSTACLES);
        &self.obstacles[..count]
    }
}

/// Static $N$-link robot collision model with configurable self-collision matrix and safety margin.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuRobotCollisionModel<const N: usize> {
    link_shapes: [CuCollisionShape; N],
    safety_margin: Length64,
    check_pair: [[bool; N]; N],
}

#[derive(Deserialize)]
struct CollisionModelConfig<const N: usize> {
    #[serde(with = "crate::serde_arrays::collision_shape_array")]
    link_shapes: [CuCollisionShape; N],
    safety_margin: Length64,
}

impl<const N: usize> CuRobotCollisionModel<N> {
    /// Loads required `collision_model` configuration for a serial chain.
    /// Contains exactly `N` local `link_shapes` and a typed `safety_margin`.
    /// Non-adjacent link pairs are checked by default.
    pub fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let config =
            config.ok_or_else(|| CuError::from("Missing collision model configuration"))?;
        let model = config
            .get_value::<CollisionModelConfig<N>>("collision_model")?
            .ok_or_else(|| CuError::from("Missing required collision_model configuration"))?;
        Ok(Self::new(model.link_shapes, model.safety_margin))
    }

    /// Creates a robot collision model from per-link local shapes and a safety margin.
    ///
    /// By default, all non-adjacent link pairs (`|i - j| > 1`) are enabled for self-collision checking.
    pub fn new(link_shapes: [CuCollisionShape; N], safety_margin: Length64) -> Self {
        let mut check_pair = [[false; N]; N];
        let mut i = 0;
        while i < N {
            let mut j = i + 2;
            while j < N {
                check_pair[i][j] = true;
                check_pair[j][i] = true;
                j += 1;
            }
            i += 1;
        }
        Self {
            link_shapes,
            safety_margin,
            check_pair,
        }
    }

    /// Borrows the local collision shapes for each link `0..N`.
    pub const fn link_shapes(&self) -> &[CuCollisionShape; N] {
        &self.link_shapes
    }

    /// Returns the configured safety margin.
    pub const fn safety_margin(&self) -> Length64 {
        self.safety_margin
    }

    /// Sets the safety margin required for a pair to be considered collision-free.
    pub fn set_safety_margin(&mut self, safety_margin: Length64) {
        self.safety_margin = safety_margin;
    }

    /// Enables or disables self-collision checking between `link_a` and `link_b`.
    pub fn set_pair_enabled(
        &mut self,
        link_a: usize,
        link_b: usize,
        enabled: bool,
    ) -> CuResult<()> {
        if link_a >= N || link_b >= N {
            return Err(CuError::from("cu_collision: link index out of bounds"));
        }
        self.check_pair[link_a][link_b] = enabled;
        self.check_pair[link_b][link_a] = enabled;
        Ok(())
    }

    /// Returns `true` if self-collision checking is enabled between `link_a` and `link_b`.
    pub const fn is_pair_enabled(&self, link_a: usize, link_b: usize) -> bool {
        if link_a >= N || link_b >= N {
            return false;
        }
        self.check_pair[link_a][link_b]
    }

    /// Transforms all local link shapes into world coordinates in place using `link_frames`.
    pub fn world_link_shapes(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; N],
    ) {
        let mut i = 0;
        while i < N {
            transform_shape(&link_frames[i], &self.link_shapes[i], &mut out_shapes[i]);
            i += 1;
        }
    }

    /// Evaluates self-collision and optional environment collision for precomputed `link_frames`,
    /// writing contacts and minimum signed distance into `out_report`.
    pub fn check_frames<const MAX_OBS: usize, const MAX_CONTACTS: usize>(
        &self,
        link_frames: &[Transform3D<f64>; N],
        world: Option<&CuCollisionWorld<MAX_OBS>>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) {
        out_report.clear();

        let mut world_shapes = [CuCollisionShape::default(); N];
        self.world_link_shapes(link_frames, &mut world_shapes);

        let margin_m = self.safety_margin.get::<meter>();
        let mut min_dist_m = f64::INFINITY;

        // 1. Self-collision checks
        let mut i = 0;
        while i < N {
            let mut j = i + 1;
            while j < N {
                if self.check_pair[i][j] {
                    let dist = shape_signed_distance(&world_shapes[i], &world_shapes[j]);
                    let d_m = dist.get::<meter>();
                    if d_m < min_dist_m {
                        min_dist_m = d_m;
                    }
                    if d_m < margin_m {
                        out_report.push_contact(CuCollisionContact {
                            kind: CuContactKind::SelfCollision,
                            link_index: i as u16,
                            partner_index: j as u16,
                            signed_distance: dist,
                        });
                    }
                }
                j += 1;
            }
            i += 1;
        }

        // 2. Environment collision checks
        if let Some(world) = world {
            let obs_slice = world.as_slice();
            let mut link_idx = 0;
            while link_idx < N {
                let mut obs_idx = 0;
                while obs_idx < obs_slice.len() {
                    let dist = shape_signed_distance(&world_shapes[link_idx], &obs_slice[obs_idx]);
                    let d_m = dist.get::<meter>();
                    if d_m < min_dist_m {
                        min_dist_m = d_m;
                    }
                    if d_m < margin_m {
                        out_report.push_contact(CuCollisionContact {
                            kind: CuContactKind::EnvironmentCollision,
                            link_index: link_idx as u16,
                            partner_index: obs_idx as u16,
                            signed_distance: dist,
                        });
                    }
                    obs_idx += 1;
                }
                link_idx += 1;
            }
        }

        out_report.min_signed_distance = Length64::new::<meter>(min_dist_m);
    }

    /// Evaluates forward frames for joint angles `q` and populates `out_report` in place.
    pub fn check_configuration<const MAX_OBS: usize, const MAX_CONTACTS: usize>(
        &self,
        chain: &CuKinematicChain<N>,
        q: &[Angle64; N],
        world: Option<&CuCollisionWorld<MAX_OBS>>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) -> CuResult<()> {
        let mut frames = [Transform3D::<f64>::identity(); N];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_frames(q, &mut frames, &mut ee)?;
        self.check_frames(&frames, world, out_report);
        Ok(())
    }

    /// Fast early-exit predicate returning `true` if joint configuration `q` is within joint limits
    /// and free of both self-collision and environment collision.
    pub fn is_configuration_collision_free<const MAX_OBS: usize>(
        &self,
        chain: &CuKinematicChain<N>,
        q: &[Angle64; N],
        world: Option<&CuCollisionWorld<MAX_OBS>>,
    ) -> CuResult<bool> {
        if !chain.is_within_limits(q) {
            return Ok(false);
        }
        let mut frames = [Transform3D::<f64>::identity(); N];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_frames(q, &mut frames, &mut ee)?;

        let mut world_shapes = [CuCollisionShape::default(); N];
        self.world_link_shapes(&frames, &mut world_shapes);

        let margin_m = self.safety_margin.get::<meter>();

        let mut i = 0;
        while i < N {
            let mut j = i + 1;
            while j < N {
                if self.check_pair[i][j]
                    && shape_signed_distance(&world_shapes[i], &world_shapes[j]).get::<meter>()
                        < margin_m
                {
                    return Ok(false);
                }
                j += 1;
            }
            i += 1;
        }

        if let Some(world) = world {
            let obs_slice = world.as_slice();
            let mut link_idx = 0;
            while link_idx < N {
                let mut obs_idx = 0;
                while obs_idx < obs_slice.len() {
                    if shape_signed_distance(&world_shapes[link_idx], &obs_slice[obs_idx])
                        .get::<meter>()
                        < margin_m
                    {
                        return Ok(false);
                    }
                    obs_idx += 1;
                }
                link_idx += 1;
            }
        }

        Ok(true)
    }

    /// Checks whether the linearly interpolated joint-space segment from `q_start` to `q_end`
    /// is collision-free at resolution `max_angle_step`.
    pub fn is_segment_collision_free<const MAX_OBS: usize>(
        &self,
        chain: &CuKinematicChain<N>,
        q_start: &[Angle64; N],
        q_end: &[Angle64; N],
        max_angle_step: Angle64,
        world: Option<&CuCollisionWorld<MAX_OBS>>,
    ) -> CuResult<bool> {
        let step_rad = max_angle_step.get::<radian>();
        if step_rad <= 0.0 || !step_rad.is_finite() {
            return Err(CuError::from(
                "cu_collision: max_angle_step must be positive and finite",
            ));
        }

        let mut max_delta = 0.0_f64;
        let mut j = 0;
        while j < N {
            let d = libm::fabs(q_end[j].get::<radian>() - q_start[j].get::<radian>());
            if d > max_delta {
                max_delta = d;
            }
            j += 1;
        }

        let steps = (libm::ceil(max_delta / step_rad) as usize).max(1);
        let mut s = 0;
        while s <= steps {
            let alpha = (s as f64) / (steps as f64);
            let mut q_interp = [Angle64::default(); N];
            let mut k = 0;
            while k < N {
                let a = q_start[k].get::<radian>();
                let b = q_end[k].get::<radian>();
                q_interp[k] = Angle64::new::<radian>(a + alpha * (b - a));
                k += 1;
            }
            if !self.is_configuration_collision_free(chain, &q_interp, world)? {
                return Ok(false);
            }
            s += 1;
        }
        Ok(true)
    }
}
