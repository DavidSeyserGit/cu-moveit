//! Robot link collision models, fixed-capacity obstacle worlds, and in-place collision reports.

use crate::shapes::CuCollisionShape;
use crate::shapes::shape_signed_distance;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
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
    /// Robot collision-shape index.
    pub link_index: u16,
    /// Partner shape index for self-collision, or obstacle index for environment collision.
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
    /// Total detected contacts, including those exceeding storage capacity.
    pub total_contacts: u32,
    /// Minimum surface-to-surface signed distance observed across all checked pairs.
    pub min_signed_distance: Length64,
}

impl<const MAX_CONTACTS: usize> Default for CuCollisionReport<MAX_CONTACTS> {
    fn default() -> Self {
        Self {
            contacts: [CuCollisionContact::default(); MAX_CONTACTS],
            len: 0,
            total_contacts: 0,
            min_signed_distance: Length64::new::<meter>(f64::INFINITY),
        }
    }
}

impl<const MAX_CONTACTS: usize> CuCollisionReport<MAX_CONTACTS> {
    /// Resets the collision report in place for reuse.
    pub fn clear(&mut self) {
        self.len = 0;
        self.total_contacts = 0;
        self.min_signed_distance = Length64::new::<meter>(f64::INFINITY);
    }

    /// Returns `true` if no self-collisions or environment collisions were recorded.
    pub const fn is_collision_free(&self) -> bool {
        self.total_contacts == 0
    }

    /// Returns the recorded contacts slice `&contacts[..len]`.
    pub fn as_slice(&self) -> &[CuCollisionContact] {
        let count = (self.len as usize).min(MAX_CONTACTS);
        &self.contacts[..count]
    }

    fn push_contact(&mut self, contact: CuCollisionContact) {
        self.total_contacts = self.total_contacts.saturating_add(1);
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

/// Fixed-capacity model for `N` movable frames and up to `M` attached collision shapes.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuRobotCollisionModel<const N: usize, const M: usize = N> {
    link_shapes: [CuCollisionShape; M],
    safety_margin: Length64,
    check_pair: [[bool; M]; M],
    frame_indices: [Option<usize>; M],
    len: usize,
}

// The bincode derive currently emits invalid impl generics for dependent const defaults.
impl<const N: usize, const M: usize> Encode for CuRobotCollisionModel<N, M> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.link_shapes, encoder)?;
        Encode::encode(&self.safety_margin, encoder)?;
        Encode::encode(&self.check_pair, encoder)?;
        Encode::encode(&self.frame_indices, encoder)?;
        Encode::encode(&self.len, encoder)
    }
}
impl<C, const N: usize, const M: usize> Decode<C> for CuRobotCollisionModel<N, M> {
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let model = Self {
            link_shapes: Decode::decode(decoder)?,
            safety_margin: Decode::decode(decoder)?,
            check_pair: Decode::decode(decoder)?,
            frame_indices: Decode::decode(decoder)?,
            len: Decode::decode(decoder)?,
        };
        if model.len > M
            || model.frame_indices[..model.len]
                .iter()
                .flatten()
                .any(|&i| i >= N)
        {
            return Err(bincode::error::DecodeError::Other(
                "Invalid collision model snapshot",
            ));
        }
        Ok(model)
    }
}

impl<'de, C, const N: usize, const M: usize> bincode::BorrowDecode<'de, C>
    for CuRobotCollisionModel<N, M>
{
    fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Decode::decode(decoder)
    }
}

#[derive(Deserialize)]
struct CollisionModelConfig<const M: usize> {
    #[serde(with = "crate::serde_arrays::collision_shape_array")]
    link_shapes: [CuCollisionShape; M],
    safety_margin: Length64,
    #[serde(default)]
    frame_indices: Option<FrameIndices<M>>,
}

#[derive(Deserialize)]
struct FrameIndices<const M: usize>(
    #[serde(with = "crate::serde_arrays::frame_index_array")] [Option<usize>; M],
);

impl<const N: usize> CuRobotCollisionModel<N> {
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
            frame_indices: core::array::from_fn(Some),
            len: N,
        }
    }
}

impl<const N: usize, const M: usize> CuRobotCollisionModel<N, M> {
    /// Loads a collision model. Without `frame_indices`, each shape attaches to the same-index joint.
    pub fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let config =
            config.ok_or_else(|| CuError::from("Missing collision model configuration"))?;
        let model = config
            .get_value::<CollisionModelConfig<M>>("collision_model")?
            .ok_or_else(|| CuError::from("Missing required collision_model configuration"))?;
        let frames = model
            .frame_indices
            .map_or_else(|| core::array::from_fn(Some), |v| v.0);
        let mut out = Self::new_attached(model.link_shapes, frames, M, model.safety_margin)?;
        for i in 0..M {
            for j in i + 1..M {
                if let (Some(a), Some(b)) = (frames[i], frames[j])
                    && a.abs_diff(b) == 1
                {
                    out.set_pair_enabled(i, j, false)?;
                }
            }
        }
        Ok(out)
    }

    /// Creates a fixed-capacity model with shapes attached to joint frames or the base (`None`).
    /// Shape indices identify contacts and collision pairs; several shapes may share a frame.
    pub fn new_attached(
        shapes: [CuCollisionShape; M],
        frames: [Option<usize>; M],
        len: usize,
        safety_margin: Length64,
    ) -> CuResult<Self> {
        if len > M
            || M > u16::MAX as usize
            || frames[..len].iter().flatten().any(|&i| i >= N)
            || !safety_margin.get::<meter>().is_finite()
            || safety_margin.get::<meter>() < 0.0
        {
            return Err(CuError::from(
                "Invalid collision attachments, capacity, or margin",
            ));
        }
        let mut model = Self {
            link_shapes: shapes,
            frame_indices: frames,
            len,
            safety_margin,
            check_pair: [[false; M]; M],
        };
        for i in 0..len {
            for j in i + 1..len {
                // Shapes on the same rigid body cannot move relative to one another.
                model.check_pair[i][j] = frames[i] != frames[j];
                model.check_pair[j][i] = frames[i] != frames[j];
            }
        }
        Ok(model)
    }

    /// Number of active collision shapes.
    pub const fn shape_count(&self) -> usize {
        self.len
    }

    /// Borrows shape storage. Only the first `shape_count()` entries are active.
    pub const fn link_shapes(&self) -> &[CuCollisionShape; M] {
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

    /// Enables or disables self-collision checking between two shape indices.
    pub fn set_pair_enabled(
        &mut self,
        link_a: usize,
        link_b: usize,
        enabled: bool,
    ) -> CuResult<()> {
        if link_a >= self.len || link_b >= self.len {
            return Err(CuError::from("cu_collision: link index out of bounds"));
        }
        self.check_pair[link_a][link_b] = enabled;
        self.check_pair[link_b][link_a] = enabled;
        Ok(())
    }

    /// Returns `true` if self-collision checking is enabled between `link_a` and `link_b`.
    pub const fn is_pair_enabled(&self, link_a: usize, link_b: usize) -> bool {
        if link_a >= self.len || link_b >= self.len {
            return false;
        }
        self.check_pair[link_a][link_b]
    }

    /// Transforms all local link shapes into world coordinates in place using `link_frames`.
    pub fn world_link_shapes(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; M],
    ) {
        let mut i = 0;
        while i < self.len {
            if let Some(frame) = self.frame_indices[i] {
                transform_shape(
                    &link_frames[frame],
                    &self.link_shapes[i],
                    &mut out_shapes[i],
                );
            } else {
                out_shapes[i] = self.link_shapes[i];
            }
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

        let mut world_shapes = [CuCollisionShape::default(); M];
        self.world_link_shapes(link_frames, &mut world_shapes);

        let margin_m = self.safety_margin.get::<meter>();
        let mut min_dist_m = f64::INFINITY;

        // 1. Self-collision checks
        let mut i = 0;
        while i < self.len {
            let mut j = i + 1;
            while j < self.len {
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
            while link_idx < self.len {
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
        q: &[CuJointPosition; N],
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
        q: &[CuJointPosition; N],
        world: Option<&CuCollisionWorld<MAX_OBS>>,
    ) -> CuResult<bool> {
        if !chain.is_within_limits(q) {
            return Ok(false);
        }
        let mut frames = [Transform3D::<f64>::identity(); N];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_frames(q, &mut frames, &mut ee)?;

        let mut world_shapes = [CuCollisionShape::default(); M];
        self.world_link_shapes(&frames, &mut world_shapes);

        let margin_m = self.safety_margin.get::<meter>();

        let mut i = 0;
        while i < self.len {
            let mut j = i + 1;
            while j < self.len {
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
            while link_idx < self.len {
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
    /// is collision-free at the configured angular and linear sampling resolution.
    /// This is discrete sampling, not swept-volume continuous collision detection.
    pub fn is_segment_collision_free<const MAX_OBS: usize>(
        &self,
        chain: &CuKinematicChain<N>,
        q_start: &[CuJointPosition; N],
        q_end: &[CuJointPosition; N],
        max_step: impl Into<CuJointResolution>,
        world: Option<&CuCollisionWorld<MAX_OBS>>,
    ) -> CuResult<bool> {
        let metric = max_step.into().metric();
        metric.validate()?;
        let mut max_delta = 0.0_f64;
        for j in 0..N {
            chain.joints()[j].validate_coordinate(q_start[j])?;
            chain.joints()[j].validate_coordinate(q_end[j])?;
            max_delta =
                max_delta.max((metric.normalized(q_end[j]) - metric.normalized(q_start[j])).abs());
        }
        // Bound per-call work, including adversarial/non-finite inputs.
        if max_delta > 1_000_000.0 {
            return Err(CuError::from("Collision segment requires too many samples"));
        }
        let steps = (libm::ceil(max_delta) as usize).max(1);
        let mut s = 0;
        while s <= steps {
            let alpha = (s as f64) / (steps as f64);
            let mut q_interp = [CuJointPosition::default(); N];
            let mut k = 0;
            while k < N {
                q_interp[k] = q_start[k].interpolate(q_end[k], alpha)?;
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
