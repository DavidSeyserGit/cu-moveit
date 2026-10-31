//! Robot link collision models, their parameters, and in-place collision reports.

use crate::env::CuCollisionEnv;
use crate::scene_exclusions::CuSceneExclusionPartner as Partner;
use crate::scene_exclusions::validate_exclusions;
use crate::shapes::CuCollisionShape;
use crate::shapes::CuLinkShape;
use crate::shapes::shape_signed_distance;
use crate::shapes::transform_shape;
use CuContactKind::EnvironmentCollision;
use CuContactKind::SelfCollision;
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
    #[serde(with = "crate::serde_arrays::contact_array")]
    contacts: [CuCollisionContact; MAX_CONTACTS],
    /// Number of valid entries in `contacts`.
    len: u32,
    /// Total detected contacts, including those exceeding storage capacity.
    total_contacts: u32,
    /// Minimum surface-to-surface signed distance observed across all checked pairs.
    min_signed_distance: Length64,
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

    /// Returns the stored contacts (at most `MAX_CONTACTS`).
    pub fn as_slice(&self) -> &[CuCollisionContact] {
        let count = (self.len as usize).min(MAX_CONTACTS);
        &self.contacts[..count]
    }

    /// Total detected contacts, including those that did not fit into storage.
    pub const fn total_contacts(&self) -> usize {
        self.total_contacts as usize
    }

    /// Minimum surface-to-surface signed distance over all checked pairs
    /// (`+inf` when no pair was checked; negative means penetration).
    pub const fn min_signed_distance(&self) -> Length64 {
        self.min_signed_distance
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

/// Parameters of a [`CuRobotCollisionModel`].
///
/// ```ron
/// {"safety_margin": 0.005, "skip_adjacent_links": true}
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuCollisionModelParams {
    /// Minimum surface distance for a pair to count as collision-free (finite, `>= 0`).
    pub safety_margin: Length64,
    /// Disable self-collision pairs between shapes on adjacent joint frames
    /// (`Some(i)` and `Some(i ± 1)`). Shapes on the same frame are never checked against each
    /// other regardless of this flag.
    pub skip_adjacent_links: bool,
}

impl Default for CuCollisionModelParams {
    fn default() -> Self {
        Self {
            safety_margin: Length64::new::<meter>(0.0),
            skip_adjacent_links: true,
        }
    }
}

impl CuCollisionModelParams {
    /// Checks that the safety margin is finite and non-negative.
    pub fn validate(&self) -> CuResult<()> {
        validate_margin(self.safety_margin)
    }
}

fn validate_margin(margin: Length64) -> CuResult<()> {
    let m = margin.get::<meter>();
    if m.is_finite() && m >= 0.0 {
        Ok(())
    } else {
        Err(CuError::from("cu_collision: invalid safety margin"))
    }
}

/// Fixed-capacity model for `N` movable frames and up to `M` collision shapes.
///
/// Each shape is a [`CuLinkShape`] on a joint frame or the base. Shape indices identify contacts
/// and collision pairs; several shapes may share a frame.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuRobotCollisionModel<const N: usize, const M: usize = N> {
    shapes: [CuLinkShape; M],
    safety_margin: Length64,
    check_pair: [[bool; M]; M],
    len: usize,
}

// The bincode derive currently emits invalid impl generics for dependent const defaults.
impl<const N: usize, const M: usize> Encode for CuRobotCollisionModel<N, M> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.shapes, encoder)?;
        Encode::encode(&self.safety_margin, encoder)?;
        Encode::encode(&self.check_pair, encoder)?;
        Encode::encode(&self.len, encoder)
    }
}
impl<C, const N: usize, const M: usize> Decode<C> for CuRobotCollisionModel<N, M> {
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let model = Self {
            shapes: Decode::decode(decoder)?,
            safety_margin: Decode::decode(decoder)?,
            check_pair: Decode::decode(decoder)?,
            len: Decode::decode(decoder)?,
        };
        if model.len > M || model.link_shapes().iter().any(|s| s.frame >= Some(N)) {
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

impl<const N: usize, const M: usize> CuRobotCollisionModel<N, M> {
    /// Creates a model from up to `M` link shapes.
    ///
    /// Self-collision is checked between every pair of shapes on different frames, except
    /// adjacent joint frames when `params.skip_adjacent_links` is set. Use
    /// [`set_pair_enabled`](Self::set_pair_enabled) for further adjustments.
    ///
    /// Fails if more than `M` shapes are given, a frame index is `>= N`, a shape has invalid
    /// geometry, or the params are invalid.
    pub fn new(shapes: &[CuLinkShape], params: CuCollisionModelParams) -> CuResult<Self> {
        params.validate()?;
        if shapes.len() > M || M > u16::MAX as usize {
            return Err(CuError::from(
                "cu_collision: collision model shape count exceeds capacity",
            ));
        }
        if shapes.iter().any(|s| s.frame >= Some(N)) {
            return Err(CuError::from(
                "cu_collision: collision model frame index out of range",
            ));
        }
        for s in shapes {
            s.shape.validate()?;
        }
        Ok(Self::assemble(shapes, params))
    }

    /// Builds the model from already validated inputs.
    pub(crate) fn assemble(shapes: &[CuLinkShape], params: CuCollisionModelParams) -> Self {
        let mut model = Self {
            shapes: [CuLinkShape::default(); M],
            safety_margin: params.safety_margin,
            check_pair: [[false; M]; M],
            len: shapes.len(),
        };
        model.shapes[..shapes.len()].copy_from_slice(shapes);
        for i in 0..shapes.len() {
            for j in i + 1..shapes.len() {
                let (a, b) = (shapes[i].frame, shapes[j].frame);
                // Shapes on the same rigid body cannot move relative to one another.
                let adjacent = matches!((a, b), (Some(a), Some(b)) if a.abs_diff(b) == 1);
                let enabled = a != b && !(params.skip_adjacent_links && adjacent);
                model.check_pair[i][j] = enabled;
                model.check_pair[j][i] = enabled;
            }
        }
        model
    }

    /// Number of active collision shapes.
    pub const fn shape_count(&self) -> usize {
        self.len
    }

    /// Borrows the active link shapes.
    pub fn link_shapes(&self) -> &[CuLinkShape] {
        &self.shapes[..self.len]
    }

    /// Returns the configured safety margin.
    pub const fn safety_margin(&self) -> Length64 {
        self.safety_margin
    }

    /// Sets the safety margin required for a pair to be considered collision-free.
    pub fn set_safety_margin(&mut self, safety_margin: Length64) -> CuResult<()> {
        validate_margin(safety_margin)?;
        self.safety_margin = safety_margin;
        Ok(())
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

    /// Transforms the active link shapes into world coordinates in place using `link_frames`.
    /// Only the first [`shape_count`](Self::shape_count) entries of `out_shapes` are written.
    pub fn world_link_shapes(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; M],
    ) {
        for (link, out) in self.link_shapes().iter().zip(out_shapes.iter_mut()) {
            *out = world_shape(link_frames, link.frame, &link.shape);
        }
    }

    /// Evaluates self-collision and `env` (obstacles, attached shapes, exclusions) for
    /// precomputed `link_frames`, writing contacts and the minimum signed distance into
    /// `out_report` (cleared first).
    ///
    /// Fails, leaving `out_report` unmodified, if `env` is invalid for this model (see
    /// [`CuCollisionEnv`]).
    pub fn check_frames<const MAX_CONTACTS: usize>(
        &self,
        link_frames: &[Transform3D<f64>; N],
        env: &CuCollisionEnv<'_>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) -> CuResult<()> {
        self.validate_env(env)?;
        out_report.clear();
        let margin_m = self.safety_margin.get::<meter>();
        let mut min_dist_m = f64::INFINITY;
        self.visit_pairs(link_frames, env, |contact| {
            let d_m = contact.signed_distance.get::<meter>();
            min_dist_m = min_dist_m.min(d_m);
            if d_m < margin_m {
                out_report.push_contact(contact);
            }
            true
        });
        out_report.min_signed_distance = Length64::new::<meter>(min_dist_m);
        Ok(())
    }

    /// Runs forward kinematics for `q` and calls [`check_frames`](Self::check_frames).
    pub fn check_configuration<const MAX_CONTACTS: usize>(
        &self,
        chain: &CuKinematicChain<N>,
        q: &[CuJointPosition; N],
        env: &CuCollisionEnv<'_>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) -> CuResult<()> {
        let mut frames = [Transform3D::<f64>::identity(); N];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_frames(q, &mut frames, &mut ee)?;
        self.check_frames(&frames, env, out_report)
    }

    /// Early-exit predicate: `true` if `q` is within joint limits and no checked pair (self,
    /// environment, attached) violates the safety margin.
    pub fn is_configuration_collision_free(
        &self,
        chain: &CuKinematicChain<N>,
        q: &[CuJointPosition; N],
        env: &CuCollisionEnv<'_>,
    ) -> CuResult<bool> {
        self.validate_env(env)?;
        if !chain.is_within_limits(q) {
            return Ok(false);
        }
        let mut frames = [Transform3D::<f64>::identity(); N];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_frames(q, &mut frames, &mut ee)?;
        let margin_m = self.safety_margin.get::<meter>();
        let mut free = true;
        self.visit_pairs(&frames, env, |contact| {
            free = contact.signed_distance.get::<meter>() >= margin_m;
            free
        });
        Ok(free)
    }

    /// Checks whether the linearly interpolated joint-space segment from `q_start` to `q_end`
    /// is collision-free at the given angular and linear sampling resolution.
    /// This is discrete sampling, not swept-volume continuous collision detection.
    pub fn is_segment_collision_free(
        &self,
        chain: &CuKinematicChain<N>,
        q_start: &[CuJointPosition; N],
        q_end: &[CuJointPosition; N],
        max_step: impl Into<CuJointResolution>,
        env: &CuCollisionEnv<'_>,
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
        if max_delta.is_nan() || max_delta > 1_000_000.0 {
            return Err(CuError::from("Collision segment requires too many samples"));
        }
        let steps = (libm::ceil(max_delta) as usize).max(1);
        for s in 0..=steps {
            let alpha = (s as f64) / (steps as f64);
            let mut q_interp = [CuJointPosition::default(); N];
            for k in 0..N {
                q_interp[k] = q_start[k].interpolate(q_end[k], alpha)?;
            }
            if !self.is_configuration_collision_free(chain, &q_interp, env)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Checks that `env` can be evaluated against this model.
    fn validate_env(&self, env: &CuCollisionEnv<'_>) -> CuResult<()> {
        if env.obstacles().len() > u16::MAX as usize
            || self.len + env.attached().len() > u16::MAX as usize
        {
            return Err(CuError::from(
                "cu_collision: scene query shape count out of range",
            ));
        }
        if env.attached().iter().any(|a| a.frame >= Some(N)) {
            return Err(CuError::from(
                "cu_collision: attached shape frame out of range",
            ));
        }
        validate_exclusions(env.exclusions(), env, self.len)
    }

    /// Visits every checked pair in a fixed order (robot self, robot vs. obstacles, attached vs.
    /// attached, attached vs. robot, attached vs. obstacles); `visit` returns `false` to stop.
    fn visit_pairs(
        &self,
        link_frames: &[Transform3D<f64>; N],
        env: &CuCollisionEnv<'_>,
        mut visit: impl FnMut(CuCollisionContact) -> bool,
    ) {
        let mut robot = [CuCollisionShape::default(); M];
        self.world_link_shapes(link_frames, &mut robot);
        let robot = &robot[..self.len];
        let obstacles = env.obstacles();
        let attached = env.attached();
        let robot_count = robot.len();
        let mut emit =
            |kind, link: usize, partner: usize, a: &CuCollisionShape, b: &CuCollisionShape| {
                visit(CuCollisionContact {
                    kind,
                    link_index: link as u16,
                    partner_index: partner as u16,
                    signed_distance: shape_signed_distance(a, b),
                })
            };

        for i in 0..robot_count {
            for j in i + 1..robot_count {
                if self.check_pair[i][j] && !emit(SelfCollision, i, j, &robot[i], &robot[j]) {
                    return;
                }
            }
        }
        for (i, r) in robot.iter().enumerate() {
            for (o, obstacle) in obstacles.iter().enumerate() {
                if !emit(EnvironmentCollision, i, o, r, obstacle) {
                    return;
                }
            }
        }
        for (i, att) in attached.iter().enumerate() {
            let w = world_shape(link_frames, att.frame, &att.shape);
            let link = robot_count + i;
            for (j, other) in attached.iter().enumerate().skip(i + 1) {
                // Equal frames (including `None == None`) are one rigid body.
                if att.frame == other.frame
                    || env.is_excluded(
                        att.id,
                        Partner::Attached {
                            attached_id: other.id,
                        },
                    )
                {
                    continue;
                }
                let wo = world_shape(link_frames, other.frame, &other.shape);
                if !emit(SelfCollision, link, robot_count + j, &w, &wo) {
                    return;
                }
            }
        }
        for (i, att) in attached.iter().enumerate() {
            let w = world_shape(link_frames, att.frame, &att.shape);
            let link = robot_count + i;
            for (r, shape) in robot.iter().enumerate() {
                let partner = Partner::RobotShape {
                    shape_index: r as u16,
                };
                if !env.is_excluded(att.id, partner) && !emit(SelfCollision, link, r, &w, shape) {
                    return;
                }
            }
        }
        for (i, att) in attached.iter().enumerate() {
            let w = world_shape(link_frames, att.frame, &att.shape);
            let link = robot_count + i;
            for (o, obstacle) in obstacles.iter().enumerate() {
                let excluded = env.obstacle_id(o).is_some_and(|obstacle_id| {
                    env.is_excluded(att.id, Partner::Obstacle { obstacle_id })
                });
                if !excluded && !emit(EnvironmentCollision, link, o, &w, obstacle) {
                    return;
                }
            }
        }
    }
}

/// World-space copy of `shape` expressed in `frame` (`None` = base, unchanged).
fn world_shape<const N: usize>(
    link_frames: &[Transform3D<f64>; N],
    frame: Option<usize>,
    shape: &CuCollisionShape,
) -> CuCollisionShape {
    match frame {
        Some(f) => {
            let mut out = *shape;
            transform_shape(&link_frames[f], shape, &mut out);
            out
        }
        None => *shape,
    }
}
