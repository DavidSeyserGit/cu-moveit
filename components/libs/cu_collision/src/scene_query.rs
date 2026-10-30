//! Allocation-free scene-query helpers for extra attached collision shapes.
//!
//! This module is deliberately independent of any planning-scene type. It
//! operates on the existing fixed-capacity primitives ([`CuRobotCollisionModel`],
//! [`CuCollisionWorld`], [`CuCollisionReport`], [`CuCollisionShape`]) so a
//! future `CuPlanningScene` in `scene.rs` can compose it without changing the
//! query contract: transform the scene's attached shapes by forward kinematics,
//! then check them against the robot's own world shapes and the obstacle world.
//!
//! All entry points are allocation-free: world-space shapes are written into
//! caller-provided scratch arrays and contacts are appended in place into a
//! caller-provided [`CuCollisionReport`]. Nothing is cleared here, so a caller
//! can first run [`CuRobotCollisionModel::check_frames`] (which clears and
//! fills the report) and then append attached-shape contacts into the same
//! report. `min_signed_distance` is merged with the report's existing value.
//!
//! # Contact index encoding
//!
//! Attached-shape contacts reuse [`CuContactKind`]:
//!
//! - attached vs. robot and attached vs. attached use
//!   [`CuContactKind::SelfCollision`];
//! - attached vs. obstacle use [`CuContactKind::EnvironmentCollision`].
//!
//! For attached-shape contacts, `link_index` is
//! `robot_shape_count + attached_index`, so it never collides with the robot
//! shape indices (`0..robot_shape_count`) written by
//! [`CuRobotCollisionModel::check_frames`]. `partner_index` is the robot shape
//! index for robot partners, `robot_shape_count + attached_index` for attached
//! partners, and the obstacle index for obstacle partners. Consumers that only
//! need a boolean result can use the `*_collision_free` entry points and ignore
//! this encoding entirely.
//!
//! # Exclusions
//!
//! Pairs that must not be checked (for example a gripper shape rigidly mounted
//! on the link it overlaps by construction) are listed explicitly in
//! [`CuSceneQueryExclusions`]. Attached shapes attached to the same frame are
//! always skipped, mirroring `CuRobotCollisionModel::new_attached`, because
//! shapes on one rigid body cannot move relative to each other. There is no
//! equivalent automatic skip for attached-vs-robot pairs: the model does not
//! expose its per-shape frame indices, so those pairs must be excluded
//! explicitly until it does.
//!
//! A `None` frame means a shape fixed in the base frame, so two `None` shapes
//! are also one rigid body and are skipped just like equal `Some` frames.
//!
//! Scene-level callers address exclusions by stable object ID with
//! [`CuSceneExclusions`](crate::scene_exclusions::CuSceneExclusions) and
//! resolve them against the current snapshot into this index-based set.

use crate::model::CuCollisionReport;
use crate::model::CuCollisionWorld;
use crate::model::CuContactKind;
use crate::model::CuRobotCollisionModel;
use crate::shapes::CuCollisionShape;
use crate::shapes::shape_signed_distance;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Partner category of a [`CuSceneQueryExclusion`].
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuQueryPartnerKind {
    /// A robot link shape from `CuRobotCollisionModel::link_shapes`.
    #[default]
    RobotShape,
    /// Another extra attached shape checked by this module.
    AttachedShape,
    /// An obstacle from `CuCollisionWorld::as_slice`.
    Obstacle,
}

/// One explicitly excluded attached-shape pair.
///
/// Indices are plain (non-offset) indices into the attached shape list,
/// the robot shape list, or the obstacle list, according to `partner_kind`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuSceneQueryExclusion {
    /// Index of the extra attached shape.
    pub attached_index: u16,
    /// Category of the partner that `attached_index` must not be checked against.
    pub partner_kind: CuQueryPartnerKind,
    /// Robot, attached, or obstacle index, selected by `partner_kind`.
    pub partner_index: u16,
}

impl CuSceneQueryExclusion {
    /// Excludes `attached_index` against robot shape `robot_index`.
    pub const fn robot(attached_index: u16, robot_index: u16) -> Self {
        Self {
            attached_index,
            partner_kind: CuQueryPartnerKind::RobotShape,
            partner_index: robot_index,
        }
    }

    /// Excludes `attached_index` against attached shape `other_attached_index`.
    pub const fn attached(attached_index: u16, other_attached_index: u16) -> Self {
        Self {
            attached_index,
            partner_kind: CuQueryPartnerKind::AttachedShape,
            partner_index: other_attached_index,
        }
    }

    /// Excludes `attached_index` against obstacle `obstacle_index`.
    pub const fn obstacle(attached_index: u16, obstacle_index: u16) -> Self {
        Self {
            attached_index,
            partner_kind: CuQueryPartnerKind::Obstacle,
            partner_index: obstacle_index,
        }
    }
}

/// Fixed-capacity set of explicit pair exclusions for scene queries.
///
/// This is a per-query parameter, not a message payload: it is built once
/// (usually at construction time) and borrowed by every query call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CuSceneQueryExclusions<const MAX_EXCLUSIONS: usize> {
    entries: [CuSceneQueryExclusion; MAX_EXCLUSIONS],
    len: usize,
}

impl<const MAX_EXCLUSIONS: usize> Default for CuSceneQueryExclusions<MAX_EXCLUSIONS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_EXCLUSIONS: usize> CuSceneQueryExclusions<MAX_EXCLUSIONS> {
    /// Creates an empty exclusion set.
    pub const fn new() -> Self {
        Self {
            entries: [CuSceneQueryExclusion {
                attached_index: 0,
                partner_kind: CuQueryPartnerKind::RobotShape,
                partner_index: 0,
            }; MAX_EXCLUSIONS],
            len: 0,
        }
    }

    /// Appends an exclusion if capacity permits.
    pub fn add(&mut self, exclusion: CuSceneQueryExclusion) -> CuResult<()> {
        if self.len >= MAX_EXCLUSIONS {
            return Err(CuError::from(
                "cu_collision: scene query exclusion capacity exceeded",
            ));
        }
        self.entries[self.len] = exclusion;
        self.len += 1;
        Ok(())
    }

    /// Number of active exclusions.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if no exclusions are active.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the maximum number of exclusions.
    pub const fn capacity(&self) -> usize {
        MAX_EXCLUSIONS
    }

    /// Removes all active exclusions in place.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Borrows the active exclusions.
    pub fn as_slice(&self) -> &[CuSceneQueryExclusion] {
        &self.entries[..self.len]
    }

    /// Returns `true` if the pair is explicitly excluded, in either order for
    /// attached-vs-attached pairs.
    pub fn is_excluded(
        &self,
        attached_index: usize,
        partner_kind: CuQueryPartnerKind,
        partner_index: usize,
    ) -> bool {
        self.as_slice().iter().any(|entry| {
            let attached = entry.attached_index as usize;
            let partner = entry.partner_index as usize;
            (attached == attached_index
                && entry.partner_kind == partner_kind
                && partner == partner_index)
                || (partner_kind == CuQueryPartnerKind::AttachedShape
                    && attached == partner_index
                    && partner == attached_index)
        })
    }
}

/// Transforms local attached shapes into world coordinates in place.
///
/// `attached_frames[i]` selects the link frame for shape `i`, or `None` for a
/// shape fixed in the base frame (copied unchanged), mirroring
/// `CuRobotCollisionModel::new_attached`. Only the first `attached_len`
/// entries are read and written.
pub fn transform_attached_shapes<const N: usize, const A: usize>(
    link_frames: &[Transform3D<f64>; N],
    attached_shapes: &[CuCollisionShape; A],
    attached_frames: &[Option<usize>; A],
    attached_len: usize,
    out_world_shapes: &mut [CuCollisionShape; A],
) -> CuResult<()> {
    validate_attached(attached_len, A, attached_frames, N)?;
    let mut i = 0;
    while i < attached_len {
        if let Some(frame) = attached_frames[i] {
            transform_shape(
                &link_frames[frame],
                &attached_shapes[i],
                &mut out_world_shapes[i],
            );
        } else {
            out_world_shapes[i] = attached_shapes[i];
        }
        i += 1;
    }
    Ok(())
}

/// Checks pre-transformed attached shapes against robot and obstacle shapes,
/// appending contacts into `out_report` without clearing it.
///
/// `attached_frames` runs parallel to `attached_world_shapes` and is only used
/// for the automatic same-frame skip between attached shapes; pass the frames
/// the shapes were transformed with. Distances are also merged into
/// `out_report.min_signed_distance`, including for pairs that do not violate
/// `safety_margin`.
///
/// Transactional with respect to the report: all inputs are validated before
/// the first pair is observed, so on error `out_report` is left unmodified.
pub fn check_attached_world_shapes<const MAX_CONTACTS: usize, const MAX_EXCLUSIONS: usize>(
    attached_world_shapes: &[CuCollisionShape],
    attached_frames: &[Option<usize>],
    robot_world_shapes: &[CuCollisionShape],
    obstacle_shapes: &[CuCollisionShape],
    exclusions: &CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    safety_margin: Length64,
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
) -> CuResult<()> {
    validate_margin(safety_margin)?;
    if attached_frames.len() != attached_world_shapes.len()
        || attached_world_shapes.len() > u16::MAX as usize
        || robot_world_shapes.len() > u16::MAX as usize
        || obstacle_shapes.len() > u16::MAX as usize
        || attached_world_shapes.len() + robot_world_shapes.len() > u16::MAX as usize
    {
        return Err(CuError::from(
            "cu_collision: scene query shape count out of range",
        ));
    }
    validate_exclusions(
        exclusions,
        attached_world_shapes.len(),
        robot_world_shapes.len(),
        obstacle_shapes.len(),
    )?;

    let robot_count = robot_world_shapes.len();
    let margin_m = safety_margin.get::<meter>();
    let mut min_dist_m = out_report.min_signed_distance.get::<meter>();

    // 1. Attached vs. attached (same-frame pairs cannot move relatively).
    let mut i = 0;
    while i < attached_world_shapes.len() {
        let mut j = i + 1;
        while j < attached_world_shapes.len() {
            // Equal frames are one rigid body, including `None == None` (both
            // fixed in the base frame); they cannot move relative to each other.
            let same_frame = attached_frames[i] == attached_frames[j];
            if !same_frame && !exclusions.is_excluded(i, CuQueryPartnerKind::AttachedShape, j) {
                let dist =
                    shape_signed_distance(&attached_world_shapes[i], &attached_world_shapes[j]);
                observe(
                    out_report,
                    &mut min_dist_m,
                    margin_m,
                    dist,
                    CuContactKind::SelfCollision,
                    (robot_count + i) as u16,
                    (robot_count + j) as u16,
                );
            }
            j += 1;
        }
        i += 1;
    }

    // 2. Attached vs. robot link shapes.
    let mut i = 0;
    while i < attached_world_shapes.len() {
        let mut r = 0;
        while r < robot_count {
            if !exclusions.is_excluded(i, CuQueryPartnerKind::RobotShape, r) {
                let dist = shape_signed_distance(&attached_world_shapes[i], &robot_world_shapes[r]);
                observe(
                    out_report,
                    &mut min_dist_m,
                    margin_m,
                    dist,
                    CuContactKind::SelfCollision,
                    (robot_count + i) as u16,
                    r as u16,
                );
            }
            r += 1;
        }
        i += 1;
    }

    // 3. Attached vs. environment obstacles.
    let mut i = 0;
    while i < attached_world_shapes.len() {
        let mut o = 0;
        while o < obstacle_shapes.len() {
            if !exclusions.is_excluded(i, CuQueryPartnerKind::Obstacle, o) {
                let dist = shape_signed_distance(&attached_world_shapes[i], &obstacle_shapes[o]);
                observe(
                    out_report,
                    &mut min_dist_m,
                    margin_m,
                    dist,
                    CuContactKind::EnvironmentCollision,
                    (robot_count + i) as u16,
                    o as u16,
                );
            }
            o += 1;
        }
        i += 1;
    }

    out_report.min_signed_distance = Length64::new::<meter>(min_dist_m);
    Ok(())
}

/// Early-exit predicate over pre-transformed shapes. Returns `true` when no
/// non-excluded pair violates `safety_margin`.
pub fn attached_world_shapes_collision_free<const MAX_EXCLUSIONS: usize>(
    attached_world_shapes: &[CuCollisionShape],
    attached_frames: &[Option<usize>],
    robot_world_shapes: &[CuCollisionShape],
    obstacle_shapes: &[CuCollisionShape],
    exclusions: &CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    safety_margin: Length64,
) -> CuResult<bool> {
    let mut report = CuCollisionReport::<0>::default();
    check_attached_world_shapes(
        attached_world_shapes,
        attached_frames,
        robot_world_shapes,
        obstacle_shapes,
        exclusions,
        safety_margin,
        &mut report,
    )?;
    Ok(report.is_collision_free())
}

/// Transforms robot and attached shapes for precomputed `link_frames` and
/// appends attached-shape contacts into `out_report` without clearing it.
///
/// Scratch arrays are caller-owned so repeated queries reuse the same storage.
/// Pair the call with `model.check_frames(link_frames, world, out_report)`
/// (which clears the report first) for a combined robot + attached result.
/// On error, `out_report` is left unmodified.
#[allow(clippy::too_many_arguments)]
pub fn check_attached_shapes<
    const N: usize,
    const M: usize,
    const A: usize,
    const MAX_OBS: usize,
    const MAX_CONTACTS: usize,
    const MAX_EXCLUSIONS: usize,
>(
    model: &CuRobotCollisionModel<N, M>,
    link_frames: &[Transform3D<f64>; N],
    attached_shapes: &[CuCollisionShape; A],
    attached_frames: &[Option<usize>; A],
    attached_len: usize,
    world: Option<&CuCollisionWorld<MAX_OBS>>,
    exclusions: &CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    out_attached_world_shapes: &mut [CuCollisionShape; A],
    out_robot_world_shapes: &mut [CuCollisionShape; M],
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
) -> CuResult<()> {
    if model.shape_count() > M {
        return Err(CuError::from(
            "cu_collision: robot shape count exceeds output capacity",
        ));
    }
    transform_attached_shapes(
        link_frames,
        attached_shapes,
        attached_frames,
        attached_len,
        out_attached_world_shapes,
    )?;
    model.world_link_shapes(link_frames, out_robot_world_shapes);
    let robot_count = model.shape_count();
    let obstacles: &[CuCollisionShape] = match world {
        Some(world) => world.as_slice(),
        None => &[],
    };
    check_attached_world_shapes(
        &out_attached_world_shapes[..attached_len],
        &attached_frames[..attached_len],
        &out_robot_world_shapes[..robot_count],
        obstacles,
        exclusions,
        model.safety_margin(),
        out_report,
    )
}

/// Runs forward kinematics for `q` and appends attached-shape contacts into
/// `out_report` without clearing it. See [`check_attached_shapes`]. On error,
/// `out_report` is left unmodified.
#[allow(clippy::too_many_arguments)]
pub fn check_attached_configuration<
    const N: usize,
    const M: usize,
    const A: usize,
    const MAX_OBS: usize,
    const MAX_CONTACTS: usize,
    const MAX_EXCLUSIONS: usize,
>(
    model: &CuRobotCollisionModel<N, M>,
    chain: &CuKinematicChain<N>,
    q: &[CuJointPosition; N],
    attached_shapes: &[CuCollisionShape; A],
    attached_frames: &[Option<usize>; A],
    attached_len: usize,
    world: Option<&CuCollisionWorld<MAX_OBS>>,
    exclusions: &CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    out_attached_world_shapes: &mut [CuCollisionShape; A],
    out_robot_world_shapes: &mut [CuCollisionShape; M],
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
) -> CuResult<()> {
    let mut frames = [Transform3D::<f64>::identity(); N];
    let mut ee = Transform3D::<f64>::identity();
    chain.forward_frames(q, &mut frames, &mut ee)?;
    check_attached_shapes(
        model,
        &frames,
        attached_shapes,
        attached_frames,
        attached_len,
        world,
        exclusions,
        out_attached_world_shapes,
        out_robot_world_shapes,
        out_report,
    )
}

fn validate_attached<const A: usize>(
    attached_len: usize,
    capacity: usize,
    attached_frames: &[Option<usize>; A],
    frame_count: usize,
) -> CuResult<()> {
    if attached_len > capacity
        || capacity > u16::MAX as usize
        || attached_frames[..attached_len.min(A)]
            .iter()
            .flatten()
            .any(|&frame| frame >= frame_count)
    {
        return Err(CuError::from(
            "cu_collision: attached shape frame or capacity out of range",
        ));
    }
    Ok(())
}

fn validate_margin(safety_margin: Length64) -> CuResult<()> {
    let margin_m = safety_margin.get::<meter>();
    if !margin_m.is_finite() || margin_m < 0.0 {
        return Err(CuError::from("cu_collision: invalid safety margin"));
    }
    Ok(())
}

fn validate_exclusions<const MAX_EXCLUSIONS: usize>(
    exclusions: &CuSceneQueryExclusions<MAX_EXCLUSIONS>,
    attached_count: usize,
    robot_count: usize,
    obstacle_count: usize,
) -> CuResult<()> {
    let in_range = exclusions.as_slice().iter().all(|entry| {
        let attached = entry.attached_index as usize;
        let partner = entry.partner_index as usize;
        attached < attached_count
            && match entry.partner_kind {
                CuQueryPartnerKind::RobotShape => partner < robot_count,
                CuQueryPartnerKind::AttachedShape => partner < attached_count,
                CuQueryPartnerKind::Obstacle => partner < obstacle_count,
            }
    });
    if in_range {
        Ok(())
    } else {
        Err(CuError::from(
            "cu_collision: scene query exclusion index out of range",
        ))
    }
}

/// Merges one pair distance into the report, recording a contact when the
/// pair violates the margin. Mirrors the private `CuCollisionReport::push_contact`
/// accounting through the report's public fields.
#[allow(clippy::too_many_arguments)]
fn observe<const MAX_CONTACTS: usize>(
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    min_dist_m: &mut f64,
    margin_m: f64,
    dist: Length64,
    kind: CuContactKind,
    link_index: u16,
    partner_index: u16,
) {
    let d_m = dist.get::<meter>();
    if d_m < *min_dist_m {
        *min_dist_m = d_m;
    }
    if d_m < margin_m {
        out_report.total_contacts = out_report.total_contacts.saturating_add(1);
        let idx = out_report.len as usize;
        if idx < MAX_CONTACTS {
            out_report.contacts[idx] = crate::model::CuCollisionContact {
                kind,
                link_index,
                partner_index,
                signed_distance: dist,
            };
            out_report.len += 1;
        }
    }
}
