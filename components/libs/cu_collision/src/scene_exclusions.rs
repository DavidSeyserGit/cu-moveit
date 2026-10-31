//! Stable-ID scene exclusions for attached-object collision pairs.
//!
//! Mounting contact between a grasped object and the robot must be permitted:
//! an attached shape overlaps its mount link by construction. Exclusions name
//! those pairs with stable [`CuObjectId`]s so they survive scene compaction,
//! alongside robot shape indices for attached-vs-robot pairs.
//!
//! [`CuSceneExclusions`] is a fixed-capacity `CuMsgPayload` published alongside
//! [`CuPlanningScene`]. Pass it to a check with
//! [`CuCollisionEnv::with_exclusions`](crate::CuCollisionEnv::with_exclusions); every
//! check validates it against the environment and rejects invalid references
//! (unassigned IDs, IDs absent from the snapshot, self-references, out-of-range
//! robot shape indices) instead of silently skipping them.

use crate::env::CuCollisionEnv;
use crate::scene::CuObjectId;
use crate::scene::CuPlanningScene;
use crate::scene::CuSceneError;
use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Default capacity for stable-ID exclusions in [`CuSceneExclusions`].
pub const DEFAULT_MAX_EXCLUSIONS: usize = 16;

/// Partner of an attached shape in a [`CuSceneExclusion`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuSceneExclusionPartner {
    /// A robot link shape from `CuRobotCollisionModel::link_shapes`, by shape index.
    RobotShape {
        /// Index into the robot collision model's active shapes.
        shape_index: u16,
    },
    /// Another attached shape in the same scene snapshot, by stable ID.
    Attached {
        /// Stable identifier of the partner attached shape.
        attached_id: CuObjectId,
    },
    /// An environment obstacle in the same scene snapshot, by stable ID.
    Obstacle {
        /// Stable identifier of the partner obstacle.
        obstacle_id: CuObjectId,
    },
}

/// One excluded attached-shape pair, addressed by stable IDs where the scene owns them.
///
/// The subject is always an attached shape; the partner selects robot, attached,
/// or obstacle geometry. Attached-vs-attached pairs apply symmetrically.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuSceneExclusion {
    /// Stable identifier of the attached shape this exclusion applies to.
    pub attached_id: CuObjectId,
    /// Partner geometry the attached shape must not be checked against.
    pub partner: CuSceneExclusionPartner,
}

impl Default for CuSceneExclusionPartner {
    fn default() -> Self {
        Self::RobotShape { shape_index: 0 }
    }
}

impl CuSceneExclusion {
    /// Sentinel exclusion with unassigned IDs, used for unused storage slots.
    pub const NIL: Self = Self {
        attached_id: CuObjectId::NIL,
        partner: CuSceneExclusionPartner::RobotShape { shape_index: 0 },
    };

    /// Excludes attached object `attached_id` against robot shape `shape_index`.
    pub const fn robot(attached_id: CuObjectId, shape_index: u16) -> Self {
        Self {
            attached_id,
            partner: CuSceneExclusionPartner::RobotShape { shape_index },
        }
    }

    /// Excludes attached object `attached_id` against attached object `other_id`.
    pub const fn attached(attached_id: CuObjectId, other_id: CuObjectId) -> Self {
        Self {
            attached_id,
            partner: CuSceneExclusionPartner::Attached {
                attached_id: other_id,
            },
        }
    }

    /// Excludes attached object `attached_id` against obstacle `obstacle_id`.
    pub const fn obstacle(attached_id: CuObjectId, obstacle_id: CuObjectId) -> Self {
        Self {
            attached_id,
            partner: CuSceneExclusionPartner::Obstacle { obstacle_id },
        }
    }
}

/// Fixed-capacity stable-ID exclusion set (`CuMsgPayload`-compliant).
///
/// Published alongside a [`CuPlanningScene`] snapshot and validated against it on every check,
/// so exclusions always bind to the current snapshot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuSceneExclusions<const MAX_EXCLUSIONS: usize = DEFAULT_MAX_EXCLUSIONS> {
    #[serde(with = "crate::serde_arrays::exclusion_array")]
    exclusions: [CuSceneExclusion; MAX_EXCLUSIONS],
    len: u32,
}

impl<const MAX_EXCLUSIONS: usize> Default for CuSceneExclusions<MAX_EXCLUSIONS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_EXCLUSIONS: usize> Encode for CuSceneExclusions<MAX_EXCLUSIONS> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.len, encoder)?;
        Encode::encode(&self.exclusions, encoder)
    }
}

impl<C, const MAX_EXCLUSIONS: usize> Decode<C> for CuSceneExclusions<MAX_EXCLUSIONS> {
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let len: u32 = Decode::decode(decoder)?;
        let exclusions = Decode::decode(decoder)?;
        if len as usize > MAX_EXCLUSIONS {
            return Err(bincode::error::DecodeError::Other(
                "Invalid scene exclusions snapshot: count exceeds capacity",
            ));
        }
        Ok(Self { exclusions, len })
    }
}

impl<'de, C, const MAX_EXCLUSIONS: usize> bincode::BorrowDecode<'de, C>
    for CuSceneExclusions<MAX_EXCLUSIONS>
{
    fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Decode::decode(decoder)
    }
}

impl<const MAX_EXCLUSIONS: usize> CuSceneExclusions<MAX_EXCLUSIONS> {
    /// Creates an empty exclusion set.
    pub const fn new() -> Self {
        Self {
            exclusions: [CuSceneExclusion::NIL; MAX_EXCLUSIONS],
            len: 0,
        }
    }

    /// Returns the number of active exclusions.
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns `true` if no exclusions are active.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the maximum number of exclusions.
    pub const fn capacity(&self) -> usize {
        MAX_EXCLUSIONS
    }

    /// Borrows the active exclusions.
    pub fn as_slice(&self) -> &[CuSceneExclusion] {
        &self.exclusions[..self.len as usize]
    }

    /// Appends an exclusion if capacity permits.
    pub fn add(&mut self, exclusion: CuSceneExclusion) -> CuResult<()> {
        let idx = self.len as usize;
        if idx >= MAX_EXCLUSIONS {
            return Err(CuError::from(
                "cu_collision: scene exclusion capacity exceeded",
            ));
        }
        self.exclusions[idx] = exclusion;
        self.len += 1;
        Ok(())
    }

    /// Removes all exclusions in place.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Checks every entry against `scene` and a robot model with `robot_shape_count` shapes.
    ///
    /// Unassigned or unknown IDs, self-references, and out-of-range robot shape indices fail.
    /// Checks perform the same validation; this lets a caller reject a snapshot up front.
    pub fn validate<const MAX_OBS: usize, const MAX_ATT: usize>(
        &self,
        scene: &CuPlanningScene<MAX_OBS, MAX_ATT>,
        robot_shape_count: usize,
    ) -> CuResult<()> {
        validate_exclusions(self.as_slice(), &scene.collision_env(), robot_shape_count)
    }
}

/// Validates `exclusions` against the IDs present in `env`.
pub(crate) fn validate_exclusions(
    exclusions: &[CuSceneExclusion],
    env: &CuCollisionEnv<'_>,
    robot_shape_count: usize,
) -> CuResult<()> {
    for entry in exclusions {
        if !entry.attached_id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(entry.attached_id).into());
        }
        if !env.find_attached(entry.attached_id) {
            return Err(CuSceneError::ObjectNotFound(entry.attached_id).into());
        }
        match entry.partner {
            CuSceneExclusionPartner::RobotShape { shape_index } => {
                if shape_index as usize >= robot_shape_count {
                    return Err(CuError::from(
                        "cu_collision: scene exclusion robot shape index out of range",
                    ));
                }
            }
            CuSceneExclusionPartner::Attached { attached_id } => {
                if !attached_id.is_valid() {
                    return Err(CuSceneError::InvalidObjectId(attached_id).into());
                }
                if attached_id == entry.attached_id {
                    return Err(CuError::from(
                        "cu_collision: scene exclusion references its own attached shape",
                    ));
                }
                if !env.find_attached(attached_id) {
                    return Err(CuSceneError::ObjectNotFound(attached_id).into());
                }
            }
            CuSceneExclusionPartner::Obstacle { obstacle_id } => {
                if !obstacle_id.is_valid() {
                    return Err(CuSceneError::InvalidObjectId(obstacle_id).into());
                }
                if !env.find_obstacle(obstacle_id) {
                    return Err(CuSceneError::ObjectNotFound(obstacle_id).into());
                }
            }
        }
    }
    Ok(())
}
