//! Fixed-capacity, versioned planning scene storage using deterministic collision primitives.
//!
//! A planning scene maintains the set of active collision objects in the robot environment:
//! - Static/free obstacles in world coordinates.
//! - Attached objects rigidly affixed to robot frames (or the base) with local geometry.
//!
//! All storage is pre-allocated with compile-time const capacities (`MAX_OBSTACLES`, `MAX_ATTACHED`),
//! strictly allocation-free on the execution path, and versioned via monotonic [`CuSceneRevision`].
//! Mutations are transactional: failed operations (capacity overflow, missing ID, invalid links,
//! invalid geometry, or revision exhaustion) leave state unchanged, while successful modifications
//! advance the scene revision without wrapping.

use crate::env::CuCollisionEnv;
use crate::shapes::CuCollisionShape;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Default obstacle capacity of [`CuPlanningScene`] and `CuCollisionCheckTask`.
pub const DEFAULT_MAX_OBSTACLES: usize = 32;

/// Default attached shape capacity of [`CuPlanningScene`] and `CuCollisionCheckTask`.
pub const DEFAULT_MAX_ATTACHED: usize = 8;

/// Returns `true` if `frame` has strictly finite 4x4 matrix components.
fn is_finite_transform(frame: &Transform3D<f64>) -> bool {
    frame.to_matrix().iter().flatten().all(|v| v.is_finite())
}

/// Strongly typed, stable object identifier for obstacles and attached shapes in a planning scene.
#[derive(
    Default,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Encode,
    Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
pub struct CuObjectId(u32);

impl CuObjectId {
    /// Sentinel identifier representing an unassigned or invalid object ID.
    pub const NIL: Self = Self(0);

    /// Creates a new object identifier from a raw numeric value.
    pub const fn new(id: u32) -> Self {
        Self(id)
    }

    /// Returns the raw numeric value.
    pub const fn as_u32(self) -> u32 {
        self.0
    }

    /// Returns `true` if this identifier is non-zero, not reserved, and valid.
    pub const fn is_valid(self) -> bool {
        self.0 != 0 && self.0 < u32::MAX
    }
}

impl core::fmt::Display for CuObjectId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "ObjectId({})", self.0)
    }
}

/// Monotonically increasing revision counter tracking changes to the planning scene.
#[derive(
    Default,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Encode,
    Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
pub struct CuSceneRevision(u64);

impl CuSceneRevision {
    /// Initial revision for an empty or newly initialized planning scene.
    pub const INITIAL: Self = Self(0);

    /// Creates a new revision counter with the given raw value.
    pub const fn new(revision: u64) -> Self {
        Self(revision)
    }

    /// Returns the underlying raw revision number.
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// Returns the next revision in sequence, or `None` if the counter has reached maximum.
    #[must_use]
    pub const fn checked_next(self) -> Option<Self> {
        match self.0.checked_add(1) {
            Some(v) => Some(Self(v)),
            None => None,
        }
    }

    /// Returns the next revision in sequence, saturating at [`u64::MAX`] to avoid wrapping.
    #[must_use]
    pub const fn next(self) -> Self {
        Self(self.0.saturating_add(1))
    }
}

impl core::fmt::Display for CuSceneRevision {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Revision({})", self.0)
    }
}

/// Errors that can occur during planning scene modifications and queries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CuSceneError {
    /// Attempted to add an obstacle when obstacle storage is full.
    ObstacleCapacityExceeded,
    /// Attempted to add an attached shape when attached storage is full.
    AttachedCapacityExceeded,
    /// The specified object identifier was not found in the scene.
    ObjectNotFound(CuObjectId),
    /// The specified object identifier is invalid or reserved.
    InvalidObjectId(CuObjectId),
    /// An object with the specified identifier already exists.
    DuplicateObjectId(CuObjectId),
    /// Specified robot link index is out of bounds for the kinematic chain.
    InvalidLinkIndex { link_index: usize, max_links: usize },
    /// Target destination capacity is too small to receive the exported data.
    OutputCapacityExceeded,
    /// Scene revision counter has reached maximum value and cannot advance.
    RevisionExhausted,
    /// Collision shape contains invalid or non-finite geometry.
    InvalidGeometry,
}

impl core::fmt::Display for CuSceneError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ObstacleCapacityExceeded => {
                write!(f, "planning scene obstacle capacity exceeded")
            }
            Self::AttachedCapacityExceeded => {
                write!(f, "planning scene attached shape capacity exceeded")
            }
            Self::ObjectNotFound(id) => {
                write!(f, "planning scene object {id} not found")
            }
            Self::InvalidObjectId(id) => {
                write!(f, "planning scene object {id} is invalid")
            }
            Self::DuplicateObjectId(id) => {
                write!(f, "planning scene object {id} already exists")
            }
            Self::InvalidLinkIndex {
                link_index,
                max_links,
            } => {
                write!(
                    f,
                    "invalid robot link index {link_index} (chain has {max_links} links)"
                )
            }
            Self::OutputCapacityExceeded => {
                write!(f, "target output buffer capacity exceeded")
            }
            Self::RevisionExhausted => {
                write!(f, "planning scene revision counter exhausted")
            }
            Self::InvalidGeometry => {
                write!(
                    f,
                    "planning scene collision shape contains invalid or non-finite geometry"
                )
            }
        }
    }
}

impl CuSceneError {
    /// Static description, used where no formatting is available.
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::ObstacleCapacityExceeded => {
                "cu_collision: planning scene obstacle capacity exceeded"
            }
            Self::AttachedCapacityExceeded => {
                "cu_collision: planning scene attached shape capacity exceeded"
            }
            Self::ObjectNotFound(_) => "cu_collision: planning scene object not found",
            Self::InvalidObjectId(_) => "cu_collision: planning scene object id is invalid",
            Self::DuplicateObjectId(_) => "cu_collision: planning scene duplicate object id",
            Self::InvalidLinkIndex { .. } => "cu_collision: invalid robot link index",
            Self::OutputCapacityExceeded => "cu_collision: target output buffer capacity exceeded",
            Self::RevisionExhausted => "cu_collision: planning scene revision exhausted",
            Self::InvalidGeometry => "cu_collision: invalid or non-finite collision shape geometry",
        }
    }
}

impl From<CuSceneError> for CuError {
    fn from(err: CuSceneError) -> Self {
        CuError::from(err.as_str())
    }
}

/// An environment obstacle in world coordinates, as stored in a planning scene.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuSceneObstacle {
    /// Stable typed identifier of this obstacle.
    pub id: CuObjectId,
    /// Collision geometry in world coordinates.
    pub shape: CuCollisionShape,
}

impl CuSceneObstacle {
    /// Creates a scene obstacle, validating ID and geometry.
    pub fn new(id: CuObjectId, shape: CuCollisionShape) -> CuResult<Self> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        shape.validate()?;
        Ok(Self { id, shape })
    }
}

/// A collision shape rigidly attached to a robot frame or the base.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuAttachedShape {
    /// Stable typed identifier of this attached object.
    pub id: CuObjectId,
    /// Joint-frame index this shape is attached to (`0..N`), or `None` for the base frame.
    pub frame: Option<usize>,
    /// Collision geometry expressed in the local coordinates of `frame`.
    pub shape: CuCollisionShape,
}

impl CuAttachedShape {
    /// Creates an attached shape, validating ID and geometry. The frame index is checked against
    /// the robot when the shape is used.
    pub fn new(id: CuObjectId, frame: Option<usize>, shape: CuCollisionShape) -> CuResult<Self> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        shape.validate()?;
        Ok(Self { id, frame, shape })
    }

    /// Re-expresses the shape through a frame-relative transform (`frame_from_shape`),
    /// rejecting non-finite transforms and results.
    pub fn with_transform(mut self, frame_from_shape: &Transform3D<f64>) -> CuResult<Self> {
        if !is_finite_transform(frame_from_shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let local = self.shape;
        transform_shape(frame_from_shape, &local, &mut self.shape);
        self.shape.validate()?;
        Ok(self)
    }
}

/// Changes applied by [`CuPlanningScene::update_attached`]; unset parts are kept.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CuAttachedUpdate {
    frame: Option<Option<usize>>,
    shape: Option<CuCollisionShape>,
}

impl CuAttachedUpdate {
    /// An update that changes nothing.
    pub const fn new() -> Self {
        Self {
            frame: None,
            shape: None,
        }
    }

    /// Moves the shape to `frame` (`None` = base frame).
    pub const fn with_frame(mut self, frame: Option<usize>) -> Self {
        self.frame = Some(frame);
        self
    }

    /// Replaces the frame-local geometry.
    pub const fn with_shape(mut self, shape: CuCollisionShape) -> Self {
        self.shape = Some(shape);
        self
    }
}

#[derive(Serialize, Deserialize)]
struct CuPlanningSceneSnapshot<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> {
    revision: CuSceneRevision,
    next_id: u32,
    #[serde(with = "crate::serde_arrays::obstacle_array")]
    obstacles: [CuSceneObstacle; MAX_OBSTACLES],
    obstacle_count: u32,
    #[serde(with = "crate::serde_arrays::attached_array")]
    attached_shapes: [CuAttachedShape; MAX_ATTACHED],
    attached_count: u32,
}

/// Fixed-capacity, versioned planning scene storage (`CuMsgPayload`-compliant).
///
/// Holds environment obstacles in world frame and attached shapes in robot frames.
/// Every mutation is transactional and increments [`CuSceneRevision`] on success.
/// [`collision_env`](Self::collision_env) borrows the scene as a zero-copy [`CuCollisionEnv`].
#[derive(Debug, Clone, PartialEq, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuPlanningScene<
    const MAX_OBSTACLES: usize = DEFAULT_MAX_OBSTACLES,
    const MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED,
> {
    revision: CuSceneRevision,
    next_id: u32,
    obstacle_ids: [CuObjectId; MAX_OBSTACLES],
    obstacle_shapes: [CuCollisionShape; MAX_OBSTACLES],
    obstacle_count: u32,
    attached_shapes: [CuAttachedShape; MAX_ATTACHED],
    attached_count: u32,
}

impl<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Default
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Serialize
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        if let Err(err) = self.validate_invariants() {
            return Err(serde::ser::Error::custom(err.as_str()));
        }
        let snapshot = CuPlanningSceneSnapshot::<MAX_OBSTACLES, MAX_ATTACHED> {
            revision: self.revision,
            next_id: self.next_id,
            obstacles: core::array::from_fn(|i| CuSceneObstacle {
                id: self.obstacle_ids[i],
                shape: self.obstacle_shapes[i],
            }),
            obstacle_count: self.obstacle_count,
            attached_shapes: self.attached_shapes,
            attached_count: self.attached_count,
        };
        snapshot.serialize(serializer)
    }
}

impl<'de, const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Deserialize<'de>
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let raw =
            CuPlanningSceneSnapshot::<MAX_OBSTACLES, MAX_ATTACHED>::deserialize(deserializer)?;
        let scene = Self {
            revision: raw.revision,
            next_id: raw.next_id,
            obstacle_ids: raw.obstacles.map(|o| o.id),
            obstacle_shapes: raw.obstacles.map(|o| o.shape),
            obstacle_count: raw.obstacle_count,
            attached_shapes: raw.attached_shapes,
            attached_count: raw.attached_count,
        };
        if let Err(err) = scene.validate_invariants() {
            return Err(serde::de::Error::custom(err.as_str()));
        }
        Ok(scene)
    }
}

impl<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Encode
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        if let Err(err) = self.validate_invariants() {
            return Err(bincode::error::EncodeError::Other(err.as_str()));
        }
        Encode::encode(&self.revision, encoder)?;
        Encode::encode(&self.next_id, encoder)?;
        Encode::encode(&self.obstacle_count, encoder)?;
        Encode::encode(&self.obstacle_ids, encoder)?;
        Encode::encode(&self.obstacle_shapes, encoder)?;
        Encode::encode(&self.attached_count, encoder)?;
        Encode::encode(&self.attached_shapes, encoder)
    }
}

impl<C, const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Decode<C>
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let scene = Self {
            revision: Decode::decode(decoder)?,
            next_id: Decode::decode(decoder)?,
            obstacle_count: Decode::decode(decoder)?,
            obstacle_ids: Decode::decode(decoder)?,
            obstacle_shapes: Decode::decode(decoder)?,
            attached_count: Decode::decode(decoder)?,
            attached_shapes: Decode::decode(decoder)?,
        };
        if let Err(err) = scene.validate_invariants() {
            return Err(bincode::error::DecodeError::Other(err.as_str()));
        }
        Ok(scene)
    }
}

impl<'de, C, const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> bincode::BorrowDecode<'de, C>
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Decode::decode(decoder)
    }
}

impl<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize>
    CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    /// Maximum number of environment obstacles.
    pub const OBSTACLE_CAPACITY: usize = MAX_OBSTACLES;
    /// Maximum number of attached shapes.
    pub const ATTACHED_CAPACITY: usize = MAX_ATTACHED;

    /// Creates an empty planning scene initialized at [`CuSceneRevision::INITIAL`].
    pub fn new() -> Self {
        Self {
            revision: CuSceneRevision::INITIAL,
            next_id: 1,
            obstacle_ids: [CuObjectId::NIL; MAX_OBSTACLES],
            obstacle_shapes: [CuCollisionShape::default(); MAX_OBSTACLES],
            obstacle_count: 0,
            attached_shapes: [CuAttachedShape::default(); MAX_ATTACHED],
            attached_count: 0,
        }
    }

    /// Validates structural, identity, and geometric invariants:
    /// counts within capacity, non-zero `next_id` greater than every active ID, valid and
    /// pairwise-unique IDs across obstacles and attached shapes, and finite geometry.
    pub(crate) fn validate_invariants(&self) -> Result<(), CuSceneError> {
        let obs_count = self.obstacle_count as usize;
        if obs_count > MAX_OBSTACLES {
            return Err(CuSceneError::ObstacleCapacityExceeded);
        }
        let att_count = self.attached_count as usize;
        if att_count > MAX_ATTACHED {
            return Err(CuSceneError::AttachedCapacityExceeded);
        }
        if self.next_id == 0 {
            return Err(CuSceneError::InvalidObjectId(CuObjectId::NIL));
        }
        let obstacle_ids = &self.obstacle_ids[..obs_count];
        let check = |id: CuObjectId, shape: &CuCollisionShape| {
            if !id.is_valid() || id.as_u32() >= self.next_id {
                return Err(CuSceneError::InvalidObjectId(id));
            }
            if shape.validate().is_err() {
                return Err(CuSceneError::InvalidGeometry);
            }
            Ok(())
        };
        for (i, (&id, shape)) in obstacle_ids
            .iter()
            .zip(&self.obstacle_shapes[..obs_count])
            .enumerate()
        {
            check(id, shape)?;
            if obstacle_ids[i + 1..].contains(&id) {
                return Err(CuSceneError::DuplicateObjectId(id));
            }
        }
        let attached = self.attached_shapes();
        for (i, att) in attached.iter().enumerate() {
            check(att.id, &att.shape)?;
            if attached[i + 1..].iter().any(|o| o.id == att.id) || obstacle_ids.contains(&att.id) {
                return Err(CuSceneError::DuplicateObjectId(att.id));
            }
        }
        Ok(())
    }

    /// Returns the current scene revision.
    pub const fn revision(&self) -> CuSceneRevision {
        self.revision
    }

    /// Returns `true` if the scene has neither obstacles nor attached shapes.
    pub const fn is_empty(&self) -> bool {
        self.obstacle_count == 0 && self.attached_count == 0
    }

    /// Active obstacle shapes in world coordinates; the count is `obstacle_shapes().len()`.
    pub fn obstacle_shapes(&self) -> &[CuCollisionShape] {
        &self.obstacle_shapes[..self.obstacle_len()]
    }

    /// Stable IDs of the active obstacles, parallel to [`obstacle_shapes`](Self::obstacle_shapes).
    pub fn obstacle_ids(&self) -> &[CuObjectId] {
        &self.obstacle_ids[..self.obstacle_len()]
    }

    /// Iterates over the active obstacles with their IDs.
    pub fn obstacles(&self) -> impl Iterator<Item = CuSceneObstacle> + '_ {
        self.obstacle_ids()
            .iter()
            .zip(self.obstacle_shapes())
            .map(|(&id, &shape)| CuSceneObstacle { id, shape })
    }

    /// Active attached shapes; the count is `attached_shapes().len()`.
    pub fn attached_shapes(&self) -> &[CuAttachedShape] {
        let count = (self.attached_count as usize).min(MAX_ATTACHED);
        &self.attached_shapes[..count]
    }

    /// Looks up an obstacle shape by its typed object ID.
    pub fn obstacle(&self, id: CuObjectId) -> Option<&CuCollisionShape> {
        self.find_obstacle_index(id)
            .map(|idx| &self.obstacle_shapes[idx])
    }

    /// Looks up an attached shape by its typed object ID.
    pub fn attached_shape(&self, id: CuObjectId) -> Option<&CuAttachedShape> {
        self.find_attached_index(id)
            .map(|idx| &self.attached_shapes[idx])
    }

    /// Returns `true` if the specified object identifier is present in the scene.
    pub fn contains(&self, id: CuObjectId) -> bool {
        self.find_obstacle_index(id).is_some() || self.find_attached_index(id).is_some()
    }

    /// Zero-copy view of the obstacles (with IDs) and attached shapes for collision checks.
    /// Add exclusions with [`CuCollisionEnv::with_exclusions`].
    pub fn collision_env(&self) -> CuCollisionEnv<'_> {
        CuCollisionEnv::with_obstacle_ids(self.obstacle_shapes(), self.obstacle_ids())
            .with_attached(self.attached_shapes())
    }

    fn obstacle_len(&self) -> usize {
        (self.obstacle_count as usize).min(MAX_OBSTACLES)
    }

    pub(crate) fn find_obstacle_index(&self, id: CuObjectId) -> Option<usize> {
        if !id.is_valid() {
            return None;
        }
        self.obstacle_ids().iter().position(|&o| o == id)
    }

    pub(crate) fn find_attached_index(&self, id: CuObjectId) -> Option<usize> {
        if !id.is_valid() {
            return None;
        }
        self.attached_shapes().iter().position(|a| a.id == id)
    }

    fn next_revision(&self) -> CuResult<CuSceneRevision> {
        Ok(self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?)
    }

    /// Chooses the ID for a new object and the resulting `next_id`, without mutating.
    fn plan_id(&self, id: Option<CuObjectId>) -> CuResult<(CuObjectId, u32)> {
        let overflow = || CuError::from("cu_collision: object id overflow");
        match id {
            Some(id) => {
                if !id.is_valid() {
                    return Err(CuSceneError::InvalidObjectId(id).into());
                }
                if self.contains(id) {
                    return Err(CuSceneError::DuplicateObjectId(id).into());
                }
                let next = if id.as_u32() >= self.next_id {
                    id.as_u32()
                        .checked_add(1)
                        .ok_or(CuSceneError::InvalidObjectId(id))?
                } else {
                    self.next_id
                };
                Ok((id, next))
            }
            None => {
                let mut candidate = self.next_id;
                while candidate == 0 || self.contains(CuObjectId::new(candidate)) {
                    candidate = candidate.checked_add(1).ok_or_else(overflow)?;
                }
                let id = CuObjectId::new(candidate);
                if !id.is_valid() {
                    return Err(overflow());
                }
                Ok((id, candidate.checked_add(1).ok_or_else(overflow)?))
            }
        }
    }

    fn push_obstacle(&mut self, id: CuObjectId, shape: CuCollisionShape) {
        let idx = self.obstacle_count as usize;
        self.obstacle_ids[idx] = id;
        self.obstacle_shapes[idx] = shape;
        self.obstacle_count += 1;
    }

    fn take_obstacle(&mut self, idx: usize) -> CuCollisionShape {
        let count = self.obstacle_len();
        let removed = self.obstacle_shapes[idx];
        self.obstacle_ids.copy_within(idx + 1..count, idx);
        self.obstacle_shapes.copy_within(idx + 1..count, idx);
        self.obstacle_ids[count - 1] = CuObjectId::NIL;
        self.obstacle_shapes[count - 1] = CuCollisionShape::default();
        self.obstacle_count -= 1;
        removed
    }

    fn push_attached(&mut self, attached: CuAttachedShape) {
        self.attached_shapes[self.attached_count as usize] = attached;
        self.attached_count += 1;
    }

    fn take_attached(&mut self, idx: usize) -> CuAttachedShape {
        let count = self.attached_shapes().len();
        let removed = self.attached_shapes[idx];
        self.attached_shapes.copy_within(idx + 1..count, idx);
        self.attached_shapes[count - 1] = CuAttachedShape::default();
        self.attached_count -= 1;
        removed
    }

    /// Adds a world-frame obstacle and returns its ID.
    ///
    /// With `id = None` a fresh unique ID is assigned; with `Some(id)` that ID is used and must be
    /// valid and unused. Fails on capacity, invalid geometry, invalid or duplicate ID, or revision
    /// exhaustion; on failure, state is unmodified.
    pub fn add_obstacle(
        &mut self,
        shape: CuCollisionShape,
        id: Option<CuObjectId>,
    ) -> CuResult<CuObjectId> {
        if self.obstacle_len() >= MAX_OBSTACLES {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        if shape.validate().is_err() {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let revision = self.next_revision()?;
        let (id, next_id) = self.plan_id(id)?;
        self.push_obstacle(id, shape);
        self.next_id = next_id;
        self.revision = revision;
        Ok(id)
    }

    /// Replaces the geometry of an existing obstacle. On failure, state is unmodified.
    pub fn update_obstacle(&mut self, id: CuObjectId, shape: CuCollisionShape) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if shape.validate().is_err() {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        self.obstacle_shapes[idx] = shape;
        Ok(())
    }

    /// Removes an obstacle, preserving the order of the others. On failure, state is unmodified.
    pub fn remove_obstacle(&mut self, id: CuObjectId) -> CuResult<CuCollisionShape> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        let idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        Ok(self.take_obstacle(idx))
    }

    /// Adds a shape attached to `frame` (`None` = base frame) and returns its ID.
    ///
    /// ID assignment follows [`add_obstacle`](Self::add_obstacle). On failure, state is unmodified.
    pub fn add_attached(
        &mut self,
        frame: Option<usize>,
        shape: CuCollisionShape,
        id: Option<CuObjectId>,
    ) -> CuResult<CuObjectId> {
        if self.attached_shapes().len() >= MAX_ATTACHED {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if shape.validate().is_err() {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let revision = self.next_revision()?;
        let (id, next_id) = self.plan_id(id)?;
        self.push_attached(CuAttachedShape { id, frame, shape });
        self.next_id = next_id;
        self.revision = revision;
        Ok(id)
    }

    /// Changes the frame and/or geometry of an attached shape. On failure, state is unmodified.
    pub fn update_attached(&mut self, id: CuObjectId, update: CuAttachedUpdate) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if update.shape.is_some_and(|s| s.validate().is_err()) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        let attached = &mut self.attached_shapes[idx];
        if let Some(frame) = update.frame {
            attached.frame = frame;
        }
        if let Some(shape) = update.shape {
            attached.shape = shape;
        }
        Ok(())
    }

    /// Removes an attached shape, preserving the order of the others. On failure, state is
    /// unmodified.
    pub fn remove_attached(&mut self, id: CuObjectId) -> CuResult<CuAttachedShape> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        Ok(self.take_attached(idx))
    }

    /// Turns an existing obstacle into a shape attached to `frame` (e.g. object grasp), keeping
    /// its [`CuObjectId`]. `local_shape` is the geometry in `frame` coordinates.
    ///
    /// Fails if the obstacle is not found, geometry is invalid, attached storage is full, or the
    /// revision is exhausted. On failure, state is unmodified.
    pub fn attach(
        &mut self,
        id: CuObjectId,
        frame: Option<usize>,
        local_shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if local_shape.validate().is_err() {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.attached_shapes().len() >= MAX_ATTACHED {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        let idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        self.take_obstacle(idx);
        self.push_attached(CuAttachedShape {
            id,
            frame,
            shape: local_shape,
        });
        Ok(())
    }

    /// Turns an attached shape into a world-frame obstacle (e.g. object release), keeping its
    /// [`CuObjectId`].
    ///
    /// Fails if the attached shape is not found, geometry is invalid, obstacle storage is full,
    /// or the revision is exhausted. On failure, state is unmodified.
    pub fn detach(&mut self, id: CuObjectId, world_shape: CuCollisionShape) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if world_shape.validate().is_err() {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.obstacle_len() >= MAX_OBSTACLES {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        self.revision = self.next_revision()?;
        self.take_attached(idx);
        self.push_obstacle(id, world_shape);
        Ok(())
    }

    /// Clears all obstacles and attached shapes, advancing the revision if the scene was not
    /// empty. On failure (revision exhaustion), state is unmodified.
    pub fn clear(&mut self) -> CuResult<()> {
        if !self.is_empty() {
            self.revision = self.next_revision()?;
            *self = Self {
                revision: self.revision,
                next_id: self.next_id,
                ..Self::new()
            };
        }
        Ok(())
    }

    /// Clears all obstacles, advancing the revision if any were present. On failure, state is
    /// unmodified.
    pub fn clear_obstacles(&mut self) -> CuResult<()> {
        if self.obstacle_count > 0 {
            self.revision = self.next_revision()?;
            self.obstacle_ids = [CuObjectId::NIL; MAX_OBSTACLES];
            self.obstacle_shapes = [CuCollisionShape::default(); MAX_OBSTACLES];
            self.obstacle_count = 0;
        }
        Ok(())
    }

    /// Clears all attached shapes, advancing the revision if any were present. On failure, state
    /// is unmodified.
    pub fn clear_attached(&mut self) -> CuResult<()> {
        if self.attached_count > 0 {
            self.revision = self.next_revision()?;
            self.attached_shapes = [CuAttachedShape::default(); MAX_ATTACHED];
            self.attached_count = 0;
        }
        Ok(())
    }

    /// Executes a transaction block on the planning scene.
    ///
    /// If the closure `f` succeeds, modifications are committed and the revision advances once
    /// if state was modified without advancing it. If `f` returns an error or revision
    /// exhaustion occurs, the scene rolls back to its pre-transaction state.
    pub fn transaction<F, R>(&mut self, f: F) -> CuResult<R>
    where
        F: FnOnce(&mut Self) -> CuResult<R>,
    {
        let backup = self.clone();
        match f(self) {
            Ok(result) => {
                if *self != backup && self.revision == backup.revision {
                    match self.revision.checked_next() {
                        Some(rev) => self.revision = rev,
                        None => {
                            *self = backup;
                            return Err(CuSceneError::RevisionExhausted.into());
                        }
                    }
                }
                Ok(result)
            }
            Err(e) => {
                *self = backup;
                Err(e)
            }
        }
    }

    /// Writes world-space copies of the active attached shapes for `link_frames` into
    /// `out_shapes` and returns how many were written.
    ///
    /// Validates capacity, frame indices, transforms, and resulting geometry first: on error,
    /// `out_shapes` is unmodified.
    pub fn world_attached_shapes<const N: usize, const MAX_OUT: usize>(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; MAX_OUT],
    ) -> CuResult<usize> {
        let attached = self.attached_shapes();
        if attached.len() > MAX_OUT {
            return Err(CuSceneError::OutputCapacityExceeded.into());
        }
        let world = |att: &CuAttachedShape| -> CuResult<CuCollisionShape> {
            let Some(frame) = att.frame else {
                return Ok(att.shape);
            };
            let Some(transform) = link_frames.get(frame) else {
                return Err(CuSceneError::InvalidLinkIndex {
                    link_index: frame,
                    max_links: N,
                }
                .into());
            };
            if !is_finite_transform(transform) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
            let mut out = att.shape;
            transform_shape(transform, &att.shape, &mut out);
            if out.validate().is_err() {
                return Err(CuSceneError::InvalidGeometry.into());
            }
            Ok(out)
        };
        for att in attached {
            world(att)?;
        }
        for (att, out) in attached.iter().zip(out_shapes.iter_mut()) {
            *out = world(att)?;
        }
        Ok(attached.len())
    }
}
