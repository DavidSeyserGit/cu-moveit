//! Fixed-capacity, versioned planning scene storage using deterministic collision primitives.
//!
//! A planning scene maintains the set of active collision objects in the robot environment:
//! - Static/free obstacles in world coordinates.
//! - Attached objects rigidly affixed to specific robot links with local geometry.
//!
//! All storage is pre-allocated with compile-time const capacities (`MAX_OBSTACLES`, `MAX_ATTACHED`),
//! strictly allocation-free on the execution path, and versioned via monotonic [`CuSceneRevision`].
//! Mutations are transactional: failed operations (capacity overflow, missing ID, invalid links)
//! leave state unchanged, while successful modifications advance the scene revision.

use crate::model::CuCollisionWorld;
use crate::shapes::CuCollisionShape;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Default capacity for environment obstacles in [`CuPlanningScene`].
pub const DEFAULT_MAX_SCENE_OBSTACLES: usize = 32;

/// Default capacity for attached collision shapes in [`CuPlanningScene`].
pub const DEFAULT_MAX_ATTACHED_SHAPES: usize = 8;

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
pub struct CuObjectId(pub u32);

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

    /// Returns `true` if this identifier is non-zero and valid.
    pub const fn is_valid(self) -> bool {
        self.0 != 0
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
pub struct CuSceneRevision(pub u64);

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

    /// Returns the next revision in sequence.
    #[must_use]
    pub const fn next(self) -> Self {
        Self(self.0.wrapping_add(1))
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
        }
    }
}

impl From<CuSceneError> for CuError {
    fn from(err: CuSceneError) -> Self {
        match err {
            CuSceneError::ObstacleCapacityExceeded => {
                CuError::from("cu_collision: planning scene obstacle capacity exceeded")
            }
            CuSceneError::AttachedCapacityExceeded => {
                CuError::from("cu_collision: planning scene attached shape capacity exceeded")
            }
            CuSceneError::ObjectNotFound(_) => {
                CuError::from("cu_collision: planning scene object not found")
            }
            CuSceneError::InvalidObjectId(_) => {
                CuError::from("cu_collision: planning scene object id is invalid")
            }
            CuSceneError::DuplicateObjectId(_) => {
                CuError::from("cu_collision: planning scene duplicate object id")
            }
            CuSceneError::InvalidLinkIndex { .. } => {
                CuError::from("cu_collision: invalid robot link index")
            }
            CuSceneError::OutputCapacityExceeded => {
                CuError::from("cu_collision: target output buffer capacity exceeded")
            }
        }
    }
}

/// An environment obstacle stored within the planning scene in world coordinates.
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
    /// Creates a new scene obstacle with the specified ID and collision shape.
    pub const fn new(id: CuObjectId, shape: CuCollisionShape) -> Self {
        Self { id, shape }
    }
}

/// A collision shape rigidly attached to a robot link/frame.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuAttachedShape {
    /// Stable typed identifier of this attached object.
    pub id: CuObjectId,
    /// Robot kinematic link / frame index this shape is attached to.
    pub link_index: usize,
    /// Collision geometry expressed in the local coordinate frame of `link_index`.
    pub shape: CuCollisionShape,
}

impl CuAttachedShape {
    /// Creates a new attached shape with specified ID, link index, and link-local geometry.
    pub const fn new(id: CuObjectId, link_index: usize, shape: CuCollisionShape) -> Self {
        Self {
            id,
            link_index,
            shape,
        }
    }

    /// Creates an attached shape by applying a link-relative transform to an input shape in place.
    pub fn new_with_transform(
        id: CuObjectId,
        link_index: usize,
        shape: &CuCollisionShape,
        link_from_shape: &Transform3D<f64>,
    ) -> Self {
        let mut local_shape = *shape;
        transform_shape(link_from_shape, shape, &mut local_shape);
        Self {
            id,
            link_index,
            shape: local_shape,
        }
    }
}

mod serde_scene_arrays {
    use super::CuAttachedShape;
    use super::CuSceneObstacle;
    use serde::Deserializer;
    use serde::Serializer;
    use serde::de::Error as DeError;
    use serde::de::SeqAccess;
    use serde::de::Visitor;
    use serde::ser::SerializeTuple;

    pub mod obstacle_array {
        use super::*;

        pub fn serialize<S, const N: usize>(
            values: &[CuSceneObstacle; N],
            serializer: S,
        ) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            let mut seq = serializer.serialize_tuple(N)?;
            for item in values.iter() {
                seq.serialize_element(item)?;
            }
            seq.end()
        }

        pub fn deserialize<'de, D, const N: usize>(
            deserializer: D,
        ) -> Result<[CuSceneObstacle; N], D::Error>
        where
            D: Deserializer<'de>,
        {
            struct ArrayVisitor<const M: usize>;

            impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
                type Value = [CuSceneObstacle; M];

                fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                    write!(formatter, "an array of {M} scene obstacle values")
                }

                fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
                where
                    A: SeqAccess<'de>,
                {
                    let mut out = [CuSceneObstacle::default(); M];
                    let mut i = 0;
                    while i < M {
                        out[i] = seq
                            .next_element()?
                            .ok_or_else(|| DeError::invalid_length(i, &self))?;
                        i += 1;
                    }
                    if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                        return Err(DeError::invalid_length(M + 1, &self));
                    }
                    Ok(out)
                }
            }

            deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
        }
    }

    pub mod attached_array {
        use super::*;

        pub fn serialize<S, const N: usize>(
            values: &[CuAttachedShape; N],
            serializer: S,
        ) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            let mut seq = serializer.serialize_tuple(N)?;
            for item in values.iter() {
                seq.serialize_element(item)?;
            }
            seq.end()
        }

        pub fn deserialize<'de, D, const N: usize>(
            deserializer: D,
        ) -> Result<[CuAttachedShape; N], D::Error>
        where
            D: Deserializer<'de>,
        {
            struct ArrayVisitor<const M: usize>;

            impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
                type Value = [CuAttachedShape; M];

                fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                    write!(formatter, "an array of {M} attached shape values")
                }

                fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
                where
                    A: SeqAccess<'de>,
                {
                    let mut out = [CuAttachedShape::default(); M];
                    let mut i = 0;
                    while i < M {
                        out[i] = seq
                            .next_element()?
                            .ok_or_else(|| DeError::invalid_length(i, &self))?;
                        i += 1;
                    }
                    if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                        return Err(DeError::invalid_length(M + 1, &self));
                    }
                    Ok(out)
                }
            }

            deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
        }
    }
}

/// Fixed-capacity, versioned planning scene storage (`CuMsgPayload`-compliant).
///
/// Holds environment obstacles in world frame and attached shapes in robot link frames.
/// Every mutation is transactional and increments [`CuSceneRevision`] on success.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuPlanningScene<
    const MAX_OBSTACLES: usize = DEFAULT_MAX_SCENE_OBSTACLES,
    const MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED_SHAPES,
> {
    revision: CuSceneRevision,
    next_id: u32,
    #[serde(with = "serde_scene_arrays::obstacle_array")]
    obstacles: [CuSceneObstacle; MAX_OBSTACLES],
    obstacle_count: u32,
    #[serde(with = "serde_scene_arrays::attached_array")]
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

impl<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> Encode
    for CuPlanningScene<MAX_OBSTACLES, MAX_ATTACHED>
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.revision, encoder)?;
        Encode::encode(&self.next_id, encoder)?;
        Encode::encode(&self.obstacle_count, encoder)?;
        Encode::encode(&self.obstacles, encoder)?;
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
        let revision = Decode::decode(decoder)?;
        let next_id = Decode::decode(decoder)?;
        let obstacle_count: u32 = Decode::decode(decoder)?;
        let obstacles = Decode::decode(decoder)?;
        let attached_count: u32 = Decode::decode(decoder)?;
        let attached_shapes = Decode::decode(decoder)?;

        if obstacle_count as usize > MAX_OBSTACLES || attached_count as usize > MAX_ATTACHED {
            return Err(bincode::error::DecodeError::Other(
                "Invalid planning scene snapshot: count exceeds capacity",
            ));
        }

        Ok(Self {
            revision,
            next_id,
            obstacles,
            obstacle_count,
            attached_shapes,
            attached_count,
        })
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
    /// Creates an empty planning scene initialized at [`CuSceneRevision::INITIAL`].
    pub fn new() -> Self {
        Self {
            revision: CuSceneRevision::INITIAL,
            next_id: 1,
            obstacles: [CuSceneObstacle::default(); MAX_OBSTACLES],
            obstacle_count: 0,
            attached_shapes: [CuAttachedShape::default(); MAX_ATTACHED],
            attached_count: 0,
        }
    }

    /// Returns the current scene revision.
    pub const fn revision(&self) -> CuSceneRevision {
        self.revision
    }

    /// Returns the next candidate object identifier.
    pub const fn next_id(&self) -> CuObjectId {
        CuObjectId(self.next_id)
    }

    /// Returns the maximum capacity for environment obstacles.
    pub const fn obstacle_capacity(&self) -> usize {
        MAX_OBSTACLES
    }

    /// Returns the maximum capacity for attached shapes.
    pub const fn attached_capacity(&self) -> usize {
        MAX_ATTACHED
    }

    /// Returns the number of active environment obstacles.
    pub const fn obstacle_count(&self) -> usize {
        self.obstacle_count as usize
    }

    /// Returns the number of active attached shapes.
    pub const fn attached_count(&self) -> usize {
        self.attached_count as usize
    }

    /// Returns the total number of objects (obstacles and attached shapes) in the scene.
    pub const fn total_object_count(&self) -> usize {
        (self.obstacle_count + self.attached_count) as usize
    }

    /// Returns `true` if obstacle storage is at capacity.
    pub const fn is_obstacle_full(&self) -> bool {
        self.obstacle_count as usize >= MAX_OBSTACLES
    }

    /// Returns `true` if attached shape storage is at capacity.
    pub const fn is_attached_full(&self) -> bool {
        self.attached_count as usize >= MAX_ATTACHED
    }

    /// Returns `true` if both obstacles and attached shapes are empty.
    pub const fn is_empty(&self) -> bool {
        self.obstacle_count == 0 && self.attached_count == 0
    }

    /// Borrows the active environment obstacles as a slice.
    pub fn obstacles(&self) -> &[CuSceneObstacle] {
        &self.obstacles[..self.obstacle_count as usize]
    }

    /// Borrows the active attached shapes as a slice.
    pub fn attached_shapes(&self) -> &[CuAttachedShape] {
        &self.attached_shapes[..self.attached_count as usize]
    }

    /// Looks up an obstacle shape by its typed object ID.
    pub fn obstacle(&self, id: CuObjectId) -> Option<&CuCollisionShape> {
        self.find_obstacle_index(id)
            .map(|idx| &self.obstacles[idx].shape)
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

    /// Finds the internal array index of an obstacle by its typed object ID.
    pub fn find_obstacle_index(&self, id: CuObjectId) -> Option<usize> {
        let count = self.obstacle_count as usize;
        (0..count).find(|&i| self.obstacles[i].id == id)
    }

    /// Finds the internal array index of an attached shape by its typed object ID.
    pub fn find_attached_index(&self, id: CuObjectId) -> Option<usize> {
        let count = self.attached_count as usize;
        (0..count).find(|&i| self.attached_shapes[i].id == id)
    }

    /// Appends a new obstacle to the scene with an automatically generated unique [`CuObjectId`].
    ///
    /// On success, advances [`CuSceneRevision`]. On capacity failure, state is unmodified.
    pub fn add_obstacle(&mut self, shape: CuCollisionShape) -> CuResult<CuObjectId> {
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        let id = self.alloc_id()?;
        let idx = self.obstacle_count as usize;
        self.obstacles[idx] = CuSceneObstacle::new(id, shape);
        self.obstacle_count += 1;
        self.revision = self.revision.next();
        Ok(id)
    }

    /// Adds an obstacle with an explicit [`CuObjectId`].
    ///
    /// Fails if the ID is invalid, already in use, or if capacity is exceeded.
    pub fn add_obstacle_with_id(
        &mut self,
        id: CuObjectId,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        if self.contains(id) {
            return Err(CuSceneError::DuplicateObjectId(id).into());
        }
        let idx = self.obstacle_count as usize;
        self.obstacles[idx] = CuSceneObstacle::new(id, shape);
        self.obstacle_count += 1;
        if id.as_u32() >= self.next_id {
            self.next_id = id.as_u32().saturating_add(1);
        }
        self.revision = self.revision.next();
        Ok(())
    }

    /// Updates the collision geometry of an existing obstacle in place.
    ///
    /// Fails with [`CuSceneError::ObjectNotFound`] if the obstacle is not present.
    pub fn update_obstacle(&mut self, id: CuObjectId, shape: CuCollisionShape) -> CuResult<()> {
        let Some(idx) = self.find_obstacle_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        self.obstacles[idx].shape = shape;
        self.revision = self.revision.next();
        Ok(())
    }

    /// Removes an obstacle by ID and compacts the storage in place, preserving insertion order.
    pub fn remove_obstacle(&mut self, id: CuObjectId) -> CuResult<CuCollisionShape> {
        let count = self.obstacle_count as usize;
        let Some(idx) = self.find_obstacle_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        let removed = self.obstacles[idx].shape;
        let mut i = idx;
        while i + 1 < count {
            self.obstacles[i] = self.obstacles[i + 1];
            i += 1;
        }
        self.obstacles[count - 1] = CuSceneObstacle::default();
        self.obstacle_count -= 1;
        self.revision = self.revision.next();
        Ok(removed)
    }

    /// Appends an attached shape to the scene with an automatically generated unique [`CuObjectId`].
    pub fn add_attached_shape(
        &mut self,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<CuObjectId> {
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        let id = self.alloc_id()?;
        let idx = self.attached_count as usize;
        self.attached_shapes[idx] = CuAttachedShape::new(id, link_index, shape);
        self.attached_count += 1;
        self.revision = self.revision.next();
        Ok(id)
    }

    /// Adds an attached shape with an explicit [`CuObjectId`].
    pub fn add_attached_shape_with_id(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if self.contains(id) {
            return Err(CuSceneError::DuplicateObjectId(id).into());
        }
        let idx = self.attached_count as usize;
        self.attached_shapes[idx] = CuAttachedShape::new(id, link_index, shape);
        self.attached_count += 1;
        if id.as_u32() >= self.next_id {
            self.next_id = id.as_u32().saturating_add(1);
        }
        self.revision = self.revision.next();
        Ok(())
    }

    /// Updates the collision geometry of an attached shape in place.
    pub fn update_attached_shape(
        &mut self,
        id: CuObjectId,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        let Some(idx) = self.find_attached_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        self.attached_shapes[idx].shape = shape;
        self.revision = self.revision.next();
        Ok(())
    }

    /// Updates the attached link index of an attached shape in place.
    pub fn update_attached_link(&mut self, id: CuObjectId, link_index: usize) -> CuResult<()> {
        let Some(idx) = self.find_attached_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        self.attached_shapes[idx].link_index = link_index;
        self.revision = self.revision.next();
        Ok(())
    }

    /// Updates both link index and geometry of an attached shape in place.
    pub fn update_attached(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        let Some(idx) = self.find_attached_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        self.attached_shapes[idx].link_index = link_index;
        self.attached_shapes[idx].shape = shape;
        self.revision = self.revision.next();
        Ok(())
    }

    /// Removes an attached shape by ID and compacts the storage in place.
    pub fn remove_attached_shape(&mut self, id: CuObjectId) -> CuResult<CuAttachedShape> {
        let count = self.attached_count as usize;
        let Some(idx) = self.find_attached_index(id) else {
            return Err(CuSceneError::ObjectNotFound(id).into());
        };
        let removed = self.attached_shapes[idx];
        let mut i = idx;
        while i + 1 < count {
            self.attached_shapes[i] = self.attached_shapes[i + 1];
            i += 1;
        }
        self.attached_shapes[count - 1] = CuAttachedShape::default();
        self.attached_count -= 1;
        self.revision = self.revision.next();
        Ok(removed)
    }

    /// Transitions an existing obstacle into an attached shape on `link_index` (e.g. object grasp).
    ///
    /// Preserves the stable [`CuObjectId`]. Fails if obstacle not found or attached storage full.
    pub fn attach_obstacle(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        local_shape: CuCollisionShape,
    ) -> CuResult<()> {
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        let obs_idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;

        // Remove from obstacles in place
        let obs_count = self.obstacle_count as usize;
        let mut i = obs_idx;
        while i + 1 < obs_count {
            self.obstacles[i] = self.obstacles[i + 1];
            i += 1;
        }
        self.obstacles[obs_count - 1] = CuSceneObstacle::default();
        self.obstacle_count -= 1;

        // Insert into attached shapes
        let att_idx = self.attached_count as usize;
        self.attached_shapes[att_idx] = CuAttachedShape::new(id, link_index, local_shape);
        self.attached_count += 1;

        self.revision = self.revision.next();
        Ok(())
    }

    /// Detaches an attached shape into an environment obstacle in world coordinates (e.g. object release).
    ///
    /// Preserves the stable [`CuObjectId`]. Fails if attached shape not found or obstacle storage full.
    pub fn detach_shape(&mut self, id: CuObjectId, world_shape: CuCollisionShape) -> CuResult<()> {
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        let att_idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;

        // Remove from attached shapes in place
        let att_count = self.attached_count as usize;
        let mut i = att_idx;
        while i + 1 < att_count {
            self.attached_shapes[i] = self.attached_shapes[i + 1];
            i += 1;
        }
        self.attached_shapes[att_count - 1] = CuAttachedShape::default();
        self.attached_count -= 1;

        // Insert into obstacles
        let obs_idx = self.obstacle_count as usize;
        self.obstacles[obs_idx] = CuSceneObstacle::new(id, world_shape);
        self.obstacle_count += 1;

        self.revision = self.revision.next();
        Ok(())
    }

    /// Clears all obstacles and attached shapes in place, advancing revision if not already empty.
    pub fn clear(&mut self) {
        if !self.is_empty() {
            self.obstacles = [CuSceneObstacle::default(); MAX_OBSTACLES];
            self.obstacle_count = 0;
            self.attached_shapes = [CuAttachedShape::default(); MAX_ATTACHED];
            self.attached_count = 0;
            self.revision = self.revision.next();
        }
    }

    /// Clears all obstacles in place, advancing revision if obstacles were present.
    pub fn clear_obstacles(&mut self) {
        if self.obstacle_count > 0 {
            self.obstacles = [CuSceneObstacle::default(); MAX_OBSTACLES];
            self.obstacle_count = 0;
            self.revision = self.revision.next();
        }
    }

    /// Clears all attached shapes in place, advancing revision if attached shapes were present.
    pub fn clear_attached_shapes(&mut self) {
        if self.attached_count > 0 {
            self.attached_shapes = [CuAttachedShape::default(); MAX_ATTACHED];
            self.attached_count = 0;
            self.revision = self.revision.next();
        }
    }

    /// Executes a transaction block on the planning scene.
    ///
    /// If the closure `f` succeeds, modifications are committed and revision changes if state was modified.
    /// If `f` returns an error, the scene rolls back to its pre-transaction state and revision is preserved.
    pub fn transaction<F, R>(&mut self, f: F) -> CuResult<R>
    where
        F: FnOnce(&mut Self) -> CuResult<R>,
    {
        let backup = self.clone();
        match f(self) {
            Ok(result) => {
                if *self != backup && self.revision == backup.revision {
                    self.revision = self.revision.next();
                }
                Ok(result)
            }
            Err(e) => {
                *self = backup;
                Err(e)
            }
        }
    }

    /// Populates a caller-provided [`CuCollisionWorld`] in place with all active scene obstacles.
    pub fn write_collision_world<const MAX_OUT: usize>(
        &self,
        out_world: &mut CuCollisionWorld<MAX_OUT>,
    ) -> CuResult<()> {
        let count = self.obstacle_count as usize;
        if count > MAX_OUT {
            return Err(CuSceneError::OutputCapacityExceeded.into());
        }
        out_world.clear();
        for obs in self.obstacles[..count].iter() {
            out_world.add_obstacle(obs.shape)?;
        }
        Ok(())
    }

    /// Computes world-space shapes for all active attached shapes using `link_frames`,
    /// writing them into `out_shapes` and setting `out_len` in place.
    pub fn world_attached_shapes<const N: usize, const MAX_OUT: usize>(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; MAX_OUT],
        out_len: &mut usize,
    ) -> CuResult<()> {
        let count = self.attached_count as usize;
        if count > MAX_OUT {
            return Err(CuSceneError::OutputCapacityExceeded.into());
        }
        for (att, out_shape) in self.attached_shapes[..count]
            .iter()
            .zip(out_shapes[..count].iter_mut())
        {
            if att.link_index >= N {
                return Err(CuSceneError::InvalidLinkIndex {
                    link_index: att.link_index,
                    max_links: N,
                }
                .into());
            }
            transform_shape(&link_frames[att.link_index], &att.shape, out_shape);
        }
        *out_len = count;
        Ok(())
    }

    fn alloc_id(&mut self) -> CuResult<CuObjectId> {
        let mut candidate = self.next_id;
        while candidate == 0 || self.contains(CuObjectId::new(candidate)) {
            candidate = candidate
                .checked_add(1)
                .ok_or_else(|| CuError::from("cu_collision: object id overflow"))?;
        }
        self.next_id = candidate
            .checked_add(1)
            .ok_or_else(|| CuError::from("cu_collision: object id overflow"))?;
        Ok(CuObjectId::new(candidate))
    }
}
