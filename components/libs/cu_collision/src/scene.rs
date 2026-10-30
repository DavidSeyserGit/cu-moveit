//! Fixed-capacity, versioned planning scene storage using deterministic collision primitives.
//!
//! A planning scene maintains the set of active collision objects in the robot environment:
//! - Static/free obstacles in world coordinates.
//! - Attached objects rigidly affixed to specific robot links with local geometry.
//!
//! All storage is pre-allocated with compile-time const capacities (`MAX_OBSTACLES`, `MAX_ATTACHED`),
//! strictly allocation-free on the execution path, and versioned via monotonic [`CuSceneRevision`].
//! Mutations are transactional: failed operations (capacity overflow, missing ID, invalid links,
//! invalid geometry, or revision exhaustion) leave state unchanged, while successful modifications
//! advance the scene revision without wrapping.

use crate::model::CuCollisionWorld;
use crate::shapes::CuCollisionShape;
use crate::shapes::transform_shape;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Default capacity for environment obstacles in [`CuPlanningScene`].
pub const DEFAULT_MAX_SCENE_OBSTACLES: usize = 32;

/// Default capacity for attached collision shapes in [`CuPlanningScene`].
pub const DEFAULT_MAX_ATTACHED_SHAPES: usize = 8;

/// Returns `true` if `shape` contains strictly finite coordinates and non-negative finite radius.
fn is_valid_shape(shape: &CuCollisionShape) -> bool {
    match shape {
        CuCollisionShape::Sphere(sph) => {
            let c0 = sph.center[0].get::<meter>();
            let c1 = sph.center[1].get::<meter>();
            let c2 = sph.center[2].get::<meter>();
            let r = sph.radius.get::<meter>();
            c0.is_finite() && c1.is_finite() && c2.is_finite() && r.is_finite() && r >= 0.0
        }
        CuCollisionShape::Capsule(cap) => {
            let a0 = cap.a[0].get::<meter>();
            let a1 = cap.a[1].get::<meter>();
            let a2 = cap.a[2].get::<meter>();
            let b0 = cap.b[0].get::<meter>();
            let b1 = cap.b[1].get::<meter>();
            let b2 = cap.b[2].get::<meter>();
            let r = cap.radius.get::<meter>();
            a0.is_finite()
                && a1.is_finite()
                && a2.is_finite()
                && b0.is_finite()
                && b1.is_finite()
                && b2.is_finite()
                && r.is_finite()
                && r >= 0.0
        }
    }
}

/// Returns `true` if `frame` has strictly finite 4x4 matrix components.
fn is_finite_transform(frame: &Transform3D<f64>) -> bool {
    let mat = frame.to_matrix();
    let mut r = 0;
    while r < 4 {
        let mut c = 0;
        while c < 4 {
            if !mat[r][c].is_finite() {
                return false;
            }
            c += 1;
        }
        r += 1;
    }
    true
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
            CuSceneError::RevisionExhausted => {
                CuError::from("cu_collision: planning scene revision exhausted")
            }
            CuSceneError::InvalidGeometry => {
                CuError::from("cu_collision: invalid or non-finite collision shape geometry")
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

    /// Creates a new scene obstacle, validating ID and geometry.
    pub fn try_new(id: CuObjectId, shape: CuCollisionShape) -> CuResult<Self> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        Ok(Self { id, shape })
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

    /// Creates a new attached shape, validating ID and geometry.
    pub fn try_new(id: CuObjectId, link_index: usize, shape: CuCollisionShape) -> CuResult<Self> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        Ok(Self {
            id,
            link_index,
            shape,
        })
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

    /// Creates an attached shape by applying a link-relative transform, validating ID, frame, and geometry.
    pub fn try_new_with_transform(
        id: CuObjectId,
        link_index: usize,
        shape: &CuCollisionShape,
        link_from_shape: &Transform3D<f64>,
    ) -> CuResult<Self> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if !is_finite_transform(link_from_shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let mut local_shape = *shape;
        transform_shape(link_from_shape, shape, &mut local_shape);
        if !is_valid_shape(&local_shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        Ok(Self {
            id,
            link_index,
            shape: local_shape,
        })
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

#[derive(Serialize, Deserialize)]
struct CuPlanningSceneSnapshot<const MAX_OBSTACLES: usize, const MAX_ATTACHED: usize> {
    revision: CuSceneRevision,
    next_id: u32,
    #[serde(with = "serde_scene_arrays::obstacle_array")]
    obstacles: [CuSceneObstacle; MAX_OBSTACLES],
    obstacle_count: u32,
    #[serde(with = "serde_scene_arrays::attached_array")]
    attached_shapes: [CuAttachedShape; MAX_ATTACHED],
    attached_count: u32,
}

/// Fixed-capacity, versioned planning scene storage (`CuMsgPayload`-compliant).
///
/// Holds environment obstacles in world frame and attached shapes in robot link frames.
/// Every mutation is transactional and increments [`CuSceneRevision`] on success.
#[derive(Debug, Clone, PartialEq, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuPlanningScene<
    const MAX_OBSTACLES: usize = DEFAULT_MAX_SCENE_OBSTACLES,
    const MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED_SHAPES,
> {
    revision: CuSceneRevision,
    next_id: u32,
    obstacles: [CuSceneObstacle; MAX_OBSTACLES],
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
        if let Err(msg) = self.check_invariants() {
            return Err(serde::ser::Error::custom(msg));
        }
        let snapshot = CuPlanningSceneSnapshot {
            revision: self.revision,
            next_id: self.next_id,
            obstacles: self.obstacles,
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
            obstacles: raw.obstacles,
            obstacle_count: raw.obstacle_count,
            attached_shapes: raw.attached_shapes,
            attached_count: raw.attached_count,
        };
        if let Err(msg) = scene.check_invariants() {
            return Err(serde::de::Error::custom(msg));
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
        if let Err(msg) = self.check_invariants() {
            return Err(bincode::error::EncodeError::Other(msg));
        }
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

        let scene = Self {
            revision,
            next_id,
            obstacles,
            obstacle_count,
            attached_shapes,
            attached_count,
        };

        if let Err(msg) = scene.check_invariants() {
            return Err(bincode::error::DecodeError::Other(msg));
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

    /// Internal invariant checker returning static error string for serialization formats.
    fn check_invariants(&self) -> Result<(), &'static str> {
        if self.obstacle_count as usize > MAX_OBSTACLES {
            return Err("planning scene obstacle count exceeds capacity");
        }
        if self.attached_count as usize > MAX_ATTACHED {
            return Err("planning scene attached shape count exceeds capacity");
        }
        if self.next_id == 0 {
            return Err("planning scene next_id must be non-zero");
        }

        let obs_count = self.obstacle_count as usize;
        let att_count = self.attached_count as usize;

        let mut i = 0;
        while i < obs_count {
            let obs = &self.obstacles[i];
            if !obs.id.is_valid() {
                return Err("active scene obstacle has invalid or zero object id");
            }
            if obs.id.as_u32() >= self.next_id {
                return Err("scene obstacle id is greater than or equal to next_id");
            }
            if !is_valid_shape(&obs.shape) {
                return Err("scene obstacle has invalid or non-finite geometry");
            }
            let mut j = i + 1;
            while j < obs_count {
                if self.obstacles[j].id == obs.id {
                    return Err("duplicate object id in scene obstacles");
                }
                j += 1;
            }
            i += 1;
        }

        let mut i = 0;
        while i < att_count {
            let att = &self.attached_shapes[i];
            if !att.id.is_valid() {
                return Err("active attached shape has invalid or zero object id");
            }
            if att.id.as_u32() >= self.next_id {
                return Err("attached shape id is greater than or equal to next_id");
            }
            if !is_valid_shape(&att.shape) {
                return Err("attached shape has invalid or non-finite geometry");
            }
            let mut j = i + 1;
            while j < att_count {
                if self.attached_shapes[j].id == att.id {
                    return Err("duplicate object id in attached shapes");
                }
                j += 1;
            }
            let mut j = 0;
            while j < obs_count {
                if self.obstacles[j].id == att.id {
                    return Err("duplicate object id between obstacle and attached shape");
                }
                j += 1;
            }
            i += 1;
        }

        Ok(())
    }

    /// Validates all structural, identity, and geometric invariants of the planning scene.
    ///
    /// Checks that:
    /// - Active counts do not exceed static capacities.
    /// - `next_id` is strictly non-zero and greater than all active object IDs.
    /// - All active obstacles and attached shapes have non-zero, valid IDs.
    /// - All active object IDs are pairwise unique across obstacles and attached shapes.
    /// - All active collision primitives contain finite coordinates and non-negative finite radii.
    pub fn validate_invariants(&self) -> CuResult<()> {
        let obs_count = self.obstacle_count as usize;
        if obs_count > MAX_OBSTACLES {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        let att_count = self.attached_count as usize;
        if att_count > MAX_ATTACHED {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if self.next_id == 0 {
            return Err(CuSceneError::InvalidObjectId(CuObjectId::NIL).into());
        }

        for (i, obs) in self.obstacles[..obs_count].iter().enumerate() {
            if !obs.id.is_valid() {
                return Err(CuSceneError::InvalidObjectId(obs.id).into());
            }
            if obs.id.as_u32() >= self.next_id {
                return Err(CuSceneError::InvalidObjectId(obs.id).into());
            }
            if !is_valid_shape(&obs.shape) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
            for other in self.obstacles[i + 1..obs_count].iter() {
                if other.id == obs.id {
                    return Err(CuSceneError::DuplicateObjectId(obs.id).into());
                }
            }
        }

        for (i, att) in self.attached_shapes[..att_count].iter().enumerate() {
            if !att.id.is_valid() {
                return Err(CuSceneError::InvalidObjectId(att.id).into());
            }
            if att.id.as_u32() >= self.next_id {
                return Err(CuSceneError::InvalidObjectId(att.id).into());
            }
            if !is_valid_shape(&att.shape) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
            for other in self.attached_shapes[i + 1..att_count].iter() {
                if other.id == att.id {
                    return Err(CuSceneError::DuplicateObjectId(att.id).into());
                }
            }
            for obs in self.obstacles[..obs_count].iter() {
                if obs.id == att.id {
                    return Err(CuSceneError::DuplicateObjectId(att.id).into());
                }
            }
        }

        Ok(())
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
        self.obstacle_count.saturating_add(self.attached_count) as usize
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
        let count = (self.obstacle_count as usize).min(MAX_OBSTACLES);
        &self.obstacles[..count]
    }

    /// Borrows the active attached shapes as a slice.
    pub fn attached_shapes(&self) -> &[CuAttachedShape] {
        let count = (self.attached_count as usize).min(MAX_ATTACHED);
        &self.attached_shapes[..count]
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
        if !id.is_valid() {
            return false;
        }
        self.find_obstacle_index(id).is_some() || self.find_attached_index(id).is_some()
    }

    /// Finds the internal array index of an obstacle by its typed object ID.
    pub fn find_obstacle_index(&self, id: CuObjectId) -> Option<usize> {
        if !id.is_valid() {
            return None;
        }
        let count = (self.obstacle_count as usize).min(MAX_OBSTACLES);
        (0..count).find(|&i| self.obstacles[i].id == id)
    }

    /// Finds the internal array index of an attached shape by its typed object ID.
    pub fn find_attached_index(&self, id: CuObjectId) -> Option<usize> {
        if !id.is_valid() {
            return None;
        }
        let count = (self.attached_count as usize).min(MAX_ATTACHED);
        (0..count).find(|&i| self.attached_shapes[i].id == id)
    }

    /// Appends a new obstacle to the scene with an automatically generated unique [`CuObjectId`].
    ///
    /// On success, advances [`CuSceneRevision`]. On failure (capacity, geometry, or revision exhaustion),
    /// state is unmodified.
    pub fn add_obstacle(&mut self, shape: CuCollisionShape) -> CuResult<CuObjectId> {
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let id = self.alloc_id()?;
        let idx = self.obstacle_count as usize;
        self.obstacles[idx] = CuSceneObstacle::new(id, shape);
        self.obstacle_count += 1;
        self.revision = next_rev;
        Ok(id)
    }

    /// Adds an obstacle with an explicit [`CuObjectId`].
    ///
    /// Fails if the ID is invalid, already in use, geometry is non-finite, capacity is exceeded,
    /// or revision is exhausted. On failure, state is unmodified.
    pub fn add_obstacle_with_id(
        &mut self,
        id: CuObjectId,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        if self.contains(id) {
            return Err(CuSceneError::DuplicateObjectId(id).into());
        }
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let new_next_id = if id.as_u32() >= self.next_id {
            id.as_u32()
                .checked_add(1)
                .ok_or(CuSceneError::InvalidObjectId(id))?
        } else {
            self.next_id
        };
        let idx = self.obstacle_count as usize;
        self.obstacles[idx] = CuSceneObstacle::new(id, shape);
        self.obstacle_count += 1;
        self.next_id = new_next_id;
        self.revision = next_rev;
        Ok(())
    }

    /// Updates the collision geometry of an existing obstacle in place.
    ///
    /// Fails with [`CuSceneError::ObjectNotFound`] if the obstacle is not present,
    /// [`CuSceneError::InvalidGeometry`] if geometry is invalid, or
    /// [`CuSceneError::RevisionExhausted`] on revision exhaustion. On failure, state is unmodified.
    pub fn update_obstacle(&mut self, id: CuObjectId, shape: CuCollisionShape) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        self.obstacles[idx].shape = shape;
        self.revision = next_rev;
        Ok(())
    }

    /// Removes an obstacle by ID and compacts the storage in place, preserving insertion order.
    ///
    /// On failure, state is unmodified.
    pub fn remove_obstacle(&mut self, id: CuObjectId) -> CuResult<CuCollisionShape> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        let count = (self.obstacle_count as usize).min(MAX_OBSTACLES);
        let idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let removed = self.obstacles[idx].shape;
        let mut i = idx;
        while i + 1 < count {
            self.obstacles[i] = self.obstacles[i + 1];
            i += 1;
        }
        self.obstacles[count - 1] = CuSceneObstacle::default();
        self.obstacle_count -= 1;
        self.revision = next_rev;
        Ok(removed)
    }

    /// Appends an attached shape to the scene with an automatically generated unique [`CuObjectId`].
    ///
    /// On failure (capacity, geometry, or revision exhaustion), state is unmodified.
    pub fn add_attached_shape(
        &mut self,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<CuObjectId> {
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let id = self.alloc_id()?;
        let idx = self.attached_count as usize;
        self.attached_shapes[idx] = CuAttachedShape::new(id, link_index, shape);
        self.attached_count += 1;
        self.revision = next_rev;
        Ok(id)
    }

    /// Adds an attached shape with an explicit [`CuObjectId`].
    ///
    /// Fails if the ID is invalid, already in use, geometry is non-finite, capacity is exceeded,
    /// or revision is exhausted. On failure, state is unmodified.
    pub fn add_attached_shape_with_id(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if self.contains(id) {
            return Err(CuSceneError::DuplicateObjectId(id).into());
        }
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let new_next_id = if id.as_u32() >= self.next_id {
            id.as_u32()
                .checked_add(1)
                .ok_or(CuSceneError::InvalidObjectId(id))?
        } else {
            self.next_id
        };
        let idx = self.attached_count as usize;
        self.attached_shapes[idx] = CuAttachedShape::new(id, link_index, shape);
        self.attached_count += 1;
        self.next_id = new_next_id;
        self.revision = next_rev;
        Ok(())
    }

    /// Updates the collision geometry of an attached shape in place.
    ///
    /// On failure, state is unmodified.
    pub fn update_attached_shape(
        &mut self,
        id: CuObjectId,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        self.attached_shapes[idx].shape = shape;
        self.revision = next_rev;
        Ok(())
    }

    /// Updates the attached link index of an attached shape in place.
    ///
    /// On failure, state is unmodified.
    pub fn update_attached_link(&mut self, id: CuObjectId, link_index: usize) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        self.attached_shapes[idx].link_index = link_index;
        self.revision = next_rev;
        Ok(())
    }

    /// Updates both link index and geometry of an attached shape in place.
    ///
    /// On failure, state is unmodified.
    pub fn update_attached(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        self.attached_shapes[idx].link_index = link_index;
        self.attached_shapes[idx].shape = shape;
        self.revision = next_rev;
        Ok(())
    }

    /// Removes an attached shape by ID and compacts the storage in place.
    ///
    /// On failure, state is unmodified.
    pub fn remove_attached_shape(&mut self, id: CuObjectId) -> CuResult<CuAttachedShape> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        let count = (self.attached_count as usize).min(MAX_ATTACHED);
        let idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;
        let removed = self.attached_shapes[idx];
        let mut i = idx;
        while i + 1 < count {
            self.attached_shapes[i] = self.attached_shapes[i + 1];
            i += 1;
        }
        self.attached_shapes[count - 1] = CuAttachedShape::default();
        self.attached_count -= 1;
        self.revision = next_rev;
        Ok(removed)
    }

    /// Transitions an existing obstacle into an attached shape on `link_index` (e.g. object grasp).
    ///
    /// Preserves the stable [`CuObjectId`]. Fails if obstacle not found, geometry is invalid,
    /// attached storage is full, or revision is exhausted. On failure, state is unmodified.
    pub fn attach_obstacle(
        &mut self,
        id: CuObjectId,
        link_index: usize,
        local_shape: CuCollisionShape,
    ) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&local_shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.is_attached_full() {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        let obs_idx = self
            .find_obstacle_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;

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

        self.revision = next_rev;
        Ok(())
    }

    /// Detaches an attached shape into an environment obstacle in world coordinates (e.g. object release).
    ///
    /// Preserves the stable [`CuObjectId`]. Fails if attached shape not found, geometry is invalid,
    /// obstacle storage is full, or revision is exhausted. On failure, state is unmodified.
    pub fn detach_shape(&mut self, id: CuObjectId, world_shape: CuCollisionShape) -> CuResult<()> {
        if !id.is_valid() {
            return Err(CuSceneError::InvalidObjectId(id).into());
        }
        if !is_valid_shape(&world_shape) {
            return Err(CuSceneError::InvalidGeometry.into());
        }
        if self.is_obstacle_full() {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        let att_idx = self
            .find_attached_index(id)
            .ok_or(CuSceneError::ObjectNotFound(id))?;
        let next_rev = self
            .revision
            .checked_next()
            .ok_or(CuSceneError::RevisionExhausted)?;

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

        self.revision = next_rev;
        Ok(())
    }

    /// Clears all obstacles and attached shapes in place, advancing revision if not already empty.
    ///
    /// On failure (such as revision exhaustion), state is unmodified.
    pub fn clear(&mut self) -> CuResult<()> {
        if !self.is_empty() {
            let next_rev = self
                .revision
                .checked_next()
                .ok_or(CuSceneError::RevisionExhausted)?;
            self.obstacles = [CuSceneObstacle::default(); MAX_OBSTACLES];
            self.obstacle_count = 0;
            self.attached_shapes = [CuAttachedShape::default(); MAX_ATTACHED];
            self.attached_count = 0;
            self.revision = next_rev;
        }
        Ok(())
    }

    /// Clears all obstacles in place, advancing revision if obstacles were present.
    ///
    /// On failure (such as revision exhaustion), state is unmodified.
    pub fn clear_obstacles(&mut self) -> CuResult<()> {
        if self.obstacle_count > 0 {
            let next_rev = self
                .revision
                .checked_next()
                .ok_or(CuSceneError::RevisionExhausted)?;
            self.obstacles = [CuSceneObstacle::default(); MAX_OBSTACLES];
            self.obstacle_count = 0;
            self.revision = next_rev;
        }
        Ok(())
    }

    /// Clears all attached shapes in place, advancing revision if attached shapes were present.
    ///
    /// On failure (such as revision exhaustion), state is unmodified.
    pub fn clear_attached_shapes(&mut self) -> CuResult<()> {
        if self.attached_count > 0 {
            let next_rev = self
                .revision
                .checked_next()
                .ok_or(CuSceneError::RevisionExhausted)?;
            self.attached_shapes = [CuAttachedShape::default(); MAX_ATTACHED];
            self.attached_count = 0;
            self.revision = next_rev;
        }
        Ok(())
    }

    /// Executes a transaction block on the planning scene.
    ///
    /// If the closure `f` succeeds, modifications are committed and revision changes if state was modified.
    /// If `f` returns an error or revision exhaustion occurs, the scene rolls back to its pre-transaction state
    /// and revision is preserved.
    pub fn transaction<F, R>(&mut self, f: F) -> CuResult<R>
    where
        F: FnOnce(&mut Self) -> CuResult<R>,
    {
        let backup = self.clone();
        match f(self) {
            Ok(result) => {
                if *self != backup && self.revision == backup.revision {
                    let next_rev = match self.revision.checked_next() {
                        Some(rev) => rev,
                        None => {
                            *self = backup;
                            return Err(CuSceneError::RevisionExhausted.into());
                        }
                    };
                    self.revision = next_rev;
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
    ///
    /// Validates count capacity and geometry invariants transactionally:
    /// on error, `out_world` is left completely unmodified.
    pub fn write_collision_world<const MAX_OUT: usize>(
        &self,
        out_world: &mut CuCollisionWorld<MAX_OUT>,
    ) -> CuResult<()> {
        let count = self.obstacle_count as usize;
        if count > MAX_OBSTACLES {
            return Err(CuSceneError::ObstacleCapacityExceeded.into());
        }
        if count > MAX_OUT {
            return Err(CuSceneError::OutputCapacityExceeded.into());
        }
        for obs in self.obstacles[..count].iter() {
            if !is_valid_shape(&obs.shape) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
        }
        out_world.clear();
        for obs in self.obstacles[..count].iter() {
            out_world.add_obstacle(obs.shape)?;
        }
        Ok(())
    }

    /// Computes world-space shapes for all active attached shapes using `link_frames`,
    /// writing them into `out_shapes` and setting `out_len` in place.
    ///
    /// Validates count capacity, link indices, frame transforms, and geometry invariants transactionally:
    /// on error, neither `out_shapes` nor `out_len` is modified.
    pub fn world_attached_shapes<const N: usize, const MAX_OUT: usize>(
        &self,
        link_frames: &[Transform3D<f64>; N],
        out_shapes: &mut [CuCollisionShape; MAX_OUT],
        out_len: &mut usize,
    ) -> CuResult<()> {
        let count = self.attached_count as usize;
        if count > MAX_ATTACHED {
            return Err(CuSceneError::AttachedCapacityExceeded.into());
        }
        if count > MAX_OUT {
            return Err(CuSceneError::OutputCapacityExceeded.into());
        }
        for att in self.attached_shapes[..count].iter() {
            if att.link_index >= N {
                return Err(CuSceneError::InvalidLinkIndex {
                    link_index: att.link_index,
                    max_links: N,
                }
                .into());
            }
            if !is_valid_shape(&att.shape) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
            if !is_finite_transform(&link_frames[att.link_index]) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
        }
        // Validate every transform result before writing anything so that a
        // failure leaves `out_shapes` completely unmodified (transactional).
        let mut scratch = CuCollisionShape::default();
        for att in self.attached_shapes[..count].iter() {
            transform_shape(&link_frames[att.link_index], &att.shape, &mut scratch);
            if !is_valid_shape(&scratch) {
                return Err(CuSceneError::InvalidGeometry.into());
            }
        }
        for (att, out_shape) in self.attached_shapes[..count]
            .iter()
            .zip(out_shapes[..count].iter_mut())
        {
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
