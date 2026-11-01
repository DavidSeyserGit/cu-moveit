//! Borrowed, zero-copy view of everything a robot is checked against.

use crate::scene::CuAttachedShape;
use crate::scene::CuObjectId;
use crate::scene_exclusions::CuSceneExclusion;
use crate::scene_exclusions::CuSceneExclusionPartner;
use crate::scene_exclusions::CuSceneExclusions;
use crate::shapes::CuCollisionShape;

/// Borrowed view of the collision environment: world obstacles, attached shapes, and exclusions.
///
/// The view copies nothing. Build it from raw obstacle shapes with [`new`](Self::new), or from a
/// planning scene with [`CuPlanningScene::collision_env`](crate::CuPlanningScene::collision_env)
/// (which also carries obstacle IDs, so obstacle exclusions resolve). [`EMPTY`](Self::EMPTY)
/// checks the robot against itself only.
///
/// # Attached shapes
///
/// Attached shapes are transformed by their frame (`None` = base frame) and checked against each
/// other, the robot's own shapes, and the obstacles:
///
/// - attached vs. robot and attached vs. attached report [`CuContactKind::SelfCollision`];
/// - attached vs. obstacle reports [`CuContactKind::EnvironmentCollision`].
///
/// Attached shapes on the same frame (including two `None`) are one rigid body and are never
/// checked against each other. Attached-vs-robot pairs are never skipped automatically; exclude
/// mounting contact explicitly with [`CuSceneExclusions`].
///
/// # Contact index encoding
///
/// For attached-shape contacts `link_index` is `robot_shape_count + attached_index`, so it never
/// collides with robot shape indices `0..robot_shape_count`. `partner_index` is the robot shape
/// index, `robot_shape_count + attached_index` for attached partners, or the obstacle index.
///
/// # Exclusions
///
/// Exclusions are validated at the start of every check: unassigned IDs, IDs absent from the view,
/// self-references, and out-of-range robot shape indices fail the check and leave the report
/// unmodified.
///
/// [`CuContactKind::SelfCollision`]: crate::CuContactKind::SelfCollision
/// [`CuContactKind::EnvironmentCollision`]: crate::CuContactKind::EnvironmentCollision
#[derive(Debug, Clone, Copy, Default)]
pub struct CuCollisionEnv<'a> {
    obstacles: &'a [CuCollisionShape],
    /// Either empty (anonymous obstacles) or parallel to `obstacles`.
    obstacle_ids: &'a [CuObjectId],
    attached: &'a [CuAttachedShape],
    exclusions: &'a [CuSceneExclusion],
}

impl<'a> CuCollisionEnv<'a> {
    /// No obstacles, attached shapes, or exclusions: self-collision only.
    pub const EMPTY: CuCollisionEnv<'static> = CuCollisionEnv {
        obstacles: &[],
        obstacle_ids: &[],
        attached: &[],
        exclusions: &[],
    };

    /// Environment of anonymous world-frame obstacles.
    pub const fn new(obstacles: &'a [CuCollisionShape]) -> Self {
        Self {
            obstacles,
            obstacle_ids: &[],
            attached: &[],
            exclusions: &[],
        }
    }

    /// Obstacles with stable IDs; `ids` must be parallel to `obstacles`.
    pub(crate) const fn with_obstacle_ids(
        obstacles: &'a [CuCollisionShape],
        obstacle_ids: &'a [CuObjectId],
    ) -> Self {
        Self {
            obstacles,
            obstacle_ids,
            attached: &[],
            exclusions: &[],
        }
    }

    /// Adds shapes attached to robot frames (or the base frame).
    pub const fn with_attached(mut self, attached: &'a [CuAttachedShape]) -> Self {
        self.attached = attached;
        self
    }

    /// Adds stable-ID pair exclusions for the attached shapes.
    pub fn with_exclusions<const MAX_EXCLUSIONS: usize>(
        mut self,
        exclusions: &'a CuSceneExclusions<MAX_EXCLUSIONS>,
    ) -> Self {
        self.exclusions = exclusions.as_slice();
        self
    }

    /// World-frame obstacle shapes.
    pub const fn obstacles(&self) -> &'a [CuCollisionShape] {
        self.obstacles
    }

    /// Attached shapes.
    pub const fn attached(&self) -> &'a [CuAttachedShape] {
        self.attached
    }

    /// Stable-ID exclusions.
    pub const fn exclusions(&self) -> &'a [CuSceneExclusion] {
        self.exclusions
    }

    pub(crate) fn obstacle_id(&self, index: usize) -> Option<CuObjectId> {
        self.obstacle_ids.get(index).copied()
    }

    pub(crate) fn find_attached(&self, id: CuObjectId) -> bool {
        self.attached.iter().any(|a| a.id == id)
    }

    pub(crate) fn find_obstacle(&self, id: CuObjectId) -> bool {
        self.obstacle_ids.contains(&id)
    }

    /// `true` if the pair (attached `attached_id`, `partner`) is excluded, in either order for
    /// attached-vs-attached pairs.
    pub(crate) fn is_excluded(
        &self,
        attached_id: CuObjectId,
        partner: CuSceneExclusionPartner,
    ) -> bool {
        self.exclusions.iter().any(|e| {
            (e.attached_id == attached_id && e.partner == partner)
                || matches!(partner, CuSceneExclusionPartner::Attached { attached_id: other }
                    if e.attached_id == other
                        && e.partner == CuSceneExclusionPartner::Attached { attached_id })
        })
    }
}
