//! Copper `CuTask` wrapper for real-time self-collision, environment, and attached shape collision checking.

use crate::model::CuCollisionReport;
use crate::model::CuCollisionWorld;
use crate::model::CuRobotCollisionModel;
use crate::scene::CuPlanningScene;
use crate::scene::DEFAULT_MAX_ATTACHED_SHAPES;
use crate::scene::DEFAULT_MAX_SCENE_OBSTACLES;
use crate::scene_query::CuSceneQueryExclusions;
use crate::scene_query::check_attached_shapes;
use crate::shapes::CuCollisionShape;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;

/// Default obstacle capacity for `CuCollisionCheckTask`.
pub const DEFAULT_MAX_OBSTACLES: usize = DEFAULT_MAX_SCENE_OBSTACLES;
/// Default attached shape capacity for `CuCollisionCheckTask`.
pub const DEFAULT_MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED_SHAPES;
/// Default contact capacity for `CuCollisionCheckTask`.
pub const DEFAULT_MAX_CONTACTS: usize = 16;

/// Stateless `CuTask` evaluating self-collision, environment obstacles, and attached shapes in place.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuCollisionCheckTask<
    const N: usize,
    const MAX_OBS: usize = DEFAULT_MAX_OBSTACLES,
    const MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED,
    const MAX_CONTACTS: usize = DEFAULT_MAX_CONTACTS,
    const M: usize = N,
> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    #[reflect(ignore)]
    model: CuRobotCollisionModel<N, M>,
}

impl<
    const N: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const M: usize,
> CuCollisionCheckTask<N, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, M>
{
    /// Creates a collision check task from an explicit kinematic chain and robot collision model.
    pub const fn new_with_model(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
    ) -> Self {
        Self { chain, model }
    }

    /// Borrows the kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the robot collision model.
    pub const fn model(&self) -> &CuRobotCollisionModel<N, M> {
        &self.model
    }

    /// Checks self-collision, environment obstacles, and attached shapes for configuration `q`
    /// against an optional planning scene snapshot, populating `out_report` in place.
    pub fn check_configuration(
        &self,
        q: &[CuJointPosition; N],
        scene: Option<&CuPlanningScene<MAX_OBS, MAX_ATTACHED>>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) -> CuResult<()> {
        check_scene_configuration(&self.chain, &self.model, q, scene, out_report)
    }
}

impl<
    const N: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const M: usize,
> Freezable for CuCollisionCheckTask<N, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, M>
{
}

impl<
    const N: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const M: usize,
> CuTask for CuCollisionCheckTask<N, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, M>
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        CuJointPositionsPayload<N>,
        CuPlanningScene<MAX_OBS, MAX_ATTACHED>
    );
    type Output<'m> = output_msg!(CuCollisionReport<MAX_CONTACTS>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let chain = CuKinematicChain::from_config(cfg)?;
        let mut model = CuRobotCollisionModel::from_config(cfg)?;
        if let Some(cfg) = cfg
            && let Some(margin_m) = cfg.get::<f64>("safety_margin_m")?
        {
            model.set_safety_margin(Length64::new::<meter>(margin_m));
        }
        Ok(Self::new_with_model(chain, model))
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = if !matches!(input.0.tov, Tov::None) {
            input.0.tov
        } else {
            input.1.tov
        };

        let Some(joints) = input.0.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let scene = input.1.payload();
        let mut report = CuCollisionReport::<MAX_CONTACTS>::default();
        check_scene_configuration(
            &self.chain,
            &self.model,
            &joints.positions,
            scene,
            &mut report,
        )?;
        output.set_payload(report);
        Ok(())
    }
}

/// Evaluates self-collision, environment obstacles, and attached shapes for a robot configuration
/// against an optional planning scene snapshot, writing contacts and distance into `out_report`.
fn check_scene_configuration<
    const N: usize,
    const M: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
>(
    chain: &CuKinematicChain<N>,
    model: &CuRobotCollisionModel<N, M>,
    q: &[CuJointPosition; N],
    scene: Option<&CuPlanningScene<MAX_OBS, MAX_ATTACHED>>,
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
) -> CuResult<()> {
    let mut frames = [Transform3D::<f64>::identity(); N];
    let mut ee = Transform3D::<f64>::identity();
    chain.forward_frames(q, &mut frames, &mut ee)?;
    check_scene_frames(model, &frames, scene, out_report)
}

/// Evaluates self-collision, environment obstacles, and attached shapes for precomputed `link_frames`
/// against an optional planning scene snapshot, writing contacts and distance into `out_report`.
fn check_scene_frames<
    const N: usize,
    const M: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
>(
    model: &CuRobotCollisionModel<N, M>,
    link_frames: &[Transform3D<f64>; N],
    scene: Option<&CuPlanningScene<MAX_OBS, MAX_ATTACHED>>,
    out_report: &mut CuCollisionReport<MAX_CONTACTS>,
) -> CuResult<()> {
    match scene {
        None => {
            model.check_frames::<MAX_OBS, MAX_CONTACTS>(link_frames, None, out_report);
            Ok(())
        }
        Some(scene) => {
            let mut world = CuCollisionWorld::<MAX_OBS>::new();
            scene.write_collision_world(&mut world)?;
            model.check_frames(link_frames, Some(&world), out_report);

            let attached_len = scene.attached_count();
            if attached_len > 0 {
                let attached_slice = scene.attached_shapes();
                let mut attached_shapes = [CuCollisionShape::default(); MAX_ATTACHED];
                let mut attached_frames = [None; MAX_ATTACHED];
                for (i, att) in attached_slice.iter().enumerate() {
                    attached_shapes[i] = att.shape;
                    attached_frames[i] = Some(att.link_index);
                }

                let exclusions = CuSceneQueryExclusions::<0>::new();
                let mut out_attached_world_shapes = [CuCollisionShape::default(); MAX_ATTACHED];
                let mut out_robot_world_shapes = [CuCollisionShape::default(); M];
                check_attached_shapes(
                    model,
                    link_frames,
                    &attached_shapes,
                    &attached_frames,
                    attached_len,
                    Some(&world),
                    &exclusions,
                    &mut out_attached_world_shapes,
                    &mut out_robot_world_shapes,
                    out_report,
                )?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_collision_task_requires_matching_explicit_models() {
        assert!(CuCollisionCheckTask::<1>::new(None, ()).is_err());
        let chain_only: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]}}"#).unwrap();
        assert!(CuCollisionCheckTask::<1>::new(Some(&chain_only), ()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]}, "collision_model": {"link_shapes": [{"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}], "safety_margin": 0.01}}"#).unwrap();
        let task = CuCollisionCheckTask::<1>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.model().link_shapes().len(), 1);
        assert_eq!(task.model().safety_margin().get::<meter>(), 0.01);
        assert!(CuRobotCollisionModel::<2>::from_config(Some(&cfg)).is_err());
    }
}
