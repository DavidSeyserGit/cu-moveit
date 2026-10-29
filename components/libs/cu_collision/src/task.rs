//! Copper `CuTask` wrapper for real-time self-collision and environment collision checking.

use crate::model::CuCollisionReport;
use crate::model::CuCollisionWorld;
use crate::model::CuRobotCollisionModel;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuKinematicChain;
use cu29::prelude::*;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;

/// Default obstacle capacity for `CuCollisionCheckTask`.
pub const DEFAULT_MAX_OBSTACLES: usize = 16;
/// Default contact capacity for `CuCollisionCheckTask`.
pub const DEFAULT_MAX_CONTACTS: usize = 16;

/// Stateless `CuTask` evaluating self-collision and environment collision in place.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuCollisionCheckTask<
    const N: usize,
    const MAX_OBS: usize = DEFAULT_MAX_OBSTACLES,
    const MAX_CONTACTS: usize = DEFAULT_MAX_CONTACTS,
> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    #[reflect(ignore)]
    model: CuRobotCollisionModel<N>,
}

impl<const N: usize, const MAX_OBS: usize, const MAX_CONTACTS: usize>
    CuCollisionCheckTask<N, MAX_OBS, MAX_CONTACTS>
{
    /// Creates a collision check task from an explicit kinematic chain and robot collision model.
    pub const fn new_with_model(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N>,
    ) -> Self {
        Self { chain, model }
    }

    /// Borrows the kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the robot collision model.
    pub const fn model(&self) -> &CuRobotCollisionModel<N> {
        &self.model
    }
}

impl<const N: usize, const MAX_OBS: usize, const MAX_CONTACTS: usize> Freezable
    for CuCollisionCheckTask<N, MAX_OBS, MAX_CONTACTS>
{
}

impl<const N: usize, const MAX_OBS: usize, const MAX_CONTACTS: usize> CuTask
    for CuCollisionCheckTask<N, MAX_OBS, MAX_CONTACTS>
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuJointPositionsPayload<N>, CuCollisionWorld<MAX_OBS>);
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

        let world = input.1.payload();
        let mut report = CuCollisionReport::<MAX_CONTACTS>::default();
        self.model
            .check_configuration(&self.chain, &joints.positions, world, &mut report)?;
        output.set_payload(report);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_collision_task_requires_matching_explicit_models() {
        assert!(CuCollisionCheckTask::<1>::new(None, ()).is_err());
        let chain_only: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}]}}"#).unwrap();
        assert!(CuCollisionCheckTask::<1>::new(Some(&chain_only), ()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}]},"collision_model": {"link_shapes": [{"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}], "safety_margin": 0.01}}"#).unwrap();
        let task = CuCollisionCheckTask::<1>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.model().link_shapes().len(), 1);
        assert_eq!(task.model().safety_margin().get::<meter>(), 0.01);
        assert!(CuRobotCollisionModel::<2>::from_config(Some(&cfg)).is_err());
    }
}
