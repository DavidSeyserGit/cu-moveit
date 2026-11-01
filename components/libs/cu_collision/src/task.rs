//! Copper `CuTask` for real-time self-collision, environment, and attached-shape collision checking.

use crate::env::CuCollisionEnv;
use crate::model::CuCollisionReport;
use crate::model::CuRobotCollisionModel;
use crate::robots::CuCollisionModelSpec;
use crate::scene::CuPlanningScene;
use crate::scene::DEFAULT_MAX_ATTACHED;
use crate::scene::DEFAULT_MAX_OBSTACLES;
use crate::scene_exclusions::CuSceneExclusions;
use crate::scene_exclusions::DEFAULT_MAX_EXCLUSIONS;
use cu_kinematics::CuChainSpec;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointPositionsPayload;
use cu_kinematics::CuKinematicChain;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Default contact capacity of [`CuCollisionCheckTask`].
pub const DEFAULT_MAX_CONTACTS: usize = 16;

/// Node config of [`CuCollisionCheckTask`].
///
/// ```ron
/// config: {"chain": "igus_rebel", "collision_model": "igus_rebel"}
/// ```
///
/// Both keys are required. `chain` is a [`CuChainSpec`] and `collision_model` a
/// [`CuCollisionModelSpec`] (preset name or inline model).
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuCollisionCheckParams<const N: usize, const M: usize = N> {
    /// Required robot chain: a preset name (`"igus_rebel"`) or an inline chain.
    #[serde(default)]
    #[reflect(ignore)]
    pub chain: Option<CuChainSpec<N>>,
    /// Required collision model: a preset name (`"igus_rebel"`) or an inline model.
    #[serde(default)]
    #[reflect(ignore)]
    pub collision_model: Option<CuCollisionModelSpec<M>>,
}

impl<const N: usize, const M: usize> Default for CuCollisionCheckParams<N, M> {
    fn default() -> Self {
        Self {
            chain: None,
            collision_model: None,
        }
    }
}

impl<const N: usize, const M: usize> CuCollisionCheckParams<N, M> {
    /// Resolves and validates the configured chain.
    pub fn resolve_chain(&self) -> CuResult<CuKinematicChain<N>> {
        self.chain
            .as_ref()
            .ok_or_else(|| CuError::from("Missing required `chain` configuration"))?
            .resolve()
    }

    /// Resolves and validates the configured collision model.
    pub fn resolve_collision_model(&self) -> CuResult<CuRobotCollisionModel<N, M>> {
        self.collision_model
            .as_ref()
            .ok_or_else(|| CuError::from("Missing required `collision_model` configuration"))?
            .resolve()
    }

    /// Checks that both the chain and the collision model are present and valid.
    pub fn validate(&self) -> CuResult<()> {
        self.resolve_chain()?;
        self.resolve_collision_model()?;
        Ok(())
    }
}

/// Stateless `CuTask` evaluating self-collision, environment obstacles, and attached shapes in place.
///
/// Node config: [`CuCollisionCheckParams`]. Inputs: joint positions, planning scene, exclusions.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuCollisionCheckTask<
    const N: usize,
    const M: usize = N,
    const MAX_OBS: usize = DEFAULT_MAX_OBSTACLES,
    const MAX_ATTACHED: usize = DEFAULT_MAX_ATTACHED,
    const MAX_CONTACTS: usize = DEFAULT_MAX_CONTACTS,
    const MAX_EXCL: usize = DEFAULT_MAX_EXCLUSIONS,
> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    #[reflect(ignore)]
    model: CuRobotCollisionModel<N, M>,
}

impl<
    const N: usize,
    const M: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const MAX_EXCL: usize,
> CuCollisionCheckTask<N, M, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, MAX_EXCL>
{
    /// Creates a collision check task from a kinematic chain and robot collision model.
    pub fn from_params(
        chain: CuKinematicChain<N>,
        model: CuRobotCollisionModel<N, M>,
    ) -> CuResult<Self> {
        chain.validate()?;
        Ok(Self { chain, model })
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
    ///
    /// Stable-ID `exclusions` are validated against `scene`; invalid references, or non-empty
    /// exclusions without a scene, fail the check.
    pub fn check_configuration(
        &self,
        q: &[CuJointPosition; N],
        scene: Option<&CuPlanningScene<MAX_OBS, MAX_ATTACHED>>,
        exclusions: Option<&CuSceneExclusions<MAX_EXCL>>,
        out_report: &mut CuCollisionReport<MAX_CONTACTS>,
    ) -> CuResult<()> {
        let env = match (scene, exclusions) {
            (Some(scene), Some(exclusions)) => scene.collision_env().with_exclusions(exclusions),
            (Some(scene), None) => scene.collision_env(),
            (None, Some(exclusions)) if !exclusions.is_empty() => {
                return Err(CuError::from(
                    "cu_collision: scene exclusions require a planning scene snapshot",
                ));
            }
            (None, _) => CuCollisionEnv::EMPTY,
        };
        self.model
            .check_configuration(&self.chain, q, &env, out_report)
    }
}

impl<
    const N: usize,
    const M: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const MAX_EXCL: usize,
> Freezable for CuCollisionCheckTask<N, M, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, MAX_EXCL>
{
}

impl<
    const N: usize,
    const M: usize,
    const MAX_OBS: usize,
    const MAX_ATTACHED: usize,
    const MAX_CONTACTS: usize,
    const MAX_EXCL: usize,
> CuTask for CuCollisionCheckTask<N, M, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, MAX_EXCL>
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        CuJointPositionsPayload<N>,
        CuPlanningScene<MAX_OBS, MAX_ATTACHED>,
        CuSceneExclusions<MAX_EXCL>
    );
    type Output<'m> = output_msg!(CuCollisionReport<MAX_CONTACTS>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params: CuCollisionCheckParams<N, M> = cfg
            .map(ComponentConfig::deserialize_into::<CuCollisionCheckParams<N, M>>)
            .transpose()?
            .unwrap_or_default();
        params.validate()?;
        Self::from_params(params.resolve_chain()?, params.resolve_collision_model()?)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = if !matches!(input.0.tov, Tov::None) {
            input.0.tov
        } else if !matches!(input.1.tov, Tov::None) {
            input.1.tov
        } else {
            input.2.tov
        };

        let Some(joints) = input.0.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let mut report = CuCollisionReport::<MAX_CONTACTS>::default();
        self.check_configuration(
            &joints.positions,
            input.1.payload(),
            input.2.payload(),
            &mut report,
        )?;
        output.set_payload(report);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::length::meter;

    const ONE_JOINT_CHAIN: &str = r#"{"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]}"#;

    fn config(body: &str) -> ComponentConfig {
        ron::from_str(&alloc::format!("#![enable(unwrap_newtypes)]\n{{{body}}}")).unwrap()
    }

    #[test]
    fn serial_collision_task_requires_matching_explicit_models() {
        assert!(CuCollisionCheckTask::<1>::new(None, ()).is_err());
        let chain_only = config(&alloc::format!(r#""chain": {ONE_JOINT_CHAIN}"#));
        assert!(CuCollisionCheckTask::<1>::new(Some(&chain_only), ()).is_err());
        let model = r#"{"shapes": [{"shape": {"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}, "frame": 0}], "safety_margin": 0.01}"#;
        let cfg = config(&alloc::format!(
            r#""chain": {ONE_JOINT_CHAIN}, "collision_model": {model}"#
        ));
        let task = CuCollisionCheckTask::<1>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.model().link_shapes().len(), 1);
        assert_eq!(task.model().safety_margin().get::<meter>(), 0.01);
        assert!(
            cfg.deserialize_into::<CuCollisionCheckParams<1, 2>>()
                .is_err()
        );
        // The old second spelling of the margin is rejected instead of silently ignored.
        let typo = config(&alloc::format!(
            r#""chain": {ONE_JOINT_CHAIN}, "collision_model": {model}, "safety_margin_m": 0.02"#
        ));
        assert!(CuCollisionCheckTask::<1>::new(Some(&typo), ()).is_err());
    }

    #[test]
    fn collision_task_accepts_presets() {
        let cfg = config(r#""chain": "igus_rebel", "collision_model": "igus_rebel""#);
        let task = CuCollisionCheckTask::<6>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.model(), &crate::igus_rebel_collision_model());
        assert!(CuCollisionCheckTask::<6, 4>::new(Some(&cfg), ()).is_err());
    }
}
