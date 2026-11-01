//! Copper `CuTask` wrappers for real-time forward kinematics, inverse kinematics, and twist servoing.

use crate::chain::CuKinematicChain;
use crate::ik::CuDiffIkParams;
use crate::ik::CuDiffIkStatus;
use crate::ik::CuIkParams;
use crate::ik::CuIkStatus;
use crate::ik::CuTwist6D;
use crate::joint::CuJointPosition;
use crate::joint::CuJointVelocity;
use crate::robots::CuChainSpec;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Fixed-DOF joint position payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointPositionsPayload<const N: usize> {
    #[serde(with = "crate::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
}

impl<const N: usize> Default for CuJointPositionsPayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
        }
    }
}

/// Health reported by the robot or simulator bridge.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuRobotStatus {
    /// The bridge has not confirmed readiness.
    #[default]
    NotReady,
    /// The robot is ready to accept commands.
    Ready,
    /// The robot reports a device-specific fault code.
    Fault(u32),
    /// An emergency stop is active.
    EmergencyStop,
}

/// Measured joint state: the only state payload a bridge has to produce.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointStatePayload<const N: usize> {
    /// Current health reported by the bridge.
    pub status: CuRobotStatus,
    #[serde(with = "crate::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    #[serde(with = "crate::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
}

impl<const N: usize> Default for CuJointStatePayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
            status: CuRobotStatus::NotReady,
        }
    }
}

/// Fixed-DOF joint velocity command/state payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointVelocitiesPayload<const N: usize> {
    #[serde(with = "crate::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
    pub status: CuDiffIkStatus,
}

impl<const N: usize> Default for CuJointVelocitiesPayload<N> {
    fn default() -> Self {
        Self {
            velocities: [CuJointVelocity::default(); N],
            status: CuDiffIkStatus::default(),
        }
    }
}

/// Inverse kinematics solution payload carrying solved joint angles and convergence status.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuIkSolutionPayload<const N: usize> {
    #[serde(with = "crate::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    pub status: CuIkStatus,
}

impl<const N: usize> Default for CuIkSolutionPayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            status: CuIkStatus::default(),
        }
    }
}

fn resolve_chain<const N: usize>(
    spec: Option<&CuChainSpec<N>>,
    tool_offset: Option<Transform3D<f64>>,
) -> CuResult<CuKinematicChain<N>> {
    let spec = spec.ok_or_else(|| CuError::from("Missing required `chain` configuration"))?;
    let mut chain = spec.resolve()?;
    if let Some(tool_offset) = tool_offset {
        chain.set_tool_offset(tool_offset);
    }
    chain.validate()?;
    Ok(chain)
}

fn read_params<T: Default + serde::de::DeserializeOwned>(
    cfg: Option<&ComponentConfig>,
) -> CuResult<T> {
    Ok(cfg
        .map(ComponentConfig::deserialize_into::<T>)
        .transpose()?
        .unwrap_or_default())
}

/// Node config of [`CuForwardKinematicsTask`].
///
/// ```ron
/// config: {"chain": "igus_rebel", "tool_offset": /* optional Transform3D */}
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuForwardKinematicsParams<const N: usize> {
    /// Required robot chain: a preset name (`"igus_rebel"`) or an inline chain.
    #[serde(default)]
    #[reflect(ignore)]
    pub chain: Option<CuChainSpec<N>>,
    /// Optional flange-to-tool transform replacing the chain's own tool offset.
    #[serde(default)]
    #[reflect(ignore)]
    pub tool_offset: Option<Transform3D<f64>>,
}

impl<const N: usize> Default for CuForwardKinematicsParams<N> {
    fn default() -> Self {
        Self {
            chain: None,
            tool_offset: None,
        }
    }
}

impl<const N: usize> CuForwardKinematicsParams<N> {
    /// Resolves the configured chain (with `tool_offset` applied) and validates it.
    pub fn resolve_chain(&self) -> CuResult<CuKinematicChain<N>> {
        resolve_chain(self.chain.as_ref(), self.tool_offset)
    }

    /// Checks that the chain is present and valid along with every other parameter.
    pub fn validate(&self) -> CuResult<()> {
        self.resolve_chain()?;
        Ok(())
    }
}

/// Node config of [`CuInverseKinematicsTask`].
///
/// ```ron
/// config: {"chain": "igus_rebel", "ik": {"max_iterations": 128, "damping": 0.05}}
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuInverseKinematicsParams<const N: usize> {
    /// Required robot chain: a preset name (`"igus_rebel"`) or an inline chain.
    #[serde(default)]
    #[reflect(ignore)]
    pub chain: Option<CuChainSpec<N>>,
    /// Optional flange-to-tool transform replacing the chain's own tool offset.
    #[serde(default)]
    #[reflect(ignore)]
    pub tool_offset: Option<Transform3D<f64>>,
    /// DLS solver parameters.
    #[serde(default)]
    pub ik: CuIkParams,
}

impl<const N: usize> Default for CuInverseKinematicsParams<N> {
    fn default() -> Self {
        Self {
            chain: None,
            tool_offset: None,
            ik: CuIkParams::default(),
        }
    }
}

impl<const N: usize> CuInverseKinematicsParams<N> {
    /// Resolves the configured chain (with `tool_offset` applied) and validates it.
    pub fn resolve_chain(&self) -> CuResult<CuKinematicChain<N>> {
        resolve_chain(self.chain.as_ref(), self.tool_offset)
    }

    /// Checks that the chain is present and valid along with every other parameter.
    pub fn validate(&self) -> CuResult<()> {
        self.resolve_chain()?;
        self.ik.validate()?;
        Ok(())
    }
}

/// Node config of [`CuServoTask`].
///
/// ```ron
/// config: {"chain": "igus_rebel", "diff_ik": {"base_damping": 0.02, "limit_margin": 0.15}}
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuServoParams<const N: usize> {
    /// Required robot chain: a preset name (`"igus_rebel"`) or an inline chain.
    #[serde(default)]
    #[reflect(ignore)]
    pub chain: Option<CuChainSpec<N>>,
    /// Optional flange-to-tool transform replacing the chain's own tool offset.
    #[serde(default)]
    #[reflect(ignore)]
    pub tool_offset: Option<Transform3D<f64>>,
    /// Differential IK parameters.
    #[serde(default)]
    pub diff_ik: CuDiffIkParams,
}

impl<const N: usize> Default for CuServoParams<N> {
    fn default() -> Self {
        Self {
            chain: None,
            tool_offset: None,
            diff_ik: CuDiffIkParams::default(),
        }
    }
}

impl<const N: usize> CuServoParams<N> {
    /// Resolves the configured chain (with `tool_offset` applied) and validates it.
    pub fn resolve_chain(&self) -> CuResult<CuKinematicChain<N>> {
        resolve_chain(self.chain.as_ref(), self.tool_offset)
    }

    /// Checks that the chain is present and valid along with every other parameter.
    pub fn validate(&self) -> CuResult<()> {
        self.resolve_chain()?;
        self.diff_ik.validate()?;
        Ok(())
    }
}

/// Stateless `CuTask` computing end-effector `Transform3D<f64>` from joint angles.
///
/// Node config: [`CuForwardKinematicsParams`].
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuForwardKinematicsTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
}

impl<const N: usize> CuForwardKinematicsTask<N> {
    /// Creates a forward kinematics task from a chain, validating it.
    pub fn from_params(chain: CuKinematicChain<N>) -> CuResult<Self> {
        chain.validate()?;
        Ok(Self { chain })
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }
}

impl<const N: usize> Freezable for CuForwardKinematicsTask<N> {}

impl<const N: usize> CuTask for CuForwardKinematicsTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuJointPositionsPayload<N>);
    type Output<'m> = output_msg!(Transform3D<f64>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params: CuForwardKinematicsParams<N> = read_params(cfg)?;
        params.validate()?;
        Self::from_params(params.resolve_chain()?)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let Some(joint_msg) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let mut pose = Transform3D::<f64>::identity();
        self.chain
            .forward_kinematics(&joint_msg.positions, &mut pose)?;
        output.set_payload(pose);
        Ok(())
    }
}

/// Stateless `CuTask` solving Damped Least-Squares inverse kinematics from a target pose and seed joints.
///
/// Node config: [`CuInverseKinematicsParams`].
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuInverseKinematicsTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    params: CuIkParams,
}

impl<const N: usize> CuInverseKinematicsTask<N> {
    /// Creates an inverse kinematics task from a chain and solver parameters, validating both.
    pub fn from_params(chain: CuKinematicChain<N>, params: CuIkParams) -> CuResult<Self> {
        chain.validate()?;
        params.validate()?;
        Ok(Self { chain, params })
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the IK solver parameters.
    pub const fn params(&self) -> &CuIkParams {
        &self.params
    }
}

impl<const N: usize> Freezable for CuInverseKinematicsTask<N> {}

impl<const N: usize> CuTask for CuInverseKinematicsTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, Transform3D<f64>, CuJointPositionsPayload<N>);
    type Output<'m> = output_msg!(CuIkSolutionPayload<N>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params: CuInverseKinematicsParams<N> = read_params(cfg)?;
        params.validate()?;
        Self::from_params(params.resolve_chain()?, params.ik)
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

        let (Some(target_pose), Some(seed_joints)) = (input.0.payload(), input.1.payload()) else {
            output.clear_payload();
            return Ok(());
        };

        let mut positions = [CuJointPosition::default(); N];
        let status = self.chain.inverse_kinematics_dls(
            target_pose,
            &seed_joints.positions,
            &self.params,
            &mut positions,
        )?;

        output.set_payload(CuIkSolutionPayload { positions, status });
        Ok(())
    }
}

/// Real-time Jacobian velocity/twist servoing `CuTask` with singularity and joint-limit damping.
///
/// Node config: [`CuServoParams`].
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuServoTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    params: CuDiffIkParams,
}

impl<const N: usize> CuServoTask<N> {
    /// Creates a twist servo task from a chain and differential IK parameters, validating both.
    pub fn from_params(chain: CuKinematicChain<N>, params: CuDiffIkParams) -> CuResult<Self> {
        chain.validate()?;
        params.validate()?;
        Ok(Self { chain, params })
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the differential IK parameters.
    pub const fn params(&self) -> &CuDiffIkParams {
        &self.params
    }
}

impl<const N: usize> Freezable for CuServoTask<N> {}

impl<const N: usize> CuTask for CuServoTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuTwist6D, CuJointPositionsPayload<N>);
    type Output<'m> = output_msg!(CuJointVelocitiesPayload<N>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params: CuServoParams<N> = read_params(cfg)?;
        params.validate()?;
        Self::from_params(params.resolve_chain()?, params.diff_ik)
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

        let (Some(twist), Some(joints)) = (input.0.payload(), input.1.payload()) else {
            output.clear_payload();
            return Ok(());
        };

        let mut velocities = [CuJointVelocity::default(); N];
        let status =
            self.chain
                .differential_ik(&joints.positions, twist, &self.params, &mut velocities)?;

        output.set_payload(CuJointVelocitiesPayload { velocities, status });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_task_configuration_is_required_and_dof_checked() {
        assert!(CuForwardKinematicsTask::<1>::new(None, ()).is_err());
        assert!(CuInverseKinematicsTask::<1>::new(None, ()).is_err());
        assert!(CuServoTask::<1>::new(None, ()).is_err());
        let cfg: ComponentConfig = ron::from_str(r#"#![enable(unwrap_newtypes)]
{"chain": {"joints": [{"joint_type": "revolute", "origin_translation": [0.0, 0.0, 0.0], "origin_rpy": [0.0, 0.0, 0.0], "axis": "pos_z", "limits": {"revolute": {"lower": -3.0, "upper": 3.0, "max_velocity": 1.0}}}]}}"#).unwrap();
        let mut task = CuForwardKinematicsTask::<1>::new(Some(&cfg), ()).unwrap();
        assert!(CuForwardKinematicsTask::<2>::new(Some(&cfg), ()).is_err());
        assert!(CuInverseKinematicsTask::<1>::new(Some(&cfg), ()).is_ok());
        assert!(CuServoTask::<1>::new(Some(&cfg), ()).is_ok());
        let typo: ComponentConfig =
            ron::from_str(r#"#![enable(unwrap_newtypes)] {"chain": "igus_rebel", "ikk": {}}"#)
                .unwrap();
        let err = CuInverseKinematicsTask::<6>::new(Some(&typo), ())
            .err()
            .unwrap();
        assert!(alloc::format!("{err}").contains("ikk"), "{err}");
        let (ctx, _) = CuContext::new_mock_clock();
        let input = CuMsg::new(Some(CuJointPositionsPayload::<1>::default()));
        let mut output = CuMsg::new(None);
        task.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(
            output.payload().unwrap().to_matrix(),
            Transform3D::<f64>::identity().to_matrix()
        );
    }
}

#[cfg(test)]
mod preset_tests {
    use super::*;

    #[test]
    fn chain_preset_name_resolves_and_checks_dof() {
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"chain": "igus_rebel", "ik": {"max_iterations": 10, "position_tolerance": 1e-3}}"#,
        )
        .unwrap();
        let task = CuInverseKinematicsTask::<6>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.chain(), &crate::igus_rebel_chain());
        assert_eq!(task.params().max_iterations, 10);
        let err = CuInverseKinematicsTask::<5>::new(Some(&cfg), ())
            .err()
            .unwrap();
        assert!(alloc::format!("{err}").contains("N = 5"), "{err}");
        let unknown: ComponentConfig =
            ron::from_str(r#"#![enable(unwrap_newtypes)] {"chain": "ur5"}"#).unwrap();
        let err = CuForwardKinematicsTask::<6>::new(Some(&unknown), ())
            .err()
            .unwrap();
        assert!(alloc::format!("{err}").contains("igus_rebel"), "{err}");
        let servo: ComponentConfig =
            ron::from_str(r#"#![enable(unwrap_newtypes)] {"chain": "igus_rebel", "diff_ik": {"base_damping": -1.0}}"#).unwrap();
        assert!(CuServoTask::<6>::new(Some(&servo), ()).is_err());
    }
}
