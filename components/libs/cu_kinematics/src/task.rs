//! Copper `CuTask` wrappers for real-time forward kinematics, inverse kinematics, and twist servoing.

use crate::chain::CuKinematicChain;
use crate::ik::CuDiffIkConfig;
use crate::ik::CuDiffIkStatus;
use crate::ik::CuIkConfig;
use crate::ik::CuIkStatus;
use crate::ik::CuTwist6D;
use crate::joint::CuJointPosition;
use crate::joint::CuJointVelocity;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Fixed-DOF joint position payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
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

/// Fixed-DOF joint velocity command/state payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
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
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
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

/// Stateless `CuTask` computing end-effector `Transform3D<f64>` from joint angles.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuForwardKinematicsTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
}

impl<const N: usize> CuForwardKinematicsTask<N> {
    /// Creates a forward kinematics task from an explicit `CuKinematicChain<N>`.
    pub const fn new_with_chain(chain: CuKinematicChain<N>) -> Self {
        Self { chain }
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
        let chain = CuKinematicChain::from_config(cfg)?;
        Ok(Self::new_with_chain(chain))
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
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuInverseKinematicsTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    cfg: CuIkConfig,
}

impl<const N: usize> CuInverseKinematicsTask<N> {
    /// Creates an inverse kinematics task from an explicit chain and solver configuration.
    pub const fn new_with_chain(chain: CuKinematicChain<N>, cfg: CuIkConfig) -> Self {
        Self { chain, cfg }
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the IK solver configuration.
    pub const fn config(&self) -> &CuIkConfig {
        &self.cfg
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
        let chain = CuKinematicChain::from_config(cfg)?;
        let ik_cfg = CuIkConfig::from_config(cfg)?;
        Ok(Self::new_with_chain(chain, ik_cfg))
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
            &self.cfg,
            &mut positions,
        )?;

        output.set_payload(CuIkSolutionPayload { positions, status });
        Ok(())
    }
}

/// Real-time Jacobian velocity/twist servoing `CuTask` with singularity and joint-limit damping.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuServoTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
    cfg: CuDiffIkConfig,
}

impl<const N: usize> CuServoTask<N> {
    /// Creates a twist servo task from an explicit chain and differential IK configuration.
    pub const fn new_with_chain(chain: CuKinematicChain<N>, cfg: CuDiffIkConfig) -> Self {
        Self { chain, cfg }
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }

    /// Borrows the differential IK configuration.
    pub const fn config(&self) -> &CuDiffIkConfig {
        &self.cfg
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
        let chain = CuKinematicChain::from_config(cfg)?;
        let diff_cfg = CuDiffIkConfig::from_config(cfg)?;
        Ok(Self::new_with_chain(chain, diff_cfg))
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
                .differential_ik(&joints.positions, twist, &self.cfg, &mut velocities)?;

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
