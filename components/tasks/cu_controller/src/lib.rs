//! Robot-agnostic unified joint controller for `copper-rs`.
//!
//! [`CuControllerTask`] sits between a trajectory follower
//! (`CuTrajectoryFollowOutput`) and a robot bridge. It consumes the follower
//! setpoint plus the measured joint state and emits one unified
//! [`CuJointCommandPayload`] — the only command type a bridge has to carry.
//! Simulators and hardware bridges implement the same
//! `joint_command`/`joint_state` channels, so swapping sim for a real robot
//! is a RON graph change, not an application change.
//!
//! Behaviour per cycle:
//! - No setpoint yet: `Idle`, holding the last measured state (zeros before
//!   the first state arrives).
//! - Setpoint + fresh state: command mirrors the setpoint; status maps the
//!   follower status (`Running` → `Running`, `Finished` → `Holding`,
//!   `ErrorEmptyTrajectory` → `Fault`).
//! - Tracking error above `tracking_error_limit_rad`: `Fault`, command
//!   holds the measured state instead of chasing the setpoint.
//! - Setpoint without any measured state: `Fault`, holding the last command.
//!
//! State that affects later cycles (last command, last status) round-trips
//! through `Freezable`, so deterministic replay resumes correctly.

#![cfg_attr(not(feature = "std"), no_std)]

use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu_trajectory::CuFollowStatus;
use cu_trajectory::CuTrajectoryFollowOutput;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// Controller status carried by [`CuJointCommandPayload`].
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuControllerStatus {
    /// No setpoint has been received yet.
    #[default]
    Idle,
    /// Tracking a running setpoint within the error limit.
    Running,
    /// Holding a finished setpoint (or the measured state while idle).
    Holding,
    /// Tracking error exceeded the limit, the follower reported an error,
    /// or a setpoint arrived without measured state.
    Fault,
}

/// Measured joint state: the only state payload a bridge has to produce.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointStatePayload<const N: usize> {
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
}

impl<const N: usize> Default for CuJointStatePayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
        }
    }
}

/// Unified joint command: the only command payload a bridge has to carry.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointCommandPayload<const N: usize> {
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
    pub status: CuControllerStatus,
    /// Largest absolute joint error (setpoint − measured) seen this cycle.
    pub max_tracking_error: CuJointPosition,
}

impl<const N: usize> Default for CuJointCommandPayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
            status: CuControllerStatus::default(),
            max_tracking_error: CuJointPosition::default(),
        }
    }
}

/// Unified robot-agnostic joint controller task.
///
/// Inputs are `(setpoint, state)`: a `CuTrajectoryFollowOutput<N>` and a
/// [`CuJointStatePayload<N>`]. Output is a [`CuJointCommandPayload<N>`]
/// addressed to any bridge exposing a `joint_command` channel.
///
/// Config keys:
/// - `tracking_error_limit_rad` (`f64`, default `0.5`): max allowed absolute
///   joint error in radians before the controller faults and holds the
///   measured state.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuControllerTask<const N: usize> {
    tracking_error_limit_rad: f64,
    #[reflect(ignore)]
    last_command: [CuJointPosition; N],
    #[reflect(ignore)]
    last_status: CuControllerStatus,
    has_command: bool,
}

impl<const N: usize> CuControllerTask<N> {
    /// Creates a controller with an explicit tracking-error limit (radians).
    pub fn new_with_limit(tracking_error_limit_rad: f64) -> Self {
        Self {
            tracking_error_limit_rad,
            last_command: [CuJointPosition::default(); N],
            last_status: CuControllerStatus::Idle,
            has_command: false,
        }
    }
}

impl<const N: usize> Default for CuControllerTask<N> {
    fn default() -> Self {
        Self::new_with_limit(0.5)
    }
}

impl<const N: usize> Freezable for CuControllerTask<N> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.tracking_error_limit_rad, encoder)?;
        Encode::encode(&self.last_command, encoder)?;
        Encode::encode(&self.last_status, encoder)?;
        Encode::encode(&self.has_command, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.tracking_error_limit_rad = Decode::decode(decoder)?;
        self.last_command = Decode::decode(decoder)?;
        self.last_status = Decode::decode(decoder)?;
        self.has_command = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<const N: usize> CuTask for CuControllerTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuTrajectoryFollowOutput<N>, CuJointStatePayload<N>);
    type Output<'m> = output_msg!(CuJointCommandPayload<N>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let limit = match cfg {
            Some(cfg) => cfg.get::<f64>("tracking_error_limit_rad")?.unwrap_or(0.5),
            None => 0.5,
        };
        if !limit.is_finite() || limit <= 0.0 {
            return Err(CuError::from(
                "cu-controller: 'tracking_error_limit_rad' must be positive and finite",
            ));
        }
        Ok(Self::new_with_limit(limit))
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.1.tov;
        let state = input.1.payload();
        let setpoint = input.0.payload();

        let mut command = CuJointCommandPayload::<N>::default();
        match (setpoint, state) {
            (None, None) => {
                command.positions = self.last_command;
                command.status = if self.has_command {
                    CuControllerStatus::Holding
                } else {
                    CuControllerStatus::Idle
                };
            }
            (None, Some(state)) => {
                command.positions = state.positions;
                command.status = CuControllerStatus::Idle;
            }
            (Some(_), None) => {
                command.positions = self.last_command;
                command.status = CuControllerStatus::Fault;
            }
            (Some(setpoint), Some(state)) => {
                let mut max_err = 0.0f64;
                let mut max_kind = CuJointPosition::default();
                for j in 0..N {
                    let err =
                        (setpoint.positions[j].si_value() - state.positions[j].si_value()).abs();
                    if err > max_err {
                        max_err = err;
                        max_kind = setpoint.positions[j];
                    }
                }
                command.max_tracking_error = max_kind.with_si_value(max_err);
                if setpoint.status == CuFollowStatus::ErrorEmptyTrajectory
                    || max_err > self.tracking_error_limit_rad
                {
                    command.positions = state.positions;
                    command.status = CuControllerStatus::Fault;
                } else {
                    command.positions = setpoint.positions;
                    command.velocities = setpoint.velocities;
                    command.status = match setpoint.status {
                        CuFollowStatus::Running => CuControllerStatus::Running,
                        CuFollowStatus::Finished => CuControllerStatus::Holding,
                        CuFollowStatus::Idle => CuControllerStatus::Idle,
                        CuFollowStatus::ErrorEmptyTrajectory => CuControllerStatus::Fault,
                    };
                }
            }
        }

        self.last_command = command.positions;
        self.last_status = command.status;
        self.has_command = true;
        output.set_payload(command);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::angular_velocity::radian_per_second;
    use cu29::units::si::f64::{Angle, AngularVelocity};

    fn follow(pos: f64, status: CuFollowStatus) -> CuTrajectoryFollowOutput<2> {
        CuTrajectoryFollowOutput {
            positions: [CuJointPosition::Revolute(Angle::new::<radian>(pos)); 2],
            velocities: [CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(0.1));
                2],
            status,
        }
    }

    fn state(pos: f64) -> CuJointStatePayload<2> {
        CuJointStatePayload {
            positions: [CuJointPosition::Revolute(Angle::new::<radian>(pos)); 2],
            velocities: [CuJointVelocity::default(); 2],
        }
    }

    fn run(
        task: &mut CuControllerTask<2>,
        setpoint: Option<CuTrajectoryFollowOutput<2>>,
        state: Option<CuJointStatePayload<2>>,
    ) -> CuJointCommandPayload<2> {
        let setpoint_msg = CuMsg::new(setpoint);
        let state_msg = CuMsg::new(state);
        let input = (&setpoint_msg, &state_msg);
        let mut output = CuMsg::<CuJointCommandPayload<2>>::default();
        let (ctx, _clock) = CuContext::new_mock_clock();
        task.process(&ctx, &input, &mut output).unwrap();
        output.payload().unwrap().clone()
    }

    #[test]
    fn test_idle_holds_measured_state() {
        let mut task = CuControllerTask::<2>::default();
        let out = run(&mut task, None, Some(state(0.3)));
        assert_eq!(out.status, CuControllerStatus::Idle);
        assert_eq!(out.positions[0].si_value(), 0.3);
    }

    #[test]
    fn test_running_mirrors_setpoint() {
        let mut task = CuControllerTask::<2>::default();
        let out = run(
            &mut task,
            Some(follow(0.4, CuFollowStatus::Running)),
            Some(state(0.35)),
        );
        assert_eq!(out.status, CuControllerStatus::Running);
        assert_eq!(out.positions[0].si_value(), 0.4);
        assert!((out.max_tracking_error.si_value() - 0.05).abs() < 1e-12);
    }

    #[test]
    fn test_finished_holds() {
        let mut task = CuControllerTask::<2>::default();
        let out = run(
            &mut task,
            Some(follow(0.4, CuFollowStatus::Finished)),
            Some(state(0.4)),
        );
        assert_eq!(out.status, CuControllerStatus::Holding);
    }

    #[test]
    fn test_tracking_fault_holds_measured() {
        let mut task = CuControllerTask::<2>::new_with_limit(0.1);
        let out = run(
            &mut task,
            Some(follow(1.0, CuFollowStatus::Running)),
            Some(state(0.0)),
        );
        assert_eq!(out.status, CuControllerStatus::Fault);
        assert_eq!(out.positions[0].si_value(), 0.0);
    }

    #[test]
    fn test_setpoint_without_state_faults() {
        let mut task = CuControllerTask::<2>::default();
        let out = run(&mut task, Some(follow(0.4, CuFollowStatus::Running)), None);
        assert_eq!(out.status, CuControllerStatus::Fault);
    }

    #[test]
    fn test_freeze_thaw_round_trip() {
        let mut task = CuControllerTask::<2>::new_with_limit(0.25);
        let _ = run(
            &mut task,
            Some(follow(0.4, CuFollowStatus::Running)),
            Some(state(0.35)),
        );
        let mut bytes = [0u8; 256];
        let mut encoder = bincode::enc::EncoderImpl::new(
            bincode::enc::write::SliceWriter::new(&mut bytes),
            bincode::config::standard(),
        );
        task.freeze(&mut encoder).unwrap();
        let len = encoder.into_writer().bytes_written();
        let mut restored = CuControllerTask::<2>::default();
        restored
            .thaw(&mut bincode::de::DecoderImpl::new(
                bincode::de::read::SliceReader::new(&bytes[..len]),
                bincode::config::standard(),
                (),
            ))
            .unwrap();
        assert_eq!(restored.last_command[0].si_value(), 0.4);
        assert_eq!(restored.last_status, CuControllerStatus::Running);
    }
}
