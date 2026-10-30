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
use cu29::units::si::f64::{Force, Torque};
use cu29::units::si::force::newton;
use cu29::units::si::torque::newton_meter;
use serde::Deserialize;
use serde::Serialize;

mod serde_arrays;

/// Command level a bridge is asked to apply.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuControllerMode {
    /// Position setpoints; the bridge closes the servo loop (with velocity
    /// feedforward from the command velocities).
    #[default]
    Position,
    /// Velocity setpoints; the bridge servo drives the commanded joint
    /// velocities (positions ride along as a drift reference).
    Velocity,
    /// Effort setpoints computed by the controller's own PD law; the bridge
    /// applies them as raw torque/force (plus its own gravity compensation).
    Effort,
}

/// A joint effort carries the physical unit of its joint (torque or force).
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuJointEffort {
    Revolute(Torque),
    Prismatic(Force),
    Fixed,
}

impl Default for CuJointEffort {
    fn default() -> Self {
        Self::Revolute(Torque::default())
    }
}

impl CuJointEffort {
    /// Builds an effort matching the kind of the given joint coordinate.
    pub fn from_position(position: CuJointPosition, value: f64) -> Self {
        match position {
            CuJointPosition::Revolute(_) => Self::Revolute(Torque::new::<newton_meter>(value)),
            CuJointPosition::Prismatic(_) => Self::Prismatic(Force::new::<newton>(value)),
            CuJointPosition::Fixed => Self::Fixed,
        }
    }
    #[doc(hidden)]
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<newton_meter>(),
            Self::Prismatic(v) => v.get::<newton>(),
            Self::Fixed => 0.0,
        }
    }
}

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
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointStatePayload<const N: usize> {
    /// Current health reported by the bridge.
    pub status: CuRobotStatus,
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
            status: CuRobotStatus::NotReady,
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
    /// Command level the bridge is asked to apply.
    pub mode: CuControllerMode,
    /// Effort setpoints (only meaningful when `mode` is `Effort`).
    #[serde(with = "crate::serde_arrays::effort_array")]
    pub efforts: [CuJointEffort; N],
}

impl<const N: usize> Default for CuJointCommandPayload<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
            status: CuControllerStatus::default(),
            max_tracking_error: CuJointPosition::default(),
            mode: CuControllerMode::default(),
            efforts: [CuJointEffort::default(); N],
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
/// - `mode` (`CuControllerMode`, default `position`): command level —
///   `position`, `velocity`, or `effort`.
/// - `effort_kp` / `effort_kv` (`f64`, defaults `20.0` / `0.8`): PD gains
///   for effort mode, per joint kind (N m/rad resp. N/m proportional).
///   In effort mode, hold/fault paths apply a damping brake
///   (`-effort_kv * measured velocity`); with no measured state the effort
///   is zero and the bridge's gravity compensation is all that holds the arm.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuControllerTask<const N: usize> {
    tracking_error_limit_rad: f64,
    mode: CuControllerMode,
    effort_kp: f64,
    effort_kv: f64,
    #[reflect(ignore)]
    last_command: [CuJointPosition; N],
    #[reflect(ignore)]
    last_status: CuControllerStatus,
    has_command: bool,
    state_timeout_ns: Option<u64>,
    max_speed: Option<f64>,
    max_acceleration: Option<f64>,
    last_time: Option<CuTime>,
    #[reflect(ignore)]
    last_velocity: [CuJointVelocity; N],
}

impl<const N: usize> CuControllerTask<N> {
    /// Creates a controller with an explicit tracking-error limit (radians).
    pub fn new_with_limit(tracking_error_limit_rad: f64) -> Self {
        Self {
            tracking_error_limit_rad,
            mode: CuControllerMode::default(),
            effort_kp: 20.0,
            effort_kv: 0.8,
            last_command: [CuJointPosition::default(); N],
            last_status: CuControllerStatus::Idle,
            has_command: false,
            state_timeout_ns: None,
            max_speed: None,
            max_acceleration: None,
            last_time: None,
            last_velocity: [CuJointVelocity::default(); N],
        }
    }

    /// Sets the command level (takes effect on the next cycle).
    pub fn set_mode(&mut self, mode: CuControllerMode) {
        self.mode = mode;
    }

    /// Returns the active command level.
    pub fn mode(&self) -> CuControllerMode {
        self.mode
    }

    /// Sets the effort-mode PD gains (proportional, derivative). Units are
    /// per joint kind: N m/rad and N m s/rad for revolute joints, N/m and
    /// N s/m for prismatic joints. Non-finite or negative values are ignored.
    pub fn set_effort_gains(&mut self, kp: f64, kv: f64) {
        if kp.is_finite() && kp >= 0.0 && kv.is_finite() && kv >= 0.0 {
            self.effort_kp = kp;
            self.effort_kv = kv;
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
        Encode::encode(&self.mode, encoder)?;
        Encode::encode(&self.effort_kp, encoder)?;
        Encode::encode(&self.effort_kv, encoder)?;
        Encode::encode(&self.last_command, encoder)?;
        Encode::encode(&self.last_status, encoder)?;
        Encode::encode(&self.has_command, encoder)?;
        Encode::encode(&self.state_timeout_ns, encoder)?;
        Encode::encode(&self.max_speed, encoder)?;
        Encode::encode(&self.max_acceleration, encoder)?;
        Encode::encode(&self.last_time, encoder)?;
        Encode::encode(&self.last_velocity, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.tracking_error_limit_rad = Decode::decode(decoder)?;
        self.mode = Decode::decode(decoder)?;
        self.effort_kp = Decode::decode(decoder)?;
        self.effort_kv = Decode::decode(decoder)?;
        self.last_command = Decode::decode(decoder)?;
        self.last_status = Decode::decode(decoder)?;
        self.has_command = Decode::decode(decoder)?;
        self.state_timeout_ns = Decode::decode(decoder)?;
        self.max_speed = Decode::decode(decoder)?;
        self.max_acceleration = Decode::decode(decoder)?;
        self.last_time = Decode::decode(decoder)?;
        self.last_velocity = Decode::decode(decoder)?;
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
        let mut task = Self::new_with_limit(limit);
        if let Some(cfg) = cfg {
            let timeout = cfg.get::<u64>("state_timeout_ns")?;
            let speed = cfg.get::<f64>("max_joint_speed_si")?;
            let acceleration = cfg.get::<f64>("max_joint_acceleration_si")?;
            if timeout == Some(0)
                || speed.is_some_and(|v| !v.is_finite() || v <= 0.0)
                || acceleration.is_some_and(|v| !v.is_finite() || v <= 0.0)
                || speed.is_some() != acceleration.is_some()
            {
                return Err(CuError::from(
                    "cu-controller: invalid watchdog or rate limits",
                ));
            }
            task.state_timeout_ns = timeout;
            task.max_speed = speed;
            task.max_acceleration = acceleration;
            let kp = cfg.get::<f64>("effort_kp")?;
            let kv = cfg.get::<f64>("effort_kv")?;
            if kp.is_some_and(|v| !v.is_finite() || v < 0.0)
                || kv.is_some_and(|v| !v.is_finite() || v < 0.0)
            {
                return Err(CuError::from(
                    "cu-controller: 'effort_kp'/'effort_kv' must be finite and >= 0",
                ));
            }
            task.mode = cfg
                .get_value::<CuControllerMode>("mode")?
                .unwrap_or_default();
            if task.max_speed.is_some() && task.mode != CuControllerMode::Position {
                return Err(CuError::from(
                    "cu-controller: rate limits require 'mode: position'",
                ));
            }
            if let Some(kp) = kp {
                task.effort_kp = kp;
            }
            if let Some(kv) = kv {
                task.effort_kv = kv;
            }
        }
        Ok(task)
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.1.tov;
        let state = input.1.payload();
        let setpoint = input.0.payload();

        let mut command = CuJointCommandPayload::<N> {
            mode: self.mode,
            ..Default::default()
        };
        let now = ctx.now();
        let stale = self
            .state_timeout_ns
            .is_some_and(|timeout| match input.1.tov {
                Tov::Time(t) => {
                    t.as_nanos() > now.as_nanos() || now.as_nanos() - t.as_nanos() > timeout
                }
                _ => true,
            });
        match (setpoint, state) {
            (_, _) if stale => {
                command.positions = self.last_command;
                command.status = CuControllerStatus::Fault;
            }
            (_, Some(state)) if state.status != CuRobotStatus::Ready => {
                command.positions = state.positions;
                command.status = CuControllerStatus::Fault;
            }
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
                if self.mode == CuControllerMode::Effort {
                    for j in 0..N {
                        command.efforts[j] = CuJointEffort::from_position(
                            state.positions[j],
                            -self.effort_kv * state.velocities[j].si_value(),
                        );
                    }
                }
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
                    if self.mode == CuControllerMode::Effort {
                        // Damping brake: no stiffness towards the setpoint,
                        // but resist motion instead of going limp.
                        for j in 0..N {
                            command.efforts[j] = CuJointEffort::from_position(
                                state.positions[j],
                                -self.effort_kv * state.velocities[j].si_value(),
                            );
                        }
                    }
                } else {
                    command.positions = setpoint.positions;
                    command.velocities = setpoint.velocities;
                    if self.mode == CuControllerMode::Effort {
                        for j in 0..N {
                            let pos_err =
                                setpoint.positions[j].si_value() - state.positions[j].si_value();
                            let vel_err =
                                setpoint.velocities[j].si_value() - state.velocities[j].si_value();
                            let effort = self.effort_kp * pos_err + self.effort_kv * vel_err;
                            command.efforts[j] =
                                CuJointEffort::from_position(setpoint.positions[j], effort);
                        }
                    }
                    command.status = match setpoint.status {
                        CuFollowStatus::Running => CuControllerStatus::Running,
                        CuFollowStatus::Finished => CuControllerStatus::Holding,
                        CuFollowStatus::Idle => CuControllerStatus::Idle,
                        CuFollowStatus::ErrorEmptyTrajectory => CuControllerStatus::Fault,
                    };
                }
            }
        }

        // Rate limiting applies while a setpoint is followed (running or
        // holding its end). It starts from the measured state after idle or
        // fault, so no stale command or velocity carries over.
        if let (Some(max_speed), Some(max_acceleration)) = (self.max_speed, self.max_acceleration)
            && matches!(
                command.status,
                CuControllerStatus::Running | CuControllerStatus::Holding
            )
            && let Some(measured) = state
        {
            let following = matches!(
                self.last_status,
                CuControllerStatus::Running | CuControllerStatus::Holding
            );
            if !following {
                self.last_command = measured.positions;
                self.last_velocity = measured
                    .positions
                    .map(|q| CuJointVelocity::from_coordinate(q, 0.0));
            }
            let nanos = match self.last_time {
                Some(previous) if following => now.as_nanos().saturating_sub(previous.as_nanos()),
                _ => 0,
            };
            if nanos == 0 {
                // No elapsed time (first cycle or a repeated tick): repeat the
                // last limited command.
                command.positions = self.last_command;
                command.velocities = self.last_velocity;
            } else {
                let dt = nanos as f64 * 1e-9;
                let mut settled = true;
                for j in 0..N {
                    let current = self.last_command[j].si_value();
                    let old_v = self.last_velocity[j].si_value();
                    let desired = ((command.positions[j].si_value() - current) / dt)
                        .clamp(-max_speed, max_speed);
                    let v = desired
                        .clamp(old_v - max_acceleration * dt, old_v + max_acceleration * dt)
                        .clamp(-max_speed, max_speed);
                    let target = command.positions[j].si_value();
                    command.positions[j] = self.last_command[j].with_si_value(current + v * dt);
                    command.velocities[j] =
                        CuJointVelocity::from_coordinate(command.positions[j], v);
                    settled &=
                        (command.positions[j].si_value() - target).abs() <= 1e-9 && v.abs() <= 1e-9;
                }
                // Holding is only reported once the limited command has
                // actually reached the held setpoint.
                if command.status == CuControllerStatus::Holding && !settled {
                    command.status = CuControllerStatus::Running;
                }
            }
        }
        self.last_time = Some(now);
        self.last_velocity = command.velocities;
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
            status: CuRobotStatus::Ready,
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
    fn test_effort_mode_computes_pd_effort() {
        let mut task = CuControllerTask::<2>::new_with_limit(0.5);
        task.set_mode(CuControllerMode::Effort);
        task.set_effort_gains(10.0, 2.0);
        let out = run(
            &mut task,
            Some(follow(0.5, CuFollowStatus::Running)),
            Some(state(0.0)),
        );
        assert_eq!(out.mode, CuControllerMode::Effort);
        assert_eq!(out.status, CuControllerStatus::Running);
        // tau = 10 * (0.5 - 0) + 2 * (0.1 - 0) = 5.2 N m
        assert!((out.efforts[0].si_value() - 5.2).abs() < 1e-9);
    }

    #[test]
    fn test_effort_fault_outputs_zero_effort() {
        let mut task = CuControllerTask::<2>::new_with_limit(0.1);
        task.set_mode(CuControllerMode::Effort);
        let out = run(
            &mut task,
            Some(follow(1.0, CuFollowStatus::Running)),
            Some(state(0.0)),
        );
        assert_eq!(out.status, CuControllerStatus::Fault);
        assert_eq!(out.efforts[0].si_value(), 0.0);
    }

    #[test]
    fn test_velocity_mode_tags_command() {
        let mut task = CuControllerTask::<2>::new_with_limit(0.5);
        task.set_mode(CuControllerMode::Velocity);
        let out = run(
            &mut task,
            Some(follow(0.4, CuFollowStatus::Running)),
            Some(state(0.35)),
        );
        assert_eq!(out.mode, CuControllerMode::Velocity);
        assert_eq!(out.status, CuControllerStatus::Running);
        assert_eq!(out.efforts[0].si_value(), 0.0);
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
