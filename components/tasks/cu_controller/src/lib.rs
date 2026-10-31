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
//! - Tracking error above `tracking_error_limit`: `Fault`, command
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
use cu_kinematics::CuJointStatePayload;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuRobotStatus;
use cu_trajectory::CuFollowStatus;
use cu_trajectory::CuTrajectoryFollowOutput;
use cu29::prelude::*;
use cu29::units::si::f64::{Force, Time, Torque};
use cu29::units::si::force::newton;
use cu29::units::si::time::second;
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
    /// Returns the effort in its joint's SI unit (N m or N; `0.0` for fixed).
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

/// Effort-mode PD gains.
///
/// Both gains are dimensionless SI multipliers applied per joint kind:
/// `kp` is N m/rad (revolute) or N/m (prismatic), `kv` is N m s/rad
/// (revolute) or N s/m (prismatic). Both must be finite and `>= 0`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuEffortGains {
    /// Proportional gain (dimensionless SI, see type docs).
    pub kp: f64,
    /// Derivative gain (dimensionless SI, see type docs).
    pub kv: f64,
}

impl Default for CuEffortGains {
    /// `kp = 20.0`, `kv = 0.8`.
    fn default() -> Self {
        Self { kp: 20.0, kv: 0.8 }
    }
}

impl CuEffortGains {
    /// Checks that both gains are finite and non-negative.
    pub fn validate(&self) -> CuResult<()> {
        if !self.kp.is_finite() || self.kp < 0.0 || !self.kv.is_finite() || self.kv < 0.0 {
            return Err(CuError::from(
                "cu-controller: effort gains 'kp'/'kv' must be finite and >= 0",
            ));
        }
        Ok(())
    }
}

/// Per-joint command rate limit (position mode only).
///
/// Both limits are scalars applied to every joint in that joint's SI unit
/// (dimensionless SI magnitude): `max_speed` is rad/s for revolute and m/s
/// for prismatic joints, `max_acceleration` is rad/s² resp. m/s². Both must
/// be positive and finite.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(deny_unknown_fields)]
pub struct CuRateLimit {
    /// Max joint speed (dimensionless SI magnitude, see type docs).
    pub max_speed: f64,
    /// Max joint acceleration (dimensionless SI magnitude, see type docs).
    pub max_acceleration: f64,
}

impl CuRateLimit {
    /// Checks that both limits are positive and finite.
    pub fn validate(&self) -> CuResult<()> {
        let ok = |v: f64| v.is_finite() && v > 0.0;
        if !ok(self.max_speed) || !ok(self.max_acceleration) {
            return Err(CuError::from(
                "cu-controller: rate limit 'max_speed'/'max_acceleration' must be positive and finite",
            ));
        }
        Ok(())
    }
}

/// Parameters of [`CuControllerTask`]; the node `config:` block.
///
/// ```ron
/// config: {
///     "mode": "position",            // position | velocity | effort
///     "tracking_error_limit": 0.5,   // SI per joint kind (rad / m)
///     "state_timeout": 0.2,          // s; omit to disable the watchdog
///     "rate_limit": {"max_speed": 1.5, "max_acceleration": 6.0},
///     "effort_gains": {"kp": 20.0, "kv": 0.8},
/// }
/// ```
///
/// ```rust
/// use cu_controller::{CuControllerParams, CuControllerTask, CuRateLimit};
///
/// let params = CuControllerParams {
///     rate_limit: Some(CuRateLimit { max_speed: 1.5, max_acceleration: 6.0 }),
///     ..CuControllerParams::default()
/// };
/// let task = CuControllerTask::<6>::from_params(params).unwrap();
/// assert_eq!(task.params().tracking_error_limit, 0.5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuControllerParams {
    /// Command level: `position`, `velocity`, or `effort`.
    pub mode: CuControllerMode,
    /// Max allowed absolute joint error (setpoint − measured) before the
    /// controller faults and holds the measured state. Dimensionless SI
    /// magnitude compared per joint in that joint's unit: radians for
    /// revolute, meters for prismatic joints. Positive and finite.
    pub tracking_error_limit: f64,
    /// State watchdog: a joint state whose `Tov` is older than this (or in
    /// the future, or missing a time) faults the controller. `None`
    /// disables the watchdog.
    pub state_timeout: Option<Time>,
    /// Command rate limit; requires `mode: position`. `None` disables it.
    pub rate_limit: Option<CuRateLimit>,
    /// Effort-mode PD gains. In effort mode, hold/fault paths apply a
    /// damping brake (`-kv * measured velocity`); with no measured state
    /// the effort is zero and the bridge's gravity compensation is all that
    /// holds the arm.
    pub effort_gains: CuEffortGains,
}

impl Default for CuControllerParams {
    /// Position mode, 0.5 tracking-error limit, no watchdog, no rate limit,
    /// effort gains `kp = 20.0`, `kv = 0.8`.
    fn default() -> Self {
        Self {
            mode: CuControllerMode::Position,
            tracking_error_limit: 0.5,
            state_timeout: None,
            rate_limit: None,
            effort_gains: CuEffortGains::default(),
        }
    }
}

impl CuControllerParams {
    /// Checks the limit, watchdog, rate limit (position mode only) and gains.
    pub fn validate(&self) -> CuResult<()> {
        if !self.tracking_error_limit.is_finite() || self.tracking_error_limit <= 0.0 {
            return Err(CuError::from(
                "cu-controller: 'tracking_error_limit' must be positive and finite",
            ));
        }
        if let Some(timeout) = self.state_timeout {
            let s = timeout.get::<second>();
            if !s.is_finite() || s <= 0.0 {
                return Err(CuError::from(
                    "cu-controller: 'state_timeout' must be positive and finite",
                ));
            }
        }
        if let Some(rate_limit) = self.rate_limit {
            rate_limit.validate()?;
            if self.mode != CuControllerMode::Position {
                return Err(CuError::from(
                    "cu-controller: 'rate_limit' requires 'mode: position'",
                ));
            }
        }
        self.effort_gains.validate()
    }
}

/// Converts a positive time to nanoseconds, rounded to nearest and at least
/// 1 ns (`as` saturates beyond `u64::MAX`).
fn to_nanos(t: Time) -> u64 {
    ((t.get::<second>() * 1e9 + 0.5) as u64).max(1)
}

/// Unified robot-agnostic joint controller task.
///
/// Inputs are `(setpoint, state)`: a `CuTrajectoryFollowOutput<N>` and a
/// `cu_kinematics::CuJointStatePayload<N>`. Output is a
/// [`CuJointCommandPayload<N>`] addressed to any bridge exposing a
/// `joint_command` channel. The node `config:` block is a
/// [`CuControllerParams`].
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuControllerTask<const N: usize> {
    #[reflect(ignore)]
    params: CuControllerParams,
    #[reflect(ignore)]
    state_timeout_ns: Option<u64>,
    #[reflect(ignore)]
    last_command: [CuJointPosition; N],
    #[reflect(ignore)]
    last_status: CuControllerStatus,
    has_command: bool,
    last_time: Option<CuTime>,
    #[reflect(ignore)]
    last_velocity: [CuJointVelocity; N],
}

impl<const N: usize> CuControllerTask<N> {
    /// Creates a controller from validated params.
    pub fn from_params(params: CuControllerParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self::with_valid_params(params))
    }

    fn with_valid_params(params: CuControllerParams) -> Self {
        Self {
            state_timeout_ns: params.state_timeout.map(to_nanos),
            params,
            last_command: [CuJointPosition::default(); N],
            last_status: CuControllerStatus::Idle,
            has_command: false,
            last_time: None,
            last_velocity: [CuJointVelocity::default(); N],
        }
    }

    /// Params this task runs with (including `set_mode`/`set_effort_gains`
    /// changes).
    pub fn params(&self) -> &CuControllerParams {
        &self.params
    }

    /// Sets the command level (takes effect on the next cycle).
    pub fn set_mode(&mut self, mode: CuControllerMode) {
        self.params.mode = mode;
    }

    /// Returns the active command level.
    pub fn mode(&self) -> CuControllerMode {
        self.params.mode
    }

    /// Sets the effort-mode PD gains. Invalid gains (non-finite or negative)
    /// are rejected and leave the current gains unchanged.
    pub fn set_effort_gains(&mut self, gains: CuEffortGains) -> CuResult<()> {
        gains.validate()?;
        self.params.effort_gains = gains;
        Ok(())
    }
}

impl<const N: usize> Default for CuControllerTask<N> {
    fn default() -> Self {
        Self::with_valid_params(CuControllerParams::default())
    }
}

impl<const N: usize> Freezable for CuControllerTask<N> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let p = &self.params;
        Encode::encode(&p.tracking_error_limit, encoder)?;
        Encode::encode(&p.mode, encoder)?;
        Encode::encode(&p.effort_gains.kp, encoder)?;
        Encode::encode(&p.effort_gains.kv, encoder)?;
        Encode::encode(&self.last_command, encoder)?;
        Encode::encode(&self.last_status, encoder)?;
        Encode::encode(&self.has_command, encoder)?;
        Encode::encode(&self.state_timeout_ns, encoder)?;
        Encode::encode(&p.state_timeout.map(|t| t.get::<second>()), encoder)?;
        Encode::encode(&p.rate_limit.map(|r| r.max_speed), encoder)?;
        Encode::encode(&p.rate_limit.map(|r| r.max_acceleration), encoder)?;
        Encode::encode(&self.last_time, encoder)?;
        Encode::encode(&self.last_velocity, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.params.tracking_error_limit = Decode::decode(decoder)?;
        self.params.mode = Decode::decode(decoder)?;
        self.params.effort_gains.kp = Decode::decode(decoder)?;
        self.params.effort_gains.kv = Decode::decode(decoder)?;
        self.last_command = Decode::decode(decoder)?;
        self.last_status = Decode::decode(decoder)?;
        self.has_command = Decode::decode(decoder)?;
        self.state_timeout_ns = Decode::decode(decoder)?;
        let timeout: Option<f64> = Decode::decode(decoder)?;
        self.params.state_timeout = timeout.map(Time::new::<second>);
        let speed: Option<f64> = Decode::decode(decoder)?;
        let acceleration: Option<f64> = Decode::decode(decoder)?;
        self.params.rate_limit = speed
            .zip(acceleration)
            .map(|(max_speed, max_acceleration)| CuRateLimit {
                max_speed,
                max_acceleration,
            });
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
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuControllerParams>)
            .transpose()?
            .unwrap_or_default();
        Self::from_params(params)
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

        let mode = self.params.mode;
        let CuEffortGains { kp, kv } = self.params.effort_gains;
        let mut command = CuJointCommandPayload::<N> {
            mode,
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
                if mode == CuControllerMode::Effort {
                    for j in 0..N {
                        command.efforts[j] = CuJointEffort::from_position(
                            state.positions[j],
                            -kv * state.velocities[j].si_value(),
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
                    || max_err > self.params.tracking_error_limit
                {
                    command.positions = state.positions;
                    command.status = CuControllerStatus::Fault;
                    if mode == CuControllerMode::Effort {
                        // Damping brake: no stiffness towards the setpoint,
                        // but resist motion instead of going limp.
                        for j in 0..N {
                            command.efforts[j] = CuJointEffort::from_position(
                                state.positions[j],
                                -kv * state.velocities[j].si_value(),
                            );
                        }
                    }
                } else {
                    command.positions = setpoint.positions;
                    command.velocities = setpoint.velocities;
                    if mode == CuControllerMode::Effort {
                        for j in 0..N {
                            let pos_err =
                                setpoint.positions[j].si_value() - state.positions[j].si_value();
                            let vel_err =
                                setpoint.velocities[j].si_value() - state.velocities[j].si_value();
                            let effort = kp * pos_err + kv * vel_err;
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
        if let Some(CuRateLimit {
            max_speed,
            max_acceleration,
        }) = self.params.rate_limit
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

    fn with_limit(limit: f64) -> CuControllerTask<2> {
        CuControllerTask::from_params(CuControllerParams {
            tracking_error_limit: limit,
            ..CuControllerParams::default()
        })
        .unwrap()
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
        let mut task = with_limit(0.1);
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
        let mut task = with_limit(0.5);
        task.set_mode(CuControllerMode::Effort);
        task.set_effort_gains(CuEffortGains { kp: 10.0, kv: 2.0 })
            .unwrap();
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
        let mut task = with_limit(0.1);
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
        let mut task = with_limit(0.5);
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
        let mut task = with_limit(0.25);
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

    #[test]
    fn test_invalid_params_rejected() {
        let bad = |p: CuControllerParams| CuControllerTask::<2>::from_params(p).is_err();
        let d = CuControllerParams::default();
        assert!(bad(CuControllerParams {
            tracking_error_limit: 0.0,
            ..d
        }));
        assert!(bad(CuControllerParams {
            tracking_error_limit: f64::NAN,
            ..d
        }));
        assert!(bad(CuControllerParams {
            state_timeout: Some(Time::new::<second>(0.0)),
            ..d
        }));
        let rate = CuRateLimit {
            max_speed: 1.0,
            max_acceleration: 2.0,
        };
        assert!(bad(CuControllerParams {
            rate_limit: Some(CuRateLimit {
                max_speed: -1.0,
                ..rate
            }),
            ..d
        }));
        assert!(bad(CuControllerParams {
            rate_limit: Some(CuRateLimit {
                max_acceleration: f64::INFINITY,
                ..rate
            }),
            ..d
        }));
        assert!(bad(CuControllerParams {
            mode: CuControllerMode::Velocity,
            rate_limit: Some(rate),
            ..d
        }));
        assert!(bad(CuControllerParams {
            effort_gains: CuEffortGains { kp: -1.0, kv: 0.0 },
            ..d
        }));
        assert!(!bad(CuControllerParams {
            rate_limit: Some(rate),
            ..d
        }));
    }

    #[test]
    fn test_set_effort_gains_rejects_invalid() {
        let mut task = CuControllerTask::<2>::default();
        assert!(
            task.set_effort_gains(CuEffortGains {
                kp: f64::NAN,
                kv: 1.0
            })
            .is_err()
        );
        assert_eq!(task.params().effort_gains, CuEffortGains::default());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_reads_config() {
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"mode": "position", "tracking_error_limit": 0.25,
            "state_timeout": 0.2, "rate_limit": {"max_speed": 1.5, "max_acceleration": 6.0},
            "effort_gains": {"kp": 10.0, "kv": 2.0}}"#,
        )
        .unwrap();
        let task = CuControllerTask::<2>::new(Some(&cfg), ()).unwrap();
        let p = task.params();
        assert_eq!(p.tracking_error_limit, 0.25);
        assert_eq!(p.state_timeout, Some(Time::new::<second>(0.2)));
        assert_eq!(task.state_timeout_ns, Some(200_000_000));
        assert_eq!(
            p.rate_limit,
            Some(CuRateLimit {
                max_speed: 1.5,
                max_acceleration: 6.0
            })
        );
        assert_eq!(p.effort_gains, CuEffortGains { kp: 10.0, kv: 2.0 });
        let task = CuControllerTask::<2>::new(None, ()).unwrap();
        assert_eq!(task.params(), &CuControllerParams::default());
        for bad in [
            r#"{"tracking_error_limit_rad": 0.5}"#,
            r#"{"rate_limit": {"max_speed": 1.5}}"#,
            r#"{"mode": "effort", "rate_limit": {"max_speed": 1.5, "max_acceleration": 6.0}}"#,
        ] {
            let cfg: ComponentConfig =
                ron::from_str(&format!("#![enable(unwrap_newtypes)] {bad}")).unwrap();
            assert!(CuControllerTask::<2>::new(Some(&cfg), ()).is_err(), "{bad}");
        }
    }
}
