//! Robot-agnostic digital IO for `copper-rs`, and a gripper built on it.
//!
//! A bridge that exposes digital IO carries exactly two payloads:
//! - [`CuDigitalInputPayload<N>`] on its `digital_in` channel (read values
//!   plus the IO device status),
//! - [`CuDigitalOutputPayload<N>`] on its `digital_out` channel (values plus
//!   a mask of the outputs this command drives; unmasked outputs keep their
//!   current level).
//!
//! End-effector hardware wired to that IO needs no payload of its own.
//! [`CuGripperTask`] maps open/close commands onto configured output bits
//! and reads optional sensor inputs back into a [`CuGripperState`], so a
//! pneumatic or electric gripper on any robot is a RON configuration, not new
//! code.

#![cfg_attr(not(feature = "std"), no_std)]

use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;

mod serde_arrays;

/// Health reported by the IO device behind a bridge.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuIoStatus {
    /// The bridge has not confirmed the IO device yet.
    #[default]
    NotReady,
    /// Input values are valid and outputs are applied.
    Ready,
    /// The IO device reports a device-specific fault code.
    Fault(u32),
}

/// Digital input levels read by a bridge (`digital_in` channel).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuDigitalInputPayload<const N: usize> {
    pub status: CuIoStatus,
    /// Level of each input; `true` is the active (energized) level.
    #[serde(with = "crate::serde_arrays::bool_array")]
    pub values: [bool; N],
}

impl<const N: usize> Default for CuDigitalInputPayload<N> {
    fn default() -> Self {
        Self {
            status: CuIoStatus::NotReady,
            values: [false; N],
        }
    }
}

/// Digital output command for a bridge (`digital_out` channel).
///
/// Only outputs whose `mask` bit is set are driven to `values`; the bridge
/// leaves the others at their current level, so a command never touches IO
/// it does not own.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuDigitalOutputPayload<const N: usize> {
    #[serde(with = "crate::serde_arrays::bool_array")]
    pub values: [bool; N],
    #[serde(with = "crate::serde_arrays::bool_array")]
    pub mask: [bool; N],
}

impl<const N: usize> Default for CuDigitalOutputPayload<N> {
    fn default() -> Self {
        Self {
            values: [false; N],
            mask: [false; N],
        }
    }
}

/// Requested gripper action.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuGripperAction {
    #[default]
    Open,
    Close,
}

/// Gripper command; an empty message keeps the last action.
#[derive(Default, Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuGripperCommandPayload {
    pub action: CuGripperAction,
}

/// Gripper state derived from the commanded action and its sensors.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuGripperState {
    /// No action has been commanded yet; outputs are not driven.
    #[default]
    Unknown,
    Opening,
    Open,
    Closing,
    Closed,
    /// IO not ready or faulted, contradictory sensors, or the motion did not
    /// reach its sensor in time. Outputs hold their last commanded levels.
    Fault,
}

/// Gripper state reported every cycle.
#[derive(Default, Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuGripperStatePayload {
    pub state: CuGripperState,
}

/// Parameters of a gripper wired to digital IO; this is the gripper task's
/// node `config:` block.
///
/// `close_output` is required. `open_output` is set for double-acting
/// actuators (two valves or an explicit open line); otherwise opening means
/// releasing `close_output`. Without a sensor for the target position the
/// motion counts as done after `settle`; with one it must assert within
/// `motion_timeout`. Indices count from 0 and must address the task's `IN`
/// inputs / `OUT` outputs.
///
/// RON (times in seconds):
///
/// ```ron
/// config: {
///     "close_output": 0,
///     "open_output": 1,      // optional
///     "closed_input": 0,     // optional
///     "open_input": 1,       // optional
///     "settle": 0.3,         // default 0.3 s
///     "motion_timeout": 2.0, // default 2 s
/// }
/// ```
///
/// ```
/// use cu_io::{CuGripperParams, CuGripperTask};
/// use cu29::units::si::f64::Time;
/// use cu29::units::si::time::second;
///
/// let params = CuGripperParams {
///     open_output: Some(1),
///     motion_timeout: Time::new::<second>(1.0),
///     ..CuGripperParams::single_acting(0)
/// };
/// let task = CuGripperTask::<2, 2>::from_params(params).unwrap();
/// # let _ = task;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuGripperParams {
    /// Output that closes the gripper (required).
    pub close_output: Option<usize>,
    /// Output that opens the gripper (double-acting actuators).
    pub open_output: Option<usize>,
    /// Input asserted when the gripper is closed.
    pub closed_input: Option<usize>,
    /// Input asserted when the gripper is open.
    pub open_input: Option<usize>,
    /// Time after which a motion without a target sensor counts as done.
    pub settle: Time,
    /// Time within which a target sensor must assert.
    pub motion_timeout: Time,
}

impl Default for CuGripperParams {
    /// No close output (rejected by [`validate`](Self::validate)), 0.3 s
    /// settle, 2 s sensor timeout.
    fn default() -> Self {
        Self {
            close_output: None,
            open_output: None,
            closed_input: None,
            open_input: None,
            settle: Time::new::<second>(0.3),
            motion_timeout: Time::new::<second>(2.0),
        }
    }
}

impl CuGripperParams {
    /// Params with only a close output and default timings (0.3 s settle,
    /// 2 s sensor timeout).
    pub fn single_acting(close_output: usize) -> Self {
        Self {
            close_output: Some(close_output),
            ..Self::default()
        }
    }

    /// Checks that `close_output` is set, indices fit `IN` inputs / `OUT`
    /// outputs and are not shared, and timings are positive.
    pub fn validate<const IN: usize, const OUT: usize>(&self) -> CuResult<()> {
        let close_output = self
            .close_output
            .ok_or_else(|| CuError::from("cu-io: gripper needs 'close_output'"))?;
        let output_ok = |i: usize| i < OUT;
        let input_ok = |i: usize| i < IN;
        let positive = |t: Time| t.get::<second>() > 0.0;
        if !output_ok(close_output)
            || self
                .open_output
                .is_some_and(|o| !output_ok(o) || o == close_output)
            || self.closed_input.is_some_and(|i| !input_ok(i))
            || self.open_input.is_some_and(|i| !input_ok(i))
            || self
                .closed_input
                .is_some_and(|i| self.open_input == Some(i))
            || !positive(self.settle)
            || !positive(self.motion_timeout)
        {
            return Err(CuError::from(
                "cu-io: gripper indices out of range, shared, or non-positive timings",
            ));
        }
        Ok(())
    }
}

/// Gripper driven through digital IO.
///
/// Inputs are `(command, digital_in)`; outputs are `(digital_out, state)`.
/// Only the configured output bits are masked in the output command, so the
/// task can share a bridge's `digital_out` with other IO users.
///
/// The node `config:` block is a [`CuGripperParams`].
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuGripperTask<const IN: usize, const OUT: usize> {
    #[reflect(ignore)]
    params: CuGripperParams,
    #[reflect(ignore)]
    close_output: usize,
    #[reflect(ignore)]
    settle_ns: u64,
    #[reflect(ignore)]
    motion_timeout_ns: u64,
    #[reflect(ignore)]
    action: Option<CuGripperAction>,
    #[reflect(ignore)]
    state: CuGripperState,
    motion_start: Option<CuTime>,
}

/// Converts a positive time to nanoseconds, rounded to nearest and at
/// least 1 ns.
fn to_nanos(t: Time) -> u64 {
    // `as` saturates for values beyond `u64::MAX`.
    ((t.get::<second>() * 1e9 + 0.5) as u64).max(1)
}

impl<const IN: usize, const OUT: usize> CuGripperTask<IN, OUT> {
    /// Creates a gripper task from validated params.
    pub fn from_params(params: CuGripperParams) -> CuResult<Self> {
        params.validate::<IN, OUT>()?;
        Ok(Self {
            close_output: params.close_output.unwrap_or_default(),
            settle_ns: to_nanos(params.settle),
            motion_timeout_ns: to_nanos(params.motion_timeout),
            params,
            action: None,
            state: CuGripperState::Unknown,
            motion_start: None,
        })
    }

    /// Params this task was built from.
    pub fn params(&self) -> &CuGripperParams {
        &self.params
    }

    /// Current gripper state.
    pub fn state(&self) -> CuGripperState {
        self.state
    }

    fn sensor(&self, input: Option<usize>, values: &[bool; IN]) -> Option<bool> {
        input.map(|i| values[i])
    }
}

impl<const IN: usize, const OUT: usize> Freezable for CuGripperTask<IN, OUT> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.action, encoder)?;
        Encode::encode(&self.state, encoder)?;
        Encode::encode(&self.motion_start, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.action = Decode::decode(decoder)?;
        self.state = Decode::decode(decoder)?;
        self.motion_start = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<const IN: usize, const OUT: usize> CuTask for CuGripperTask<IN, OUT> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuGripperCommandPayload, CuDigitalInputPayload<IN>);
    type Output<'m> = output_msg!(CuDigitalOutputPayload<OUT>, CuGripperStatePayload);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuGripperParams>)
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
        let (command, io) = *input;
        let (digital_out, state_out) = output;
        digital_out.tov = io.tov;
        state_out.tov = io.tov;
        let now = ctx.now();

        // A new action restarts the motion; repeating the active one does not.
        if let Some(command) = command.payload()
            && self.action != Some(command.action)
        {
            self.action = Some(command.action);
            self.motion_start = Some(now);
            self.state = match command.action {
                CuGripperAction::Open => CuGripperState::Opening,
                CuGripperAction::Close => CuGripperState::Closing,
            };
        }

        let io_ready = io.payload().filter(|io| io.status == CuIoStatus::Ready);
        self.state = match (self.action, io_ready) {
            (None, _) => CuGripperState::Unknown,
            (Some(_), None) => CuGripperState::Fault,
            // A fault latches until a new action is commanded.
            (Some(_), Some(_)) if self.state == CuGripperState::Fault => CuGripperState::Fault,
            (Some(action), Some(io)) => {
                let closed = self.sensor(self.params.closed_input, &io.values);
                let open = self.sensor(self.params.open_input, &io.values);
                let (reached, other, done, moving) = match action {
                    CuGripperAction::Close => (
                        closed,
                        open,
                        CuGripperState::Closed,
                        CuGripperState::Closing,
                    ),
                    CuGripperAction::Open => {
                        (open, closed, CuGripperState::Open, CuGripperState::Opening)
                    }
                };
                let elapsed = self
                    .motion_start
                    .map_or(0, |start| now.as_nanos().saturating_sub(start.as_nanos()));
                if closed == Some(true) && open == Some(true) {
                    CuGripperState::Fault
                } else {
                    match reached {
                        Some(true) => done,
                        Some(false) if elapsed >= self.motion_timeout_ns => CuGripperState::Fault,
                        Some(false) => moving,
                        // No target sensor: trust the settle time, but an
                        // opposite sensor still asserted after it is a jam.
                        None if elapsed >= self.settle_ns => {
                            if other == Some(true) {
                                CuGripperState::Fault
                            } else {
                                done
                            }
                        }
                        None => moving,
                    }
                }
            }
        };

        match self.action {
            None => digital_out.clear_payload(),
            Some(action) => {
                let close = action == CuGripperAction::Close;
                let mut command = CuDigitalOutputPayload::<OUT>::default();
                command.values[self.close_output] = close;
                command.mask[self.close_output] = true;
                if let Some(open_output) = self.params.open_output {
                    command.values[open_output] = !close;
                    command.mask[open_output] = true;
                }
                digital_out.set_payload(command);
            }
        }
        state_out.set_payload(CuGripperStatePayload { state: self.state });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> CuGripperParams {
        CuGripperParams {
            open_output: Some(1),
            closed_input: Some(0),
            ..CuGripperParams::single_acting(0)
        }
    }

    fn step(
        task: &mut CuGripperTask<2, 2>,
        ctx: &CuContext,
        action: Option<CuGripperAction>,
        io: Option<CuDigitalInputPayload<2>>,
    ) -> (Option<CuDigitalOutputPayload<2>>, CuGripperState) {
        let command = CuMsg::new(action.map(|action| CuGripperCommandPayload { action }));
        let io = CuMsg::new(io);
        let input = (&command, &io);
        let mut output = (
            CuMsg::<CuDigitalOutputPayload<2>>::default(),
            CuMsg::<CuGripperStatePayload>::default(),
        );
        task.process(ctx, &input, &mut output).unwrap();
        (
            output.0.payload().cloned(),
            output.1.payload().unwrap().state,
        )
    }

    fn ready(closed: bool) -> Option<CuDigitalInputPayload<2>> {
        Some(CuDigitalInputPayload {
            status: CuIoStatus::Ready,
            values: [closed, false],
        })
    }

    #[test]
    fn close_drives_masked_outputs_until_sensor_confirms() {
        let mut task = CuGripperTask::<2, 2>::from_params(params()).unwrap();
        let (ctx, _clock) = CuContext::new_mock_clock();
        let (out, state) = step(&mut task, &ctx, None, ready(false));
        assert!(out.is_none());
        assert_eq!(state, CuGripperState::Unknown);
        let (out, state) = step(&mut task, &ctx, Some(CuGripperAction::Close), ready(false));
        let out = out.unwrap();
        assert_eq!(out.values, [true, false]);
        assert_eq!(out.mask, [true, true]);
        assert_eq!(state, CuGripperState::Closing);
        let (_, state) = step(&mut task, &ctx, None, ready(true));
        assert_eq!(state, CuGripperState::Closed);
    }

    #[test]
    fn missing_sensor_times_out_and_io_fault_latches() {
        let mut task = CuGripperTask::<2, 2>::from_params(CuGripperParams {
            motion_timeout: Time::new::<second>(1e-6),
            ..params()
        })
        .unwrap();
        let (ctx, clock) = CuContext::new_mock_clock();
        step(&mut task, &ctx, Some(CuGripperAction::Close), ready(false));
        clock.increment(CuDuration::from_nanos(2_000));
        let (_, state) = step(&mut task, &ctx, None, ready(false));
        assert_eq!(state, CuGripperState::Fault);
        let (out, state) = step(&mut task, &ctx, None, ready(true));
        assert_eq!(state, CuGripperState::Fault);
        assert_eq!(out.unwrap().values, [true, false]);
        let (_, state) = step(&mut task, &ctx, Some(CuGripperAction::Open), None);
        assert_eq!(state, CuGripperState::Fault);
    }

    #[test]
    fn rejects_out_of_range_wiring() {
        assert!(CuGripperTask::<2, 2>::from_params(CuGripperParams::single_acting(2)).is_err());
        assert!(
            CuGripperTask::<2, 2>::from_params(CuGripperParams {
                open_output: Some(0),
                ..CuGripperParams::single_acting(0)
            })
            .is_err()
        );
    }

    #[test]
    fn missing_close_output_is_rejected() {
        assert!(CuGripperTask::<2, 2>::new(None, ()).is_err());
        assert!(CuGripperParams::default().validate::<2, 2>().is_err());
        assert!(
            CuGripperTask::<2, 2>::from_params(CuGripperParams {
                settle: Time::new::<second>(0.0),
                ..CuGripperParams::single_acting(0)
            })
            .is_err()
        );
    }

    #[test]
    fn default_timings_match_previous_nanoseconds() {
        let task = CuGripperTask::<2, 2>::from_params(params()).unwrap();
        assert_eq!(task.settle_ns, 300_000_000);
        assert_eq!(task.motion_timeout_ns, 2_000_000_000);
    }
}
