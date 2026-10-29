//! `CuTask` following a stored [`CuTrajectoryPayload`] with preemption.
//!
//! [`CuFollowTrajectoryTask`] adopts the latest non-empty trajectory from its
//! input, samples it at the elapsed time since adoption, and emits joint
//! positions, velocities, and a [`CuFollowStatus`]. A differing trajectory
//! preempts the running one; an empty trajectory cancels it.

use crate::CuTrajectoryPayload;
use crate::MAX_TRAJECTORY_POINTS;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;

/// Follower outcome mirroring the [`CuIkStatus`](cu_kinematics::ik::CuIkStatus) derive style.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuFollowStatus {
    /// No trajectory has been adopted yet.
    #[default]
    Idle,
    /// Sampling a stored trajectory before its end time.
    Running,
    /// Elapsed time reached the trajectory duration; the final point is held.
    Finished,
    /// The input carried an empty trajectory; any stored trajectory is cancelled.
    ErrorEmptyTrajectory,
}

/// Follower setpoint carrying sampled joints plus the follow status.
///
/// Single-output analogue of `CuIkSolutionPayload`/`CuJointVelocitiesPayload`:
/// `CuTask` supports exactly one `Output` type, so status rides with the data.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuTrajectoryFollowOutput<const N: usize> {
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
    pub status: CuFollowStatus,
}

impl<const N: usize> Default for CuTrajectoryFollowOutput<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
            status: CuFollowStatus::default(),
        }
    }
}

/// `CuTask` sampling a stored joint trajectory against the message `Tov` clock.
///
/// The trajectory clock is the input message's `Tov`: adoption records the
/// `Tov::Time` of the adopting call and each call samples at `now - start`,
/// where `now` is the current input `Tov::Time` (`ctx.now()` fallback when the
/// input carries no stamp). This mirrors the sibling tasks' `output.tov =
/// input.tov` propagation while keeping elapsed tracking on the
/// measurement-time axis rather than processing time.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuFollowTrajectoryTask<const N: usize, const MAX_POINTS: usize = MAX_TRAJECTORY_POINTS> {
    #[reflect(ignore)]
    trajectory: Option<CuTrajectoryPayload<N, MAX_POINTS>>,
    #[reflect(ignore)]
    start: CuTime,
}

impl<const N: usize, const MAX_POINTS: usize> CuFollowTrajectoryTask<N, MAX_POINTS> {
    /// Creates an idle follower holding no trajectory.
    pub const fn new() -> Self {
        Self {
            trajectory: None,
            start: CuTime::MIN,
        }
    }
}

impl<const N: usize, const MAX_POINTS: usize> Default for CuFollowTrajectoryTask<N, MAX_POINTS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const MAX_POINTS: usize> Freezable for CuFollowTrajectoryTask<N, MAX_POINTS> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.trajectory, encoder)?;
        Encode::encode(&self.start, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.trajectory = Decode::decode(decoder)?;
        self.start = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<const N: usize, const MAX_POINTS: usize> CuTask for CuFollowTrajectoryTask<N, MAX_POINTS> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuTrajectoryPayload<N, MAX_POINTS>);
    type Output<'m> = output_msg!(CuTrajectoryFollowOutput<N>);

    fn new(_cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::new())
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let now = match input.tov {
            Tov::Time(t) => t,
            _ => ctx.now(),
        };

        if let Some(traj) = input.payload() {
            if traj.is_empty() {
                self.trajectory = None;
                output.set_payload(CuTrajectoryFollowOutput {
                    status: CuFollowStatus::ErrorEmptyTrajectory,
                    ..Default::default()
                });
                return Ok(());
            }
            if self.trajectory.as_ref().is_none_or(|stored| stored != traj) {
                self.trajectory = Some(traj.clone());
                self.start = now;
            }
        }

        let Some(stored) = self.trajectory.as_ref() else {
            output.set_payload(CuTrajectoryFollowOutput::default());
            return Ok(());
        };

        let elapsed = now - self.start;
        let elapsed_s = elapsed.as_nanos() as f64 / 1_000_000_000.0;
        let mut positions = [CuJointPosition::default(); N];
        let mut velocities = [CuJointVelocity::default(); N];
        stored.sample(
            Time::new::<second>(elapsed_s),
            &mut positions,
            &mut velocities,
        )?;
        let status = if elapsed_s >= stored.duration().get::<second>() {
            CuFollowStatus::Finished
        } else {
            CuFollowStatus::Running
        };
        output.set_payload(CuTrajectoryFollowOutput {
            positions,
            velocities,
            status,
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuTrajectoryPoint;
    use cu29::units::si::angle::radian;
    use cu29::units::si::angular_velocity::radian_per_second;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::f64::AngularVelocity;

    fn point(t: f64, q: f64, v: f64) -> CuTrajectoryPoint<1> {
        CuTrajectoryPoint {
            time_from_start: Time::new::<second>(t),
            positions: [CuJointPosition::Revolute(Angle::new::<radian>(q))],
            velocities: [CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(v))],
        }
    }

    fn two_point_traj() -> CuTrajectoryPayload<1> {
        let mut traj = CuTrajectoryPayload::new();
        traj.push(point(0.0, 0.0, 0.0)).unwrap();
        traj.push(point(2.0, 2.0, 4.0)).unwrap();
        traj
    }

    fn msg_at(traj: Option<CuTrajectoryPayload<1>>, secs: u64) -> CuMsg<CuTrajectoryPayload<1>> {
        let mut msg = CuMsg::new(traj);
        msg.tov = Tov::Time(CuTime::from_secs(secs));
        msg
    }

    #[test]
    fn test_idle_before_any_trajectory() {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut task = CuFollowTrajectoryTask::<1>::new();
        let input = CuMsg::new(None);
        let mut output = CuMsg::new(None);
        task.process(&ctx, &input, &mut output).unwrap();
        let out = output.payload().unwrap();
        assert_eq!(out.status, CuFollowStatus::Idle);
        assert_eq!(out.positions[0].si_value(), 0.0);
    }

    #[test]
    fn test_midpoint_is_running() {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut task = CuFollowTrajectoryTask::<1>::new();
        let mut output = CuMsg::new(None);
        task.process(&ctx, &msg_at(Some(two_point_traj()), 100), &mut output)
            .unwrap();
        assert_eq!(output.payload().unwrap().status, CuFollowStatus::Running);
        task.process(&ctx, &msg_at(Some(two_point_traj()), 101), &mut output)
            .unwrap();
        let out = output.payload().unwrap();
        assert_eq!(out.status, CuFollowStatus::Running);
        assert!((out.positions[0].si_value() - 1.0).abs() < 1e-12);
        assert!((out.velocities[0].si_value() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_finished_holds_final_point() {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut task = CuFollowTrajectoryTask::<1>::new();
        let mut output = CuMsg::new(None);
        task.process(&ctx, &msg_at(Some(two_point_traj()), 100), &mut output)
            .unwrap();
        task.process(&ctx, &msg_at(Some(two_point_traj()), 102), &mut output)
            .unwrap();
        assert_eq!(output.payload().unwrap().status, CuFollowStatus::Finished);
        task.process(&ctx, &msg_at(Some(two_point_traj()), 110), &mut output)
            .unwrap();
        let out = output.payload().unwrap();
        assert_eq!(out.status, CuFollowStatus::Finished);
        assert_eq!(out.positions[0].si_value(), 2.0);
        assert_eq!(out.velocities[0].si_value(), 4.0);
    }

    #[test]
    fn test_new_trajectory_preempts_and_resets_clock() {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut task = CuFollowTrajectoryTask::<1>::new();
        let mut output = CuMsg::new(None);
        task.process(&ctx, &msg_at(Some(two_point_traj()), 100), &mut output)
            .unwrap();
        task.process(&ctx, &msg_at(Some(two_point_traj()), 101), &mut output)
            .unwrap();
        let mut other = CuTrajectoryPayload::new();
        other.push(point(0.0, 10.0, 0.0)).unwrap();
        other.push(point(4.0, 14.0, 0.0)).unwrap();
        task.process(&ctx, &msg_at(Some(other), 150), &mut output)
            .unwrap();
        let out = output.payload().unwrap();
        assert_eq!(out.status, CuFollowStatus::Running);
        assert_eq!(out.positions[0].si_value(), 10.0);
    }

    #[test]
    fn test_empty_trajectory_reports_error() {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut task = CuFollowTrajectoryTask::<1>::new();
        let mut output = CuMsg::new(None);
        task.process(
            &ctx,
            &msg_at(Some(CuTrajectoryPayload::new()), 100),
            &mut output,
        )
        .unwrap();
        assert_eq!(
            output.payload().unwrap().status,
            CuFollowStatus::ErrorEmptyTrajectory
        );
        task.process(&ctx, &CuMsg::new(None), &mut output).unwrap();
        assert_eq!(output.payload().unwrap().status, CuFollowStatus::Idle);
    }
}
