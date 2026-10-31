//! Time planned joint paths into fixed-capacity trajectories.

use crate::CuJointPlanPath;
use crate::MAX_WAYPOINTS;
use cu_kinematics::CuJointPosition;
use cu_trajectory::CuTimingOverrides;
use cu_trajectory::CuTimingParams;
use cu_trajectory::CuTimingStatus;
use cu_trajectory::CuTrajectoryPayload;
use cu_trajectory::parameterize;
use cu29::prelude::*;

/// Times each published plan into a [`CuTrajectoryPayload<N, CAP>`].
///
/// Input is a planner's [`CuJointPlanPath<N>`]. A path with
/// [`CuPlanStatus::has_path`](crate::CuPlanStatus::has_path) is the final
/// result of its planning job and is timed; failure statuses and empty
/// cycles publish nothing. A path that cannot be timed within the
/// configured limits or the trajectory capacity `CAP` is an error.
///
/// Node config: [`CuTimingParams<N>`] (`limits` required, `profile` default
/// `"trapezoidal"`, `sample_period` in seconds, default `0.02`):
///
/// ```ron
/// // CuPlanToTrajectoryTask<6, 1024>
/// config: {
///     "limits": {
///         "max_velocities": [{"revolute": 0.8}, /* ... */],
///         "max_accelerations": [{"revolute": 2.0}, /* ... */],
///     },
///     "profile": "s_curve",
///     "sample_period": 0.01,
/// }
/// ```
///
/// Timing does not recheck the interpolated trajectory for collisions; the
/// path's `scene_revision` identifies the scene it was planned against.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuPlanToTrajectoryTask<const N: usize, const CAP: usize> {
    #[reflect(ignore)]
    params: CuTimingParams<N>,
    #[reflect(ignore)]
    waypoints: [[CuJointPosition; N]; MAX_WAYPOINTS],
}

impl<const N: usize, const CAP: usize> CuPlanToTrajectoryTask<N, CAP> {
    /// Creates a timing task from validated timing parameters.
    pub fn from_params(params: CuTimingParams<N>) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            params,
            waypoints: [[CuJointPosition::default(); N]; MAX_WAYPOINTS],
        })
    }

    /// The timing parameters.
    pub const fn params(&self) -> &CuTimingParams<N> {
        &self.params
    }
}

impl<const N: usize, const CAP: usize> Freezable for CuPlanToTrajectoryTask<N, CAP> {}

impl<const N: usize, const CAP: usize> CuTask for CuPlanToTrajectoryTask<N, CAP> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuJointPlanPath<N>);
    type Output<'m> = output_msg!(CuTrajectoryPayload<N, CAP>);

    fn new(cfg: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let cfg = cfg.ok_or_else(|| CuError::from("plan timing: missing 'limits'"))?;
        Self::from_params(cfg.deserialize_into::<CuTimingParams<N>>()?)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        output.clear_payload();
        let Some(path) = input.payload().filter(|path| path.status.has_path()) else {
            return Ok(());
        };
        let len = path.len as usize;
        if !(1..=MAX_WAYPOINTS).contains(&len) {
            return Err(CuError::from("plan timing: invalid path length"));
        }
        for (dst, src) in self.waypoints.iter_mut().zip(&path.waypoints[..len]) {
            *dst = src.positions;
        }
        let trajectory = output
            .payload_mut()
            .get_or_insert_with(CuTrajectoryPayload::new);
        let status = parameterize(
            &self.waypoints[..len],
            &self.params,
            CuTimingOverrides::default(),
            trajectory,
        )?;
        if status != CuTimingStatus::Success {
            output.clear_payload();
            return Err(CuError::from("plan timing: parameterization failed"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuPlanStatus;
    use cu_kinematics::CuJointAcceleration;
    use cu_kinematics::CuJointVelocity;
    use cu_trajectory::CuJointTrajectoryLimits;
    use cu_trajectory::CuTrajectoryProfile;
    use cu29::units::si::angle::radian;
    use cu29::units::si::angular_acceleration::radian_per_second_squared;
    use cu29::units::si::angular_velocity::radian_per_second;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::f64::AngularAcceleration;
    use cu29::units::si::f64::AngularVelocity;
    use cu29::units::si::f64::Time;
    use cu29::units::si::time::second;

    fn joint(rad: f64) -> CuJointPosition {
        CuJointPosition::Revolute(Angle::new::<radian>(rad))
    }

    fn task() -> CuPlanToTrajectoryTask<1, 256> {
        let limits = CuJointTrajectoryLimits::new(
            [CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(1.0))],
            [CuJointAcceleration::Revolute(AngularAcceleration::new::<
                radian_per_second_squared,
            >(2.0))],
        );
        CuPlanToTrajectoryTask::from_params(CuTimingParams {
            limits,
            profile: CuTrajectoryProfile::Trapezoidal,
            sample_period: Time::new::<second>(0.02),
        })
        .unwrap()
    }

    fn path(status: CuPlanStatus) -> CuJointPlanPath<1> {
        let mut path = CuJointPlanPath::<1> {
            status,
            ..Default::default()
        };
        if status.has_path() {
            path.waypoints[0].positions = [joint(0.0)];
            path.waypoints[1].positions = [joint(0.5)];
            path.len = 2;
        }
        path
    }

    fn run(
        task: &mut CuPlanToTrajectoryTask<1, 256>,
        path: CuJointPlanPath<1>,
    ) -> CuResult<Option<CuTrajectoryPayload<1, 256>>> {
        let input = CuMsg::new(Some(path));
        let mut output = CuMsg::<CuTrajectoryPayload<1, 256>>::default();
        let (ctx, _clock) = CuContext::new_mock_clock();
        task.process(&ctx, &input, &mut output)?;
        Ok(output.payload().cloned())
    }

    #[test]
    fn test_times_final_paths() {
        let mut task = task();
        for status in [CuPlanStatus::Improving, CuPlanStatus::Converged] {
            let trajectory = run(&mut task, path(status)).unwrap().unwrap();
            assert!(!trajectory.is_empty());
        }
    }

    #[test]
    fn test_ignores_failed_plans() {
        let mut task = task();
        for status in [CuPlanStatus::Searching, CuPlanStatus::InvalidGoal] {
            assert!(run(&mut task, path(status)).unwrap().is_none());
        }
    }

    #[test]
    fn test_reads_config() {
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"limits": {"max_velocities": [{"revolute": 1.0}], "max_accelerations": [{"revolute": 2.0}]},
                "profile": "cubic", "sample_period": 0.01}"#,
        )
        .unwrap();
        let task = CuPlanToTrajectoryTask::<1, 256>::new(Some(&cfg), ()).unwrap();
        assert_eq!(task.params().profile, CuTrajectoryProfile::Cubic);
        assert_eq!(task.params().sample_period, Time::new::<second>(0.01));
        let legacy: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"limits": {"max_velocities": [{"revolute": 1.0}], "max_accelerations": [{"revolute": 2.0}]},
                "sample_period_s": 0.01}"#,
        )
        .unwrap();
        assert!(CuPlanToTrajectoryTask::<1, 256>::new(Some(&legacy), ()).is_err());
        assert!(CuPlanToTrajectoryTask::<1, 256>::new(None, ()).is_err());
    }
}
