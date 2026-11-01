//! Turn measured joint state and a configured joint goal into a planning request.

use crate::CuJointPlanRequest;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointStatePayload;
use cu_kinematics::CuRobotStatus;
use cu29::prelude::*;
use serde::Deserialize;
use serde::Serialize;

/// `Option<[CuJointPosition; N]>` serde helper.
mod option_position_array {
    use cu_kinematics::CuJointPosition;
    use serde::Deserialize;
    use serde::Deserializer;
    use serde::Serialize;
    use serde::Serializer;

    #[derive(Serialize, Deserialize)]
    #[serde(transparent)]
    struct Positions<const N: usize>(
        #[serde(with = "cu_kinematics::serde_arrays::position_array")] [CuJointPosition; N],
    );

    pub(super) fn serialize<S: Serializer, const N: usize>(
        values: &Option<[CuJointPosition; N]>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        values.map(Positions).serialize(serializer)
    }

    pub(super) fn deserialize<'de, D: Deserializer<'de>, const N: usize>(
        deserializer: D,
    ) -> Result<Option<[CuJointPosition; N]>, D::Error> {
        Ok(Option::<Positions<N>>::deserialize(deserializer)?.map(|p| p.0))
    }
}

/// Node config of [`CuJointGoalTask`].
///
/// ```ron
/// // CuJointGoalTask<2>: radians for revolute joints, meters for prismatic ones
/// config: {"goal": [{"revolute": 0.4}, {"prismatic": 0.1}]}
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuJointGoalParams<const N: usize> {
    /// Required joint goal: `N` typed joint coordinates.
    #[serde(default, with = "option_position_array")]
    #[reflect(ignore)]
    pub goal: Option<[CuJointPosition; N]>,
}

impl<const N: usize> Default for CuJointGoalParams<N> {
    fn default() -> Self {
        Self { goal: None }
    }
}

impl<const N: usize> CuJointGoalParams<N> {
    /// Parameters requesting a plan to `goal`.
    pub const fn new(goal: [CuJointPosition; N]) -> Self {
        Self { goal: Some(goal) }
    }

    /// Checks that the goal is present.
    pub fn validate(&self) -> CuResult<()> {
        self.goal
            .map(|_| ())
            .ok_or_else(|| CuError::from("joint goal: missing required 'goal'"))
    }
}

/// Requests one plan from the measured joint state to a configured goal.
///
/// Input is the robot's [`CuJointStatePayload<N>`]; output is a
/// [`CuJointPlanRequest<N>`] for a [`crate::CuJointPlannerTask`]. The first
/// cycle whose state reports [`CuRobotStatus::Ready`] becomes the plan start
/// and the request is published exactly once; every other cycle publishes
/// nothing, so the planner runs one job.
///
/// Node config: [`CuJointGoalParams`]. The request carries an empty planning
/// scene; self-collision is still checked against the planner's collision
/// model.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointGoalTask<const N: usize> {
    #[reflect(ignore)]
    request: CuJointPlanRequest<N>,
    requested: bool,
}

impl<const N: usize> CuJointGoalTask<N> {
    /// Creates a task that requests one plan to the configured goal.
    pub fn from_params(params: CuJointGoalParams<N>) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            request: CuJointPlanRequest {
                goal: params.goal.unwrap_or([CuJointPosition::default(); N]),
                ..Default::default()
            },
            requested: false,
        })
    }

    /// `true` once the request has been published.
    pub const fn is_requested(&self) -> bool {
        self.requested
    }
}

impl<const N: usize> Freezable for CuJointGoalTask<N> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.requested, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.requested = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<const N: usize> CuTask for CuJointGoalTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuJointStatePayload<N>);
    type Output<'m> = output_msg!(CuJointPlanRequest<N>);

    fn new(cfg: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuJointGoalParams<N>>)
            .transpose()?
            .unwrap_or_default();
        Self::from_params(params)
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        output.clear_payload();
        if self.requested {
            return Ok(());
        }
        let Some(state) = input.payload() else {
            return Ok(());
        };
        if state.status != CuRobotStatus::Ready {
            return Ok(());
        }
        self.request.start = state.positions;
        output.set_payload(self.request.clone());
        self.requested = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle;

    fn joint(rad: f64) -> CuJointPosition {
        CuJointPosition::Revolute(Angle::new::<radian>(rad))
    }

    fn run(task: &mut CuJointGoalTask<2>, status: CuRobotStatus) -> Option<CuJointPlanRequest<2>> {
        let state = CuMsg::new(Some(CuJointStatePayload {
            status,
            positions: [joint(0.1); 2],
            ..Default::default()
        }));
        let mut output = CuMsg::<CuJointPlanRequest<2>>::default();
        let (ctx, _clock) = CuContext::new_mock_clock();
        task.process(&ctx, &state, &mut output).unwrap();
        output.payload().cloned()
    }

    #[test]
    fn test_requests_once_from_first_ready_state() {
        let mut task =
            CuJointGoalTask::from_params(CuJointGoalParams::new([joint(0.5); 2])).unwrap();
        assert!(run(&mut task, CuRobotStatus::NotReady).is_none());
        let request = run(&mut task, CuRobotStatus::Ready).unwrap();
        assert_eq!(request.start, [joint(0.1); 2]);
        assert_eq!(request.goal, [joint(0.5); 2]);
        assert!(run(&mut task, CuRobotStatus::Ready).is_none());
    }

    #[test]
    fn test_reads_typed_goal() {
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"goal": [{"revolute": 0.4}, {"revolute": -0.3}]}"#,
        )
        .unwrap();
        let mut task = CuJointGoalTask::<2>::new(Some(&cfg), ()).unwrap();
        let request = run(&mut task, CuRobotStatus::Ready).unwrap();
        assert_eq!(request.goal, [joint(0.4), joint(-0.3)]);
        assert!(CuJointGoalTask::<2>::new(None, ()).is_err());
        let typo: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"goal": [{"revolute": 0.4}, {"revolute": -0.3}], "gaol": 1}"#,
        )
        .unwrap();
        assert!(CuJointGoalTask::<2>::new(Some(&typo), ()).is_err());
    }
}
