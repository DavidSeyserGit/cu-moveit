//! Import a mixed revolute/prismatic serial arm and verify its planning pipeline.

use cu_spatial_payloads::Transform3D;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use repose_collision::CuCollisionEnv;
use repose_collision::CuCollisionModelParams;
use repose_collision::CuCollisionReport;
use repose_collision::CuPlanningScene;
use repose_collision::CuSceneExclusions;
use repose_joint_rrt_star::CuJointRrtStar;
use repose_joint_rrt_star::CuJointRrtStarParams;
use repose_joint_rrt_star::CuJointRrtStarTask;
use repose_kinematics::CuIkParams;
use repose_kinematics::CuIkStatus;
use repose_kinematics::CuJointPosition;
use repose_motion_planning::CuJointPlanRequest;
use repose_motion_planning::CuJointPlanner;
use repose_robot_description::CuRobotDescription;
use repose_robot_description::CuSrdfImport;

fn main() -> CuResult<()> {
    // XML parsing, names, and SRDF state lookups happen before runtime execution.
    let robot = CuRobotDescription::<2, 6>::from_urdf_srdf(
        include_str!("../tests/fixtures/serial.urdf"),
        include_str!("../tests/fixtures/serial.srdf"),
        CuSrdfImport::new("manipulator").with_collision(CuCollisionModelParams {
            safety_margin: Length::new::<meter>(0.005),
            skip_adjacent_links: false,
        }),
    )?;
    let mut start = [CuJointPosition::default(); 2];
    let mut goal = start;
    robot.write_named_state("home", &mut start)?;
    robot.write_named_state("reach", &mut goal)?;

    // Runtime algorithms write into fixed-size caller-owned buffers.
    let mut target = Transform3D::<f64>::identity();
    robot.chain().forward_kinematics(&goal, &mut target)?;
    let mut solution = start;
    let status = robot.chain().inverse_kinematics_dls(
        &target,
        &start,
        &CuIkParams::default(),
        &mut solution,
    )?;
    if !matches!(status, CuIkStatus::Converged { .. }) {
        return Err(CuError::from("Example IK did not converge"));
    }
    let mut report = CuCollisionReport::<16>::default();
    let env = CuCollisionEnv::EMPTY;
    robot
        .collision_model()
        .check_configuration(robot.chain(), &solution, &env, &mut report)?;
    if !report.is_collision_free() {
        return Err(CuError::from("Example IK solution is in collision"));
    }

    let mut planner = CuJointRrtStarTask::from_planner(
        CuJointRrtStar::new(
            *robot.chain(),
            *robot.collision_model(),
            CuJointRrtStarParams::default(),
        )?,
        64,
        32,
        7,
    )?;
    let (ctx, _) = CuContext::new_mock_clock();
    let scene = CuPlanningScene::<16, 8>::new();
    let request = CuMsg::new(Some(CuJointPlanRequest {
        scene,
        exclusions: CuSceneExclusions::new(),
        start,
        goal,
    }));
    let mut output = CuMsg::new(None);
    planner.base(&ctx, &request, &mut output)?;
    for _ in 0..8 {
        planner.refine(&ctx, &mut output)?;
    }
    let path = output
        .payload()
        .ok_or_else(|| CuError::from("Example planner found no path"))?;
    if path.len < 2 || path.waypoints[path.len as usize - 1].positions != goal {
        return Err(CuError::from(
            "Example path did not reach the requested goal",
        ));
    }
    Ok(())
}
