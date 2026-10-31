#![cfg(feature = "std")]
//! Planner integration; needs `cu-joint-rrt-star` (dev-dependency).

use cu_collision::CuCollisionModelParams;
use cu_collision::CuPlanningScene;
use cu_collision::CuSceneExclusions;
use cu_joint_rrt_star::CuJointRrtStar;
use cu_joint_rrt_star::CuJointRrtStarParams;
use cu_joint_rrt_star::CuJointRrtStarTask;
use cu_kinematics::CuJointPosition;
use cu_motion_planning::CuJointPlanPath;
use cu_motion_planning::CuJointPlanRequest;
use cu_motion_planning::CuJointPlanner;
use cu_robot_description::CuRobotDescription;
use cu_robot_description::CuSrdfImport;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;

const URDF: &str = include_str!("fixtures/serial.urdf");
const SRDF: &str = include_str!("fixtures/serial.srdf");
fn robot() -> CuRobotDescription<2, 6> {
    CuRobotDescription::from_urdf_srdf(
        URDF,
        SRDF,
        CuSrdfImport::new("manipulator").with_collision(CuCollisionModelParams {
            safety_margin: Length::new::<meter>(0.005),
            skip_adjacent_links: false,
        }),
    )
    .unwrap()
}
fn joints(angle: f64, distance: f64) -> [CuJointPosition; 2] {
    [
        Angle::new::<radian>(angle).into(),
        Length::new::<meter>(distance).into(),
    ]
}

#[test]
fn named_states_drive_a_mixed_joint_anytime_planner() {
    let robot = robot();
    let mut start = joints(0.0, 0.0);
    let mut goal = start;
    robot.write_named_state("home", &mut start).unwrap();
    robot.write_named_state("reach", &mut goal).unwrap();
    assert_eq!(start[1].displacement().unwrap().get::<meter>(), 0.1);
    let mut task = CuJointRrtStarTask::from_planner(
        CuJointRrtStar::new(
            *robot.chain(),
            *robot.collision_model(),
            CuJointRrtStarParams::default(),
        )
        .unwrap(),
        64,
        32,
        7,
    )
    .unwrap();
    let (ctx, _) = CuContext::new_mock_clock();
    let input = CuMsg::new(Some(CuJointPlanRequest {
        scene: CuPlanningScene::new(),
        exclusions: CuSceneExclusions::new(),
        start,
        goal,
    }));
    let mut output = CuMsg::<CuJointPlanPath<2>>::new(None);
    task.base(&ctx, &input, &mut output).unwrap();
    let path = output.payload().unwrap();
    assert_eq!(path.waypoints[0].positions, start);
    assert_eq!(path.waypoints[path.len as usize - 1].positions, goal);
    assert!(path.cost.is_finite());
}
