#![cfg(feature = "std")]

use cu_spatial_payloads::Transform3D;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use repose_collision::CuCollisionEnv;
use repose_collision::CuCollisionModelParams;
use repose_collision::CuCollisionReport;
use repose_collision::CuPlanningScene;
use repose_collision::CuSceneExclusions;
use repose_joint_rrt_star::CuJointRrtStar;
use repose_joint_rrt_star::CuJointRrtStarParams;
use repose_joint_rrt_star::CuJointRrtStarTask;
use repose_kinematics::CuDiffIkParams;
use repose_kinematics::CuIkParams;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointVelocity;
use repose_kinematics::CuTwist6D;
use repose_kinematics::CuTwistFrame;
use repose_motion_planning::CuJointPlanRequest;
use repose_motion_planning::CuJointPlanner;
use repose_robot_description::CuRobotDescription;
use repose_robot_description::CuSrdfImport;
use std::alloc::GlobalAlloc;
use std::alloc::Layout;
use std::alloc::System;
use std::cell::Cell;

struct CountedAllocator;
thread_local! {
    static TRACK: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}
fn count_allocation() {
    if TRACK.try_with(Cell::get).unwrap_or(false) {
        let _ = ALLOCATIONS.try_with(|count| count.set(count.get() + 1));
    }
}
// SAFETY: All allocation operations are forwarded unchanged to the system allocator.
unsafe impl GlobalAlloc for CountedAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count_allocation();
        // SAFETY: The caller supplies a valid allocation layout.
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count_allocation();
        // SAFETY: The caller supplies a valid allocation layout.
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        count_allocation();
        // SAFETY: The caller supplies a system-allocated pointer and its layout.
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: The pointer and layout came from the system allocator.
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: CountedAllocator = CountedAllocator;

#[test]
fn imported_models_execute_without_heap_allocation() {
    let robot = CuRobotDescription::<2, 6>::from_urdf_srdf(
        include_str!("fixtures/serial.urdf"),
        include_str!("fixtures/serial.srdf"),
        CuSrdfImport::new("manipulator").with_collision(CuCollisionModelParams {
            safety_margin: Length::new::<meter>(0.005),
            skip_adjacent_links: false,
        }),
    )
    .unwrap();
    let mut start = [CuJointPosition::default(); 2];
    let mut goal = start;
    robot.write_named_state("home", &mut start).unwrap();
    robot.write_named_state("reach", &mut goal).unwrap();
    let mut planner = CuJointRrtStarTask::from_planner(
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
    let request = CuMsg::new(Some(CuJointPlanRequest {
        scene: CuPlanningScene::new(),
        exclusions: CuSceneExclusions::new(),
        start,
        goal,
    }));
    let mut output = CuMsg::new(None);
    let mut pose = Transform3D::<f64>::identity();
    let mut jac = [[0.0; 2]; 6];
    let mut solution = start;
    let mut velocity = [CuJointVelocity::default(); 2];
    let mut report = CuCollisionReport::<8>::default();
    let env = CuCollisionEnv::EMPTY;
    let twist = CuTwist6D::new(
        [0.0, 0.01, 0.0].map(Velocity::new::<meter_per_second>),
        [AngularVelocity::new::<radian_per_second>(0.0); 3],
        CuTwistFrame::Base,
    );
    ALLOCATIONS.with(|count| count.set(0));
    TRACK.with(|track| track.set(true));
    robot.chain().forward_kinematics(&goal, &mut pose).unwrap();
    robot.chain().geometric_jacobian(&start, &mut jac).unwrap();
    robot
        .chain()
        .inverse_kinematics_dls(&pose, &start, &CuIkParams::default(), &mut solution)
        .unwrap();
    robot
        .chain()
        .differential_ik(&start, &twist, &CuDiffIkParams::default(), &mut velocity)
        .unwrap();
    robot
        .collision_model()
        .check_configuration(robot.chain(), &start, &env, &mut report)
        .unwrap();
    planner.base(&ctx, &request, &mut output).unwrap();
    planner.refine(&ctx, &mut output).unwrap();
    TRACK.with(|track| track.set(false));
    assert_eq!(ALLOCATIONS.with(Cell::get), 0);
    assert!(output.payload().is_some());
}
