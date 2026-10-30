#![cfg(feature = "std")]

use cu_collision::CuCollisionReport;
use cu_collision::CuCollisionWorld;
use cu_collision::CuPlanningScene;
use cu_collision::CuSceneExclusions;
use cu_joint_rrt_star::CuJointPlanRequest;
use cu_joint_rrt_star::CuJointRrtParams;
use cu_joint_rrt_star::CuJointRrtStarTask;
use cu_kinematics::CuDiffIkConfig;
use cu_kinematics::CuIkConfig;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuTwist6D;
use cu_robot_description::CuRobotDescription;
use cu_spatial_payloads::Transform3D;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
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
        "manipulator",
        Length::new::<meter>(0.005),
    )
    .unwrap();
    let mut start = [CuJointPosition::default(); 2];
    let mut goal = start;
    robot.write_named_state("home", &mut start).unwrap();
    robot.write_named_state("reach", &mut goal).unwrap();
    let mut planner = CuJointRrtStarTask::new_with_model(
        robot.chain,
        robot.collision_model,
        CuJointRrtParams::default(),
        64,
        32,
        7,
    );
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
    let world = CuCollisionWorld::<1>::new();
    let twist = CuTwist6D::from_base_mps_radps([0.0, 0.01, 0.0], [0.0; 3]);
    ALLOCATIONS.with(|count| count.set(0));
    TRACK.with(|track| track.set(true));
    robot.chain.forward_kinematics(&goal, &mut pose).unwrap();
    robot.chain.geometric_jacobian(&start, &mut jac).unwrap();
    robot
        .chain
        .inverse_kinematics_dls(&pose, &start, &CuIkConfig::default(), &mut solution)
        .unwrap();
    robot
        .chain
        .differential_ik(&start, &twist, &CuDiffIkConfig::default(), &mut velocity)
        .unwrap();
    robot
        .collision_model
        .check_configuration(&robot.chain, &start, Some(&world), &mut report)
        .unwrap();
    planner.base(&ctx, &request, &mut output).unwrap();
    planner.refine(&ctx, &mut output).unwrap();
    TRACK.with(|track| track.set(false));
    assert_eq!(ALLOCATIONS.with(Cell::get), 0);
    assert!(output.payload().is_some());
}
