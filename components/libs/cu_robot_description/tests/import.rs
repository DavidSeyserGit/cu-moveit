#![cfg(feature = "std")]

use cu_collision::CuCollisionReport;
use cu_collision::CuCollisionShape;
use cu_collision::CuCollisionWorld;
use cu_joint_rrt_star::CuJointPlanPath;
use cu_joint_rrt_star::CuJointPlanRequest;
use cu_joint_rrt_star::CuJointRrtParams;
use cu_joint_rrt_star::CuJointRrtStarTask;
use cu_kinematics::CuDiffIkConfig;
use cu_kinematics::CuIkConfig;
use cu_kinematics::CuIkStatus;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuTwist6D;
use cu_robot_description::CuRobotDescription;
use cu_spatial_payloads::Transform3D;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;

const URDF: &str = include_str!("fixtures/serial.urdf");
const SRDF: &str = include_str!("fixtures/serial.srdf");
fn margin() -> Length {
    Length::new::<meter>(0.005)
}
fn robot() -> CuRobotDescription<2, 6> {
    CuRobotDescription::from_urdf_srdf(URDF, SRDF, "manipulator", margin()).unwrap()
}
fn joints(angle: f64, distance: f64) -> [CuJointPosition; 2] {
    [
        Angle::new::<radian>(angle).into(),
        Length::new::<meter>(distance).into(),
    ]
}

#[test]
fn fixed_frames_and_prismatic_units_match_analytic_fk_and_jacobian() {
    let robot = robot();
    assert_eq!(robot.joint_names(), &["swing", "extension"]);
    assert_eq!(robot.collision_model.shape_count(), 6);
    let q = joints(0.0, 0.2);
    let mut pose = Transform3D::<f64>::identity();
    robot.chain.forward_kinematics(&q, &mut pose).unwrap();
    let m = pose.to_matrix();
    assert!(m[0][3].abs() < 1e-12);
    assert!((m[1][3] - 0.9).abs() < 1e-12);
    assert!((m[2][3] - 0.2).abs() < 1e-12);
    let mut jac = [[0.0; 2]; 6];
    robot.chain.geometric_jacobian(&q, &mut jac).unwrap();
    for col in 0..2 {
        let mut perturbed = q;
        perturbed[col] = q[col].with_si_value(q[col].si_value() + 1e-7);
        let mut p = pose;
        robot.chain.forward_kinematics(&perturbed, &mut p).unwrap();
        for row in 0..3 {
            assert!(((p.to_matrix()[row][3] - m[row][3]) / 1e-7 - jac[row][col]).abs() < 1e-6);
        }
    }
    assert_eq!(jac[5][1], 0.0);
    let wrong_units = [q[0], Angle::new::<radian>(0.2).into()];
    assert!(
        robot
            .chain
            .forward_kinematics(&wrong_units, &mut pose)
            .is_err()
    );
}

#[test]
fn mixed_ik_and_servo_keep_linear_coordinates_and_velocity() {
    let robot = robot();
    let mut target = Transform3D::<f64>::identity();
    robot
        .chain
        .forward_kinematics(&joints(0.4, 0.25), &mut target)
        .unwrap();
    let mut solution = joints(0.0, 0.1);
    let status = robot
        .chain
        .inverse_kinematics_dls(
            &target,
            &joints(0.0, 0.1),
            &CuIkConfig::default(),
            &mut solution,
        )
        .unwrap();
    assert!(matches!(status, CuIkStatus::Converged { .. }));
    assert!((solution[1].displacement().unwrap().get::<meter>() - 0.25).abs() < 1e-3);
    let mut velocity = [CuJointVelocity::default(); 2];
    robot
        .chain
        .differential_ik(
            &joints(0.0, 0.2),
            &CuTwist6D::from_base_mps_radps([0.0, 0.05, 0.0], [0.0; 3]),
            &CuDiffIkConfig::default(),
            &mut velocity,
        )
        .unwrap();
    assert!(matches!(velocity[0], CuJointVelocity::Revolute(_)));
    assert!(matches!(velocity[1], CuJointVelocity::Prismatic(_)));
    assert!(velocity[1].si_value() > 0.0 && velocity[1].si_value() <= 0.2);
}

#[test]
fn geometry_retains_fixed_base_multiple_shapes_and_srdf_pairs() {
    let raw = CuRobotDescription::<2, 6>::from_urdf(URDF, "base", "tool", margin()).unwrap();
    let robot = robot();
    assert_eq!(
        robot.collision_links(),
        &["base", "pedestal", "arm", "arm", "slide", "tool"]
    );
    assert!(raw.collision_model.is_pair_enabled(2, 4));
    assert!(!robot.collision_model.is_pair_enabled(2, 4));
    assert!(!robot.collision_model.is_pair_enabled(3, 4));
    assert!(!robot.collision_model.is_pair_enabled(4, 5)); // same rigid body
    let mut frames = [Transform3D::<f64>::identity(); 2];
    let mut pose = Transform3D::<f64>::identity();
    robot
        .chain
        .forward_frames(&joints(0.0, 0.2), &mut frames, &mut pose)
        .unwrap();
    let mut shapes = [CuCollisionShape::default(); 6];
    robot
        .collision_model
        .world_link_shapes(&frames, &mut shapes);
    if let CuCollisionShape::Sphere(s) = shapes[0] {
        assert_eq!(s.center[2].get::<meter>(), -1.0);
    } else {
        panic!("expected base sphere");
    }
    if let CuCollisionShape::Capsule(c) = shapes[2] {
        assert!((c.a[1].get::<meter>() - 0.15).abs() < 1e-12);
        assert!((c.b[1].get::<meter>() - 0.35).abs() < 1e-12);
    } else {
        panic!("expected cylinder enclosure");
    }
    let mut world = CuCollisionWorld::<1>::new();
    world.add_obstacle(shapes[0]).unwrap();
    let mut report = CuCollisionReport::<8>::default();
    robot
        .collision_model
        .check_configuration(&robot.chain, &joints(0.0, 0.2), Some(&world), &mut report)
        .unwrap();
    assert!(!report.is_collision_free()); // base geometry participates in environment checks
}

#[test]
fn named_states_drive_a_mixed_joint_anytime_planner() {
    let robot = robot();
    let mut start = joints(0.0, 0.0);
    let mut goal = start;
    robot.write_named_state("home", &mut start).unwrap();
    robot.write_named_state("reach", &mut goal).unwrap();
    assert_eq!(start[1].displacement().unwrap().get::<meter>(), 0.1);
    let mut task = CuJointRrtStarTask::new_with_model(
        robot.chain,
        robot.collision_model,
        CuJointRrtParams::default(),
        64,
        32,
        7,
    );
    let (ctx, _) = CuContext::new_mock_clock();
    let input = CuMsg::new(Some(CuJointPlanRequest {
        world: CuCollisionWorld::new(),
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

#[test]
fn invalid_models_and_unsupported_features_fail_explicitly() {
    assert!(CuRobotDescription::<3, 6>::from_urdf(URDF, "base", "tool", margin()).is_err());
    assert!(CuRobotDescription::<2, 5>::from_urdf(URDF, "base", "tool", margin()).is_err());
    assert!(CuRobotDescription::<2, 6>::from_urdf(URDF, "tool", "base", margin()).is_err());
    for xml in [
        URDF.replace("radius=\"0.1\"", "radius=\"NaN\""),
        URDF.replace("xyz=\"0 0 2\"", "xyz=\"0 0 0\""),
        URDF.replace("type=\"prismatic\"", "type=\"floating\""),
        URDF.replace("<sphere radius=\"0.1\"/>", "<mesh filename=\"robot.stl\"/>"),
        URDF.replace("<axis xyz=\"1 0 0\"/>", "<mimic joint=\"swing\"/>"),
    ] {
        assert!(CuRobotDescription::<2, 6>::from_urdf(&xml, "base", "tool", margin()).is_err());
    }
    let mut robot = robot();
    for xml in [
        SRDF.replace("serial_demo", "other"),
        SRDF.replace("value=\"0.1\"", "value=\"8\""),
        SRDF.replace("<joint name=\"extension\" value=\"0.1\"/>", ""),
        SRDF.replace(
            "<chain base_link=\"base\" tip_link=\"tool\"/>",
            "<joint name=\"swing\"/>",
        ),
    ] {
        assert!(robot.apply_srdf(&xml, "manipulator").is_err());
    }
    let mut home = joints(0.0, 0.0);
    robot.write_named_state("home", &mut home).unwrap(); // failed updates are atomic
    assert_eq!(home[1].si_value(), 0.1);
}

#[test]
fn continuous_rotation_has_no_position_limit_and_empty_geometry_has_no_ghost_shape() {
    let urdf = r#"<robot name="continuous"><link name="base"/><link name="tip"/>
        <joint name="spin" type="continuous"><parent link="base"/><child link="tip"/>
        <limit velocity="2" effort="1"/></joint></robot>"#;
    let robot = CuRobotDescription::<1, 4>::from_urdf(urdf, "base", "tip", margin()).unwrap();
    let mut q = [Angle::new::<radian>(20.0).into()];
    assert!(robot.chain.is_within_limits(&q));
    robot.chain.clamp_to_limits(&mut q).unwrap();
    assert_eq!(q[0].angle().unwrap().get::<radian>(), 20.0);
    assert_eq!(robot.collision_model.shape_count(), 0);
    let mut report = CuCollisionReport::<0>::default();
    let world = CuCollisionWorld::<0>::new();
    robot
        .collision_model
        .check_configuration(&robot.chain, &q, Some(&world), &mut report)
        .unwrap();
    assert!(report.is_collision_free());
}

#[test]
fn folded_intermediate_rotations_preserve_the_full_transform() {
    for pitch in [
        0.7,
        core::f64::consts::FRAC_PI_2,
        -core::f64::consts::FRAC_PI_2,
    ] {
        let urdf = format!(
            r#"<robot name="folded">
        <link name="base"/><link name="a"/><link name="bracket"/><link name="b"/><link name="tip"/>
        <joint name="a" type="revolute"><parent link="base"/><child link="a"/>
          <origin xyz="0.1 0.2 0.3" rpy="0.2 0.3 0.4"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1"/></joint>
        <joint name="fixed" type="fixed"><parent link="a"/><child link="bracket"/>
          <origin xyz="0.2 -0.1 0.4" rpy="0.4 {pitch} -0.3"/></joint>
        <joint name="b" type="prismatic"><parent link="bracket"/><child link="b"/>
          <axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="1"/></joint>
        <joint name="tip" type="fixed"><parent link="b"/><child link="tip"/>
          <origin xyz="0.1 0 0.2" rpy="-0.2 0.1 0.3"/></joint></robot>"#
        );
        let robot = CuRobotDescription::<2, 0>::from_urdf(&urdf, "base", "tip", margin()).unwrap();
        let transform = |xyz: [f64; 3], rpy: [f64; 3]| {
            Transform3D::<f64>::from_translation_euler_xyz(
                xyz.map(Length::new::<meter>),
                rpy.map(Angle::new::<radian>),
            )
        };
        let expected = transform([0.1, 0.2, 0.3], [0.2, 0.3, 0.4])
            .compose(transform([0.0; 3], [0.0, 0.0, 0.2]))
            .compose(transform([0.2, -0.1, 0.4], [0.4, pitch, -0.3]))
            .compose(transform([0.15, 0.0, 0.0], [0.0; 3]))
            .compose(transform([0.1, 0.0, 0.2], [-0.2, 0.1, 0.3]));
        let mut actual = Transform3D::<f64>::identity();
        robot
            .chain
            .forward_kinematics(&joints(0.2, 0.15), &mut actual)
            .unwrap();
        for (a, b) in actual
            .to_matrix()
            .iter()
            .flatten()
            .zip(expected.to_matrix().iter().flatten())
        {
            assert!((a - b).abs() < 1e-10);
        }
    }
}

#[test]
fn topology_semantics_and_zero_capacity_reports_fail_safely() {
    let bad_cycle = URDF.replace(
        "<parent link=\"base\"/><child link=\"pedestal\"/>",
        "<parent link=\"tool\"/><child link=\"pedestal\"/>",
    );
    assert!(CuRobotDescription::<2, 6>::from_urdf(&bad_cycle, "base", "tool", margin()).is_err());
    let duplicate = URDF.replace("</robot>", "<link name=\"arm\"/></robot>");
    assert!(CuRobotDescription::<2, 6>::from_urdf(&duplicate, "base", "tool", margin()).is_err());
    let mut robot = robot();
    assert!(
        robot
            .apply_srdf(
                &SRDF.replace("link2=\"slide\"", "link2=\"typo\""),
                "manipulator"
            )
            .is_err()
    );
    let mut world = CuCollisionWorld::<1>::new();
    world
        .add_obstacle(CuCollisionShape::Sphere(
            cu_collision::CuSphere::from_meters([0.0, 0.0, -1.0], 0.1),
        ))
        .unwrap();
    let mut report = CuCollisionReport::<0>::default();
    robot
        .collision_model
        .check_configuration(&robot.chain, &joints(0.0, 0.2), Some(&world), &mut report)
        .unwrap();
    assert!(!report.is_collision_free());
    assert_eq!(report.len, 0);
    assert!(report.total_contacts > 0);
}
