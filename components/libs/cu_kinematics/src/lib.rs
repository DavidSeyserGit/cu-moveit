//! Deterministic, allocation-free serial robot kinematics (FK, Jacobian, IK, Servo) for `copper-rs`.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod chain;
mod ik;
mod joint;
mod robots;
pub mod serde_arrays;
mod task;

pub use chain::CuJointAxis;
pub use chain::CuJointSpec;
pub use chain::CuJointType;
pub use chain::CuKinematicChain;
pub use ik::CuDiffIkParams;
pub use ik::CuDiffIkStatus;
pub use ik::CuIkParams;
pub use ik::CuIkStatus;
pub use ik::CuTwist6D;
pub use ik::CuTwistFrame;
pub use joint::CuJointAcceleration;
pub use joint::CuJointJerk;
pub use joint::CuJointLimits;
pub use joint::CuJointMetric;
pub use joint::CuJointPosition;
pub use joint::CuJointResolution;
pub use joint::CuJointVelocity;
pub use robots::CuChainSpec;
pub use robots::CuRobotPreset;
pub use robots::igus_rebel_chain;
pub use task::CuForwardKinematicsParams;
pub use task::CuForwardKinematicsTask;
pub use task::CuIkSolutionPayload;
pub use task::CuInverseKinematicsParams;
pub use task::CuInverseKinematicsTask;
pub use task::CuJointPositionsPayload;
pub use task::CuJointStatePayload;
pub use task::CuJointVelocitiesPayload;
pub use task::CuRobotStatus;
pub use task::CuServoParams;
pub use task::CuServoTask;

#[cfg(test)]
mod tests {
    use crate::CuDiffIkParams;
    use crate::CuDiffIkStatus;
    use crate::CuIkParams;
    use crate::CuInverseKinematicsTask;
    use crate::CuJointAxis;
    use crate::CuJointLimits;
    use crate::CuJointPosition;
    use crate::CuJointPositionsPayload;
    use crate::CuJointSpec;
    use crate::CuJointVelocity;
    use crate::CuKinematicChain;
    use crate::CuServoTask;
    use crate::CuTwist6D;
    use crate::CuTwistFrame;
    use crate::igus_rebel_chain;
    use cu_spatial_payloads::Transform3D;
    use cu29::prelude::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::angular_velocity::radian_per_second;
    use cu29::units::si::f64::Angle as Angle64;
    use cu29::units::si::f64::AngularVelocity;
    use cu29::units::si::f64::Length;
    use cu29::units::si::f64::Velocity;
    use cu29::units::si::length::meter;
    use cu29::units::si::velocity::meter_per_second;

    fn m3(v: [f64; 3]) -> [Length; 3] {
        v.map(Length::new::<meter>)
    }
    fn rad3(v: [f64; 3]) -> [Angle64; 3] {
        v.map(Angle64::new::<radian>)
    }
    fn rad_limits(lo: f64, hi: f64, v: f64) -> CuJointLimits {
        CuJointLimits::revolute(
            Angle64::new::<radian>(lo),
            Angle64::new::<radian>(hi),
            AngularVelocity::new::<radian_per_second>(v),
        )
    }
    fn mk_twist(lin: [f64; 3], ang: [f64; 3], frame: CuTwistFrame) -> CuTwist6D {
        CuTwist6D::new(
            lin.map(Velocity::new::<meter_per_second>),
            ang.map(AngularVelocity::new::<radian_per_second>),
            frame,
        )
    }

    #[test]
    fn mixed_joint_units_fixed_slots_and_metric_are_preserved() {
        let chain = CuKinematicChain::new([
            CuJointSpec::revolute(
                m3([0.0; 3]),
                rad3([0.0; 3]),
                CuJointAxis::PosZ,
                rad_limits(-3.0, 3.0, 1.0),
            ),
            CuJointSpec {
                joint_type: crate::CuJointType::Fixed,
                ..Default::default()
            },
            CuJointSpec::prismatic(
                m3([0.0; 3]),
                rad3([0.0; 3]),
                CuJointAxis::Arbitrary([2.0, 0.0, 0.0]),
                CuJointLimits::prismatic(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(1.0),
                    Velocity::new::<meter_per_second>(0.2),
                ),
            ),
        ]);
        let q = [
            Angle64::new::<radian>(0.0).into(),
            CuJointPosition::Fixed,
            cu29::units::si::f64::Length::new::<cu29::units::si::length::meter>(0.2).into(),
        ];
        let mut pose = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q, &mut pose).unwrap();
        assert!((pose.to_matrix()[0][3] - 0.2).abs() < 1e-12);
        let mut jac = [[0.0; 3]; 6];
        chain.geometric_jacobian(&q, &mut jac).unwrap();
        assert_eq!(jac[0][2], 1.0);
        assert!(jac.iter().all(|row| row[1] == 0.0));
        assert_eq!(crate::CuJointMetric::default().normalized(q[2]), 0.2);
        let wrong = [q[0], q[1], q[0]];
        assert!(!chain.is_within_limits(&wrong));
        assert!(chain.forward_kinematics(&wrong, &mut pose).is_err());
    }

    #[test]
    fn test_igus_rebel_zero_pose_fk() {
        let chain = igus_rebel_chain();
        let q_zero = [Angle64::new::<radian>(0.0).into(); 6];
        let mut ee = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q_zero, &mut ee).unwrap();

        let mat = ee.to_matrix();
        // At q = [0; 6], y offsets (+0.0265 and -0.0265, -0.0182 and +0.0182) cancel out.
        // Total height z = 0.1462 + 0.106 + 0.24152 + 0.157479407401356 + 0.142 + 0.0768 = 0.869999407401356 m.
        // Because joint 1 has rpy = [0, 0, pi], the x offset of J_motor4 (+0.00134498...) points in -X in base_link.
        assert!((mat[0][3] - (-0.0013449814287804)).abs() < 1e-12);
        assert!(mat[1][3].abs() < 1e-12);
        assert!((mat[2][3] - 0.869999407401356).abs() < 1e-12);
    }

    #[test]
    fn test_geometric_jacobian_matches_finite_differences() {
        let chain = igus_rebel_chain();
        let q = [
            Angle64::new::<radian>(0.25).into(),
            Angle64::new::<radian>(-0.4).into(),
            Angle64::new::<radian>(0.55).into(),
            Angle64::new::<radian>(-0.3).into(),
            Angle64::new::<radian>(0.45).into(),
            Angle64::new::<radian>(0.15).into(),
        ];

        let mut jac = [[0.0_f64; 6]; 6];
        chain.geometric_jacobian(&q, &mut jac).unwrap();

        let eps = 1e-7;
        for col in 0..6 {
            let mut q_plus = q;
            let mut q_minus = q;
            q_plus[col] = Angle64::new::<radian>(q[col].si_value() + eps).into();
            q_minus[col] = Angle64::new::<radian>(q[col].si_value() - eps).into();

            let mut pose_plus = Transform3D::<f64>::identity();
            let mut pose_minus = Transform3D::<f64>::identity();
            chain.forward_kinematics(&q_plus, &mut pose_plus).unwrap();
            chain.forward_kinematics(&q_minus, &mut pose_minus).unwrap();

            let mp = pose_plus.to_matrix();
            let mm = pose_minus.to_matrix();
            for row in 0..3 {
                let fd_vel = (mp[row][3] - mm[row][3]) / (2.0 * eps);
                assert!(
                    (jac[row][col] - fd_vel).abs() < 1e-5,
                    "linear Jacobian mismatch at ({row}, {col}): analytic={} fd={fd_vel}",
                    jac[row][col]
                );
            }
        }
    }

    #[test]
    fn test_body_jacobian_matches_finite_differences() {
        let chain = igus_rebel_chain();
        let q = [
            Angle64::new::<radian>(0.25).into(),
            Angle64::new::<radian>(-0.4).into(),
            Angle64::new::<radian>(0.55).into(),
            Angle64::new::<radian>(-0.3).into(),
            Angle64::new::<radian>(0.45).into(),
            Angle64::new::<radian>(0.15).into(),
        ];

        let mut ee = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q, &mut ee).unwrap();
        let ee_inv = ee.inverse();

        let mut body_jac = [[0.0_f64; 6]; 6];
        chain.body_jacobian(&q, &mut body_jac).unwrap();

        let eps = 1e-7;
        for col in 0..6 {
            let mut q_plus = q;
            let mut q_minus = q;
            q_plus[col] = Angle64::new::<radian>(q[col].si_value() + eps).into();
            q_minus[col] = Angle64::new::<radian>(q[col].si_value() - eps).into();

            let mut pose_plus = Transform3D::<f64>::identity();
            let mut pose_minus = Transform3D::<f64>::identity();
            chain.forward_kinematics(&q_plus, &mut pose_plus).unwrap();
            chain.forward_kinematics(&q_minus, &mut pose_minus).unwrap();

            // Relative transforms in end-effector frame: T_ee^-1 * T(q +- eps)
            let rel_plus = ee_inv.compose(pose_plus).to_matrix();
            let rel_minus = ee_inv.compose(pose_minus).to_matrix();

            for row in 0..3 {
                let fd_vel = (rel_plus[row][3] - rel_minus[row][3]) / (2.0 * eps);
                assert!(
                    (body_jac[row][col] - fd_vel).abs() < 1e-5,
                    "body linear Jacobian mismatch at ({row}, {col}): analytic={} fd={fd_vel}",
                    body_jac[row][col]
                );
            }

            // Angular velocity components from skew-symmetric part of dR_rel / dq
            let wx_fd = ((rel_plus[2][1] - rel_plus[1][2]) - (rel_minus[2][1] - rel_minus[1][2]))
                / (4.0 * eps);
            let wy_fd = ((rel_plus[0][2] - rel_plus[2][0]) - (rel_minus[0][2] - rel_minus[2][0]))
                / (4.0 * eps);
            let wz_fd = ((rel_plus[1][0] - rel_plus[0][1]) - (rel_minus[1][0] - rel_minus[0][1]))
                / (4.0 * eps);

            assert!((body_jac[3][col] - wx_fd).abs() < 1e-5);
            assert!((body_jac[4][col] - wy_fd).abs() < 1e-5);
            assert!((body_jac[5][col] - wz_fd).abs() < 1e-5);
        }
    }

    #[test]
    fn test_differential_ik_tracks_base_and_body_twists() {
        let chain = igus_rebel_chain();
        let q = [
            Angle64::new::<radian>(0.2).into(),
            Angle64::new::<radian>(0.45).into(),
            Angle64::new::<radian>(-0.6).into(),
            Angle64::new::<radian>(0.3).into(),
            Angle64::new::<radian>(-0.5).into(),
            Angle64::new::<radian>(0.2).into(),
        ];

        let cfg = CuDiffIkParams {
            base_damping: 1e-4,
            max_singularity_damping: 0.0,
            singularity_threshold_sq: 0.0,
            limit_margin: Angle64::new::<radian>(0.0),
            linear_limit_margin: Default::default(),
            enforce_velocity_limits: true,
        };

        let twist_base = mk_twist([0.05, -0.03, 0.04], [0.1, -0.05, 0.08], CuTwistFrame::Base);
        let mut qd = [CuJointVelocity::default(); 6];
        let status = chain
            .differential_ik(&q, &twist_base, &cfg, &mut qd)
            .unwrap();
        assert!(matches!(status, CuDiffIkStatus::Nominal { .. }));

        // Verify J_s * qd reproduces twist_base
        let mut jac = [[0.0_f64; 6]; 6];
        chain.geometric_jacobian(&q, &mut jac).unwrap();
        let expected = twist_base.to_raw_array();
        for (r, exp_r) in expected.iter().enumerate() {
            let mut achieved = 0.0;
            for (c, qd_c) in qd.iter().enumerate() {
                achieved += jac[r][c] * qd_c.si_value();
            }
            assert!((achieved - exp_r).abs() < 1e-3);
        }

        // Verify body-frame twist tracking
        let twist_body = mk_twist([0.04, 0.02, -0.03], [-0.05, 0.06, 0.04], CuTwistFrame::Body);
        let mut qd_body = [CuJointVelocity::default(); 6];
        chain
            .differential_ik(&q, &twist_body, &cfg, &mut qd_body)
            .unwrap();

        let mut bjac = [[0.0_f64; 6]; 6];
        chain.body_jacobian(&q, &mut bjac).unwrap();
        let expected_body = twist_body.to_raw_array();
        for (r, exp_r) in expected_body.iter().enumerate() {
            let mut achieved = 0.0;
            for (c, qd_c) in qd_body.iter().enumerate() {
                achieved += bjac[r][c] * qd_c.si_value();
            }
            assert!((achieved - exp_r).abs() < 1e-3);
        }
    }

    #[test]
    fn test_differential_ik_singularity_and_velocity_scaling() {
        let chain = igus_rebel_chain();
        // Zero pose has wrist joint 5 = 0 (wrist singularity!)
        let q_singular = [Angle64::new::<radian>(0.0).into(); 6];
        let w_sq = chain.manipulability_squared(&q_singular).unwrap();
        assert!(w_sq < 1e-6);

        let cfg = CuDiffIkParams::default();
        let twist = mk_twist([0.05, 0.0, 0.0], [0.0, 0.0, 0.0], CuTwistFrame::Base);
        let mut qd = [CuJointVelocity::default(); 6];
        let status = chain
            .differential_ik(&q_singular, &twist, &cfg, &mut qd)
            .unwrap();
        assert!(matches!(status, CuDiffIkStatus::SingularityDamped { .. }));

        // High twist triggers uniform velocity scaling
        let fast_twist = mk_twist([5.0, 5.0, 5.0], [5.0, 5.0, 5.0], CuTwistFrame::Base);
        let status_scaled = chain
            .differential_ik(&q_singular, &fast_twist, &cfg, &mut qd)
            .unwrap();
        assert!(matches!(
            status_scaled,
            CuDiffIkStatus::VelocityScaled { .. }
        ));
        for (j, qd_j) in qd.iter().enumerate() {
            let v = qd_j.si_value().abs();
            let max_v = chain.joints()[j].limits.max_velocity().si_value();
            assert!(v <= max_v + 1e-9);
        }
    }

    #[test]
    fn test_dls_ik_roundtrip_on_igus_rebel() {
        let chain = igus_rebel_chain();
        let q_goal = [
            Angle64::new::<radian>(0.3).into(),
            Angle64::new::<radian>(0.45).into(),
            Angle64::new::<radian>(-0.35).into(),
            Angle64::new::<radian>(0.25).into(),
            Angle64::new::<radian>(-0.4).into(),
            Angle64::new::<radian>(0.5).into(),
        ];

        let mut target_pose = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q_goal, &mut target_pose).unwrap();

        let q_seed = [
            Angle64::new::<radian>(0.1).into(),
            Angle64::new::<radian>(0.2).into(),
            Angle64::new::<radian>(-0.1).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(-0.1).into(),
            Angle64::new::<radian>(0.1).into(),
        ];

        let cfg = CuIkParams::default();
        let mut q_sol = [CuJointPosition::default(); 6];
        let status = chain
            .inverse_kinematics_dls(&target_pose, &q_seed, &cfg, &mut q_sol)
            .unwrap();

        assert!(status.is_converged(), "IK did not converge: {status:?}");
        assert!(chain.is_within_limits(&q_sol));

        let mut solved_pose = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q_sol, &mut solved_pose).unwrap();

        let tgt = target_pose.to_matrix();
        let sol = solved_pose.to_matrix();
        for r in 0..3 {
            assert!((tgt[r][3] - sol[r][3]).abs() < 1e-4);
        }
    }

    #[test]
    fn test_ik_and_servo_tasks() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut ik_task =
            CuInverseKinematicsTask::<6>::from_params(igus_rebel_chain(), CuIkParams::default())
                .unwrap();
        let mut servo_task =
            CuServoTask::<6>::from_params(igus_rebel_chain(), CuDiffIkParams::default()).unwrap();

        let q_goal = [
            Angle64::new::<radian>(0.2).into(),
            Angle64::new::<radian>(0.35).into(),
            Angle64::new::<radian>(-0.3).into(),
            Angle64::new::<radian>(0.15).into(),
            Angle64::new::<radian>(-0.35).into(),
            Angle64::new::<radian>(0.25).into(),
        ];
        let mut target_pose = Transform3D::<f64>::identity();
        ik_task
            .chain()
            .forward_kinematics(&q_goal, &mut target_pose)
            .unwrap();

        let seed_msg = CuMsg::new(Some(CuJointPositionsPayload {
            positions: [Angle64::new::<radian>(0.05).into(); 6],
        }));
        let pose_msg = CuMsg::new(Some(target_pose));
        let mut ik_out = CuMsg::new(None);

        ik_task
            .process(&ctx, &(&pose_msg, &seed_msg), &mut ik_out)
            .unwrap();
        let sol = ik_out.payload().unwrap();
        assert!(sol.status.is_converged());

        let twist_msg = CuMsg::new(Some(mk_twist(
            [0.02, 0.0, -0.01],
            [0.0, 0.05, 0.0],
            CuTwistFrame::Base,
        )));
        let joints_msg = CuMsg::new(Some(CuJointPositionsPayload { positions: q_goal }));
        let mut servo_out = CuMsg::new(None);
        servo_task
            .process(&ctx, &(&twist_msg, &joints_msg), &mut servo_out)
            .unwrap();
        assert!(servo_out.payload().is_some());
    }

    #[test]
    fn test_two_link_planar_arm() {
        let chain = CuKinematicChain::new([
            CuJointSpec::revolute(
                m3([0.0, 0.0, 0.0]),
                rad3([0.0, 0.0, 0.0]),
                CuJointAxis::PosZ,
                rad_limits(-core::f64::consts::PI, core::f64::consts::PI, 1.0),
            ),
            CuJointSpec::revolute(
                m3([1.0, 0.0, 0.0]),
                rad3([0.0, 0.0, 0.0]),
                CuJointAxis::PosZ,
                rad_limits(-core::f64::consts::PI, core::f64::consts::PI, 1.0),
            ),
        ]);
        let mut ee = Transform3D::<f64>::identity();
        chain
            .forward_kinematics(
                &[
                    Angle64::new::<radian>(core::f64::consts::FRAC_PI_2).into(),
                    Angle64::new::<radian>(0.0).into(),
                ],
                &mut ee,
            )
            .unwrap();
        let m = ee.to_matrix();
        assert!(m[0][3].abs() < 1e-12);
        assert!((m[1][3] - 1.0).abs() < 1e-12);
    }
}
