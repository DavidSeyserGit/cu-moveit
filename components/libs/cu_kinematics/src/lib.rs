//! Deterministic, allocation-free serial robot kinematics (FK, Jacobian, IK) for `copper-rs`.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod chain;
pub mod ik;
pub mod robots;
pub mod task;

pub use chain::CuJointAxis;
pub use chain::CuJointLimits;
pub use chain::CuJointSpec;
pub use chain::CuJointType;
pub use chain::CuKinematicChain;
pub use ik::CuIkConfig;
pub use ik::CuIkStatus;
pub use robots::igus_rebel_chain;
pub use task::CuForwardKinematicsTask;
pub use task::CuJointPositionsPayload;

#[cfg(test)]
mod tests {
    use crate::CuIkConfig;
    use crate::CuJointAxis;
    use crate::CuJointLimits;
    use crate::CuJointSpec;
    use crate::CuKinematicChain;
    use crate::igus_rebel_chain;
    use cu_spatial_payloads::Transform3D;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle as Angle64;

    #[test]
    fn test_igus_rebel_zero_pose_fk() {
        let chain = igus_rebel_chain();
        let q_zero = [Angle64::new::<radian>(0.0); 6];
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
            Angle64::new::<radian>(0.25),
            Angle64::new::<radian>(-0.4),
            Angle64::new::<radian>(0.55),
            Angle64::new::<radian>(-0.3),
            Angle64::new::<radian>(0.45),
            Angle64::new::<radian>(0.15),
        ];

        let mut jac = [[0.0_f64; 6]; 6];
        chain.geometric_jacobian(&q, &mut jac).unwrap();

        let eps = 1e-7;
        for col in 0..6 {
            let mut q_plus = q;
            let mut q_minus = q;
            q_plus[col] = Angle64::new::<radian>(q[col].get::<radian>() + eps);
            q_minus[col] = Angle64::new::<radian>(q[col].get::<radian>() - eps);

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
    fn test_dls_ik_roundtrip_on_igus_rebel() {
        let chain = igus_rebel_chain();
        let q_goal = [
            Angle64::new::<radian>(0.3),
            Angle64::new::<radian>(0.45),
            Angle64::new::<radian>(-0.35),
            Angle64::new::<radian>(0.25),
            Angle64::new::<radian>(-0.4),
            Angle64::new::<radian>(0.5),
        ];

        let mut target_pose = Transform3D::<f64>::identity();
        chain.forward_kinematics(&q_goal, &mut target_pose).unwrap();

        let q_seed = [
            Angle64::new::<radian>(0.1),
            Angle64::new::<radian>(0.2),
            Angle64::new::<radian>(-0.1),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(-0.1),
            Angle64::new::<radian>(0.1),
        ];

        let cfg = CuIkConfig::default();
        let mut q_sol = [Angle64::default(); 6];
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
    fn test_two_link_planar_arm() {
        let chain = CuKinematicChain::new([
            CuJointSpec::revolute(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                CuJointAxis::PosZ,
                CuJointLimits::from_radians(-core::f64::consts::PI, core::f64::consts::PI, 1.0),
            ),
            CuJointSpec::revolute(
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                CuJointAxis::PosZ,
                CuJointLimits::from_radians(-core::f64::consts::PI, core::f64::consts::PI, 1.0),
            ),
        ]);
        let mut ee = Transform3D::<f64>::identity();
        chain
            .forward_kinematics(
                &[
                    Angle64::new::<radian>(core::f64::consts::FRAC_PI_2),
                    Angle64::new::<radian>(0.0),
                ],
                &mut ee,
            )
            .unwrap();
        let m = ee.to_matrix();
        assert!(m[0][3].abs() < 1e-12);
        assert!((m[1][3] - 1.0).abs() < 1e-12);
    }
}
