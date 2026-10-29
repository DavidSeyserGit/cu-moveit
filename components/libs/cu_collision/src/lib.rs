//! Deterministic, allocation-free capsule and sphere robot collision checking for `copper-rs`.
//!
//! # Example
//! ```rust
//! use cu_collision::{CuCollisionReport, igus_rebel_collision_model};
//! use cu_kinematics::igus_rebel_chain;
//! use cu29::units::si::angle::radian;
//! use cu29::units::si::f64::Angle;
//!
//! let chain = igus_rebel_chain();
//! let model = igus_rebel_collision_model();
//! let q_home = [Angle::new::<radian>(0.0); 6];
//! let mut report = CuCollisionReport::<8>::default();
//! model.check_configuration::<8, 8>(&chain, &q_home, None, &mut report).unwrap();
//! assert!(report.is_collision_free());
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod model;
pub mod robots;
pub mod serde_arrays;
pub mod shapes;
pub mod task;

pub use model::CuCollisionContact;
pub use model::CuCollisionReport;
pub use model::CuCollisionWorld;
pub use model::CuContactKind;
pub use model::CuRobotCollisionModel;
pub use robots::igus_rebel_collision_model;
pub use shapes::CuCapsule;
pub use shapes::CuCollisionShape;
pub use shapes::CuSphere;
pub use shapes::capsule_capsule_signed_distance;
pub use shapes::capsule_sphere_signed_distance;
pub use shapes::shape_signed_distance;
pub use shapes::shapes_violate_margin;
pub use shapes::sphere_sphere_signed_distance;
pub use shapes::transform_shape;
pub use task::CuCollisionCheckTask;
pub use task::DEFAULT_MAX_CONTACTS;
pub use task::DEFAULT_MAX_OBSTACLES;

#[cfg(test)]
mod tests {
    use crate::CuCapsule;
    use crate::CuCollisionCheckTask;
    use crate::CuCollisionReport;
    use crate::CuCollisionShape;
    use crate::CuCollisionWorld;
    use crate::CuContactKind;
    use crate::CuSphere;
    use crate::capsule_capsule_signed_distance;
    use crate::capsule_sphere_signed_distance;
    use crate::igus_rebel_collision_model;
    use crate::sphere_sphere_signed_distance;
    use cu_kinematics::CuJointPositionsPayload;
    use cu_kinematics::igus_rebel_chain;
    use cu29::prelude::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle as Angle64;
    use cu29::units::si::length::meter;

    #[test]
    fn test_sphere_and_capsule_signed_distances() {
        let s1 = CuSphere::from_meters([0.0, 0.0, 0.0], 0.1);
        let s2 = CuSphere::from_meters([0.3, 0.0, 0.0], 0.05);
        let d_ss = sphere_sphere_signed_distance(&s1, &s2).get::<meter>();
        assert!((d_ss - 0.15).abs() < 1e-12);

        let c1 = CuCapsule::from_meters([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.1);
        let s3 = CuSphere::from_meters([0.25, 0.0, 0.5], 0.05);
        let d_cs = capsule_sphere_signed_distance(&c1, &s3).get::<meter>();
        assert!((d_cs - 0.10).abs() < 1e-12);

        // Crossing capsules (perpendicular skew segments at z=0.5, separated by 0.2 in X)
        let c2 = CuCapsule::from_meters([0.2, -0.5, 0.5], [0.2, 0.5, 0.5], 0.12);
        let d_cc = capsule_capsule_signed_distance(&c1, &c2).get::<meter>();
        // Segment distance is 0.2, sum of radii is 0.22 -> penetration of -0.02 m
        assert!((d_cc - (-0.02)).abs() < 1e-12);
    }

    #[test]
    fn test_igus_rebel_home_is_collision_free_and_folded_self_collides() {
        let chain = igus_rebel_chain();
        let model = igus_rebel_collision_model();

        let q_home = [Angle64::new::<radian>(0.0); 6];
        let mut report = CuCollisionReport::<16>::default();
        model
            .check_configuration::<8, 16>(&chain, &q_home, None, &mut report)
            .unwrap();
        assert!(report.is_collision_free());
        assert!(report.min_signed_distance.get::<meter>() > 0.01);

        // Fold upper arm and forearm sharply back into the shoulder/base column
        let q_folded = [
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(1.25),
            Angle64::new::<radian>(2.35),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(1.4),
            Angle64::new::<radian>(0.0),
        ];
        model
            .check_configuration::<8, 16>(&chain, &q_folded, None, &mut report)
            .unwrap();
        assert!(!report.is_collision_free());
        assert_eq!(report.as_slice()[0].kind, CuContactKind::SelfCollision);
    }

    #[test]
    fn test_environment_collision_and_segment_interpolation() {
        let chain = igus_rebel_chain();
        let model = igus_rebel_collision_model();

        let q_start = [Angle64::new::<radian>(0.0); 6];
        let q_end = [
            Angle64::new::<radian>(0.8),
            Angle64::new::<radian>(0.5),
            Angle64::new::<radian>(-0.4),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
            Angle64::new::<radian>(0.0),
        ];

        let mut world = CuCollisionWorld::<8>::new();
        assert!(
            model
                .is_segment_collision_free(
                    &chain,
                    &q_start,
                    &q_end,
                    Angle64::new::<radian>(0.05),
                    Some(&world),
                )
                .unwrap()
        );

        // Place an obstacle sphere right on the end-effector at q_home (z ≈ 0.87 m)
        world
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [0.0, 0.0, 0.87],
                0.08,
            )))
            .unwrap();

        assert!(
            !model
                .is_configuration_collision_free(&chain, &q_start, Some(&world))
                .unwrap()
        );
        assert!(
            !model
                .is_segment_collision_free(
                    &chain,
                    &q_start,
                    &q_end,
                    Angle64::new::<radian>(0.05),
                    Some(&world),
                )
                .unwrap()
        );
    }

    #[test]
    fn test_collision_check_task() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = CuCollisionCheckTask::<6, 8, 8>::new_with_model(
            igus_rebel_chain(),
            igus_rebel_collision_model(),
        );

        let joints_msg = CuMsg::new(Some(CuJointPositionsPayload {
            positions: [Angle64::new::<radian>(0.0); 6],
        }));
        let world_msg = CuMsg::new(Some(CuCollisionWorld::<8>::new()));
        let mut out_msg = CuMsg::new(None);

        task.process(&ctx, &(&joints_msg, &world_msg), &mut out_msg)
            .unwrap();
        let report = out_msg.payload().unwrap();
        assert!(report.is_collision_free());
    }
}
