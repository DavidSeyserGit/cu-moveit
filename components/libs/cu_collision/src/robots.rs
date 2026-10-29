//! Preconfigured collision models for standard manipulators.

use crate::model::CuRobotCollisionModel;
use crate::shapes::CuCapsule;
use crate::shapes::CuCollisionShape;
use crate::shapes::CuSphere;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;

/// Returns the 6-link capsule/sphere collision model for the **igus ReBeL** 6-DOF cobot.
///
/// Each shape is expressed in the local coordinate frame of its corresponding joint frame `0..6`
/// produced by `cu_kinematics::igus_rebel_chain().forward_frames(...)`.
pub fn igus_rebel_collision_model() -> CuRobotCollisionModel<6> {
    let shapes = [
        // Link 0 (shoulder column around joint 1 frame at z = 0.1462 m)
        CuCollisionShape::Capsule(CuCapsule::from_meters(
            [0.0, 0.0, -0.10],
            [0.0, 0.0, 0.04],
            0.052,
        )),
        // Link 1 (upper arm from joint 2 at z = 0.2522 m toward joint 3)
        CuCollisionShape::Capsule(CuCapsule::from_meters(
            [0.0, 0.0, 0.02],
            [0.0, 0.0, 0.20],
            0.045,
        )),
        // Link 2 (elbow housing from joint 3 toward joint 4)
        CuCollisionShape::Capsule(CuCapsule::from_meters(
            [0.0, 0.0, 0.02],
            [0.0, 0.0, 0.12],
            0.042,
        )),
        // Link 3 (forearm tube from joint 4 toward joint 5)
        CuCollisionShape::Capsule(CuCapsule::from_meters(
            [0.0, 0.0, 0.02],
            [0.0, 0.0, 0.11],
            0.038,
        )),
        // Link 4 (wrist housing from joint 5 toward joint 6)
        CuCollisionShape::Capsule(CuCapsule::from_meters(
            [0.0, 0.0, 0.01],
            [0.0, 0.0, 0.05],
            0.034,
        )),
        // Link 5 (tool flange sphere at joint 6)
        CuCollisionShape::Sphere(CuSphere::from_meters([0.0, 0.0, 0.02], 0.032)),
    ];

    CuRobotCollisionModel::new(shapes, Length64::new::<meter>(0.005))
}
