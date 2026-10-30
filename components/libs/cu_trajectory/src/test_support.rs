//! Shared test fixtures: a 3-axis Cartesian (XYZ prismatic) robot.

use crate::parameterize::CuJointTrajectoryLimits;
use cu_collision::CuCollisionShape;
use cu_collision::CuRobotCollisionModel;
use cu_collision::CuSphere;
use cu_kinematics::CuJointAxis;
use cu_kinematics::CuJointLimits;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointSpec;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuKinematicChain;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;

/// XYZ gantry whose TCP position equals its joint positions.
pub(crate) fn robot() -> (CuKinematicChain<3>, CuRobotCollisionModel<3, 1>) {
    let limits = CuJointLimits::from_meters(-1.0, 1.0, 1.0);
    let chain = CuKinematicChain::new([
        CuJointSpec::prismatic([0.0; 3], [0.0; 3], CuJointAxis::PosX, limits),
        CuJointSpec::prismatic([0.0; 3], [0.0; 3], CuJointAxis::PosY, limits),
        CuJointSpec::prismatic([0.0; 3], [0.0; 3], CuJointAxis::PosZ, limits),
    ]);
    let sphere = CuCollisionShape::Sphere(CuSphere::from_meters([0.0; 3], 0.01));
    let model =
        CuRobotCollisionModel::new_attached([sphere], [Some(2)], 1, Length::new::<meter>(0.0))
            .unwrap();
    (chain, model)
}

/// Joint configuration with the TCP at `(x, y, 0)` meters.
pub(crate) fn q(x: f64, y: f64) -> [CuJointPosition; 3] {
    [
        Length::new::<meter>(x).into(),
        Length::new::<meter>(y).into(),
        Length::new::<meter>(0.0).into(),
    ]
}

/// 0.4 m/s and 1 m/s^2 on every axis.
pub(crate) fn limits() -> CuJointTrajectoryLimits<3> {
    CuJointTrajectoryLimits::new(
        [CuJointVelocity::Prismatic(Velocity::new::<meter_per_second>(0.4)); 3],
        [1.0; 3],
    )
}
