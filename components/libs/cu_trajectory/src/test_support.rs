//! Shared test fixtures: a 3-axis Cartesian (XYZ prismatic) robot.

use crate::CuCartesianWaypoint;
use crate::CuJointTrajectoryLimits;
use cu_collision::CuCollisionModelParams;
use cu_collision::CuCollisionShape;
use cu_collision::CuLinkShape;
use cu_collision::CuRobotCollisionModel;
use cu_collision::CuSphere;
use cu_kinematics::CuJointAcceleration;
use cu_kinematics::CuJointAxis;
use cu_kinematics::CuJointLimits;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointSpec;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuKinematicChain;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Acceleration;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;

fn m(v: f64) -> Length {
    Length::new::<meter>(v)
}

/// XYZ gantry whose TCP position equals its joint positions.
pub(crate) fn robot() -> (CuKinematicChain<3>, CuRobotCollisionModel<3, 1>) {
    let limits = CuJointLimits::prismatic(m(-1.0), m(1.0), Velocity::new::<meter_per_second>(1.0));
    let origin = [m(0.0); 3];
    let rpy = [Angle::new::<radian>(0.0); 3];
    let chain = CuKinematicChain::new([
        CuJointSpec::prismatic(origin, rpy, CuJointAxis::PosX, limits),
        CuJointSpec::prismatic(origin, rpy, CuJointAxis::PosY, limits),
        CuJointSpec::prismatic(origin, rpy, CuJointAxis::PosZ, limits),
    ]);
    let sphere = CuCollisionShape::Sphere(CuSphere::new([m(0.0); 3], m(0.01)).unwrap());
    let model = CuRobotCollisionModel::new(
        &[CuLinkShape::new(sphere, Some(2))],
        CuCollisionModelParams {
            safety_margin: m(0.0),
            skip_adjacent_links: false,
        },
    )
    .unwrap();
    (chain, model)
}

/// Obstacle sphere next to the corner of the `(0,0) -> (0.3,0) -> (0.3,0.3)` path.
pub(crate) fn obstacle() -> CuCollisionShape {
    CuCollisionShape::Sphere(CuSphere::new([m(0.2775), m(0.0225), m(0.0)], m(0.025)).unwrap())
}

/// Joint configuration with the TCP at `(x, y, 0)` meters.
pub(crate) fn q(x: f64, y: f64) -> [CuJointPosition; 3] {
    [m(x).into(), m(y).into(), m(0.0).into()]
}

/// Waypoints with per-waypoint TCP zone radii in meters.
pub(crate) fn waypoints<const N: usize, const K: usize>(
    points: &[[CuJointPosition; N]; K],
    radii: [f64; K],
) -> [CuCartesianWaypoint<N>; K] {
    core::array::from_fn(|i| CuCartesianWaypoint::new(points[i], m(radii[i])))
}

/// 0.4 m/s and 1 m/s^2 on every axis.
pub(crate) fn limits() -> CuJointTrajectoryLimits<3> {
    CuJointTrajectoryLimits::new(
        [CuJointVelocity::Prismatic(Velocity::new::<meter_per_second>(0.4)); 3],
        [CuJointAcceleration::Prismatic(Acceleration::new::<meter_per_second_squared>(1.0)); 3],
    )
}
