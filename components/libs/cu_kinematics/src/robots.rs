//! Kinematic chain presets for standard manipulators.

use crate::chain::CuJointAxis;
use crate::chain::CuJointLimits;
use crate::chain::CuJointSpec;
use crate::chain::CuKinematicChain;

/// Returns the 6-DOF kinematic chain for the **igus ReBeL** cobot.
///
/// Link origins, axes, and joint limits are derived directly from `igus_rebel.urdf`
/// (combining fixed motor-housing offsets with their corresponding revolute joints).
pub fn igus_rebel_chain() -> CuKinematicChain<6> {
    CuKinematicChain::new([
        // J_motor1 (xyz: [0, 0, 0.1462], rpy: [0, 0, pi]) + joint1 (axis: +Z)
        CuJointSpec::revolute(
            [0.0, 0.0, 0.1462],
            [0.0, 0.0, core::f64::consts::PI],
            CuJointAxis::PosZ,
            CuJointLimits::from_radians(-3.1241, 3.1241, 2.16),
        ),
        // J_motor2 (xyz: [0, -0.0225, 0.106]) + joint2 (xyz: [0, 0.049, 0], axis: -Y)
        CuJointSpec::revolute(
            [0.0, 0.0265, 0.106],
            [0.0, 0.0, 0.0],
            CuJointAxis::NegY,
            CuJointLimits::from_radians(-1.3500, 2.4435, 2.16),
        ),
        // J_motor3 (xyz: [0, -0.049, 0.24152]) + joint3 (xyz: [0, 0.0225, 0], axis: -Y)
        CuJointSpec::revolute(
            [0.0, -0.0265, 0.24152],
            [0.0, 0.0, 0.0],
            CuJointAxis::NegY,
            CuJointLimits::from_radians(-1.39626, 2.4000, 2.16),
        ),
        // J_motor4 (xyz: [0.0013449814287804, 0, 0.105479407401356]) + joint4 (xyz: [0, 0, 0.052], axis: +Z)
        CuJointSpec::revolute(
            [0.0013449814287804, 0.0, 0.157479407401356],
            [0.0, 0.0, 0.0],
            CuJointAxis::PosZ,
            CuJointLimits::from_radians(-3.12414, 3.12414, 2.16),
        ),
        // J_motor5 (xyz: [0, -0.0182, 0.142]) + joint5 (xyz: [0, 0.0182, 0], axis: -Y)
        CuJointSpec::revolute(
            [0.0, 0.0, 0.142],
            [0.0, 0.0, 0.0],
            CuJointAxis::NegY,
            CuJointLimits::from_radians(-1.65806, 1.65806, 2.16),
        ),
        // joint6 (xyz: [0, 0, 0.0768], axis: +Z)
        CuJointSpec::revolute(
            [0.0, 0.0, 0.0768],
            [0.0, 0.0, 0.0],
            CuJointAxis::PosZ,
            CuJointLimits::from_radians(-3.12414, 3.12414, 2.16),
        ),
    ])
}
