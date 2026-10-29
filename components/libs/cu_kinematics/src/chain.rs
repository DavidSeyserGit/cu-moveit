//! Serial kinematic chain definitions, forward kinematics, and geometric Jacobians.

use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle as Angle64;
use cu29::units::si::f64::AngularVelocity as AngularVelocity64;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Principal or arbitrary unit axis of actuation in the joint's local frame.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuJointAxis {
    PosX,
    NegX,
    PosY,
    NegY,
    #[default]
    PosZ,
    NegZ,
    Arbitrary([f64; 3]),
}

impl CuJointAxis {
    /// Returns the unit vector `[x, y, z]` in the local joint frame.
    pub const fn unit_vector(self) -> [f64; 3] {
        match self {
            Self::PosX => [1.0, 0.0, 0.0],
            Self::NegX => [-1.0, 0.0, 0.0],
            Self::PosY => [0.0, 1.0, 0.0],
            Self::NegY => [0.0, -1.0, 0.0],
            Self::PosZ => [0.0, 0.0, 1.0],
            Self::NegZ => [0.0, 0.0, -1.0],
            Self::Arbitrary(v) => v,
        }
    }
}

/// Actuation mechanism of a single joint in a serial chain.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuJointType {
    #[default]
    Revolute,
    Prismatic,
    Fixed,
}

/// Position and velocity bounds for a single joint.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct CuJointLimits {
    pub lower: Angle64,
    pub upper: Angle64,
    pub max_velocity: AngularVelocity64,
}

impl CuJointLimits {
    /// Creates joint limits from SI base values (`radians` and `radians/s`).
    pub fn from_radians(lower_rad: f64, upper_rad: f64, max_vel_rad_s: f64) -> Self {
        Self {
            lower: Angle64::new::<radian>(lower_rad),
            upper: Angle64::new::<radian>(upper_rad),
            max_velocity: AngularVelocity64::new::<radian_per_second>(max_vel_rad_s),
        }
    }

    /// Returns `true` if `angle` lies within `[lower, upper]`.
    pub fn contains(&self, angle: Angle64) -> bool {
        let val = angle.get::<radian>();
        val >= self.lower.get::<radian>() && val <= self.upper.get::<radian>()
    }

    /// Clamps `angle` to `[lower, upper]`.
    pub fn clamp(&self, angle: Angle64) -> Angle64 {
        let val = angle
            .get::<radian>()
            .clamp(self.lower.get::<radian>(), self.upper.get::<radian>());
        Angle64::new::<radian>(val)
    }
}

/// Static kinematic specification of a single joint following URDF conventions.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct CuJointSpec {
    pub joint_type: CuJointType,
    pub origin_translation: [Length64; 3],
    pub origin_rpy: [Angle64; 3],
    pub axis: CuJointAxis,
    pub limits: CuJointLimits,
}

impl CuJointSpec {
    /// Creates a revolute joint from SI base arrays (`meters` and `radians`).
    pub fn revolute(
        xyz_m: [f64; 3],
        rpy_rad: [f64; 3],
        axis: CuJointAxis,
        limits: CuJointLimits,
    ) -> Self {
        Self {
            joint_type: CuJointType::Revolute,
            origin_translation: [
                Length64::new::<meter>(xyz_m[0]),
                Length64::new::<meter>(xyz_m[1]),
                Length64::new::<meter>(xyz_m[2]),
            ],
            origin_rpy: [
                Angle64::new::<radian>(rpy_rad[0]),
                Angle64::new::<radian>(rpy_rad[1]),
                Angle64::new::<radian>(rpy_rad[2]),
            ],
            axis,
            limits,
        }
    }

    /// Creates a prismatic joint from SI base arrays (`meters` and `radians`).
    pub fn prismatic(
        xyz_m: [f64; 3],
        rpy_rad: [f64; 3],
        axis: CuJointAxis,
        limits: CuJointLimits,
    ) -> Self {
        Self {
            joint_type: CuJointType::Prismatic,
            origin_translation: [
                Length64::new::<meter>(xyz_m[0]),
                Length64::new::<meter>(xyz_m[1]),
                Length64::new::<meter>(xyz_m[2]),
            ],
            origin_rpy: [
                Angle64::new::<radian>(rpy_rad[0]),
                Angle64::new::<radian>(rpy_rad[1]),
                Angle64::new::<radian>(rpy_rad[2]),
            ],
            axis,
            limits,
        }
    }

    /// Computes the static parent-to-joint origin transform.
    pub fn origin_transform(&self) -> Transform3D<f64> {
        Transform3D::<f64>::from_translation_euler_xyz(self.origin_translation, self.origin_rpy)
    }

    /// Computes the local motion transform for joint displacement `q`.
    pub fn motion_transform(&self, q: Angle64) -> Transform3D<f64> {
        let val = q.get::<radian>();
        let u = self.axis.unit_vector();
        match self.joint_type {
            CuJointType::Fixed => Transform3D::<f64>::identity(),
            CuJointType::Revolute => axis_angle_transform(u, val),
            CuJointType::Prismatic => Transform3D::<f64>::from_matrix([
                [1.0, 0.0, 0.0, u[0] * val],
                [0.0, 1.0, 0.0, u[1] * val],
                [0.0, 0.0, 1.0, u[2] * val],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        }
    }
}

/// Compile-time sized $N$-DOF serial robot kinematic chain.
///
/// # Example
/// ```rust
/// use cu_kinematics::{CuJointAxis, CuJointLimits, CuJointSpec, CuKinematicChain};
/// use cu_spatial_payloads::Transform3D;
/// use cu29::units::si::angle::radian;
/// use cu29::units::si::f64::Angle;
///
/// let chain = CuKinematicChain::new([
///     CuJointSpec::revolute(
///         [0.0, 0.0, 0.1],
///         [0.0, 0.0, 0.0],
///         CuJointAxis::PosZ,
///         CuJointLimits::from_radians(-core::f64::consts::PI, core::f64::consts::PI, 2.0),
///     ),
/// ]);
/// let mut ee = Transform3D::<f64>::identity();
/// chain.forward_kinematics(&[Angle::new::<radian>(0.0)], &mut ee).unwrap();
/// assert!((ee.to_matrix()[2][3] - 0.1).abs() < 1e-12);
/// ```
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuKinematicChain<const N: usize> {
    #[serde(with = "crate::serde_arrays::joint_spec_array")]
    joints: [CuJointSpec; N],
    #[serde(default = "Transform3D::<f64>::identity")]
    tool_offset: Transform3D<f64>,
}

impl<const N: usize> PartialEq for CuKinematicChain<N> {
    fn eq(&self, other: &Self) -> bool {
        self.joints == other.joints && self.tool_offset.to_matrix() == other.tool_offset.to_matrix()
    }
}

impl<const N: usize> CuKinematicChain<N> {
    /// Creates a serial kinematic chain with an identity tool offset.
    pub const fn new(joints: [CuJointSpec; N]) -> Self {
        Self {
            joints,
            tool_offset: Transform3D::<f64>::identity(),
        }
    }

    /// Creates a serial kinematic chain with an explicit flange-to-tool transform.
    pub const fn new_with_tool(joints: [CuJointSpec; N], tool_offset: Transform3D<f64>) -> Self {
        Self {
            joints,
            tool_offset,
        }
    }

    /// Loads an explicitly configured serial chain. Missing `chain` configuration is an error.
    /// The number of configured joints must match the compile-time joint count `N`.
    pub fn from_config(cfg: Option<&ComponentConfig>) -> CuResult<Self> {
        let cfg = cfg.ok_or_else(|| CuError::from("Missing serial chain configuration"))?;
        let mut chain = cfg
            .get_value::<Self>("chain")?
            .ok_or_else(|| CuError::from("Missing required chain configuration"))?;
        if let Some(tool_offset) = cfg.get_value::<Transform3D<f64>>("tool_offset")? {
            chain.tool_offset = tool_offset;
        }
        Ok(chain)
    }

    /// Borrows the static joint specifications.
    pub const fn joints(&self) -> &[CuJointSpec; N] {
        &self.joints
    }

    /// Borrows the flange-to-tool transform.
    pub const fn tool_offset(&self) -> &Transform3D<f64> {
        &self.tool_offset
    }

    /// Sets the flange-to-tool transform.
    pub fn set_tool_offset(&mut self, tool_offset: Transform3D<f64>) {
        self.tool_offset = tool_offset;
    }

    /// Returns `true` if every joint angle in `q` satisfies its joint limits.
    pub fn is_within_limits(&self, q: &[Angle64; N]) -> bool {
        let mut i = 0;
        while i < N {
            if !self.joints[i].limits.contains(q[i]) {
                return false;
            }
            i += 1;
        }
        true
    }

    /// Clamps each joint angle in `q` in place to its configured limits.
    pub fn clamp_to_limits(&self, q: &mut [Angle64; N]) {
        let mut i = 0;
        while i < N {
            q[i] = self.joints[i].limits.clamp(q[i]);
            i += 1;
        }
    }

    /// Computes the base-to-tool forward kinematics transform in place.
    pub fn forward_kinematics(&self, q: &[Angle64; N], out: &mut Transform3D<f64>) -> CuResult<()> {
        let mut acc = Transform3D::<f64>::identity();
        let mut i = 0;
        while i < N {
            if !q[i].get::<radian>().is_finite() {
                return Err(CuError::from("cu_kinematics: non-finite joint angle"));
            }
            let step = self.joints[i]
                .origin_transform()
                .compose(self.joints[i].motion_transform(q[i]));
            acc = acc.compose(step);
            i += 1;
        }
        *out = acc.compose(self.tool_offset);
        Ok(())
    }

    /// Computes every intermediate link frame (`0..N`) and the final end-effector frame in place.
    pub fn forward_frames(
        &self,
        q: &[Angle64; N],
        out_frames: &mut [Transform3D<f64>; N],
        out_ee: &mut Transform3D<f64>,
    ) -> CuResult<()> {
        let mut acc = Transform3D::<f64>::identity();
        let mut i = 0;
        while i < N {
            if !q[i].get::<radian>().is_finite() {
                return Err(CuError::from("cu_kinematics: non-finite joint angle"));
            }
            let step = self.joints[i]
                .origin_transform()
                .compose(self.joints[i].motion_transform(q[i]));
            acc = acc.compose(step);
            out_frames[i] = acc;
            i += 1;
        }
        *out_ee = acc.compose(self.tool_offset);
        Ok(())
    }

    /// Computes the $6 \times N$ geometric space Jacobian in the base frame in place.
    ///
    /// Rows `0..3` correspond to linear velocity $[v_x, v_y, v_z]^T$ in $\text{m/s}$ and
    /// rows `3..6` correspond to angular velocity $[\omega_x, \omega_y, \omega_z]^T$ in $\text{rad/s}$,
    /// both measured at the tool origin and expressed in the base frame.
    pub fn geometric_jacobian(
        &self,
        q: &[Angle64; N],
        out_jacobian: &mut [[f64; N]; 6],
    ) -> CuResult<()> {
        let mut ee = Transform3D::<f64>::identity();
        self.geometric_jacobian_and_pose(q, out_jacobian, &mut ee)
    }

    /// Computes the $6 \times N$ geometric body Jacobian in the end-effector (tool) frame in place.
    ///
    /// Rows `0..3` correspond to linear velocity $[v_x, v_y, v_z]^T$ in $\text{m/s}$ and
    /// rows `3..6` correspond to angular velocity $[\omega_x, \omega_y, \omega_z]^T$ in $\text{rad/s}$,
    /// both measured at the tool origin and expressed in the end-effector frame.
    pub fn body_jacobian(
        &self,
        q: &[Angle64; N],
        out_jacobian: &mut [[f64; N]; 6],
    ) -> CuResult<()> {
        let mut space_jac = [[0.0_f64; N]; 6];
        let mut ee = Transform3D::<f64>::identity();
        self.geometric_jacobian_and_pose(q, &mut space_jac, &mut ee)?;

        let ee_mat = ee.to_matrix();
        let mut col = 0;
        while col < N {
            let vx = space_jac[0][col];
            let vy = space_jac[1][col];
            let vz = space_jac[2][col];
            let wx = space_jac[3][col];
            let wy = space_jac[4][col];
            let wz = space_jac[5][col];

            // Multiply by R_ee^T (columns of R_ee become rows of R_ee^T)
            out_jacobian[0][col] = ee_mat[0][0] * vx + ee_mat[1][0] * vy + ee_mat[2][0] * vz;
            out_jacobian[1][col] = ee_mat[0][1] * vx + ee_mat[1][1] * vy + ee_mat[2][1] * vz;
            out_jacobian[2][col] = ee_mat[0][2] * vx + ee_mat[1][2] * vy + ee_mat[2][2] * vz;

            out_jacobian[3][col] = ee_mat[0][0] * wx + ee_mat[1][0] * wy + ee_mat[2][0] * wz;
            out_jacobian[4][col] = ee_mat[0][1] * wx + ee_mat[1][1] * wy + ee_mat[2][1] * wz;
            out_jacobian[5][col] = ee_mat[0][2] * wx + ee_mat[1][2] * wy + ee_mat[2][2] * wz;
            col += 1;
        }
        Ok(())
    }

    pub(crate) fn geometric_jacobian_and_pose(
        &self,
        q: &[Angle64; N],
        out_jacobian: &mut [[f64; N]; 6],
        out_ee: &mut Transform3D<f64>,
    ) -> CuResult<()> {
        let mut joint_origins = [[0.0_f64; 3]; N];
        let mut world_axes = [[0.0_f64; 3]; N];
        let mut acc = Transform3D::<f64>::identity();

        let mut i = 0;
        while i < N {
            if !q[i].get::<radian>().is_finite() {
                return Err(CuError::from("cu_kinematics: non-finite joint angle"));
            }
            let joint_frame = acc.compose(self.joints[i].origin_transform());
            let mat = joint_frame.to_matrix();
            joint_origins[i] = [mat[0][3], mat[1][3], mat[2][3]];

            let local_axis = self.joints[i].axis.unit_vector();
            world_axes[i] = [
                mat[0][0] * local_axis[0] + mat[0][1] * local_axis[1] + mat[0][2] * local_axis[2],
                mat[1][0] * local_axis[0] + mat[1][1] * local_axis[1] + mat[1][2] * local_axis[2],
                mat[2][0] * local_axis[0] + mat[2][1] * local_axis[1] + mat[2][2] * local_axis[2],
            ];

            acc = joint_frame.compose(self.joints[i].motion_transform(q[i]));
            i += 1;
        }

        *out_ee = acc.compose(self.tool_offset);
        let ee_mat = out_ee.to_matrix();
        let p_ee = [ee_mat[0][3], ee_mat[1][3], ee_mat[2][3]];

        let mut col = 0;
        while col < N {
            let w = world_axes[col];
            match self.joints[col].joint_type {
                CuJointType::Revolute => {
                    let r = [
                        p_ee[0] - joint_origins[col][0],
                        p_ee[1] - joint_origins[col][1],
                        p_ee[2] - joint_origins[col][2],
                    ];
                    let v = cross3(w, r);
                    out_jacobian[0][col] = v[0];
                    out_jacobian[1][col] = v[1];
                    out_jacobian[2][col] = v[2];
                    out_jacobian[3][col] = w[0];
                    out_jacobian[4][col] = w[1];
                    out_jacobian[5][col] = w[2];
                }
                CuJointType::Prismatic => {
                    out_jacobian[0][col] = w[0];
                    out_jacobian[1][col] = w[1];
                    out_jacobian[2][col] = w[2];
                    out_jacobian[3][col] = 0.0;
                    out_jacobian[4][col] = 0.0;
                    out_jacobian[5][col] = 0.0;
                }
                CuJointType::Fixed => {
                    out_jacobian[0][col] = 0.0;
                    out_jacobian[1][col] = 0.0;
                    out_jacobian[2][col] = 0.0;
                    out_jacobian[3][col] = 0.0;
                    out_jacobian[4][col] = 0.0;
                    out_jacobian[5][col] = 0.0;
                }
            }
            col += 1;
        }
        Ok(())
    }
}

const fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn axis_angle_transform(axis: [f64; 3], angle_rad: f64) -> Transform3D<f64> {
    let norm_sq = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    if norm_sq <= 1e-15 {
        return Transform3D::<f64>::identity();
    }
    let inv_norm = 1.0 / libm::sqrt(norm_sq);
    let ux = axis[0] * inv_norm;
    let uy = axis[1] * inv_norm;
    let uz = axis[2] * inv_norm;

    let s = libm::sin(angle_rad);
    let c = libm::cos(angle_rad);
    let one_minus_c = 1.0 - c;

    Transform3D::<f64>::from_matrix([
        [
            c + ux * ux * one_minus_c,
            ux * uy * one_minus_c - uz * s,
            ux * uz * one_minus_c + uy * s,
            0.0,
        ],
        [
            uy * ux * one_minus_c + uz * s,
            c + uy * uy * one_minus_c,
            uy * uz * one_minus_c - ux * s,
            0.0,
        ],
        [
            uz * ux * one_minus_c - uy * s,
            uz * uy * one_minus_c + ux * s,
            c + uz * uz * one_minus_c,
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ])
}
