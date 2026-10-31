//! Allocation-free Damped Least-Squares (Levenberg-Marquardt) position/pose inverse kinematics
//! and velocity/twist differential inverse kinematics with singularity and joint-limit damping.

use crate::chain::CuKinematicChain;
use crate::joint::CuJointPosition;
use crate::joint::CuJointVelocity;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle as Angle64;
use cu29::units::si::f64::AngularVelocity as AngularVelocity64;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::f64::Velocity as Velocity64;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use serde::Deserialize;
use serde::Serialize;

/// Parameters of the Damped Least-Squares (DLS) inverse kinematics solver.
///
/// RON (every field optional, SI values):
/// `{"max_iterations": 64, "position_tolerance": 1e-4, "orientation_tolerance": 1e-3,
///   "damping": 0.05, "max_step": 0.25, "max_linear_step": 0.025, "enforce_joint_limits": true}`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuIkParams {
    /// Maximum solver iterations per call.
    pub max_iterations: u32,
    /// Position convergence threshold.
    pub position_tolerance: Length64,
    /// Orientation convergence threshold.
    pub orientation_tolerance: Angle64,
    /// Damping factor $\lambda > 0$ stabilizing near-singular configurations (dimensionless).
    pub damping: f64,
    /// Maximum per-iteration revolute joint step.
    pub max_step: Angle64,
    /// Maximum per-iteration prismatic displacement.
    pub max_linear_step: Length64,
    /// Whether to clamp joint coordinates to joint limits at each step.
    pub enforce_joint_limits: bool,
}

impl Default for CuIkParams {
    fn default() -> Self {
        Self {
            max_iterations: 64,
            position_tolerance: Length64::new::<meter>(1e-4),
            orientation_tolerance: Angle64::new::<radian>(1e-3),
            damping: 0.05,
            max_step: Angle64::new::<radian>(0.25),
            max_linear_step: Length64::new::<meter>(0.025),
            enforce_joint_limits: true,
        }
    }
}

impl CuIkParams {
    /// Rejects non-positive or non-finite damping, tolerances, and step sizes.
    pub fn validate(&self) -> CuResult<()> {
        if self.damping <= 0.0 || !self.damping.is_finite() {
            return Err(CuError::from(
                "cu_kinematics: IK damping must be positive and finite",
            ));
        }
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(self.position_tolerance.get::<meter>())
            || !positive(self.orientation_tolerance.get::<radian>())
            || !positive(self.max_step.get::<radian>())
            || !positive(self.max_linear_step.get::<meter>())
        {
            return Err(CuError::from(
                "cu_kinematics: IK tolerances and max_step must be positive",
            ));
        }
        Ok(())
    }
}

/// Outcome of a bounded inverse kinematics solve.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuIkStatus {
    /// Target pose reached within position and orientation tolerances.
    Converged {
        iterations: u32,
        position_error: Length64,
        orientation_error: Angle64,
    },
    /// Iteration budget exhausted; `out_q` holds the best-so-far configuration.
    MaxIterationsReached {
        iterations: u32,
        position_error: Length64,
        orientation_error: Angle64,
    },
    /// Default uninitialized state prior to any solve.
    #[default]
    Uninitialized,
}

impl CuIkStatus {
    /// Returns `true` if the solver converged within tolerances.
    pub const fn is_converged(&self) -> bool {
        matches!(self, Self::Converged { .. })
    }
}

/// Coordinate frame in which a 6D task-space twist command is expressed.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuTwistFrame {
    /// Twist expressed in the robot base frame (paired with `geometric_jacobian`).
    #[default]
    Base,
    /// Twist expressed in the end-effector tool/body frame (paired with `body_jacobian`).
    Body,
}

/// 6D spatial velocity (linear + angular twist) payload with explicit frame semantics.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuTwist6D {
    /// Linear velocity of the tool origin.
    pub linear: [Velocity64; 3],
    /// Angular velocity.
    pub angular: [AngularVelocity64; 3],
    /// Frame both vectors are expressed in.
    pub frame: CuTwistFrame,
}

impl CuTwist6D {
    /// Creates a twist from linear `[vx, vy, vz]` and angular `[wx, wy, wz]` velocities
    /// expressed in `frame`.
    pub const fn new(
        linear: [Velocity64; 3],
        angular: [AngularVelocity64; 3],
        frame: CuTwistFrame,
    ) -> Self {
        Self {
            linear,
            angular,
            frame,
        }
    }

    /// Extracts the raw 6-vector `[vx, vy, vz, wx, wy, wz]` in `m/s` and `rad/s`.
    pub(crate) fn to_raw_array(self) -> [f64; 6] {
        [
            self.linear[0].get::<meter_per_second>(),
            self.linear[1].get::<meter_per_second>(),
            self.linear[2].get::<meter_per_second>(),
            self.angular[0].get::<radian_per_second>(),
            self.angular[1].get::<radian_per_second>(),
            self.angular[2].get::<radian_per_second>(),
        ]
    }
}

/// Parameters of velocity/twist differential inverse kinematics (Jacobian servoing).
///
/// RON (every field optional, SI values):
/// `{"base_damping": 0.02, "max_singularity_damping": 0.25, "singularity_threshold_sq": 1e-4,
///   "limit_margin": 0.15, "linear_limit_margin": 0.015, "enforce_velocity_limits": true}`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuDiffIkParams {
    /// Baseline damping factor $\lambda_0 > 0$ used far from singularities (dimensionless).
    pub base_damping: f64,
    /// Maximum additional damping $\lambda_{\max} \ge 0$ added at exact singularity (dimensionless).
    pub max_singularity_damping: f64,
    /// Yoshikawa manipulability squared threshold $w_{\text{thresh}}^2$ below which adaptive
    /// singularity damping ramps up smoothly (dimensionless; `0` disables it).
    pub singularity_threshold_sq: f64,
    /// Revolute joint-limit proximity margin within which motion toward a limit is smoothly damped.
    pub limit_margin: Angle64,
    /// Prismatic joint-limit damping distance.
    pub linear_limit_margin: Length64,
    /// Whether to uniformly scale output joint velocities when any joint exceeds `max_velocity`.
    pub enforce_velocity_limits: bool,
}

impl Default for CuDiffIkParams {
    fn default() -> Self {
        Self {
            base_damping: 0.02,
            max_singularity_damping: 0.25,
            singularity_threshold_sq: 1e-4,
            limit_margin: Angle64::new::<radian>(0.15),
            linear_limit_margin: Length64::new::<meter>(0.015),
            enforce_velocity_limits: true,
        }
    }
}

impl CuDiffIkParams {
    /// Rejects invalid damping parameters and negative or non-finite limit margins.
    pub fn validate(&self) -> CuResult<()> {
        if self.base_damping <= 0.0
            || !self.base_damping.is_finite()
            || self.max_singularity_damping < 0.0
            || !self.max_singularity_damping.is_finite()
            || self.singularity_threshold_sq < 0.0
            || !self.singularity_threshold_sq.is_finite()
        {
            return Err(CuError::from(
                "cu_kinematics: differential IK damping parameters must be valid finite numbers",
            ));
        }
        if !self.limit_margin.get::<radian>().is_finite()
            || self.limit_margin.get::<radian>() < 0.0
            || !self.linear_limit_margin.get::<meter>().is_finite()
            || self.linear_limit_margin.get::<meter>() < 0.0
        {
            return Err(CuError::from(
                "Joint-limit damping margins must be finite and nonnegative",
            ));
        }
        Ok(())
    }
}

/// Status reported by a single differential IK velocity solve.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuDiffIkStatus {
    /// Nominal tracking outside the singularity damping zone and within velocity limits.
    Nominal { manipulability_sq: f64 },
    /// Near-singular configuration triggered adaptive manipulability damping.
    SingularityDamped {
        manipulability_sq: f64,
        effective_damping: f64,
    },
    /// Joint velocities were uniformly scaled in `(0.0, 1.0]` to respect joint velocity limits.
    VelocityScaled { manipulability_sq: f64, scale: f64 },
}

impl Default for CuDiffIkStatus {
    fn default() -> Self {
        Self::Nominal {
            manipulability_sq: 0.0,
        }
    }
}

impl<const N: usize> CuKinematicChain<N> {
    /// Computes Yoshikawa's manipulability index squared $w^2 = \max(0, \det(J J^T))$ at `q`.
    pub fn manipulability_squared(&self, q: &[CuJointPosition; N]) -> CuResult<f64> {
        let mut jac = [[0.0_f64; N]; 6];
        self.geometric_jacobian(q, &mut jac)?;
        Ok(manipulability_sq_from_jacobian(&jac))
    }

    /// Computes joint velocities `out_qd` in place for a desired 6D Cartesian twist using
    /// singularity-robust and joint-limit-aware damped least squares.
    pub fn differential_ik(
        &self,
        q: &[CuJointPosition; N],
        twist: &CuTwist6D,
        cfg: &CuDiffIkParams,
        out_qd: &mut [CuJointVelocity; N],
    ) -> CuResult<CuDiffIkStatus> {
        cfg.validate()?;
        let v_raw = twist.to_raw_array();
        let mut idx_v = 0;
        while idx_v < 6 {
            if !v_raw[idx_v].is_finite() {
                return Err(CuError::from(
                    "cu_kinematics: non-finite twist component in differential_ik",
                ));
            }
            idx_v += 1;
        }

        let mut jac = [[0.0_f64; N]; 6];
        match twist.frame {
            CuTwistFrame::Base => self.geometric_jacobian(q, &mut jac)?,
            CuTwistFrame::Body => self.body_jacobian(q, &mut jac)?,
        }

        let w_sq = manipulability_sq_from_jacobian(&jac);
        let (lambda_sq, singularity_damped, effective_damping) =
            if cfg.singularity_threshold_sq > 0.0 && w_sq < cfg.singularity_threshold_sq {
                let ratio = 1.0 - (w_sq / cfg.singularity_threshold_sq);
                let l_sq = cfg.base_damping * cfg.base_damping
                    + cfg.max_singularity_damping * cfg.max_singularity_damping * ratio * ratio;
                (l_sq, true, libm::sqrt(l_sq))
            } else {
                (cfg.base_damping * cfg.base_damping, false, cfg.base_damping)
            };

        // Build 6x6 damped normal matrix A = J * J^T + lambda_eff^2 * I
        let mut a = [[0.0_f64; 6]; 6];
        let mut r = 0;
        while r < 6 {
            let mut c = 0;
            while c < 6 {
                let mut sum = 0.0;
                let mut k = 0;
                while k < N {
                    sum += jac[r][k] * jac[c][k];
                    k += 1;
                }
                a[r][c] = sum;
                c += 1;
            }
            a[r][r] += lambda_sq;
            r += 1;
        }

        let y = solve_6x6(&mut a, &v_raw)?;

        // Compute raw joint velocities qd = J^T * y and apply smooth joint-limit margin damping
        // when moving toward a joint limit.

        let joints = self.joints();
        let mut raw_qd = [0.0_f64; N];
        let mut velocity_scale = 1.0_f64;

        let mut j = 0;
        while j < N {
            let mut qd_j = 0.0;
            let mut row = 0;
            while row < 6 {
                qd_j += jac[row][j] * y[row];
                row += 1;
            }

            let margin = match q[j] {
                CuJointPosition::Prismatic(_) => cfg.linear_limit_margin.get::<meter>(),
                _ => cfg.limit_margin.get::<radian>(),
            };
            if margin > 0.0
                && !matches!(
                    joints[j].limits,
                    crate::joint::CuJointLimits::Continuous { .. }
                )
            {
                let q_rad = q[j].si_value();
                let lower = joints[j].limits.lower().si_value();
                let upper = joints[j].limits.upper().si_value();
                if qd_j < 0.0 {
                    let dist_low = (q_rad - lower).max(0.0);
                    if dist_low < margin {
                        let s = dist_low / margin;
                        qd_j *= s * s;
                    }
                } else if qd_j > 0.0 {
                    let dist_high = (upper - q_rad).max(0.0);
                    if dist_high < margin {
                        let s = dist_high / margin;
                        qd_j *= s * s;
                    }
                }
            }

            raw_qd[j] = qd_j;

            if cfg.enforce_velocity_limits {
                let max_v = joints[j].limits.max_velocity().si_value();
                let abs_v = libm::fabs(qd_j);
                if abs_v > max_v {
                    let candidate_scale = max_v / abs_v;
                    if candidate_scale < velocity_scale {
                        velocity_scale = candidate_scale;
                    }
                }
            }
            j += 1;
        }

        let mut i = 0;
        while i < N {
            out_qd[i] = CuJointVelocity::from_coordinate(q[i], raw_qd[i] * velocity_scale);
            i += 1;
        }

        if velocity_scale < 1.0 {
            Ok(CuDiffIkStatus::VelocityScaled {
                manipulability_sq: w_sq,
                scale: velocity_scale,
            })
        } else if singularity_damped {
            Ok(CuDiffIkStatus::SingularityDamped {
                manipulability_sq: w_sq,
                effective_damping,
            })
        } else {
            Ok(CuDiffIkStatus::Nominal {
                manipulability_sq: w_sq,
            })
        }
    }

    /// Solves inverse kinematics in place using Damped Least-Squares (DLS).
    ///
    /// Writes the resulting joint configuration into `out_q` without heap allocation.
    pub fn inverse_kinematics_dls(
        &self,
        target: &Transform3D<f64>,
        seed: &[CuJointPosition; N],
        cfg: &CuIkParams,
        out_q: &mut [CuJointPosition; N],
    ) -> CuResult<CuIkStatus> {
        cfg.validate()?;
        let pos_tol = cfg.position_tolerance.get::<meter>();
        let ori_tol = cfg.orientation_tolerance.get::<radian>();
        let max_step = cfg.max_step.get::<radian>();

        for (joint, &position) in self.joints().iter().zip(seed) {
            joint.validate_coordinate(position)?;
        }
        if target.to_matrix().iter().flatten().any(|v| !v.is_finite()) {
            return Err(CuError::from("Non-finite IK target"));
        }
        *out_q = *seed;
        if cfg.enforce_joint_limits {
            self.clamp_to_limits(out_q)?;
        }

        let mut current_pose = Transform3D::<f64>::identity();
        let mut jac = [[0.0_f64; N]; 6];
        let lambda_sq = cfg.damping * cfg.damping;

        let mut iter = 0;
        while iter < cfg.max_iterations {
            self.forward_kinematics(out_q, &mut current_pose)?;
            let (err_vec, pos_err, ori_err) = pose_error_world(&current_pose, target);

            if pos_err <= pos_tol && ori_err <= ori_tol {
                return Ok(CuIkStatus::Converged {
                    iterations: iter,
                    position_error: Length64::new::<meter>(pos_err),
                    orientation_error: Angle64::new::<radian>(ori_err),
                });
            }

            self.geometric_jacobian(out_q, &mut jac)?;

            // Build 6x6 damped normal matrix A = J * J^T + lambda^2 * I
            let mut a = [[0.0_f64; 6]; 6];
            let mut r = 0;
            while r < 6 {
                let mut c = 0;
                while c < 6 {
                    let mut sum = 0.0;
                    let mut k = 0;
                    while k < N {
                        sum += jac[r][k] * jac[c][k];
                        k += 1;
                    }
                    a[r][c] = sum;
                    c += 1;
                }
                a[r][r] += lambda_sq;
                r += 1;
            }

            let y = solve_6x6(&mut a, &err_vec)?;

            // Compute dq = J^T * y and scale if max_step is exceeded
            let mut dq = [0.0_f64; N];
            let mut max_abs_dq = 0.0_f64;
            let mut j = 0;
            while j < N {
                let mut delta = 0.0;
                let mut row = 0;
                while row < 6 {
                    delta += jac[row][j] * y[row];
                    row += 1;
                }
                dq[j] = delta;
                let joint_step = match out_q[j] {
                    CuJointPosition::Prismatic(_) => cfg.max_linear_step.get::<meter>(),
                    _ => max_step,
                };
                let abs_d = libm::fabs(delta) * max_step / joint_step;
                if abs_d > max_abs_dq {
                    max_abs_dq = abs_d;
                }
                j += 1;
            }

            let scale = if max_abs_dq > max_step {
                max_step / max_abs_dq
            } else {
                1.0
            };

            let mut idx = 0;
            while idx < N {
                let next_rad = out_q[idx].si_value() + dq[idx] * scale;
                out_q[idx] = out_q[idx].with_si_value(next_rad);
                idx += 1;
            }

            if cfg.enforce_joint_limits {
                self.clamp_to_limits(out_q)?;
            }

            iter += 1;
        }

        self.forward_kinematics(out_q, &mut current_pose)?;
        let (_, pos_err, ori_err) = pose_error_world(&current_pose, target);
        if pos_err <= pos_tol && ori_err <= ori_tol {
            Ok(CuIkStatus::Converged {
                iterations: cfg.max_iterations,
                position_error: Length64::new::<meter>(pos_err),
                orientation_error: Angle64::new::<radian>(ori_err),
            })
        } else {
            Ok(CuIkStatus::MaxIterationsReached {
                iterations: cfg.max_iterations,
                position_error: Length64::new::<meter>(pos_err),
                orientation_error: Angle64::new::<radian>(ori_err),
            })
        }
    }
}

fn manipulability_sq_from_jacobian<const N: usize>(jac: &[[f64; N]; 6]) -> f64 {
    if N < 6 {
        return 0.0;
    }
    let mut jjt = [[0.0_f64; 6]; 6];
    let mut r = 0;
    while r < 6 {
        let mut c = 0;
        while c < 6 {
            let mut sum = 0.0;
            let mut k = 0;
            while k < N {
                sum += jac[r][k] * jac[c][k];
                k += 1;
            }
            jjt[r][c] = sum;
            c += 1;
        }
        r += 1;
    }
    det_6x6(&mut jjt).max(0.0)
}

fn det_6x6(a: &mut [[f64; 6]; 6]) -> f64 {
    let mut sign = 1.0_f64;
    let mut k = 0;
    while k < 6 {
        let mut pivot_row = k;
        let mut max_val = libm::fabs(a[k][k]);
        let mut i = k + 1;
        while i < 6 {
            let v = libm::fabs(a[i][k]);
            if v > max_val {
                max_val = v;
                pivot_row = i;
            }
            i += 1;
        }

        if max_val < 1e-18 {
            return 0.0;
        }

        if pivot_row != k {
            a.swap(k, pivot_row);
            sign = -sign;
        }

        let pivot = a[k][k];
        let mut row = k + 1;
        while row < 6 {
            let factor = a[row][k] / pivot;
            let mut col = k + 1;
            while col < 6 {
                a[row][col] -= factor * a[k][col];
                col += 1;
            }
            row += 1;
        }
        k += 1;
    }

    let mut det = sign;
    let mut d = 0;
    while d < 6 {
        det *= a[d][d];
        d += 1;
    }
    det
}

/// Computes the 6D task-space error vector `[dp_x, dp_y, dp_z, dw_x, dw_y, dw_z]` in the base frame.
fn pose_error_world(current: &Transform3D<f64>, target: &Transform3D<f64>) -> ([f64; 6], f64, f64) {
    let cur = current.to_matrix();
    let tgt = target.to_matrix();

    let dx = tgt[0][3] - cur[0][3];
    let dy = tgt[1][3] - cur[1][3];
    let dz = tgt[2][3] - cur[2][3];
    let pos_err = libm::sqrt(dx * dx + dy * dy + dz * dz);

    // Relative rotation in base frame: R_err = R_target * R_current^T
    let mut r_err = [[0.0_f64; 3]; 3];
    let mut i = 0;
    while i < 3 {
        let mut j = 0;
        while j < 3 {
            r_err[i][j] = tgt[i][0] * cur[j][0] + tgt[i][1] * cur[j][1] + tgt[i][2] * cur[j][2];
            j += 1;
        }
        i += 1;
    }

    let (rot_vec, ori_err) = rotation_matrix_log(r_err);
    (
        [dx, dy, dz, rot_vec[0], rot_vec[1], rot_vec[2]],
        pos_err,
        ori_err,
    )
}

/// Matrix logarithm $\log(R) \in \mathfrak{so}(3)$ mapped to a 3D rotation vector and angle.
fn rotation_matrix_log(r: [[f64; 3]; 3]) -> ([f64; 3], f64) {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let cos_theta = ((trace - 1.0) * 0.5).clamp(-1.0, 1.0);
    let theta = libm::acos(cos_theta);

    let vx = r[2][1] - r[1][2];
    let vy = r[0][2] - r[2][0];
    let vz = r[1][0] - r[0][1];

    if theta < 1e-7 {
        return ([0.5 * vx, 0.5 * vy, 0.5 * vz], theta);
    }

    let sin_theta = libm::sin(theta);
    if libm::fabs(sin_theta) < 1e-7 {
        // Near pi: extract axis from diagonal entries of (R + I) / 2
        let ax = libm::sqrt((0.5 * (r[0][0] + 1.0)).max(0.0));
        let ay = libm::sqrt((0.5 * (r[1][1] + 1.0)).max(0.0)).copysign(r[0][1]);
        let az = libm::sqrt((0.5 * (r[2][2] + 1.0)).max(0.0)).copysign(r[0][2]);
        return ([ax * theta, ay * theta, az * theta], theta);
    }

    let factor = theta / (2.0 * sin_theta);
    ([vx * factor, vy * factor, vz * factor], theta)
}

/// Solves a $6 \times 6$ linear system $A x = b$ in place using Gaussian elimination with partial pivoting.
fn solve_6x6(a: &mut [[f64; 6]; 6], b: &[f64; 6]) -> CuResult<[f64; 6]> {
    let mut rhs = *b;
    let mut k = 0;
    while k < 6 {
        let mut pivot_row = k;
        let mut max_val = libm::fabs(a[k][k]);
        let mut i = k + 1;
        while i < 6 {
            let v = libm::fabs(a[i][k]);
            if v > max_val {
                max_val = v;
                pivot_row = i;
            }
            i += 1;
        }

        if max_val < 1e-15 {
            return Err(CuError::from(
                "cu_kinematics: singular damped normal matrix",
            ));
        }

        if pivot_row != k {
            a.swap(k, pivot_row);
            rhs.swap(k, pivot_row);
        }

        let pivot = a[k][k];
        let mut row = k + 1;
        while row < 6 {
            let factor = a[row][k] / pivot;
            a[row][k] = 0.0;
            let mut col = k + 1;
            while col < 6 {
                a[row][col] -= factor * a[k][col];
                col += 1;
            }
            rhs[row] -= factor * rhs[k];
            row += 1;
        }
        k += 1;
    }

    let mut x = [0.0_f64; 6];
    let mut idx = 6;
    while idx > 0 {
        idx -= 1;
        let mut sum = rhs[idx];
        let mut col = idx + 1;
        while col < 6 {
            sum -= a[idx][col] * x[col];
            col += 1;
        }
        x[idx] = sum / a[idx][idx];
    }
    Ok(x)
}
