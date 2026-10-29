//! Allocation-free Damped Least-Squares (Levenberg-Marquardt) inverse kinematics solver.

use crate::chain::CuKinematicChain;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle as Angle64;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// Configuration for the Damped Least-Squares (DLS) inverse kinematics solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct CuIkConfig {
    /// Maximum solver iterations per call.
    pub max_iterations: u32,
    /// Position convergence threshold.
    pub position_tolerance: Length64,
    /// Orientation convergence threshold.
    pub orientation_tolerance: Angle64,
    /// Damping factor $\lambda$ stabilizing near-singular configurations.
    pub damping: f64,
    /// Maximum per-iteration joint step magnitude.
    pub max_step: Angle64,
    /// Whether to clamp joint angles to joint limits at each step.
    pub enforce_joint_limits: bool,
}

impl Default for CuIkConfig {
    fn default() -> Self {
        Self {
            max_iterations: 64,
            position_tolerance: Length64::new::<meter>(1e-4),
            orientation_tolerance: Angle64::new::<radian>(1e-3),
            damping: 0.05,
            max_step: Angle64::new::<radian>(0.25),
            enforce_joint_limits: true,
        }
    }
}

/// Outcome of a bounded inverse kinematics solve.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
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
}

impl CuIkStatus {
    /// Returns `true` if the solver converged within tolerances.
    pub const fn is_converged(&self) -> bool {
        matches!(self, Self::Converged { .. })
    }
}

impl<const N: usize> CuKinematicChain<N> {
    /// Solves inverse kinematics in place using Damped Least-Squares (DLS).
    ///
    /// Writes the resulting joint configuration into `out_q` without heap allocation.
    pub fn inverse_kinematics_dls(
        &self,
        target: &Transform3D<f64>,
        seed: &[Angle64; N],
        cfg: &CuIkConfig,
        out_q: &mut [Angle64; N],
    ) -> CuResult<CuIkStatus> {
        if cfg.damping <= 0.0 || !cfg.damping.is_finite() {
            return Err(CuError::from(
                "cu_kinematics: IK damping must be positive and finite",
            ));
        }
        let pos_tol = cfg.position_tolerance.get::<meter>();
        let ori_tol = cfg.orientation_tolerance.get::<radian>();
        let max_step = cfg.max_step.get::<radian>();
        if pos_tol <= 0.0 || ori_tol <= 0.0 || max_step <= 0.0 {
            return Err(CuError::from(
                "cu_kinematics: IK tolerances and max_step must be positive",
            ));
        }

        *out_q = *seed;
        if cfg.enforce_joint_limits {
            self.clamp_to_limits(out_q);
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
                let abs_d = libm::fabs(delta);
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
                let next_rad = out_q[idx].get::<radian>() + dq[idx] * scale;
                out_q[idx] = Angle64::new::<radian>(next_rad);
                idx += 1;
            }

            if cfg.enforce_joint_limits {
                self.clamp_to_limits(out_q);
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
