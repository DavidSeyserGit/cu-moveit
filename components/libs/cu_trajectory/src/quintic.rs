//! Hermite reconstruction of stored joint knots and its exact limits.
//!
//! A [`CuTrajectoryPayload`] stores position, velocity and acceleration at
//! every knot. Between two knots the motion is the unique quintic matching all
//! six boundary values, which makes position, velocity *and* acceleration
//! continuous across the whole trajectory, so joint jerk is finite everywhere.
//! The cubic Hermite through positions and velocities alone is the lighter
//! reconstruction used by the cubic follower mode.
//!
//! The peaks of velocity, acceleration and jerk of the quintic are computed in
//! closed form ([`CuTrajectoryPayload::quintic_limit_ratio`]), and the velocity
//! and acceleration peaks of the cubic in [`cubic_limit_ratio`]. Stretching all
//! knot times by a factor `k` ([`CuTrajectoryPayload::stretch_time`]) keeps the shape and scales the peaks by
//! `1/k`, `1/k^2` and `1/k^3`, so a trajectory whose ratio to the limits is
//! `r > 1` satisfies them exactly after stretching by `r`.

use crate::CuJointTrajectoryLimits;
use crate::CuTrajectoryPayload;
use cu_kinematics::CuJointAcceleration;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;

/// Quintic through two knots over duration `t`, in normalized time `u = tau / t`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Quintic {
    q0: f64,
    v0: f64,
    a0: f64,
    t: f64,
    c3: f64,
    c4: f64,
    c5: f64,
}

impl Quintic {
    /// Quintic matching `(q, v, a)` at both ends of a segment of duration `t > 0`.
    pub(crate) fn new(q: [f64; 2], v: [f64; 2], a: [f64; 2], t: f64) -> Self {
        let r0 = q[1] - q[0] - v[0] * t - 0.5 * a[0] * t * t;
        let r1 = (v[1] - v[0] - a[0] * t) * t;
        let r2 = (a[1] - a[0]) * t * t;
        Self {
            q0: q[0],
            v0: v[0],
            a0: a[0],
            t,
            c3: 10.0 * r0 - 4.0 * r1 + 0.5 * r2,
            c4: -15.0 * r0 + 7.0 * r1 - r2,
            c5: 6.0 * r0 - 3.0 * r1 + 0.5 * r2,
        }
    }

    /// Position, velocity and acceleration at `u` in `[0, 1]`.
    pub(crate) fn eval(&self, u: f64) -> (f64, f64, f64) {
        let t = self.t;
        let position = self.q0
            + u * (self.v0 * t
                + u * (0.5 * self.a0 * t * t + u * (self.c3 + u * (self.c4 + u * self.c5))));
        let velocity = self.v0
            + self.a0 * t * u
            + u * u * (3.0 * self.c3 + u * (4.0 * self.c4 + 5.0 * u * self.c5)) / t;
        let acceleration =
            self.a0 + u * (6.0 * self.c3 + u * (12.0 * self.c4 + 20.0 * u * self.c5)) / (t * t);
        (position, velocity, acceleration)
    }

    fn jerk(&self, u: f64) -> f64 {
        (6.0 * self.c3 + u * (24.0 * self.c4 + 60.0 * u * self.c5)) / (self.t * self.t * self.t)
    }

    /// Exact peaks of `(|velocity|, |acceleration|, |jerk|)` over the segment.
    ///
    /// Jerk is quadratic (peak at an end or its vertex), acceleration cubic
    /// (peak where jerk vanishes), velocity quartic (peak where acceleration
    /// vanishes; those roots are bracketed between the jerk roots and bisected).
    pub(crate) fn peaks(&self) -> (f64, f64, f64) {
        let mut jerk = libm::fabs(self.jerk(0.0)).max(libm::fabs(self.jerk(1.0)));
        if libm::fabs(self.c5) > 1e-300 {
            let u = -self.c4 / (5.0 * self.c5);
            if u > 0.0 && u < 1.0 {
                jerk = jerk.max(libm::fabs(self.jerk(u)));
            }
        }
        let turning = unit_roots(10.0 * self.c5, 4.0 * self.c4, self.c3);
        let mut acceleration = libm::fabs(self.a0).max(libm::fabs(self.eval(1.0).2));
        let mut cuts = [0.0_f64; 4];
        let mut count = 1;
        for u in sorted(turning).into_iter().flatten() {
            acceleration = acceleration.max(libm::fabs(self.eval(u).2));
            cuts[count] = u;
            count += 1;
        }
        cuts[count] = 1.0;
        count += 1;
        let mut velocity = libm::fabs(self.v0).max(libm::fabs(self.eval(1.0).1));
        for w in cuts[..count].windows(2) {
            let (mut lo, mut hi) = (w[0], w[1]);
            let (flo, fhi) = (self.eval(lo).2, self.eval(hi).2);
            if flo * fhi < 0.0 {
                for _ in 0..64 {
                    let mid = 0.5 * (lo + hi);
                    if (self.eval(mid).2 < 0.0) == (flo < 0.0) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                velocity = velocity.max(libm::fabs(self.eval(0.5 * (lo + hi)).1));
            }
        }
        (velocity, acceleration, jerk)
    }
}

/// Roots of `a*u^2 + b*u + c` strictly inside `(0, 1)`.
pub(crate) fn unit_roots(a: f64, b: f64, c: f64) -> [Option<f64>; 2] {
    let inside = |u: f64| (u > 0.0 && u < 1.0).then_some(u);
    if libm::fabs(a) <= 1e-300 {
        if libm::fabs(b) <= 1e-300 {
            return [None, None];
        }
        return [inside(-c / b), None];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return [None, None];
    }
    let root = libm::sqrt(disc);
    [
        inside((-b + root) / (2.0 * a)),
        inside((-b - root) / (2.0 * a)),
    ]
}

fn sorted(roots: [Option<f64>; 2]) -> [Option<f64>; 2] {
    match roots {
        [Some(a), Some(b)] if a > b => [Some(b), Some(a)],
        other => other,
    }
}

impl<const N: usize, const CAP: usize> CuTrajectoryPayload<N, CAP> {
    /// Smallest uniform time stretch under which the quintic reconstruction of
    /// this trajectory respects `limits`, as a ratio of peak to limit (dimensionless).
    ///
    /// Returns the largest of `peak_v / v_max`, `sqrt(peak_a / a_max)` and
    /// `cbrt(peak_j / j_max)` over all segments and non-fixed joints (an infinite
    /// jerk limit contributes nothing). A result `<= 1` means the limits already
    /// hold exactly. Returns `None` for fewer than two knots, non-positive
    /// segment durations, non-finite values, joint-kind disagreements or limits
    /// that are not positive.
    pub fn quintic_limit_ratio(&self, limits: &CuJointTrajectoryLimits<N>) -> Option<f64> {
        let trajectory = self;
        if trajectory.len() < 2 || trajectory.len() > CAP {
            return None;
        }
        let mut ratio = 0.0_f64;
        for pair in trajectory.points().windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let t = b.time_from_start.get::<second>() - a.time_from_start.get::<second>();
            if !t.is_finite() || t <= 0.0 {
                return None;
            }
            for j in 0..N {
                if matches!(a.positions[j], CuJointPosition::Fixed) {
                    continue;
                }
                let kinds_agree = matches!(
                    (
                        a.positions[j],
                        b.positions[j],
                        a.velocities[j],
                        b.velocities[j],
                        limits.max_velocities[j]
                    ),
                    (
                        CuJointPosition::Revolute(_),
                        CuJointPosition::Revolute(_),
                        CuJointVelocity::Revolute(_),
                        CuJointVelocity::Revolute(_),
                        CuJointVelocity::Revolute(_)
                    ) | (
                        CuJointPosition::Prismatic(_),
                        CuJointPosition::Prismatic(_),
                        CuJointVelocity::Prismatic(_),
                        CuJointVelocity::Prismatic(_),
                        CuJointVelocity::Prismatic(_)
                    )
                );
                let (vmax, amax, jmax) = (
                    limits.max_velocities[j].si_value(),
                    limits.acc(j),
                    limits.jerk(j),
                );
                if !kinds_agree
                    || vmax.is_nan()
                    || vmax <= 0.0
                    || amax.is_nan()
                    || amax <= 0.0
                    || jmax.is_nan()
                    || jmax <= 0.0
                {
                    return None;
                }
                let quintic = Quintic::new(
                    [a.positions[j].si_value(), b.positions[j].si_value()],
                    [a.velocities[j].si_value(), b.velocities[j].si_value()],
                    [a.accelerations[j].si_value(), b.accelerations[j].si_value()],
                    t,
                );
                let (v, acc, jerk) = quintic.peaks();
                if !v.is_finite() || !acc.is_finite() || !jerk.is_finite() {
                    return None;
                }
                ratio = ratio
                    .max(v / vmax)
                    .max(libm::sqrt(acc / amax))
                    .max(libm::cbrt(jerk / jmax));
            }
        }
        Some(ratio)
    }

    /// Stretches all knot times by `factor` (dimensionless) and rescales
    /// velocities by `1/factor` and accelerations by `1/factor^2`, so the motion
    /// keeps its exact shape and every velocity, acceleration and jerk peak drops
    /// by `1/factor`, `1/factor^2` and `1/factor^3`. `factor` must be finite and
    /// positive.
    pub fn stretch_time(&mut self, factor: f64) -> CuResult<()> {
        if !factor.is_finite() || factor <= 0.0 {
            return Err(CuError::from("Time stretch factor must be positive"));
        }
        let len = self.len();
        for point in &mut self.points[..len] {
            point.time_from_start =
                Time::new::<second>(point.time_from_start.get::<second>() * factor);
            for j in 0..N {
                point.velocities[j] = CuJointVelocity::from_coordinate(
                    point.positions[j],
                    point.velocities[j].si_value() / factor,
                );
                point.accelerations[j] = CuJointAcceleration::from_coordinate(
                    point.positions[j],
                    point.accelerations[j].si_value() / (factor * factor),
                );
            }
        }
        Ok(())
    }
}

/// Exact `(max speed, min speed, max |acceleration|)` of a cubic or quintic
/// Hermite move over duration `t` with endpoint speeds `v0`, `v1`.
///
/// Cubic speed is quadratic (vertex) and acceleration linear (ends). Quintic
/// (zero end accelerations) acceleration is `u·(6a3 + 12a4·u + 20a5·u^2)/T^2`
/// and jerk quadratic, so all critical points are quadratic roots.
pub(crate) fn polynomial_extrema(
    quintic: bool,
    length: f64,
    v0: f64,
    v1: f64,
    t: f64,
) -> (f64, f64, f64) {
    let mut high = v0.max(v1);
    let mut low = v0.min(v1);
    let mut acc = 0.0_f64;
    if quintic {
        let start = length - v0 * t;
        let end = (v1 - v0) * t;
        let a3 = 10.0 * start - 4.0 * end;
        let a4 = -15.0 * start + 7.0 * end;
        let a5 = 6.0 * start - 3.0 * end;
        let speed =
            |u: f64| v0 + (3.0 * a3 * u * u + 4.0 * a4 * u * u * u + 5.0 * a5 * u * u * u * u) / t;
        let accel = |u: f64| (6.0 * a3 * u + 12.0 * a4 * u * u + 20.0 * a5 * u * u * u) / (t * t);
        for u in unit_roots(20.0 * a5, 12.0 * a4, 6.0 * a3)
            .into_iter()
            .flatten()
        {
            high = high.max(speed(u));
            low = low.min(speed(u));
        }
        for u in unit_roots(60.0 * a5, 24.0 * a4, 6.0 * a3)
            .into_iter()
            .flatten()
        {
            acc = acc.max(libm::fabs(accel(u)));
        }
    } else {
        let a = v0 * t;
        let b = 3.0 * length - 2.0 * a - v1 * t;
        let c = -2.0 * length + a + v1 * t;
        if let Some(u) = unit_roots(0.0, 6.0 * c, 2.0 * b)[0] {
            let speed = (a + u * (2.0 * b + 3.0 * c * u)) / t;
            high = high.max(speed);
            low = low.min(speed);
        }
        acc = libm::fabs(2.0 * b).max(libm::fabs(2.0 * b + 6.0 * c)) / (t * t);
    }
    (high, low, acc)
}

/// Smallest uniform time stretch under which the cubic Hermite reconstruction
/// (positions and velocities) of `trajectory` respects the velocity and
/// acceleration limits, as a ratio of peak to limit.
///
/// Returns the largest of `peak_v / v_max` and `sqrt(peak_a / a_max)` over all
/// segments and non-fixed joints. A result `<= 1` means the limits already
/// hold exactly. Returns `None` for fewer than two knots, non-positive
/// segment durations, non-finite values, joint-kind disagreements or limits
/// that are not positive.
pub(crate) fn cubic_limit_ratio<const N: usize, const CAP: usize>(
    trajectory: &CuTrajectoryPayload<N, CAP>,
    limits: &CuJointTrajectoryLimits<N>,
) -> Option<f64> {
    if trajectory.len() < 2 || trajectory.len() > CAP {
        return None;
    }
    let mut ratio = 0.0_f64;
    for pair in trajectory.points().windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let t = b.time_from_start.get::<second>() - a.time_from_start.get::<second>();
        if !t.is_finite() || t <= 0.0 {
            return None;
        }
        for j in 0..N {
            if matches!(a.positions[j], CuJointPosition::Fixed) {
                continue;
            }
            let kinds_agree = matches!(
                (
                    a.positions[j],
                    b.positions[j],
                    a.velocities[j],
                    b.velocities[j],
                    limits.max_velocities[j]
                ),
                (
                    CuJointPosition::Revolute(_),
                    CuJointPosition::Revolute(_),
                    CuJointVelocity::Revolute(_),
                    CuJointVelocity::Revolute(_),
                    CuJointVelocity::Revolute(_)
                ) | (
                    CuJointPosition::Prismatic(_),
                    CuJointPosition::Prismatic(_),
                    CuJointVelocity::Prismatic(_),
                    CuJointVelocity::Prismatic(_),
                    CuJointVelocity::Prismatic(_)
                )
            );
            let (vmax, amax) = (limits.max_velocities[j].si_value(), limits.acc(j));
            if !kinds_agree || vmax.is_nan() || vmax <= 0.0 || amax.is_nan() || amax <= 0.0 {
                return None;
            }
            let (high, low, acc) = polynomial_extrema(
                false,
                b.positions[j].si_value() - a.positions[j].si_value(),
                a.velocities[j].si_value(),
                b.velocities[j].si_value(),
                t,
            );
            if !high.is_finite() || !low.is_finite() || !acc.is_finite() {
                return None;
            }
            ratio = ratio
                .max(high / vmax)
                .max(-low / vmax)
                .max(libm::sqrt(acc / amax));
        }
    }
    Some(ratio)
}
