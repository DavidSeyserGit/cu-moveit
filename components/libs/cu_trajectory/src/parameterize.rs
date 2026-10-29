//! Synchronized trapezoidal time-parameterization (v1) for joint-space paths.
//!
//! [`parameterize_trajectory`] converts an untimed waypoint sequence into a
//! timestamped [`CuTrajectoryPayload`] by assigning each path segment a
//! trapezoidal (or triangular) duration under per-joint velocity and
//! acceleration limits, synchronizing joints so every segment ends with zero
//! velocity, then resampling the synchronized profile at a fixed period.

use bincode::Decode;
use bincode::Encode;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;

use crate::CuTrajectoryPayload;

/// Per-joint speed and acceleration limits for time-parameterization.
///
/// `max_velocities` uses the typed joint-velocity representation so revolute
/// (`rad/s`), prismatic (`m/s`), and fixed joints each carry their own unit;
/// the entry kind must match the joint kind of the parameterized waypoints.
/// `max_accelerations` holds plain `f64` values because no scalar acceleration
/// representation exists in `cu_kinematics` yet: entries are in `rad/s^2` for
/// revolute joints and `m/s^2` for prismatic joints, unused for fixed joints.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointTrajectoryLimits<const N: usize> {
    #[serde(with = "cu_kinematics::serde_arrays::velocity_array")]
    pub max_velocities: [CuJointVelocity; N],
    #[serde(with = "crate::serde_arrays::f64_array")]
    pub max_accelerations: [f64; N],
}

impl<const N: usize> CuJointTrajectoryLimits<N> {
    /// Creates limits from per-joint typed velocities and SI accelerations.
    pub fn new(max_velocities: [CuJointVelocity; N], max_accelerations: [f64; N]) -> Self {
        Self {
            max_velocities,
            max_accelerations,
        }
    }
}

/// Outcome of [`parameterize_trajectory`].
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuParameterizeStatus {
    /// Trajectory written to `out`, ending at rest at the final waypoint.
    Success,
    /// No waypoints were provided; `out` is left empty.
    #[default]
    EmptyPath,
    /// A velocity/acceleration limit or the sample period is non-positive,
    /// non-finite, or mismatched to the waypoint joint kinds.
    InvalidLimits,
    /// `out` filled before the resampled trajectory completed.
    CapacityExceeded,
}

impl CuParameterizeStatus {
    /// Returns `true` when the trajectory was fully parameterized.
    pub const fn is_success(&self) -> bool {
        matches!(self, Self::Success)
    }
}

/// Times a waypoint path with synchronized trapezoidal profiles, writing
/// timestamped points into `out` in place without allocating.
///
/// Each segment duration is the maximum over joints of the trapezoidal
/// (triangular when the displacement is too short to reach full speed) time
/// for that joint's displacement under its velocity/acceleration limits, so
/// every joint has zero velocity at each input waypoint. Points are resampled
/// at `sample_period` with per-joint velocities from the synchronized
/// profile; the final point sits exactly at the path end with zero velocity.
/// A single waypoint yields one zero-velocity point at `t = 0`.
pub fn parameterize_trajectory<const N: usize, const MAX_POINTS: usize>(
    waypoints: &[[CuJointPosition; N]],
    limits: &CuJointTrajectoryLimits<N>,
    sample_period: Time,
    out: &mut CuTrajectoryPayload<N, MAX_POINTS>,
) -> CuResult<CuParameterizeStatus> {
    out.clear();
    if waypoints.is_empty() {
        return Ok(CuParameterizeStatus::EmptyPath);
    }
    let dt = sample_period.get::<second>();
    if !dt.is_finite() || dt <= 0.0 {
        return Ok(CuParameterizeStatus::InvalidLimits);
    }
    validate_waypoints(waypoints)?;
    if !limits_valid(waypoints[0], limits) {
        return Ok(CuParameterizeStatus::InvalidLimits);
    }
    if waypoints.len() == 1 {
        return push_rest_point(out, Time::default(), waypoints[0]);
    }

    let mut total = 0.0;
    let mut s = 0;
    while s + 1 < waypoints.len() {
        total += segment_duration(&waypoints[s], &waypoints[s + 1], limits);
        s += 1;
    }
    if total <= 0.0 {
        return push_rest_point(out, Time::default(), waypoints[0]);
    }

    push_sample(out, 0.0, waypoints, limits)?;
    let mut k = 1u64;
    loop {
        let t = k as f64 * dt;
        if t >= total {
            break;
        }
        if !push_sample(out, t, waypoints, limits)? {
            return Ok(CuParameterizeStatus::CapacityExceeded);
        }
        k += 1;
    }
    let last = waypoints[waypoints.len() - 1];
    push_rest_point(out, Time::new::<second>(total), last)
}

/// Checks that all waypoints share joint kinds and hold finite coordinates.
fn validate_waypoints<const N: usize>(waypoints: &[[CuJointPosition; N]]) -> CuResult<()> {
    let mut i = 0;
    while i < waypoints.len() {
        let mut j = 0;
        while j < N {
            if core::mem::discriminant(&waypoints[i][j])
                != core::mem::discriminant(&waypoints[0][j])
                || !waypoints[i][j].si_value().is_finite()
            {
                return Err(CuError::from(
                    "Waypoint joint kinds must match and be finite",
                ));
            }
            j += 1;
        }
        i += 1;
    }
    Ok(())
}

/// Checks limit positivity, finiteness, and kind agreement with the joints.
fn limits_valid<const N: usize>(
    reference: [CuJointPosition; N],
    limits: &CuJointTrajectoryLimits<N>,
) -> bool {
    let mut j = 0;
    while j < N {
        let acc = limits.max_accelerations[j];
        if !acc.is_finite() {
            return false;
        }
        let vel = limits.max_velocities[j];
        if core::mem::discriminant(&vel)
            != core::mem::discriminant(&coordinate_velocity_kind(reference[j]))
        {
            return false;
        }
        match reference[j] {
            CuJointPosition::Fixed => {
                if acc < 0.0 {
                    return false;
                }
            }
            _ => {
                if !vel.si_value().is_finite() || vel.si_value() <= 0.0 || acc <= 0.0 {
                    return false;
                }
            }
        }
        j += 1;
    }
    true
}

/// Maps a joint kind to the matching zero-magnitude velocity variant.
fn coordinate_velocity_kind(q: CuJointPosition) -> CuJointVelocity {
    CuJointVelocity::from_coordinate(q, 0.0)
}

/// Trapezoidal (or triangular) traversal time for one joint displacement.
fn joint_profile_duration(distance: f64, max_vel: f64, max_acc: f64) -> f64 {
    if distance <= 0.0 {
        return 0.0;
    }
    let accel_time = max_vel / max_acc;
    if distance >= max_vel * accel_time {
        distance / max_vel + accel_time
    } else {
        2.0 * libm::sqrt(distance / max_acc)
    }
}

/// Segment duration synchronized across joints (slowest joint dominates).
fn segment_duration<const N: usize>(
    from: &[CuJointPosition; N],
    to: &[CuJointPosition; N],
    limits: &CuJointTrajectoryLimits<N>,
) -> f64 {
    let mut duration = 0.0;
    let mut j = 0;
    while j < N {
        let distance = (to[j].si_value() - from[j].si_value()).abs();
        let leg = joint_profile_duration(
            distance,
            limits.max_velocities[j].si_value(),
            limits.max_accelerations[j],
        );
        if leg > duration {
            duration = leg;
        }
        j += 1;
    }
    duration
}

/// Evaluates a single-joint trapezoidal profile at `offset` seconds.
///
/// Returns `(position_offset, velocity)` relative to the segment start for a
/// non-negative `distance`; both are zero-rate bounded by `max_vel`/`max_acc`.
fn trapezoid_eval(offset: f64, distance: f64, max_vel: f64, max_acc: f64) -> (f64, f64) {
    if distance <= 0.0 || offset <= 0.0 {
        return (0.0, 0.0);
    }
    let accel_time = max_vel / max_acc;
    if distance >= max_vel * accel_time {
        let cruise = distance / max_vel - accel_time;
        let total = distance / max_vel + accel_time;
        let accel_distance = 0.5 * max_vel * accel_time;
        if offset < accel_time {
            (0.5 * max_acc * offset * offset, max_acc * offset)
        } else if offset < accel_time + cruise {
            (accel_distance + max_vel * (offset - accel_time), max_vel)
        } else if offset < total {
            let remaining = total - offset;
            (
                distance - 0.5 * max_acc * remaining * remaining,
                max_acc * remaining,
            )
        } else {
            (distance, 0.0)
        }
    } else {
        let half = libm::sqrt(distance / max_acc);
        let total = 2.0 * half;
        if offset < half {
            (0.5 * max_acc * offset * offset, max_acc * offset)
        } else if offset < total {
            let remaining = total - offset;
            (
                distance - 0.5 * max_acc * remaining * remaining,
                max_acc * remaining,
            )
        } else {
            (distance, 0.0)
        }
    }
}

/// Pushes a zero-velocity waypoint copy; a full buffer maps to `CapacityExceeded`.
fn push_rest_point<const N: usize, const MAX_POINTS: usize>(
    out: &mut CuTrajectoryPayload<N, MAX_POINTS>,
    time_from_start: Time,
    q: [CuJointPosition; N],
) -> CuResult<CuParameterizeStatus> {
    let mut velocities = [CuJointVelocity::Fixed; N];
    let mut j = 0;
    while j < N {
        velocities[j] = CuJointVelocity::from_coordinate(q[j], 0.0);
        j += 1;
    }
    match out.push(crate::CuTrajectoryPoint {
        time_from_start,
        positions: q,
        velocities,
    }) {
        Ok(()) => Ok(CuParameterizeStatus::Success),
        Err(_) => Ok(CuParameterizeStatus::CapacityExceeded),
    }
}

/// Pushes the synchronized-profile state at path time `t`.
///
/// Each joint follows its own trapezoidal profile stretched onto the segment
/// duration (`scale = own_time / segment_time <= 1`), which keeps sampled
/// velocities and accelerations within limits. Returns `Ok(false)` when full.
fn push_sample<const N: usize, const MAX_POINTS: usize>(
    out: &mut CuTrajectoryPayload<N, MAX_POINTS>,
    t: f64,
    waypoints: &[[CuJointPosition; N]],
    limits: &CuJointTrajectoryLimits<N>,
) -> CuResult<bool> {
    let mut seg = 0;
    let mut seg_start = 0.0;
    while seg + 2 < waypoints.len() {
        let duration = segment_duration(&waypoints[seg], &waypoints[seg + 1], limits);
        if t < seg_start + duration {
            break;
        }
        seg_start += duration;
        seg += 1;
    }
    let seg_duration = segment_duration(&waypoints[seg], &waypoints[seg + 1], limits);
    let tau = (t - seg_start).clamp(0.0, seg_duration);

    let from = waypoints[seg];
    let to = waypoints[seg + 1];
    let mut positions = [CuJointPosition::Fixed; N];
    let mut velocities = [CuJointVelocity::Fixed; N];
    let mut j = 0;
    while j < N {
        let start = from[j].si_value();
        let displacement = to[j].si_value() - start;
        let distance = displacement.abs();
        let own_time = joint_profile_duration(
            distance,
            limits.max_velocities[j].si_value(),
            limits.max_accelerations[j],
        );
        let (offset, vel) = if seg_duration > 0.0 && own_time > 0.0 {
            let scale = own_time / seg_duration;
            let (p, v) = trapezoid_eval(
                tau * scale,
                distance,
                limits.max_velocities[j].si_value(),
                limits.max_accelerations[j],
            );
            (p, v * scale)
        } else {
            (0.0, 0.0)
        };
        let sign = if displacement >= 0.0 { 1.0 } else { -1.0 };
        positions[j] = from[j].with_si_value(start + sign * offset);
        velocities[j] = CuJointVelocity::from_coordinate(from[j], sign * vel);
        j += 1;
    }
    match out.push(crate::CuTrajectoryPoint {
        time_from_start: Time::new::<second>(t),
        positions,
        velocities,
    }) {
        Ok(()) => Ok(true),
        Err(_) => Ok(false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::angular_velocity::radian_per_second;
    use cu29::units::si::f64::Angle;
    use cu29::units::si::f64::AngularVelocity;

    fn rev(q: f64) -> CuJointPosition {
        CuJointPosition::Revolute(Angle::new::<radian>(q))
    }

    fn rev_limits(v: f64, a: f64) -> CuJointTrajectoryLimits<1> {
        CuJointTrajectoryLimits::new(
            [CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(v))],
            [a],
        )
    }

    fn max_sampled_velocity<const M: usize>(traj: &CuTrajectoryPayload<1, M>) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut i = 0;
        while i < traj.len() {
            let v = traj.points()[i].velocities[0].si_value();
            lo = lo.min(v);
            hi = hi.max(v);
            i += 1;
        }
        (lo, hi)
    }

    #[test]
    fn test_empty_path_yields_empty_status() {
        let mut out = CuTrajectoryPayload::<1>::new();
        let status = parameterize_trajectory(
            &[],
            &rev_limits(1.0, 1.0),
            Time::new::<second>(0.01),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::EmptyPath);
        assert!(out.is_empty());
    }

    #[test]
    fn test_single_waypoint_is_zero_velocity_point() {
        let mut out = CuTrajectoryPayload::<1>::new();
        let status = parameterize_trajectory(
            &[[rev(0.5)]],
            &rev_limits(1.0, 1.0),
            Time::new::<second>(0.01),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::Success);
        assert_eq!(out.len(), 1);
        assert_eq!(out.points()[0].time_from_start.get::<second>(), 0.0);
        assert_eq!(out.points()[0].positions[0].si_value(), 0.5);
        assert_eq!(out.points()[0].velocities[0].si_value(), 0.0);
    }

    #[test]
    fn test_trapezoidal_duration_and_velocity_limits() {
        let mut out = CuTrajectoryPayload::<1, 512>::new();
        let status = parameterize_trajectory(
            &[[rev(0.0)], [rev(1.0)]],
            &rev_limits(0.5, 1.0),
            Time::new::<second>(0.02),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::Success);
        // Trapezoidal: T = D/v + v/a = 2.0 + 0.5.
        assert!((out.duration().get::<second>() - 2.5).abs() < 1e-9);
        let (lo, hi) = max_sampled_velocity(&out);
        assert!(hi <= 0.5 + 1e-9, "velocity limit exceeded: {hi}");
        assert!(lo >= -1e-9, "unexpected reverse motion: {lo}");
        assert_eq!(out.points()[0].velocities[0].si_value(), 0.0);
        let last = out.len() - 1;
        assert_eq!(out.points()[last].positions[0].si_value(), 1.0);
        assert_eq!(out.points()[last].velocities[0].si_value(), 0.0);
    }

    #[test]
    fn test_triangular_duration_matches_formula() {
        let mut out = CuTrajectoryPayload::<1, 256>::new();
        let status = parameterize_trajectory(
            &[[rev(0.0)], [rev(0.1)]],
            &rev_limits(1.0, 1.0),
            Time::new::<second>(0.005),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::Success);
        // Triangular (D < v^2/a): T = 2*sqrt(D/a).
        let expected = 2.0 * (0.1f64).sqrt();
        assert!((out.duration().get::<second>() - expected).abs() < 1e-9);
        let (_, peak) = max_sampled_velocity(&out);
        assert!((peak - (0.1f64).sqrt()).abs() < 0.02);
    }

    #[test]
    fn test_slow_joint_dominates_synchronized_duration() {
        let slow = CuJointTrajectoryLimits::new(
            [
                CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(2.0)),
                CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(0.25)),
            ],
            [2.0, 0.5],
        );
        let mut out = CuTrajectoryPayload::<2, 512>::new();
        let status = parameterize_trajectory(
            &[[rev(0.0), rev(0.0)], [rev(0.1), rev(1.0)]],
            &slow,
            Time::new::<second>(0.05),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::Success);
        // Slow joint trapezoidal: T = 1/0.25 + 0.25/0.5 = 4.5; fast joint ~0.45.
        assert!((out.duration().get::<second>() - 4.5).abs() < 1e-9);
        let last = out.len() - 1;
        assert!((out.points()[last].positions[0].si_value() - 0.1).abs() < 1e-9);
        assert!((out.points()[last].positions[1].si_value() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_capacity_exceeded_status() {
        let mut out = CuTrajectoryPayload::<1, 3>::new();
        let status = parameterize_trajectory(
            &[[rev(0.0)], [rev(1.0)]],
            &rev_limits(0.5, 1.0),
            Time::new::<second>(0.01),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::CapacityExceeded);
        assert_eq!(out.len(), 3);
    }

    #[test]
    fn test_invalid_limits_rejected() {
        let wps = [[rev(0.0)], [rev(1.0)]];
        let mut out = CuTrajectoryPayload::<1>::new();
        let zero_vel = CuJointTrajectoryLimits::new(
            [CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(0.0))],
            [1.0],
        );
        let status =
            parameterize_trajectory(&wps, &zero_vel, Time::new::<second>(0.01), &mut out).unwrap();
        assert_eq!(status, CuParameterizeStatus::InvalidLimits);
        let neg_acc = rev_limits(1.0, -2.0);
        let status =
            parameterize_trajectory(&wps, &neg_acc, Time::new::<second>(0.01), &mut out).unwrap();
        assert_eq!(status, CuParameterizeStatus::InvalidLimits);
        let status = parameterize_trajectory(
            &wps,
            &rev_limits(1.0, 1.0),
            Time::new::<second>(0.0),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuParameterizeStatus::InvalidLimits);
    }
}
