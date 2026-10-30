//! Fixed-capacity joint-space trajectory payload and allocation-free sampling.
//!
//! [`CuTrajectoryPayload`] stores up to `MAX_POINTS` timestamped joint waypoints.
//! [`CuTrajectoryPayload::sample`] interpolates joint positions and velocities
//! in place into caller-provided buffers without allocating.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod cartesian_blend;
pub mod continuous;
pub mod parameterize;
pub mod serde_arrays;
pub mod task;

pub use task::CuFollowStatus;
pub use task::CuFollowTrajectoryTask;
pub use task::CuTrajectoryFollowOutput;
pub use task::CuTrajectoryInterpolation;

pub use parameterize::CuJointTrajectoryLimits;
pub use parameterize::CuParameterizeStatus;
pub use parameterize::CuTrajectoryProfile;
pub use parameterize::CuTrajectoryTiming;
pub use parameterize::parameterize_trajectory;
pub use parameterize::parameterize_with_profile;

use bincode::Decode;
use bincode::Encode;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Time;
use cu29::units::si::f64::Velocity;
use cu29::units::si::time::second;
use cu29::units::si::velocity::meter_per_second;
use serde::Deserialize;
use serde::Serialize;

/// Maximum number of waypoints in a [`CuTrajectoryPayload`].
pub const MAX_TRAJECTORY_POINTS: usize = 64;

/// Single timestamped joint waypoint with positions and velocities.
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuTrajectoryPoint<const N: usize> {
    /// Elapsed time since trajectory start.
    pub time_from_start: Time,
    #[serde(with = "cu_kinematics::serde_arrays::position_array")]
    pub positions: [CuJointPosition; N],
    #[serde(with = "cu_kinematics::serde_arrays::velocity_array")]
    pub velocities: [CuJointVelocity; N],
}

impl<const N: usize> Default for CuTrajectoryPoint<N> {
    fn default() -> Self {
        Self {
            time_from_start: Time::default(),
            positions: [CuJointPosition::default(); N],
            velocities: [CuJointVelocity::default(); N],
        }
    }
}

/// Fixed-capacity timestamped joint trajectory (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuTrajectoryPayload<const N: usize, const MAX_POINTS: usize = MAX_TRAJECTORY_POINTS> {
    #[serde(with = "crate::serde_arrays::trajectory_point_array")]
    pub points: [CuTrajectoryPoint<N>; MAX_POINTS],
    /// Number of valid waypoints in `points[..len]`.
    pub len: u32,
}

// The bincode derive currently emits invalid impl generics for const defaults.
impl<const N: usize, const MAX_POINTS: usize> Encode for CuTrajectoryPayload<N, MAX_POINTS> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.points, encoder)?;
        Encode::encode(&self.len, encoder)
    }
}
impl<C, const N: usize, const MAX_POINTS: usize> Decode<C> for CuTrajectoryPayload<N, MAX_POINTS> {
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Ok(Self {
            points: Decode::decode(decoder)?,
            len: Decode::decode(decoder)?,
        })
    }
}
impl<'de, C, const N: usize, const MAX_POINTS: usize> bincode::BorrowDecode<'de, C>
    for CuTrajectoryPayload<N, MAX_POINTS>
{
    fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Decode::decode(decoder)
    }
}

impl<const N: usize, const MAX_POINTS: usize> Default for CuTrajectoryPayload<N, MAX_POINTS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const MAX_POINTS: usize> CuTrajectoryPayload<N, MAX_POINTS> {
    /// Creates an empty trajectory.
    pub fn new() -> Self {
        Self {
            points: core::array::from_fn(|_| CuTrajectoryPoint::default()),
            len: 0,
        }
    }

    /// Returns `true` when the trajectory holds no waypoints.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the number of valid waypoints.
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns the last waypoint time, or zero when empty.
    pub fn duration(&self) -> Time {
        if self.is_empty() {
            Time::default()
        } else {
            self.points[self.len as usize - 1].time_from_start
        }
    }

    /// Borrows the valid waypoint prefix.
    pub fn points(&self) -> &[CuTrajectoryPoint<N>] {
        &self.points[..self.len as usize]
    }

    /// Appends a waypoint; times must be strictly increasing.
    pub fn push(&mut self, point: CuTrajectoryPoint<N>) -> CuResult<()> {
        if self.len as usize >= MAX_POINTS {
            return Err(CuError::from("Trajectory is full"));
        }
        if !self.is_empty()
            && point.time_from_start <= self.points[self.len as usize - 1].time_from_start
        {
            return Err(CuError::from(
                "Trajectory times must be strictly increasing",
            ));
        }
        self.points[self.len as usize] = point;
        self.len += 1;
        Ok(())
    }

    /// Drops all waypoints.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Samples joint positions and velocities at `t` in place.
    ///
    /// Linear interpolation between the bracketing waypoints (v1; cubic
    /// interpolation arrives with the time-parameterization slice). `t` is
    /// clamped to `[0, duration]`; sampling before the first or after the last
    /// waypoint holds that endpoint. Errors on an empty trajectory or on
    /// mismatched joint coordinate types.
    pub fn sample(
        &self,
        t: Time,
        out_positions: &mut [CuJointPosition; N],
        out_velocities: &mut [CuJointVelocity; N],
    ) -> CuResult<()> {
        if self.is_empty() {
            return Err(CuError::from("Cannot sample an empty trajectory"));
        }
        if self.len() == 1 {
            *out_positions = self.points[0].positions;
            *out_velocities = self.points[0].velocities;
            return Ok(());
        }
        let pts = self.points();
        let duration = self.duration();
        let zero = Time::default();
        let clamped = if t <= zero {
            zero
        } else if t >= duration {
            duration
        } else {
            t
        };
        let t_s = clamped.get::<second>();
        let last = pts.len() - 1;
        if t_s <= pts[0].time_from_start.get::<second>() {
            *out_positions = pts[0].positions;
            *out_velocities = pts[0].velocities;
            return Ok(());
        }
        if t_s >= pts[last].time_from_start.get::<second>() {
            *out_positions = pts[last].positions;
            *out_velocities = pts[last].velocities;
            return Ok(());
        }
        let mut i = 0;
        while i < last && pts[i + 1].time_from_start.get::<second>() < t_s {
            i += 1;
        }
        let t0 = pts[i].time_from_start.get::<second>();
        let t1 = pts[i + 1].time_from_start.get::<second>();
        let fraction = (t_s - t0) / (t1 - t0);
        let mut j = 0;
        while j < N {
            out_positions[j] =
                pts[i].positions[j].interpolate(pts[i + 1].positions[j], fraction)?;
            out_velocities[j] =
                lerp_velocity(pts[i].velocities[j], pts[i + 1].velocities[j], fraction)?;
            j += 1;
        }
        Ok(())
    }

    /// Samples cubic Hermite interpolation using the stored positions and velocities.
    ///
    /// Use this for trajectories generated by `parameterize_cartesian_blend`;
    /// it preserves the continuous position and velocity at each stored knot.
    pub fn sample_cubic(
        &self,
        t: Time,
        out_positions: &mut [CuJointPosition; N],
        out_velocities: &mut [CuJointVelocity; N],
    ) -> CuResult<()> {
        if self.is_empty() || self.len() > MAX_POINTS || !t.get::<second>().is_finite() {
            return Err(CuError::from("Invalid cubic trajectory or sample time"));
        }
        let pts = self.points();
        let ts = t.get::<second>();
        if ts <= pts[0].time_from_start.get::<second>() {
            *out_positions = pts[0].positions;
            *out_velocities = pts[0].velocities;
            return Ok(());
        }
        let last = pts.len() - 1;
        if ts >= pts[last].time_from_start.get::<second>() {
            *out_positions = pts[last].positions;
            *out_velocities = pts[last].velocities;
            return Ok(());
        }
        let mut i = 0;
        while i < last && pts[i + 1].time_from_start.get::<second>() < ts {
            i += 1;
        }
        let a = &pts[i];
        let b = &pts[i + 1];
        let t0 = a.time_from_start.get::<second>();
        let duration = b.time_from_start.get::<second>() - t0;
        if !duration.is_finite() || duration <= 0.0 {
            return Err(CuError::from("Invalid trajectory segment duration"));
        }
        let u = (ts - t0) / duration;
        for j in 0..N {
            if core::mem::discriminant(&a.positions[j]) != core::mem::discriminant(&b.positions[j])
                || core::mem::discriminant(&a.velocities[j])
                    != core::mem::discriminant(&b.velocities[j])
                || !matches!(
                    (a.positions[j], a.velocities[j]),
                    (CuJointPosition::Revolute(_), CuJointVelocity::Revolute(_))
                        | (CuJointPosition::Prismatic(_), CuJointVelocity::Prismatic(_))
                        | (CuJointPosition::Fixed, CuJointVelocity::Fixed)
                )
            {
                return Err(CuError::from("Trajectory joint kinds disagree"));
            }
            let q0 = a.positions[j].si_value();
            let q1 = b.positions[j].si_value();
            let v0 = a.velocities[j].si_value();
            let v1 = b.velocities[j].si_value();
            if !q0.is_finite() || !q1.is_finite() || !v0.is_finite() || !v1.is_finite() {
                return Err(CuError::from("Non-finite trajectory knot"));
            }
            let d = q1 - q0;
            let aa = v0 * duration;
            let bb = 3.0 * d - 2.0 * aa - v1 * duration;
            let cc = -2.0 * d + aa + v1 * duration;
            out_positions[j] = a.positions[j].with_si_value(q0 + u * (aa + u * (bb + u * cc)));
            out_velocities[j] = CuJointVelocity::from_coordinate(
                a.positions[j],
                (aa + u * (2.0 * bb + 3.0 * cc * u)) / duration,
            );
        }
        Ok(())
    }
}

/// Linearly interpolates joint velocities of the same joint type.
fn lerp_velocity(
    a: CuJointVelocity,
    b: CuJointVelocity,
    fraction: f64,
) -> CuResult<CuJointVelocity> {
    if !fraction.is_finite() {
        return Err(CuError::from("Non-finite interpolation fraction"));
    }
    match (a, b) {
        (CuJointVelocity::Revolute(x), CuJointVelocity::Revolute(y)) => {
            let xv = x.get::<radian_per_second>();
            let yv = y.get::<radian_per_second>();
            Ok(CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(
                xv + fraction * (yv - xv)
            )))
        }
        (CuJointVelocity::Prismatic(x), CuJointVelocity::Prismatic(y)) => {
            let xv = x.get::<meter_per_second>();
            let yv = y.get::<meter_per_second>();
            Ok(CuJointVelocity::Prismatic(
                Velocity::new::<meter_per_second>(xv + fraction * (yv - xv)),
            ))
        }
        (CuJointVelocity::Fixed, CuJointVelocity::Fixed) => Ok(CuJointVelocity::Fixed),
        _ => Err(CuError::from("Mismatched joint velocity types")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::angle::radian;

    fn point(t: f64, q: f64, v: f64) -> CuTrajectoryPoint<1> {
        CuTrajectoryPoint {
            time_from_start: Time::new::<second>(t),
            positions: [CuJointPosition::Revolute(
                cu29::units::si::f64::Angle::new::<radian>(q),
            )],
            velocities: [CuJointVelocity::Revolute(AngularVelocity::new::<
                radian_per_second,
            >(v))],
        }
    }

    fn two_point_traj() -> CuTrajectoryPayload<1> {
        let mut traj = CuTrajectoryPayload::new();
        traj.push(point(0.0, 0.0, 0.0)).unwrap();
        traj.push(point(2.0, 2.0, 4.0)).unwrap();
        traj
    }

    #[test]
    fn test_push_duration_and_points() {
        let traj = two_point_traj();
        assert_eq!(traj.len(), 2);
        assert!(!traj.is_empty());
        assert_eq!(traj.duration().get::<second>(), 2.0);
        assert_eq!(traj.points().len(), 2);
    }

    #[test]
    fn test_push_rejects_full_and_non_monotonic() {
        let mut traj = CuTrajectoryPayload::<1, 2>::new();
        traj.push(point(0.0, 0.0, 0.0)).unwrap();
        traj.push(point(1.0, 1.0, 1.0)).unwrap();
        assert!(traj.push(point(2.0, 2.0, 2.0)).is_err());
        let mut traj = CuTrajectoryPayload::<1>::new();
        traj.push(point(1.0, 0.0, 0.0)).unwrap();
        assert!(traj.push(point(1.0, 1.0, 1.0)).is_err());
        assert!(traj.push(point(0.5, 1.0, 1.0)).is_err());
        assert_eq!(traj.len(), 1);
        traj.clear();
        assert!(traj.is_empty());
        assert_eq!(traj.duration().get::<second>(), 0.0);
    }

    #[test]
    fn test_sample_endpoints_midpoint_and_clamping() {
        let traj = two_point_traj();
        let mut q = [CuJointPosition::default(); 1];
        let mut qd = [CuJointVelocity::default(); 1];
        traj.sample(Time::new::<second>(0.0), &mut q, &mut qd)
            .unwrap();
        assert_eq!(q[0].si_value(), 0.0);
        traj.sample(Time::new::<second>(1.0), &mut q, &mut qd)
            .unwrap();
        assert!((q[0].si_value() - 1.0).abs() < 1e-12);
        assert!((qd[0].si_value() - 2.0).abs() < 1e-12);
        traj.sample(Time::new::<second>(2.0), &mut q, &mut qd)
            .unwrap();
        assert_eq!(q[0].si_value(), 2.0);
        traj.sample(Time::new::<second>(-5.0), &mut q, &mut qd)
            .unwrap();
        assert_eq!(q[0].si_value(), 0.0);
        traj.sample(Time::new::<second>(99.0), &mut q, &mut qd)
            .unwrap();
        assert_eq!(q[0].si_value(), 2.0);
        assert_eq!(qd[0].si_value(), 4.0);
    }

    #[test]
    fn test_sample_empty_errors() {
        let traj = CuTrajectoryPayload::<1>::new();
        let mut q = [CuJointPosition::default(); 1];
        let mut qd = [CuJointVelocity::default(); 1];
        assert!(
            traj.sample(Time::new::<second>(0.0), &mut q, &mut qd)
                .is_err()
        );
    }
}
