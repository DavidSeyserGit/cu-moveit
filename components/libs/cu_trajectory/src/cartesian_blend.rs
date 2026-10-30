//! Sampled Cartesian TCP corner blending for serial joint-space paths.
//!
//! Each positive zone trims its incoming and outgoing Cartesian lines and
//! joins them with a quadratic Bézier curve contained in the TCP zone. Tool
//! orientation follows shortest-arc spherical interpolation. IK and discrete
//! joint-space collision checks are applied at every generated point and edge.

use bincode::Decode;
use bincode::Encode;
use cu_collision::CuCollisionWorld;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuIkConfig;
use cu_kinematics::CuIkStatus;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// A sampled joint configuration and whether it is a required stop.
#[derive(Debug, Clone, Copy)]
pub struct CuCartesianPathPoint<const N: usize> {
    pub positions: [CuJointPosition; N],
    /// True at the start, end, and every zero-radius interior waypoint.
    pub stop: bool,
    /// Index of the waypoint whose TCP zone contains this point, when blending.
    pub zone: Option<usize>,
}
impl<const N: usize> Default for CuCartesianPathPoint<N> {
    fn default() -> Self {
        Self {
            positions: [CuJointPosition::default(); N],
            stop: false,
            zone: None,
        }
    }
}

/// Fixed-capacity scratch path. Only `points[..len]` are valid.
#[derive(Debug, Clone)]
pub struct CuCartesianBlendPath<const N: usize, const CAP: usize> {
    pub points: [CuCartesianPathPoint<N>; CAP],
    pub len: usize,
}
impl<const N: usize, const CAP: usize> Default for CuCartesianBlendPath<N, CAP> {
    fn default() -> Self {
        Self {
            points: [CuCartesianPathPoint::default(); CAP],
            len: 0,
        }
    }
}
impl<const N: usize, const CAP: usize> CuCartesianBlendPath<N, CAP> {
    pub fn points(&self) -> &[CuCartesianPathPoint<N>] {
        &self.points[..self.len]
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

/// Sampling bounds and IK configuration used during Cartesian path construction.
#[derive(Debug, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuCartesianBlendConfig {
    /// Maximum TCP translation between Cartesian targets.
    pub translation_step: Length,
    /// Maximum tool rotation between Cartesian targets.
    pub rotation_step: Angle,
    /// Joint interpolation resolution for collision checks between IK solutions.
    pub collision_step: CuJointResolution,
    pub ik: CuIkConfig,
}
impl Default for CuCartesianBlendConfig {
    fn default() -> Self {
        Self {
            translation_step: Length::new::<meter>(0.01),
            rotation_step: Angle::new::<radian>(0.05),
            collision_step: CuJointResolution::default(),
            ik: CuIkConfig::default(),
        }
    }
}

/// Controlled outcome. The output path is cleared on any non-success outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuCartesianBlendStatus {
    Success,
    InvalidInput,
    InverseKinematicsFailed { waypoint: usize },
    Collision { waypoint: usize },
    ZoneExceeded { waypoint: usize },
    CapacityExceeded,
}

#[derive(Clone, Copy)]
struct Pose {
    p: [f64; 3],
    q: [f64; 4],
}
impl Default for Pose {
    fn default() -> Self {
        Self {
            p: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }
}
impl Pose {
    fn from_transform(t: Transform3D<f64>) -> Self {
        let m = t.to_matrix();
        let trace = m[0][0] + m[1][1] + m[2][2];
        let q = if trace > 0.0 {
            let s = libm::sqrt(trace + 1.0) * 2.0;
            [
                s / 4.0,
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
            ]
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = libm::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
            [
                (m[2][1] - m[1][2]) / s,
                s / 4.0,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
            ]
        } else if m[1][1] > m[2][2] {
            let s = libm::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
            [
                (m[0][2] - m[2][0]) / s,
                (m[0][1] + m[1][0]) / s,
                s / 4.0,
                (m[1][2] + m[2][1]) / s,
            ]
        } else {
            let s = libm::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
            [
                (m[1][0] - m[0][1]) / s,
                (m[0][2] + m[2][0]) / s,
                (m[1][2] + m[2][1]) / s,
                s / 4.0,
            ]
        };
        Self {
            p: [m[0][3], m[1][3], m[2][3]],
            q,
        }
    }
    fn transform(self) -> Transform3D<f64> {
        let [w, x, y, z] = self.q;
        Transform3D::<f64>::from_matrix([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
                self.p[0],
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
                self.p[1],
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
                self.p[2],
            ],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }
    fn lerp(self, other: Self, t: f64) -> Self {
        Self {
            p: core::array::from_fn(|i| self.p[i] + (other.p[i] - self.p[i]) * t),
            q: slerp(self.q, other.q, t),
        }
    }
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
}
fn slerp(a: [f64; 4], mut b: [f64; 4], t: f64) -> [f64; 4] {
    let mut dot = (0..4).map(|i| a[i] * b[i]).sum::<f64>();
    if dot < 0.0 {
        b = b.map(|v| -v);
        dot = -dot;
    }
    let (wa, wb) = if dot > 0.9995 {
        (1.0 - t, t)
    } else {
        let theta = libm::acos(dot.clamp(-1.0, 1.0));
        let denom = libm::sin(theta);
        (
            libm::sin((1.0 - t) * theta) / denom,
            libm::sin(t * theta) / denom,
        )
    };
    let mut q = core::array::from_fn(|i| wa * a[i] + wb * b[i]);
    let norm = libm::sqrt(q.iter().map(|v| v * v).sum::<f64>());
    for v in &mut q {
        *v /= norm;
    }
    q
}
fn angle(a: [f64; 4], b: [f64; 4]) -> f64 {
    let dot = (0..4).map(|i| a[i] * b[i]).sum::<f64>();
    2.0 * libm::acos(dot.abs().clamp(0.0, 1.0))
}

struct Builder<'a, const N: usize, const M: usize, const O: usize, const CAP: usize> {
    chain: &'a CuKinematicChain<N>,
    model: &'a CuRobotCollisionModel<N, M>,
    world: Option<&'a CuCollisionWorld<O>>,
    config: &'a CuCartesianBlendConfig,
    out: &'a mut CuCartesianBlendPath<N, CAP>,
}
impl<const N: usize, const M: usize, const O: usize, const CAP: usize> Builder<'_, N, M, O, CAP> {
    fn append(
        &mut self,
        target: Pose,
        exact: Option<[CuJointPosition; N]>,
        stop: bool,
        zone: Option<(usize, [f64; 3], f64)>,
        waypoint: usize,
    ) -> CuResult<CuCartesianBlendStatus> {
        if self.out.len >= CAP {
            return Ok(CuCartesianBlendStatus::CapacityExceeded);
        }
        let previous = self.out.points[self.out.len - 1].positions;
        let mut q = previous;
        if let Some(exact) = exact {
            q = exact;
        } else {
            let status = self.chain.inverse_kinematics_dls(
                &target.transform(),
                &previous,
                &self.config.ik,
                &mut q,
            )?;
            if !matches!(status, CuIkStatus::Converged { .. }) {
                return Ok(CuCartesianBlendStatus::InverseKinematicsFailed { waypoint });
            }
        }
        if !self
            .model
            .is_configuration_collision_free(self.chain, &q, self.world)?
            || !self.model.is_segment_collision_free(
                self.chain,
                &previous,
                &q,
                self.config.collision_step,
                self.world,
            )?
        {
            return Ok(CuCartesianBlendStatus::Collision { waypoint });
        }
        if let Some((index, center, radius)) = zone {
            let mut actual = Transform3D::<f64>::identity();
            self.chain.forward_kinematics(&q, &mut actual)?;
            if distance(Pose::from_transform(actual).p, center) > radius {
                return Ok(CuCartesianBlendStatus::ZoneExceeded { waypoint: index });
            }
        }
        // IK can legitimately return its seed for targets already within tolerance.
        // Coalesce identical samples so they do not create artificial pauses.
        if previous
            .iter()
            .zip(q.iter())
            .all(|(a, b)| (a.si_value() - b.si_value()).abs() <= 1e-12)
        {
            if stop {
                self.out.points[self.out.len - 1].stop = true;
            }
            return Ok(CuCartesianBlendStatus::Success);
        }
        self.out.points[self.out.len] = CuCartesianPathPoint {
            positions: q,
            stop,
            zone: zone.map(|z| z.0),
        };
        self.out.len += 1;
        Ok(CuCartesianBlendStatus::Success)
    }
    fn steps(&self, dist: f64, rotation: f64) -> usize {
        let count = libm::ceil(
            (dist / self.config.translation_step.get::<meter>())
                .max(rotation / self.config.rotation_step.get::<radian>()),
        )
        .max(1.0);
        if !count.is_finite() || count > CAP as f64 {
            CAP.saturating_add(1)
        } else {
            count as usize
        }
    }
    fn line(
        &mut self,
        from: Pose,
        to: Pose,
        exact: Option<[CuJointPosition; N]>,
        stop: bool,
        waypoint: usize,
    ) -> CuResult<CuCartesianBlendStatus> {
        let steps = self.steps(distance(from.p, to.p), angle(from.q, to.q));
        if steps > CAP.saturating_sub(self.out.len) {
            return Ok(CuCartesianBlendStatus::CapacityExceeded);
        }
        for i in 1..=steps {
            let final_point = i == steps;
            let status = self.append(
                from.lerp(to, i as f64 / steps as f64),
                if final_point { exact } else { None },
                stop && final_point,
                None,
                waypoint,
            )?;
            if status != CuCartesianBlendStatus::Success {
                return Ok(status);
            }
        }
        Ok(CuCartesianBlendStatus::Success)
    }
}

/// Construct a Cartesian path through joint waypoints with per-waypoint TCP zones.
///
/// `zones` has one radius per waypoint. First and last must be zero. A zero
/// interior radius is an exact stop; a positive radius is a fly-by zone in
/// meters. Trimming is capped to 49% of each adjacent straight segment to
/// avoid overlapping zones. Degenerate corners and zones too small relative
/// to IK tolerance are rejected. The input joint configurations define TCP
/// position *and orientation* through FK. Collision checking is discrete,
/// not swept-volume continuous collision detection. Callers must validate
/// the final timed interpolation as well as the resulting path.
///
/// All output and scratch storage is const-sized; failure clears the output.
pub fn blend_cartesian_path<const N: usize, const M: usize, const O: usize, const CAP: usize>(
    chain: &CuKinematicChain<N>,
    model: &CuRobotCollisionModel<N, M>,
    world: Option<&CuCollisionWorld<O>>,
    waypoints: &[[CuJointPosition; N]],
    zones: &[Length],
    config: &CuCartesianBlendConfig,
    out: &mut CuCartesianBlendPath<N, CAP>,
) -> CuResult<CuCartesianBlendStatus> {
    out.clear();
    if waypoints.len() < 2
        || waypoints.len() > CAP
        || zones.len() != waypoints.len()
        || zones.first().is_none_or(|r| r.get::<meter>() != 0.0)
        || zones.last().is_none_or(|r| r.get::<meter>() != 0.0)
        || !config.translation_step.get::<meter>().is_finite()
        || config.translation_step.get::<meter>() <= 0.0
        || !config.rotation_step.get::<radian>().is_finite()
        || config.rotation_step.get::<radian>() <= 0.0
        || config.collision_step.metric().validate().is_err()
    {
        return Ok(CuCartesianBlendStatus::InvalidInput);
    }
    let mut poses = [Pose::default(); CAP];
    for i in 0..waypoints.len() {
        let radius = zones[i].get::<meter>();
        if !radius.is_finite() || radius < 0.0 || !chain.is_within_limits(&waypoints[i]) {
            return Ok(CuCartesianBlendStatus::InvalidInput);
        }
        let mut transform = Transform3D::<f64>::identity();
        chain.forward_kinematics(&waypoints[i], &mut transform)?;
        poses[i] = Pose::from_transform(transform);
    }
    if !model.is_configuration_collision_free(chain, &waypoints[0], world)? {
        return Ok(CuCartesianBlendStatus::Collision { waypoint: 0 });
    }
    out.points[0] = CuCartesianPathPoint {
        positions: waypoints[0],
        stop: true,
        zone: None,
    };
    out.len = 1;
    let result = (|| -> CuResult<CuCartesianBlendStatus> {
        let mut builder = Builder {
            chain,
            model,
            world,
            config,
            out,
        };
        let mut current = poses[0];
        for i in 1..waypoints.len() - 1 {
            let r = zones[i].get::<meter>();
            if r == 0.0 {
                let s = builder.line(current, poses[i], Some(waypoints[i]), true, i)?;
                if s != CuCartesianBlendStatus::Success {
                    return Ok(s);
                }
                current = poses[i];
                continue;
            }
            let before = distance(poses[i - 1].p, poses[i].p);
            let after = distance(poses[i].p, poses[i + 1].p);
            if before <= 1e-12
                || after <= 1e-12
                || r <= 2.0 * config.ik.position_tolerance.get::<meter>()
            {
                return Ok(CuCartesianBlendStatus::InvalidInput);
            }
            let incoming =
                core::array::from_fn::<_, 3, _>(|j| (poses[i].p[j] - poses[i - 1].p[j]) / before);
            let outgoing =
                core::array::from_fn::<_, 3, _>(|j| (poses[i + 1].p[j] - poses[i].p[j]) / after);
            if (0..3).map(|j| incoming[j] * outgoing[j]).sum::<f64>() < -0.9999 {
                return Ok(CuCartesianBlendStatus::InvalidInput);
            }
            let trim = r.min(before * 0.49).min(after * 0.49) * 0.9;
            let entry = poses[i - 1].lerp(poses[i], 1.0 - trim / before);
            let exit = poses[i].lerp(poses[i + 1], trim / after);
            let status = builder.line(current, entry, None, false, i)?;
            if status != CuCartesianBlendStatus::Success {
                return Ok(status);
            }
            let steps = builder.steps(
                2.0 * trim,
                angle(entry.q, poses[i].q) + angle(poses[i].q, exit.q),
            );
            if steps > CAP.saturating_sub(builder.out.len) {
                return Ok(CuCartesianBlendStatus::CapacityExceeded);
            }
            for k in 1..=steps {
                let t = k as f64 / steps as f64;
                let pose = entry.lerp(poses[i], t).lerp(poses[i].lerp(exit, t), t);
                let status = builder.append(pose, None, false, Some((i, poses[i].p, r)), i)?;
                if status != CuCartesianBlendStatus::Success {
                    return Ok(status);
                }
            }
            current = exit;
        }
        let last = waypoints.len() - 1;
        let status = builder.line(current, poses[last], Some(waypoints[last]), true, last)?;
        if status != CuCartesianBlendStatus::Success {
            return Ok(status);
        }
        for (i, zone) in zones.iter().enumerate().take(last).skip(1) {
            if zone.get::<meter>() > 0.0 && !builder.out.points().iter().any(|p| p.zone == Some(i))
            {
                return Ok(CuCartesianBlendStatus::InvalidInput);
            }
        }
        Ok(CuCartesianBlendStatus::Success)
    })();
    if !matches!(result, Ok(CuCartesianBlendStatus::Success)) {
        out.clear();
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuTrajectoryPayload;
    use crate::continuous::CuContinuousAnchors;
    use crate::continuous::CuContinuousConfig;
    use crate::continuous::CuContinuousStatus;
    use crate::continuous::parameterize_cartesian_blend;
    use crate::parameterize::CuJointTrajectoryLimits;
    use crate::parameterize::CuTrajectoryProfile;
    use cu_collision::CuCollisionShape;
    use cu_collision::CuSphere;
    use cu_kinematics::CuJointAxis;
    use cu_kinematics::CuJointLimits;
    use cu_kinematics::CuJointSpec;
    use cu_kinematics::CuJointVelocity;
    use cu29::units::si::f64::Time;
    use cu29::units::si::f64::Velocity;
    use cu29::units::si::time::second;
    use cu29::units::si::velocity::meter_per_second;

    fn robot() -> (CuKinematicChain<3>, CuRobotCollisionModel<3, 1>) {
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
    fn q(x: f64, y: f64) -> [CuJointPosition; 3] {
        [
            Length::new::<meter>(x).into(),
            Length::new::<meter>(y).into(),
            Length::new::<meter>(0.0).into(),
        ]
    }
    fn limits() -> CuJointTrajectoryLimits<3> {
        CuJointTrajectoryLimits::new(
            [CuJointVelocity::Prismatic(Velocity::new::<meter_per_second>(0.4)); 3],
            [1.0; 3],
        )
    }

    #[test]
    fn zone_produces_a_continuous_corner_inside_tcp_radius() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        let world = CuCollisionWorld::<1>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        let mut in_zone = 0;
        for p in path.points() {
            if p.zone == Some(1) {
                in_zone += 1;
                let mut pose = Transform3D::<f64>::identity();
                chain.forward_kinematics(&p.positions, &mut pose).unwrap();
                let m = pose.to_matrix();
                let d = distance([m[0][3], m[1][3], m[2][3]], [0.3, 0.0, 0.0]);
                assert!(d < 0.1 && d > 0.001);
                assert!(!p.stop);
            }
        }
        assert!(in_zone > 2);
        let mut trajectory = CuTrajectoryPayload::<3, 1024>::new();
        assert_eq!(
            parameterize_cartesian_blend(
                &chain,
                &model,
                Some(&world),
                CuContinuousAnchors {
                    positions: &points,
                    zones: &zones
                },
                &path,
                CuContinuousConfig {
                    limits: &limits(),
                    profile: CuTrajectoryProfile::Cubic,
                    blend: &config,
                    sample_period: Time::new::<second>(0.02)
                },
                &mut trajectory
            )
            .unwrap(),
            CuContinuousStatus::Success
        );
        let mut moving_inside = false;
        for p in trajectory.points() {
            let x = p.positions[0].si_value();
            let y = p.positions[1].si_value();
            if distance([x, y, 0.0], [0.3, 0.0, 0.0]) < 0.09 && y > 0.001 {
                moving_inside |= libm::fabs(p.velocities[0].si_value())
                    + libm::fabs(p.velocities[1].si_value())
                    > 1e-5;
            }
        }
        assert!(moving_inside, "robot must not stop inside the fly-by zone");
        for span in trajectory.points().windows(2) {
            let start = span[0].time_from_start.get::<second>();
            let end = span[1].time_from_start.get::<second>();
            for k in 0..=10 {
                let mut pos = q(0.0, 0.0);
                let mut vel = [CuJointVelocity::default(); 3];
                trajectory
                    .sample_cubic(
                        Time::new::<second>(start + (end - start) * k as f64 / 10.0),
                        &mut pos,
                        &mut vel,
                    )
                    .unwrap();
                assert!(vel.iter().all(|v| v.si_value().abs() <= 0.4 + 1e-8));
                if k > 0 {
                    let mut previous = q(0.0, 0.0);
                    let mut previous_vel = [CuJointVelocity::default(); 3];
                    trajectory
                        .sample_cubic(
                            Time::new::<second>(start + (end - start) * (k - 1) as f64 / 10.0),
                            &mut previous,
                            &mut previous_vel,
                        )
                        .unwrap();
                    let dt = (end - start) / 10.0;
                    for j in 0..3 {
                        assert!(
                            ((vel[j].si_value() - previous_vel[j].si_value()) / dt).abs()
                                <= 1.0 + 1e-6
                        );
                    }
                }
            }
        }

        assert_eq!(trajectory.points()[0].positions, points[0]);
        let end = trajectory.points().last().unwrap();
        assert!((end.positions[0].si_value() - 0.3).abs() < 1e-9);
        assert!((end.positions[1].si_value() - 0.3).abs() < 1e-9);
        for p in trajectory
            .points()
            .iter()
            .skip(1)
            .take(trajectory.len() - 2)
        {
            let t = p.time_from_start.get::<second>();
            let mut before_q = q(0.0, 0.0);
            let mut after_q = before_q;
            let mut before_v = [CuJointVelocity::default(); 3];
            let mut after_v = before_v;
            trajectory
                .sample_cubic(Time::new::<second>(t - 1e-7), &mut before_q, &mut before_v)
                .unwrap();
            trajectory
                .sample_cubic(Time::new::<second>(t + 1e-7), &mut after_q, &mut after_v)
                .unwrap();
            for j in 0..3 {
                assert!((before_q[j].si_value() - after_q[j].si_value()).abs() < 1e-5);
                assert!((before_v[j].si_value() - after_v[j].si_value()).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn zero_zone_stops_exactly_and_collision_fails_closed() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zero = [Length::new::<meter>(0.0); 3];
        let mut world = CuCollisionWorld::<1>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zero,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        assert!(
            path.points()
                .iter()
                .any(|p| p.stop && p.positions == points[1])
        );
        let mut trajectory = CuTrajectoryPayload::<3, 1024>::new();
        assert_eq!(
            parameterize_cartesian_blend(
                &chain,
                &model,
                Some(&world),
                CuContinuousAnchors {
                    positions: &points,
                    zones: &zero
                },
                &path,
                CuContinuousConfig {
                    limits: &limits(),
                    profile: CuTrajectoryProfile::Cubic,
                    blend: &config,
                    sample_period: Time::new::<second>(0.02)
                },
                &mut trajectory
            )
            .unwrap(),
            CuContinuousStatus::Success
        );
        assert!(
            trajectory
                .points()
                .iter()
                .any(|p| (p.positions[0].si_value() - 0.3).abs() < 1e-10
                    && p.positions[1].si_value().abs() < 1e-10
                    && p.velocities.iter().all(|v| v.si_value().abs() < 1e-10))
        );
        world
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [0.2775, 0.0225, 0.0],
                0.025,
            )))
            .unwrap();
        let zones = [zero[0], Length::new::<meter>(0.1), zero[2]];
        assert!(matches!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Collision { .. }
        ));
        assert_eq!(path.len, 0);
    }

    #[test]
    fn invalid_and_capacity_cases_leave_no_partial_path() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let world = CuCollisionWorld::<0>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 8>::default();
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::CapacityExceeded
        );
        assert_eq!(path.len, 0);
        let invalid = [zones[0], Length::new::<meter>(-0.1), zones[2]];
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &invalid,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::InvalidInput
        );
        assert_eq!(path.len, 0);
    }
    #[test]
    fn rebel_flyby_is_reachable_and_timed() {
        let chain = cu_kinematics::igus_rebel_chain();
        let model = cu_collision::igus_rebel_collision_model();
        let to_q = |j1: f64| {
            [j1, 0.75, -0.25, 0.0, -0.3, 0.0]
                .map(|v| cu29::units::si::f64::Angle::new::<radian>(v).into())
        };
        let points = [to_q(-0.75), to_q(0.0), to_q(0.75)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.03),
            Length::new::<meter>(0.0),
        ];
        let world = CuCollisionWorld::<0>::new();
        let mut config = CuCartesianBlendConfig::default();
        config.ik.max_iterations = 256;
        let mut path = CuCartesianBlendPath::<6, 256>::default();
        let result = blend_cartesian_path(
            &chain,
            &model,
            Some(&world),
            &points,
            &zones,
            &config,
            &mut path,
        )
        .unwrap();
        assert_eq!(result, CuCartesianBlendStatus::Success);
        let velocity = cu29::units::si::f64::AngularVelocity::new::<
            cu29::units::si::angular_velocity::radian_per_second,
        >(1.0);
        let limits =
            CuJointTrajectoryLimits::new([CuJointVelocity::Revolute(velocity); 6], [2.0; 6]);
        let mut trajectory = CuTrajectoryPayload::<6, 1024>::new();
        let status = parameterize_cartesian_blend(
            &chain,
            &model,
            Some(&world),
            CuContinuousAnchors {
                positions: &points,
                zones: &zones,
            },
            &path,
            CuContinuousConfig {
                limits: &limits,
                profile: CuTrajectoryProfile::Cubic,
                blend: &config,
                sample_period: Time::new::<second>(0.02),
            },
            &mut trajectory,
        )
        .unwrap();
        assert_eq!(status, CuContinuousStatus::Success);
    }
    #[test]
    fn timing_rechecks_collision_and_rejects_a_flyby_that_must_stop() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        let mut world = CuCollisionWorld::<1>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        world
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [0.2775, 0.0225, 0.0],
                0.025,
            )))
            .unwrap();
        let mut out = CuTrajectoryPayload::<3, 1024>::new();
        let status = parameterize_cartesian_blend(
            &chain,
            &model,
            Some(&world),
            CuContinuousAnchors {
                positions: &points,
                zones: &zones,
            },
            &path,
            CuContinuousConfig {
                limits: &limits(),
                profile: CuTrajectoryProfile::Cubic,
                blend: &config,
                sample_period: Time::new::<second>(0.02),
            },
            &mut out,
        )
        .unwrap();
        assert!(matches!(status, CuContinuousStatus::Collision { .. }));
        assert!(out.is_empty());

        world.clear();
        path.len = 3;
        path.points[0] = CuCartesianPathPoint {
            positions: q(0.0, 0.0),
            stop: true,
            zone: None,
        };
        path.points[1] = CuCartesianPathPoint {
            positions: q(0.3, 0.0),
            stop: false,
            zone: Some(1),
        };
        path.points[2] = CuCartesianPathPoint {
            positions: q(0.0, 0.0),
            stop: true,
            zone: None,
        };
        let status = parameterize_cartesian_blend(
            &chain,
            &model,
            Some(&world),
            CuContinuousAnchors {
                positions: &points,
                zones: &zones,
            },
            &path,
            CuContinuousConfig {
                limits: &limits(),
                profile: CuTrajectoryProfile::Cubic,
                blend: &config,
                sample_period: Time::new::<second>(0.02),
            },
            &mut out,
        )
        .unwrap();
        assert_eq!(
            status,
            CuContinuousStatus::NoContinuousMotion { waypoint: 1 }
        );
        assert!(out.is_empty());
    }

    fn timed_with_profile<const CAP: usize, const OUT: usize>(
        chain: &CuKinematicChain<3>,
        model: &CuRobotCollisionModel<3, 1>,
        points: &[[CuJointPosition; 3]],
        zones: &[Length],
        profile: CuTrajectoryProfile,
    ) -> (CuContinuousStatus, CuTrajectoryPayload<3, OUT>) {
        let world = CuCollisionWorld::<0>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, CAP>::default();
        assert_eq!(
            blend_cartesian_path(
                chain,
                model,
                Some(&world),
                points,
                zones,
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        let mut trajectory = CuTrajectoryPayload::<3, OUT>::new();
        let status = parameterize_cartesian_blend(
            chain,
            model,
            Some(&world),
            CuContinuousAnchors {
                positions: points,
                zones,
            },
            &path,
            CuContinuousConfig {
                limits: &limits(),
                blend: &config,
                sample_period: Time::new::<second>(0.02),
                profile,
            },
            &mut trajectory,
        )
        .unwrap();
        (status, trajectory)
    }

    #[test]
    fn minjerk_profile_flies_through_zone() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        let (status, trajectory) = timed_with_profile::<256, 1024>(
            &chain,
            &model,
            &points,
            &zones,
            CuTrajectoryProfile::MinimumJerk,
        );
        assert_eq!(status, CuContinuousStatus::Success);
        let mut moving_inside = false;
        let mut peak = 0.0f64;
        for p in trajectory.points() {
            for v in &p.velocities {
                peak = peak.max(v.si_value().abs());
            }
            let x = p.positions[0].si_value();
            let y = p.positions[1].si_value();
            if distance([x, y, 0.0], [0.3, 0.0, 0.0]) < 0.09 && y > 0.001 {
                moving_inside |= p.velocities.iter().any(|v| v.si_value().abs() > 1e-5);
            }
        }
        assert!(moving_inside, "minimum-jerk must not stop inside the zone");
        assert!(peak <= 0.4 + 1e-8, "velocity limit exceeded: {peak}");
        // Quintic segments have zero endpoint acceleration by construction:
        // finite differences of the evaluator at unit scale stay near zero,
        // and endpoint velocities are honored exactly.
        let (_, v0) = crate::parameterize::quintic_eval(0.0, 0.0, 1.0, 0.2, 0.3, 1.0);
        let (_, v1) = crate::parameterize::quintic_eval(1e-6, 0.0, 1.0, 0.2, 0.3, 1.0);
        assert!(
            ((v1 - v0) / 1e-6).abs() < 1e-3,
            "endpoint acceleration not near zero"
        );
        let (_, w0) = crate::parameterize::quintic_eval(0.0, 0.0, 1.0, 0.2, 0.3, 1.0);
        let (_, w1) = crate::parameterize::quintic_eval(1.0, 0.0, 1.0, 0.2, 0.3, 1.0);
        assert!((w0 - 0.2).abs() < 1e-12 && (w1 - 0.3).abs() < 1e-12);
        let first = &trajectory.points()[0];
        assert!(first.velocities.iter().all(|v| v.si_value().abs() < 1e-12));
        let end = trajectory.points().last().unwrap();
        assert!((end.positions[0].si_value() - 0.3).abs() < 1e-9);
        assert!(end.velocities.iter().all(|v| v.si_value().abs() < 1e-12));
    }

    #[test]
    fn trapezoidal_rejects_positive_zone_with_waypoint() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        let (status, trajectory) = timed_with_profile::<256, 1024>(
            &chain,
            &model,
            &points,
            &zones,
            CuTrajectoryProfile::Trapezoidal,
        );
        assert_eq!(
            status,
            CuContinuousStatus::NoContinuousMotion { waypoint: 1 }
        );
        assert!(trajectory.is_empty());
    }

    #[test]
    fn scurve_rejects_positive_zone_with_waypoint() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
        ];
        let (status, trajectory) = timed_with_profile::<256, 1024>(
            &chain,
            &model,
            &points,
            &zones,
            CuTrajectoryProfile::SCurve,
        );
        assert_eq!(
            status,
            CuContinuousStatus::NoContinuousMotion { waypoint: 1 }
        );
        assert!(trajectory.is_empty());
    }

    #[test]
    fn trapezoidal_times_all_stop_path_with_exact_stops() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let zones = [Length::new::<meter>(0.0); 3];
        let (status, trajectory) = timed_with_profile::<256, 1024>(
            &chain,
            &model,
            &points,
            &zones,
            CuTrajectoryProfile::Trapezoidal,
        );
        assert_eq!(status, CuContinuousStatus::Success);
        // Exact rest at the middle waypoint and the final configuration.
        assert!(
            trajectory
                .points()
                .iter()
                .any(|p| (p.positions[0].si_value() - 0.3).abs() < 1e-9
                    && p.positions[1].si_value().abs() < 1e-9
                    && p.velocities.iter().all(|v| v.si_value().abs() < 1e-9))
        );
        let end = trajectory.points().last().unwrap();
        assert!((end.positions[0].si_value() - 0.3).abs() < 1e-9);
        assert!((end.positions[1].si_value() - 0.3).abs() < 1e-9);
        assert!(end.velocities.iter().all(|v| v.si_value().abs() < 1e-9));
        let mut prev = -1.0;
        for p in trajectory.points() {
            let t = p.time_from_start.get::<second>();
            assert!(t > prev, "times must be strictly increasing");
            prev = t;
            for v in &p.velocities {
                assert!(v.si_value().abs() <= 0.4 + 1e-6);
            }
        }
    }

    #[test]
    fn mixed_stop_and_flyby_zones_on_one_path() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3), q(0.0, 0.3)];
        let zones = [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
        ];
        let (status, trajectory) = timed_with_profile::<256, 2048>(
            &chain,
            &model,
            &points,
            &zones,
            CuTrajectoryProfile::Cubic,
        );
        assert_eq!(status, CuContinuousStatus::Success);
        // Fly through waypoint 1, exact stop at waypoint 2.
        let mut moving_inside = false;
        for p in trajectory.points() {
            let x = p.positions[0].si_value();
            let y = p.positions[1].si_value();
            if distance([x, y, 0.0], [0.3, 0.0, 0.0]) < 0.09 && y > 0.001 {
                moving_inside |= p.velocities.iter().any(|v| v.si_value().abs() > 1e-5);
            }
        }
        assert!(moving_inside, "cubic must not stop inside the zone");
        assert!(
            trajectory
                .points()
                .iter()
                .any(|p| (p.positions[0].si_value() - 0.3).abs() < 1e-9
                    && (p.positions[1].si_value() - 0.3).abs() < 1e-9
                    && p.velocities.iter().all(|v| v.si_value().abs() < 1e-9))
        );
        let end = trajectory.points().last().unwrap();
        assert!(end.positions[0].si_value().abs() < 1e-9);
        assert!((end.positions[1].si_value() - 0.3).abs() < 1e-9);
    }
}
