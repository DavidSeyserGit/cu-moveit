//! Sampled Cartesian TCP corner blending for serial joint-space paths.
//!
//! Each positive zone trims its incoming and outgoing Cartesian lines and
//! joins them with a quadratic Bézier curve contained in the TCP zone. Tool
//! orientation follows shortest-arc spherical interpolation. IK and discrete
//! joint-space collision checks are applied at every generated point and edge.
//!
//! The path is geometry only. Time it with any profile through
//! [`parameterize_cartesian_path`](crate::path_timing::parameterize_cartesian_path).

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

/// Programmed TCP zone of one waypoint, in the chain base frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CuTcpZone {
    /// Index of the programmed waypoint.
    pub waypoint: usize,
    /// Programmed TCP position of the waypoint.
    pub center: [Length; 3],
    pub radius: Length,
}

/// A sampled joint configuration and whether it is a required stop.
#[derive(Debug, Clone, Copy)]
pub struct CuCartesianPathPoint<const N: usize> {
    pub positions: [CuJointPosition; N],
    /// True at the start, end, and every zero-radius interior waypoint.
    pub stop: bool,
    /// The TCP zone that contains this point, when blending.
    pub zone: Option<CuTcpZone>,
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
    /// IK found no solution at sample `step` of `steps` on the move toward
    /// `waypoint`: on the rounded corner inside its zone when `corner`,
    /// otherwise on the straight TCP line leading to it (or to its zone).
    InverseKinematicsFailed {
        waypoint: usize,
        corner: bool,
        step: usize,
        steps: usize,
    },
    Collision {
        waypoint: usize,
    },
    ZoneExceeded {
        waypoint: usize,
    },
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

/// Position of a generated sample on the move toward `waypoint`.
#[derive(Clone, Copy)]
struct Progress {
    waypoint: usize,
    step: usize,
    steps: usize,
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
        progress: Progress,
    ) -> CuResult<CuCartesianBlendStatus> {
        let waypoint = progress.waypoint;
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
                return Ok(CuCartesianBlendStatus::InverseKinematicsFailed {
                    waypoint,
                    corner: zone.is_some(),
                    step: progress.step,
                    steps: progress.steps,
                });
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
            zone: zone.map(|(waypoint, center, radius)| CuTcpZone {
                waypoint,
                center: center.map(Length::new::<meter>),
                radius: Length::new::<meter>(radius),
            }),
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
                Progress {
                    waypoint,
                    step: i,
                    steps,
                },
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
/// not swept-volume continuous collision detection;
/// [`parameterize_cartesian_path`](crate::path_timing::parameterize_cartesian_path)
/// rechecks the interpolated path when timing it.
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
                let progress = Progress {
                    waypoint: i,
                    step: k,
                    steps,
                };
                let status =
                    builder.append(pose, None, false, Some((i, poses[i].p, r)), progress)?;
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
            if zone.get::<meter>() > 0.0
                && !builder
                    .out
                    .points()
                    .iter()
                    .any(|p| p.zone.is_some_and(|z| z.waypoint == i))
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
    use crate::test_support::q;
    use crate::test_support::robot;
    use cu_collision::CuCollisionShape;
    use cu_collision::CuSphere;

    fn zones<const K: usize>(radii: [f64; K]) -> [Length; K] {
        radii.map(Length::new::<meter>)
    }

    #[test]
    fn zone_produces_a_corner_inside_tcp_radius() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let world = CuCollisionWorld::<1>::new();
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones([0.0, 0.1, 0.0]),
                &CuCartesianBlendConfig::default(),
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        let mut in_zone = 0;
        for p in path.points() {
            if let Some(zone) = p.zone {
                assert_eq!(zone.waypoint, 1);
                assert_eq!(zone.radius, Length::new::<meter>(0.1));
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
        assert!(!path.points().iter().any(|p| p.positions == points[1]));
        assert!(path.points()[0].stop && path.points()[path.len - 1].stop);
    }

    #[test]
    fn zero_zone_stops_exactly_and_collision_fails_closed() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let mut world = CuCollisionWorld::<1>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones([0.0; 3]),
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
        assert!(path.points().iter().all(|p| p.zone.is_none()));
        world
            .add_obstacle(CuCollisionShape::Sphere(CuSphere::from_meters(
                [0.2775, 0.0225, 0.0],
                0.025,
            )))
            .unwrap();
        assert!(matches!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones([0.0, 0.1, 0.0]),
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Collision { .. }
        ));
        assert_eq!(path.len, 0);
    }

    #[test]
    fn ik_failure_reports_the_move_and_progress() {
        let (chain, model) = robot();
        let world = CuCollisionWorld::<0>::new();
        // 0.1 m samples need several 0.025 m-clamped IK iterations, so a
        // two-iteration budget fails on the first sample toward waypoint 1.
        let mut config = CuCartesianBlendConfig {
            translation_step: Length::new::<meter>(0.1),
            ..CuCartesianBlendConfig::default()
        };
        config.ik.max_iterations = 2;
        let mut path = CuCartesianBlendPath::<3, 64>::default();
        let status = blend_cartesian_path(
            &chain,
            &model,
            Some(&world),
            &[q(0.0, 0.0), q(0.3, 0.0)],
            &zones([0.0, 0.0]),
            &config,
            &mut path,
        )
        .unwrap();
        assert_eq!(
            status,
            CuCartesianBlendStatus::InverseKinematicsFailed {
                waypoint: 1,
                corner: false,
                step: 1,
                steps: 3,
            }
        );
        assert_eq!(path.len, 0);
    }

    #[test]
    fn invalid_and_capacity_cases_leave_no_partial_path() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let world = CuCollisionWorld::<0>::new();
        let config = CuCartesianBlendConfig::default();
        let mut path = CuCartesianBlendPath::<3, 8>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones([0.0, 0.1, 0.0]),
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::CapacityExceeded
        );
        assert_eq!(path.len, 0);
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                Some(&world),
                &points,
                &zones([0.0, -0.1, 0.0]),
                &config,
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::InvalidInput
        );
        assert_eq!(path.len, 0);
    }
}
