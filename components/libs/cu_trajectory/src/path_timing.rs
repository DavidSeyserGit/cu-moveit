//! Profile timing of sampled Cartesian paths.
//!
//! Geometry and timing are independent steps.
//! [`blend_cartesian_path`](crate::blend_cartesian_path)
//! turns waypoints and their per-point TCP zones into a joint-space path;
//! [`parameterize_cartesian_path`] times that path with any
//! [`CuTrajectoryProfile`]. A zone belongs to its waypoint, not to the
//! profile: every profile flies through every positive zone and stops
//! exactly at every zero-radius waypoint.
//!
//! The knots are joined by a C2 natural cubic spline `q(s)` over a
//! path parameter `s`. Exact stops split the path into rest-to-rest pieces.
//! Inside a piece each TCP zone is traversed at a constant corner speed
//! bounded by the zone's joint-space curvature, and every move between a stop
//! or corner and the next follows the selected profile from one boundary speed
//! to the other. Joint velocity and acceleration bounds follow from analytic
//! derivative bounds of `q(s)` and of the path-speed profile; S-curve joint jerk is bounded on the executed quintic reconstruction.
//! Every profile is finally certified on the reconstruction its follower executes — quintic
//! Hermite for the S-curve, cubic Hermite for the others — and stretched uniformly when a peak
//! exceeds a limit.

use crate::CuTrajectoryInterpolation;
use crate::CuTrajectoryPayload;
use crate::CuTrajectoryPoint;
use crate::cartesian_blend::CuCartesianBlendPath;
use crate::cartesian_blend::CuCartesianPathPoint;
use crate::parameterize::CuJointTrajectoryLimits;
use crate::parameterize::CuTimingParams;
use crate::parameterize::CuTimingStatus;
use crate::parameterize::CuTrajectoryProfile;
use crate::parameterize::cubic_eval;
use crate::parameterize::quintic_eval;
use crate::quintic::cubic_limit_ratio;
use crate::quintic::polynomial_extrema;
use cu_collision::CuCollisionEnv;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointAcceleration;
use cu_kinematics::CuJointJerk;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointResolution;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::length::meter;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;

/// Parameters of [`parameterize_cartesian_path`]: the shared joint timing
/// plus the collision recheck resolution.
///
/// ```ron
/// {
///     "timing": {
///         "limits": {
///             "max_velocities": [{"revolute": 1.0}],
///             "max_accelerations": [{"revolute": 2.0}],
///             "max_jerks": [{"revolute": 10.0}],
///         },
///         "profile": "s_curve",
///         "sample_period": 0.02,
///     },
///     "collision_step": {"angular": 0.08, "linear": 0.01},
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(deny_unknown_fields)]
pub struct CuPathTimingParams<const N: usize> {
    /// Limits, path-speed profile of every move, and sample period.
    pub timing: CuTimingParams<N>,
    /// Joint resolution for the collision recheck of the interpolated path.
    #[serde(default)]
    pub collision_step: CuJointResolution,
}

impl<const N: usize> Default for CuPathTimingParams<N> {
    fn default() -> Self {
        Self {
            timing: CuTimingParams::default(),
            collision_step: CuJointResolution::default(),
        }
    }
}

impl<const N: usize> CuPathTimingParams<N> {
    /// Checks the timing parameters and the collision resolution.
    pub fn validate(&self) -> CuResult<()> {
        self.timing.validate()?;
        self.collision_step.metric().validate()
    }
}

/// Times a sampled Cartesian path with one profile along the whole path.
///
/// Every move between an exact stop or zone corner and the next follows
/// `params.timing.profile` in the path parameter; zones are crossed at a
/// constant corner speed. Joint velocity and acceleration stay within
/// `params.timing.limits`; S-curve additionally bounds joint jerk on the
/// quintic reconstruction, including curvature contributions. The geometric
/// spline is rechecked for collisions against `env` (pass
/// [`CuCollisionEnv::EMPTY`] for self-collision only) and for zone
/// containment with discrete samples, not swept-volume continuous collision
/// detection. This check does not certify the reconstructed execution curve
/// against geometry. `out.interpolation` is set to the reconstruction the
/// limits were validated on: [`CuTrajectoryInterpolation::Quintic`] for the
/// S-curve, [`CuTrajectoryInterpolation::Cubic`] for the other profiles, so
/// [`CuTrajectoryPayload::sample`] (and the default follower) execute the
/// validated position, velocity and acceleration. All scratch storage is
/// const-sized; `out` is empty unless the status is `Success`.
pub fn parameterize_cartesian_path<
    const N: usize,
    const M: usize,
    const CAP: usize,
    const OUT: usize,
>(
    chain: &CuKinematicChain<N>,
    model: &CuRobotCollisionModel<N, M>,
    env: &CuCollisionEnv<'_>,
    path: &CuCartesianBlendPath<N, CAP>,
    params: &CuPathTimingParams<N>,
    out: &mut CuTrajectoryPayload<N, OUT>,
) -> CuResult<CuTimingStatus> {
    out.clear();
    out.interpolation = if params.timing.profile == CuTrajectoryProfile::SCurve {
        CuTrajectoryInterpolation::Quintic
    } else {
        CuTrajectoryInterpolation::Cubic
    };
    let result = time_path(chain, model, env, path, params, out);
    if !matches!(result, Ok(CuTimingStatus::Success)) {
        out.clear();
    }
    result
}

/// Path-speed bounds for one span: `ds/dt <= vel`, `|d2s/dt2| <= acc`,
/// `|d3s/dt3| <= jerk`.
#[derive(Debug, Clone, Copy, Default)]
struct SpeedLimits {
    vel: f64,
    acc: f64,
    jerk: f64,
}

/// A run of pieces timed as one unit: a move, or a zone corner.
#[derive(Debug, Clone, Copy, Default)]
struct Span {
    start: usize,
    end: usize,
    corner: Option<usize>,
    limits: SpeedLimits,
    /// Corner speed; unused for moves.
    speed: f64,
}

/// Constant-jerk phase starting with acceleration `acc`.
#[derive(Debug, Clone, Copy, Default)]
struct Phase {
    duration: f64,
    jerk: f64,
    acc: f64,
}

/// Path-speed motion of one span from boundary speed `v0` to `v1`.
#[derive(Debug, Clone, Copy)]
enum Motion {
    Constant { speed: f64 },
    Cubic { v0: f64, v1: f64 },
    Quintic { v0: f64, v1: f64 },
    Phased { v0: f64, phases: [Phase; 7] },
}

fn time_path<const N: usize, const M: usize, const CAP: usize, const OUT: usize>(
    chain: &CuKinematicChain<N>,
    model: &CuRobotCollisionModel<N, M>,
    env: &CuCollisionEnv<'_>,
    path: &CuCartesianBlendPath<N, CAP>,
    config: &CuPathTimingParams<N>,
    out: &mut CuTrajectoryPayload<N, OUT>,
) -> CuResult<CuTimingStatus> {
    let limits = &config.timing.limits;
    let profile = config.timing.profile;
    let count = path.len;
    let dt = config.timing.sample_period.get::<second>();
    if count < 2
        || count > CAP
        || !path.points[0].stop
        || !path.points[count - 1].stop
        || !dt.is_finite()
        || dt <= 0.0
        || config.collision_step.metric().validate().is_err()
    {
        return Ok(CuTimingStatus::InvalidInput);
    }
    let first = path.points[0].positions;
    for (j, &joint) in first.iter().enumerate() {
        let valid_kind = matches!(
            (joint, limits.max_velocities[j]),
            (CuJointPosition::Revolute(_), CuJointVelocity::Revolute(_))
                | (CuJointPosition::Prismatic(_), CuJointVelocity::Prismatic(_))
                | (CuJointPosition::Fixed, CuJointVelocity::Fixed)
        ) && core::mem::discriminant(&limits.max_accelerations[j])
            == core::mem::discriminant(&CuJointAcceleration::from_coordinate(joint, 0.0))
            && (profile != CuTrajectoryProfile::SCurve
                || matches!(joint, CuJointPosition::Fixed)
                || limits.max_jerks.is_none_or(|jerks| {
                    core::mem::discriminant(&jerks[j])
                        == core::mem::discriminant(&CuJointJerk::from_coordinate(joint, 0.0))
                }));
        if !valid_kind {
            return Ok(CuTimingStatus::InvalidInput);
        }
        if matches!(joint, CuJointPosition::Fixed) {
            continue;
        }
        let v = limits.max_velocities[j].si_value();
        let a = limits.acc(j);
        let jerk = limits.jerk(j);
        if !v.is_finite()
            || v <= 0.0
            || !a.is_finite()
            || a <= 0.0
            || (profile == CuTrajectoryProfile::SCurve && (jerk.is_nan() || jerk <= 0.0))
        {
            return Ok(CuTimingStatus::InvalidInput);
        }
    }

    // Path parameter: speed-weighted joint-space arc length.
    let points = path.points();
    let mut s = [0.0_f64; CAP];
    for i in 0..count {
        let q = points[i].positions;
        if !chain.is_within_limits(&q)
            || (0..N).any(|j| core::mem::discriminant(&q[j]) != core::mem::discriminant(&first[j]))
        {
            return Ok(CuTimingStatus::InvalidInput);
        }
        if i > 0 {
            let h = step(&points[i - 1].positions, &q, limits);
            if !h.is_finite() || h <= 1e-15 {
                return Ok(CuTimingStatus::InvalidInput);
            }
            s[i] = s[i - 1] + h;
        }
    }

    // C2 natural cubic spline q(s) through the knots, one rest-to-rest run at
    // a time. A non-stop knot where every joint stops moving cannot be
    // crossed at speed.
    let curvatures = spline_curvatures(points, &s);
    let curve = Curve {
        points,
        s: &s,
        curvatures: &curvatures,
    };
    for (i, _) in points.iter().enumerate().take(count - 1).skip(1) {
        if points[i].stop {
            continue;
        }
        let (_, slope, _) = curve.eval(i, 0.0);
        if slope.iter().all(|m| libm::fabs(*m) <= 1e-12) {
            return Ok(match points[i].zone {
                Some(zone) => CuTimingStatus::NoContinuousMotion {
                    waypoint: zone.waypoint,
                },
                None => CuTimingStatus::UnexpectedStop { sample: i },
            });
        }
    }

    // Geometry recheck: joint limits, collisions and zone containment along q(s).
    let metric = config.collision_step.metric();
    for i in 0..count - 1 {
        let h = s[i + 1] - s[i];
        let (slope, _, _) = curve.bounds(i);
        let mut checks = 1.0_f64;
        for (j, &peak) in slope.iter().enumerate() {
            checks = checks.max(libm::ceil(peak * h / metric.scale(first[j])));
        }
        if !checks.is_finite() || checks > 100_000.0 {
            return Ok(CuTimingStatus::InvalidInput);
        }
        let zone = points[i + 1].zone;
        let mut previous = points[i].positions;
        for k in 1..=checks as usize {
            let (q, _, _) = curve.eval(i, h * k as f64 / checks);
            if !chain.is_within_limits(&q) {
                return Ok(CuTimingStatus::InvalidInput);
            }
            if !model.is_configuration_collision_free(chain, &q, env)?
                || !model.is_segment_collision_free(
                    chain,
                    &previous,
                    &q,
                    config.collision_step,
                    env,
                )?
            {
                return Ok(CuTimingStatus::Collision { sample: i });
            }
            if let Some(zone) = zone {
                let mut pose = Transform3D::<f64>::identity();
                chain.forward_kinematics(&q, &mut pose)?;
                let m = pose.to_matrix();
                let c = zone.center.map(|c| c.get::<meter>());
                let d = [m[0][3] - c[0], m[1][3] - c[1], m[2][3] - c[2]];
                if libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) > zone.radius.get::<meter>()
                {
                    return Ok(CuTimingStatus::ZoneExceeded {
                        waypoint: zone.waypoint,
                    });
                }
            }
            previous = q;
        }
    }

    // Spans: runs of pieces sharing one corner, split at exact stops.
    let mut spans = [Span::default(); CAP];
    let mut nspans = 0;
    let mut start = 0;
    for i in 0..count - 1 {
        let corner = corner_of(points, i);
        let last = i + 1 == count - 1 || points[i + 1].stop || corner_of(points, i + 1) != corner;
        if last {
            spans[nspans] = Span {
                start,
                end: i + 1,
                corner,
                ..Span::default()
            };
            nspans += 1;
            start = i + 1;
        }
    }
    let spans = &mut spans[..nspans];
    let scurve = profile == CuTrajectoryProfile::SCurve;
    // Moves keep half the acceleration budget for curvature; corners spend
    // all of it there because their path speed is constant. Joint jerk is
    // `q3·v³ + 3·q2·v·a + q1·j` in the path derivatives `q1..q3` and the
    // path speed `v`, acceleration `a` and jerk `j`: the S-curve gives each term a third of the joint jerk limit,
    // and the path jerk takes what the other two leave unused. Speed bounds
    // come from the move interior; the acceleration and jerk budgets hold on
    // every piece at the speed that piece allows.
    for p in 0..spans.len() {
        let (lo, hi) = interior(spans, p);
        let span = &mut spans[p];
        let mut vel = f64::INFINITY;
        let mut corner_vel = f64::INFINITY;
        for i in lo..hi {
            let (move_cap, corner_cap) = speed_caps(&curve.bounds(i), limits, &first, scurve);
            vel = vel.min(move_cap);
            corner_vel = corner_vel.min(corner_cap);
        }
        if !vel.is_finite() || vel <= 0.0 {
            return Ok(CuTimingStatus::InvalidInput);
        }
        let mut acc = f64::INFINITY;
        for i in span.start..span.end {
            let bounds = curve.bounds(i);
            let (slope, curvature, _) = bounds;
            let v = vel.min(speed_caps(&bounds, limits, &first, scurve).0);
            for j in 0..N {
                if matches!(first[j], CuJointPosition::Fixed) {
                    continue;
                }
                let jmax = limits.jerk(j);
                if slope[j] > 1e-300 {
                    acc = acc.min((limits.acc(j) - curvature[j] * v * v) / slope[j]);
                }
                if scurve && jmax.is_finite() && curvature[j] > 1e-300 {
                    acc = acc.min(jmax / (9.0 * curvature[j] * v));
                }
            }
        }
        if !acc.is_finite() || acc <= 0.0 {
            return Ok(CuTimingStatus::InvalidInput);
        }
        let mut jerk = f64::INFINITY;
        if scurve {
            for i in span.start..span.end {
                let bounds = curve.bounds(i);
                let (slope, curvature, third) = bounds;
                let v = vel.min(speed_caps(&bounds, limits, &first, scurve).0);
                for j in 0..N {
                    let jmax = limits.jerk(j);
                    if matches!(first[j], CuJointPosition::Fixed)
                        || slope[j] <= 1e-300
                        || !jmax.is_finite()
                    {
                        continue;
                    }
                    let left = jmax - third[j] * v * v * v - 3.0 * curvature[j] * v * acc;
                    jerk = jerk.min(left / slope[j]);
                }
            }
            if jerk.is_nan() || jerk <= 0.0 {
                return Ok(CuTimingStatus::InvalidInput);
            }
        }
        span.limits = SpeedLimits { vel, acc, jerk };
        if span.corner.is_some() {
            span.speed = corner_vel;
        }
    }
    // Adjacent corners form a run crossed at one speed, between two moves;
    // a run never touches a stop.
    let mut p = 0;
    while p < spans.len() {
        if spans[p].corner.is_none() {
            p += 1;
            continue;
        }
        let first_corner = p;
        while p + 1 < spans.len() && spans[p + 1].corner.is_some() {
            p += 1;
        }
        if first_corner == 0
            || p + 1 == spans.len()
            || points[spans[first_corner].start].stop
            || points[spans[p].end].stop
        {
            return Ok(CuTimingStatus::InvalidInput);
        }
        let speed = spans[first_corner..=p]
            .iter()
            .fold(f64::INFINITY, |v, span| v.min(span.speed))
            .min(spans[first_corner - 1].limits.vel)
            .min(spans[p + 1].limits.vel);
        set_run_speed(spans, first_corner, speed);
        p += 1;
    }

    // Lower corner speeds until every move can join its boundary speeds.
    let mut settled = false;
    let mut blocked = None;
    for _ in 0..400 {
        settled = true;
        for p in 0..spans.len() {
            if spans[p].corner.is_some() {
                continue;
            }
            let (v0, v1) = boundary_speeds(spans, p);
            if plan_span(&curve, &s, spans, p, (v0, v1), config).is_some() {
                continue;
            }
            settled = false;
            let previous = (p > 0 && spans[p - 1].corner.is_some()).then(|| p - 1);
            let next = (p + 1 < spans.len() && spans[p + 1].corner.is_some()).then_some(p + 1);
            // Lower the faster boundary corner.
            let lower = match (previous, next) {
                (Some(before), Some(after)) => {
                    if v0 >= v1 {
                        before
                    } else {
                        after
                    }
                }
                (Some(corner), None) | (None, Some(corner)) => corner,
                (None, None) => return Ok(CuTimingStatus::InvalidInput),
            };
            set_run_speed(spans, lower, spans[lower].speed * 0.9);
            blocked = spans[lower].corner;
        }
        if settled {
            break;
        }
    }
    if !settled {
        return Ok(match blocked {
            Some(waypoint) => CuTimingStatus::NoContinuousMotion { waypoint },
            None => CuTimingStatus::InvalidInput,
        });
    }

    if OUT == 0 {
        return Ok(CuTimingStatus::CapacityExceeded);
    }
    out.push(CuTrajectoryPoint {
        time_from_start: Time::default(),
        positions: first,
        velocities: first.map(|q| CuJointVelocity::from_coordinate(q, 0.0)),
        accelerations: first.map(|q| CuJointAcceleration::from_coordinate(q, 0.0)),
    })?;
    let mut elapsed = 0.0;
    for p in 0..spans.len() {
        let span = spans[p];
        let length = s[span.end] - s[span.start];
        let (v0, v1) = boundary_speeds(spans, p);
        let (motion, duration) = if span.corner.is_some() {
            (Motion::Constant { speed: span.speed }, length / span.speed)
        } else {
            match plan_span(&curve, &s, spans, p, (v0, v1), config) {
                Some(planned) => planned,
                None => return Ok(CuTimingStatus::InvalidInput),
            }
        };
        let steps = libm::ceil(duration / dt).max(1.0);
        let step_dt = duration / steps;
        // The quintic reconstruction of the S-curve is exact only where the
        // motion is smooth, so knots are added where the path jerk changes
        // (phase boundaries) and where the path crosses a spline knot (the
        // third path derivative changes).
        let mut phase_times = [0.0_f64; 6];
        let mut phase_count = 0;
        let mut knot_times = [0.0_f64; CAP];
        let mut knot_count = 0;
        if scurve {
            if let Motion::Phased { phases, .. } = motion {
                let mut at = 0.0;
                for phase in &phases[..6] {
                    at += phase.duration;
                    phase_times[phase_count] = at;
                    phase_count += 1;
                }
            }
            for knot in span.start + 1..span.end {
                knot_times[knot_count] =
                    crossing_time(&motion, length, duration, s[knot] - s[span.start]);
                knot_count += 1;
            }
        }
        if !steps.is_finite()
            || steps + (phase_count + knot_count) as f64 > OUT.saturating_sub(out.len()) as f64
        {
            return Ok(CuTimingStatus::CapacityExceeded);
        }
        let steps = steps as usize;
        let (mut k, mut next_phase, mut next_knot) = (1, 0, 0);
        let mut emitted = 0.0_f64;
        let mut piece = span.start;
        while k <= steps {
            let grid = duration * k as f64 / steps as f64;
            // Next extra time, skipping any that would crowd an emitted knot
            // or the span end, so knot times strictly increase.
            let mut extra = f64::INFINITY;
            let mut from_phase = false;
            loop {
                let phase = phase_times
                    .get(next_phase)
                    .filter(|_| next_phase < phase_count);
                let knot = knot_times.get(next_knot).filter(|_| next_knot < knot_count);
                let (candidate, is_phase) = match (phase, knot) {
                    (Some(&p), Some(&q)) if p <= q => (p, true),
                    (_, Some(&q)) => (q, false),
                    (Some(&p), None) => (p, true),
                    (None, None) => break,
                };
                if candidate <= emitted + 0.1 * step_dt || candidate >= duration - 0.1 * step_dt {
                    if is_phase {
                        next_phase += 1;
                    } else {
                        next_knot += 1;
                    }
                    continue;
                }
                extra = candidate;
                from_phase = is_phase;
                break;
            }
            let mut take_extra = || {
                if from_phase {
                    next_phase += 1;
                } else {
                    next_knot += 1;
                }
                extra
            };
            let mut t = grid;
            let mut last = false;
            if extra < grid - 0.1 * step_dt {
                t = take_extra();
            } else {
                if extra <= grid + 0.1 * step_dt {
                    t = take_extra();
                }
                last = k == steps;
                k += 1;
            }
            emitted = t;
            let (positions, velocities, accelerations) = if last {
                let end = span.end - 1;
                let (_, slope, curvature) = curve.eval(end, s[span.end] - s[end]);
                let (_, _, acc) = motion.eval(length, duration, duration);
                joint_state(points[span.end].positions, slope, curvature, v1, acc)
            } else {
                let (offset, speed, acc) = motion.eval(length, duration, t);
                let at = s[span.start] + offset.clamp(0.0, length);
                while piece + 1 < span.end && at > s[piece + 1] {
                    piece += 1;
                }
                let (q, slope, curvature) = curve.eval(piece, at - s[piece]);
                joint_state(q, slope, curvature, speed.max(0.0), acc)
            };
            out.push(CuTrajectoryPoint {
                time_from_start: Time::new::<second>(elapsed + t),
                positions,
                velocities,
                accelerations,
            })?;
        }
        elapsed += duration;
    }

    // The follower executes a Hermite motion through these knots — quintic
    // for the S-curve, cubic for the others — so the limits are checked on
    // exactly that reconstruction. Stretching time by the worst ratio of peak
    // to limit makes every peak fit: velocity, acceleration and jerk scale by
    // 1/k, 1/k^2 and 1/k^3, keeping the shape.
    let ratio = if scurve {
        out.quintic_limit_ratio(limits)
    } else {
        cubic_limit_ratio(out, limits)
    };
    match ratio {
        Some(ratio) if ratio.is_finite() => {
            if ratio > 1.0 {
                out.stretch_time(ratio * (1.0 + 1e-9))?;
            }
        }
        _ => return Ok(CuTimingStatus::InvalidInput),
    }
    Ok(CuTimingStatus::Success)
}

/// Time at which `motion` reaches path offset `target`; the motion's offset
/// never decreases, so bisection converges to the first crossing.
fn crossing_time(motion: &Motion, length: f64, duration: f64, target: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, duration);
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        if motion.eval(length, duration, mid).0 < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// Zone corner owning piece `i`: the zone of either end knot, extended by
/// one piece before the zone entry, where the spline already bends.
fn corner_of<const N: usize>(points: &[CuCartesianPathPoint<N>], i: usize) -> Option<usize> {
    let zone = |k: usize| points.get(k).and_then(|p| p.zone).map(|z| z.waypoint);
    zone(i + 1)
        .or_else(|| zone(i).filter(|_| !points[i + 1].stop))
        .or_else(|| zone(i + 2).filter(|_| !points[i].stop))
}

/// Path-speed caps `(move, corner)` of one piece from its derivative bounds.
///
/// A move keeps half the acceleration and a third of the jerk budget for the
/// curvature terms; a corner at constant speed spends them all there.
fn speed_caps<const N: usize>(
    (slope, curvature, third): &([f64; N], [f64; N], [f64; N]),
    limits: &CuJointTrajectoryLimits<N>,
    first: &[CuJointPosition; N],
    scurve: bool,
) -> (f64, f64) {
    let mut move_cap = f64::INFINITY;
    let mut corner_cap = f64::INFINITY;
    for j in 0..N {
        if matches!(first[j], CuJointPosition::Fixed) {
            continue;
        }
        let v = limits.max_velocities[j].si_value();
        let a = limits.acc(j);
        let jmax = limits.jerk(j);
        if slope[j] > 1e-300 {
            move_cap = move_cap.min(v / slope[j]);
            corner_cap = corner_cap.min(v / slope[j]);
        }
        if curvature[j] > 1e-300 {
            move_cap = move_cap.min(libm::sqrt(a / (2.0 * curvature[j])));
            corner_cap = corner_cap.min(libm::sqrt(a / curvature[j]));
        }
        if scurve && jmax.is_finite() && third[j] > 1e-300 {
            move_cap = move_cap.min(libm::cbrt(jmax / (3.0 * third[j])));
            corner_cap = corner_cap.min(libm::cbrt(jmax / third[j]));
        }
    }
    (move_cap, corner_cap)
}

/// Pieces of a move next to a corner that are excluded from the move's
/// constant speed limits and checked against the exact joint limits instead.
///
/// The C2 spline smooths the curvature jump between a straight line and a
/// zone arc; its third derivative decays by roughly a factor of four per
/// knot into the line. Taking those pieces into the move's worst-case bounds
/// would cap the speed of the whole straight move, although the profile is
/// still near the corner speed while crossing them.
const TRANSITION_PIECES: usize = 4;

/// Piece range of span `p` that sets its speed limits: the move without its
/// transition pieces next to a corner (the whole span when that leaves
/// nothing).
fn interior(spans: &[Span], p: usize) -> (usize, usize) {
    let span = spans[p];
    if span.corner.is_some() {
        return (span.start, span.end);
    }
    let mut lo = span.start;
    let mut hi = span.end;
    if p > 0 && spans[p - 1].corner.is_some() {
        lo += TRANSITION_PIECES;
    }
    if p + 1 < spans.len() && spans[p + 1].corner.is_some() {
        hi = hi.saturating_sub(TRANSITION_PIECES);
    }
    if lo < hi {
        (lo, hi)
    } else {
        (span.start, span.end)
    }
}

/// Sets the speed of the corner run containing span `p`.
fn set_run_speed(spans: &mut [Span], p: usize, speed: f64) {
    let mut first = p;
    while first > 0 && spans[first - 1].corner.is_some() {
        first -= 1;
    }
    let mut last = p;
    while last + 1 < spans.len() && spans[last + 1].corner.is_some() {
        last += 1;
    }
    for span in &mut spans[first..=last] {
        span.speed = speed;
    }
}

/// Boundary path speeds of span `p`: corner speed next to a corner, zero at stops.
fn boundary_speeds(spans: &[Span], p: usize) -> (f64, f64) {
    if spans[p].corner.is_some() {
        return (spans[p].speed, spans[p].speed);
    }
    let v0 = if p > 0 && spans[p - 1].corner.is_some() {
        spans[p - 1].speed
    } else {
        0.0
    };
    let v1 = if p + 1 < spans.len() && spans[p + 1].corner.is_some() {
        spans[p + 1].speed
    } else {
        0.0
    };
    (v0, v1)
}

/// Path-parameter step between two knots: joint-space arc length weighted
/// by the speed limits, so constant path speed means constant joint speed
/// and curvature contributes no tangential acceleration.
fn step<const N: usize>(
    a: &[CuJointPosition; N],
    b: &[CuJointPosition; N],
    limits: &CuJointTrajectoryLimits<N>,
) -> f64 {
    let mut sum = 0.0_f64;
    for j in 0..N {
        if !matches!(a[j], CuJointPosition::Fixed) {
            let d = (b[j].si_value() - a[j].si_value()) / limits.max_velocities[j].si_value();
            sum += d * d;
        }
    }
    libm::sqrt(sum)
}

/// Joint state of one sample: positions, velocities and accelerations.
type JointState<const N: usize> = (
    [CuJointPosition; N],
    [CuJointVelocity; N],
    [CuJointAcceleration; N],
);

/// Joint state from path derivatives: `v = q'·ds/dt`, `a = q''·(ds/dt)² + q'·d2s/dt2`.
fn joint_state<const N: usize>(
    q: [CuJointPosition; N],
    slope: [f64; N],
    curvature: [f64; N],
    speed: f64,
    acc: f64,
) -> JointState<N> {
    (
        q,
        core::array::from_fn(|j| CuJointVelocity::from_coordinate(q[j], slope[j] * speed)),
        core::array::from_fn(|j| {
            CuJointAcceleration::from_coordinate(
                q[j],
                curvature[j] * speed * speed + slope[j] * acc,
            )
        }),
    )
}

/// Second derivatives `d2q/ds2` at the knots of the natural cubic spline
/// through the path, solved with the Thomas algorithm on const-sized scratch.
///
/// Exact stops pin the curvature to zero, which decouples the path into
/// rest-to-rest runs; path speed and acceleration vanish at a stop, so the
/// curvature jump there never reaches the joint motion.
fn spline_curvatures<const N: usize, const CAP: usize>(
    points: &[CuCartesianPathPoint<N>],
    s: &[f64; CAP],
) -> [[f64; N]; CAP] {
    let count = points.len();
    let mut curvatures = [[0.0_f64; N]; CAP];
    let mut upper = [0.0_f64; CAP];
    let mut rhs = [0.0_f64; CAP];
    for (j, _) in points[0].positions.iter().enumerate() {
        let y = |i: usize| points[i].positions[j].si_value();
        for i in 0..count {
            if i == 0 || i + 1 == count || points[i].stop {
                upper[i] = 0.0;
                rhs[i] = 0.0;
                continue;
            }
            let (h0, h1) = (s[i] - s[i - 1], s[i + 1] - s[i]);
            let diagonal = 2.0 * (h0 + h1) - h0 * upper[i - 1];
            let slopes = (y(i + 1) - y(i)) / h1 - (y(i) - y(i - 1)) / h0;
            upper[i] = h1 / diagonal;
            rhs[i] = (6.0 * slopes - h0 * rhs[i - 1]) / diagonal;
        }
        let mut next = 0.0;
        for i in (0..count).rev() {
            next = rhs[i] - upper[i] * next;
            curvatures[i][j] = next;
        }
    }
    curvatures
}

/// The C2 natural cubic spline `q(s)` through the path knots.
struct Curve<'a, const N: usize, const CAP: usize> {
    points: &'a [CuCartesianPathPoint<N>],
    s: &'a [f64; CAP],
    curvatures: &'a [[f64; N]; CAP],
}

impl<const N: usize, const CAP: usize> Curve<'_, N, CAP> {
    /// Cubic of piece `i`, joint `j`, in the offset `t` from its start:
    /// `(h, y0, b, m0, d)` with `q = y0 + b·t + m0·t²/2 + d·t³/6`.
    fn coefficients(&self, i: usize, j: usize) -> (f64, f64, f64, f64, f64) {
        let h = self.s[i + 1] - self.s[i];
        let y0 = self.points[i].positions[j].si_value();
        let y1 = self.points[i + 1].positions[j].si_value();
        let (m0, m1) = (self.curvatures[i][j], self.curvatures[i + 1][j]);
        (
            h,
            y0,
            (y1 - y0) / h - h * (2.0 * m0 + m1) / 6.0,
            m0,
            (m1 - m0) / h,
        )
    }

    /// Joint positions, `dq/ds` and `d2q/ds2` at `offset` along piece `i`.
    fn eval(&self, i: usize, offset: f64) -> ([CuJointPosition; N], [f64; N], [f64; N]) {
        let mut q = self.points[i].positions;
        let mut slope = [0.0; N];
        let mut curvature = [0.0; N];
        for j in 0..N {
            let (h, y0, b, m0, d) = self.coefficients(i, j);
            let t = offset.clamp(0.0, h);
            q[j] = q[j].with_si_value(y0 + t * (b + t * (0.5 * m0 + t * d / 6.0)));
            slope[j] = b + t * (m0 + 0.5 * t * d);
            curvature[j] = m0 + t * d;
        }
        (q, slope, curvature)
    }

    /// Per-joint peaks of `|dq/ds|`, `|d2q/ds2|` and `|d3q/ds3|` over piece `i`.
    ///
    /// The slope is quadratic in `s` (peak at an end or the vertex), the
    /// curvature linear (peak at an end) and the third derivative constant,
    /// so all three are exact.
    fn bounds(&self, i: usize) -> ([f64; N], [f64; N], [f64; N]) {
        let mut slope = [0.0; N];
        let mut curvature = [0.0; N];
        let mut jerk = [0.0; N];
        for j in 0..N {
            let (h, _, b, m0, d) = self.coefficients(i, j);
            let at = |t: f64| b + t * (m0 + 0.5 * t * d);
            let mut peak = libm::fabs(b).max(libm::fabs(at(h)));
            if libm::fabs(d) > 1e-300 {
                let t = -m0 / d;
                if t > 0.0 && t < h {
                    peak = peak.max(libm::fabs(at(t)));
                }
            }
            slope[j] = peak;
            curvature[j] = libm::fabs(m0).max(libm::fabs(m0 + h * d));
            jerk[j] = libm::fabs(d);
        }
        (slope, curvature, jerk)
    }
}

impl Motion {
    /// Path offset, speed and acceleration at `t` seconds into the span.
    fn eval(&self, length: f64, duration: f64, t: f64) -> (f64, f64, f64) {
        let (offset, speed, acc, _) = self.eval_jerk(length, duration, t);
        (offset, speed, acc)
    }

    /// Path offset, speed, acceleration and jerk at `t` seconds into the span.
    fn eval_jerk(&self, length: f64, duration: f64, t: f64) -> (f64, f64, f64, f64) {
        let tt = duration * duration;
        match *self {
            Motion::Constant { speed } => (speed * t, speed, 0.0, 0.0),
            Motion::Cubic { v0, v1 } => {
                let u = t / duration;
                let (offset, speed) = cubic_eval(u, 0.0, length, v0, v1, duration);
                let a = v0 * duration;
                let b = 3.0 * length - 2.0 * a - v1 * duration;
                let c = -2.0 * length + a + v1 * duration;
                (
                    offset,
                    speed,
                    (2.0 * b + 6.0 * c * u) / tt,
                    6.0 * c / (tt * duration),
                )
            }
            Motion::Quintic { v0, v1 } => {
                let u = t / duration;
                let (offset, speed) = quintic_eval(u, 0.0, length, v0, v1, duration);
                let start = length - v0 * duration;
                let end = (v1 - v0) * duration;
                let a3 = 10.0 * start - 4.0 * end;
                let a4 = -15.0 * start + 7.0 * end;
                let a5 = 6.0 * start - 3.0 * end;
                (
                    offset,
                    speed,
                    u * (6.0 * a3 + u * (12.0 * a4 + 20.0 * a5 * u)) / tt,
                    (6.0 * a3 + u * (24.0 * a4 + 60.0 * a5 * u)) / (tt * duration),
                )
            }
            Motion::Phased { v0, phases } => {
                let mut remaining = t;
                let mut offset = 0.0;
                let mut speed = v0;
                let mut acc = 0.0;
                let mut jerk = 0.0;
                for phase in phases {
                    let tau = remaining.min(phase.duration);
                    offset += speed * tau
                        + 0.5 * phase.acc * tau * tau
                        + phase.jerk * tau * tau * tau / 6.0;
                    speed += phase.acc * tau + 0.5 * phase.jerk * tau * tau;
                    acc = phase.acc + phase.jerk * tau;
                    jerk = phase.jerk;
                    remaining -= tau;
                    if remaining <= 0.0 {
                        break;
                    }
                }
                (offset, speed, acc, jerk)
            }
        }
    }
}

/// Plans move `p` from boundary speeds `(v0, v1)` with its interior limits,
/// then checks the exact joint limits on its transition pieces; `None` when
/// either fails, which lowers the adjacent corner speed.
fn plan_span<const N: usize, const CAP: usize>(
    curve: &Curve<'_, N, CAP>,
    s: &[f64; CAP],
    spans: &[Span],
    p: usize,
    (v0, v1): (f64, f64),
    config: &CuPathTimingParams<N>,
) -> Option<(Motion, f64)> {
    let span = spans[p];
    let length = s[span.end] - s[span.start];
    let (motion, duration) = plan(config.timing.profile, length, v0, v1, &span.limits)?;
    let (lo, hi) = interior(spans, p);
    if (lo, hi) == (span.start, span.end) {
        return Some((motion, duration));
    }
    let limits = &config.timing.limits;
    let jerk_limited = config.timing.profile == CuTrajectoryProfile::SCurve;
    let mut piece = span.start;
    // Dense samples of the planned motion; pieces inside the interior are
    // already covered by the span limits.
    const SAMPLES: usize = 512;
    for k in 0..=SAMPLES {
        let t = duration * k as f64 / SAMPLES as f64;
        let (offset, speed, acc, jerk) = motion.eval_jerk(length, duration, t);
        let at = s[span.start] + offset.clamp(0.0, length);
        while piece + 1 < span.end && at > s[piece + 1] {
            piece += 1;
        }
        if piece >= lo && piece < hi {
            continue;
        }
        let (_, slope, curvature) = curve.eval(piece, at - s[piece]);
        for j in 0..N {
            let (_, _, _, _, third) = curve.coefficients(piece, j);
            let velocity = libm::fabs(slope[j] * speed);
            let acceleration = libm::fabs(curvature[j] * speed * speed + slope[j] * acc);
            let joint_jerk = libm::fabs(
                third * speed * speed * speed + 3.0 * curvature[j] * speed * acc + slope[j] * jerk,
            );
            let tolerance = 1.0 + 1e-9;
            if velocity > limits.max_velocities[j].si_value() * tolerance
                || acceleration > limits.acc(j) * tolerance
                || (jerk_limited && joint_jerk > limits.jerk(j) * tolerance)
            {
                return None;
            }
        }
    }
    Some((motion, duration))
}

/// Plans one move of `length` from speed `v0` to `v1`; `None` if the
/// boundary speeds cannot be joined within `limits`.
fn plan(
    profile: CuTrajectoryProfile,
    length: f64,
    v0: f64,
    v1: f64,
    limits: &SpeedLimits,
) -> Option<(Motion, f64)> {
    match profile {
        CuTrajectoryProfile::Trapezoidal => plan_phased(length, v0, v1, limits, f64::INFINITY),
        CuTrajectoryProfile::SCurve => plan_phased(length, v0, v1, limits, limits.jerk),
        CuTrajectoryProfile::Cubic => {
            plan_polynomial(false, length, v0, v1, limits).map(|t| (Motion::Cubic { v0, v1 }, t))
        }
        CuTrajectoryProfile::MinimumJerk => {
            plan_polynomial(true, length, v0, v1, limits).map(|t| (Motion::Quintic { v0, v1 }, t))
        }
    }
}

/// Symmetric velocity change by `dv >= 0`: `(jerk_time, accel_time, peak_acc)`.
/// Infinite jerk gives a constant-acceleration (trapezoidal) ramp.
fn ramp(dv: f64, acc: f64, jerk: f64) -> (f64, f64, f64) {
    if dv <= 0.0 {
        (0.0, 0.0, 0.0)
    } else if !jerk.is_finite() {
        (0.0, dv / acc, acc)
    } else if dv >= acc * acc / jerk {
        (acc / jerk, dv / acc - acc / jerk, acc)
    } else {
        let tj = libm::sqrt(dv / jerk);
        (tj, 0.0, jerk * tj)
    }
}

/// Distance covered by a ramp between two speeds.
fn ramp_distance(from: f64, to: f64, acc: f64, jerk: f64) -> f64 {
    let (tj, ta, _) = ramp(libm::fabs(to - from), acc, jerk);
    0.5 * (from + to) * (2.0 * tj + ta)
}

/// Trapezoidal (`jerk = inf`) or S-curve move: ramp to a peak speed, cruise,
/// ramp to the end speed.
fn plan_phased(
    length: f64,
    v0: f64,
    v1: f64,
    limits: &SpeedLimits,
    jerk: f64,
) -> Option<(Motion, f64)> {
    let acc = limits.acc;
    let distance =
        |peak: f64| ramp_distance(v0, peak, acc, jerk) + ramp_distance(peak, v1, acc, jerk);
    let low = v0.max(v1);
    if low > limits.vel * (1.0 + 1e-9) || distance(low) > length * (1.0 + 1e-12) {
        return None;
    }
    let peak = if distance(limits.vel) <= length {
        limits.vel
    } else {
        let (mut lo, mut hi) = (low, limits.vel);
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            if distance(mid) <= length {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    if peak <= 0.0 {
        return None;
    }
    let cruise = (length - distance(peak)).max(0.0) / peak;
    let (uj, ua, up) = ramp(peak - v0, acc, jerk);
    let (dj, da, dp) = ramp(peak - v1, acc, jerk);
    let jerk = if jerk.is_finite() { jerk } else { 0.0 };
    let phases = [
        Phase {
            duration: uj,
            jerk,
            acc: 0.0,
        },
        Phase {
            duration: ua,
            jerk: 0.0,
            acc: up,
        },
        Phase {
            duration: uj,
            jerk: -jerk,
            acc: up,
        },
        Phase {
            duration: cruise,
            jerk: 0.0,
            acc: 0.0,
        },
        Phase {
            duration: dj,
            jerk: -jerk,
            acc: 0.0,
        },
        Phase {
            duration: da,
            jerk: 0.0,
            acc: -dp,
        },
        Phase {
            duration: dj,
            jerk,
            acc: -dp,
        },
    ];
    let duration = 2.0 * uj + ua + cruise + 2.0 * dj + da;
    Some((Motion::Phased { v0, phases }, duration))
}

/// Shortest duration for a cubic or quintic Hermite move whose speed stays
/// within `[0, vel]` and acceleration within `acc`.
fn plan_polynomial(
    quintic: bool,
    length: f64,
    v0: f64,
    v1: f64,
    limits: &SpeedLimits,
) -> Option<f64> {
    let fits = |t: f64| {
        let (high, low, acc) = polynomial_extrema(quintic, length, v0, v1, t);
        high <= limits.vel * (1.0 + 1e-9) && acc <= limits.acc * (1.0 + 1e-9) && low >= 0.0
    };
    let mut t = length / limits.vel;
    let mut found = None;
    for _ in 0..400 {
        if fits(t) {
            found = Some(t);
            break;
        }
        t *= 1.05;
    }
    let mut hi = found?;
    let mut lo = hi / 1.05;
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if fits(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuCartesianBlendParams;
    use crate::CuCartesianBlendStatus;
    use crate::CuCartesianWaypoint;
    use crate::CuTcpZone;
    use crate::blend_cartesian_path;
    use crate::test_support::limits;
    use crate::test_support::obstacle;
    use crate::test_support::q;
    use crate::test_support::robot;
    use crate::test_support::waypoints;
    use cu29::units::si::f64::Length;
    use cu29::units::si::length::meter;

    const PROFILES: [CuTrajectoryProfile; 4] = [
        CuTrajectoryProfile::Trapezoidal,
        CuTrajectoryProfile::SCurve,
        CuTrajectoryProfile::MinimumJerk,
        CuTrajectoryProfile::Cubic,
    ];

    fn timing(profile: CuTrajectoryProfile) -> CuPathTimingParams<3> {
        let jerk = CuJointJerk::from_coordinate(q(0.0, 0.0)[0], 4.0);
        CuPathTimingParams {
            timing: CuTimingParams {
                limits: limits().with_jerks([jerk; 3]),
                profile,
                sample_period: Time::new::<second>(0.02),
            },
            collision_step: CuCartesianBlendParams::default().collision_step,
        }
    }

    fn timed<const K: usize>(
        points: &[[CuJointPosition; 3]; K],
        zones: [f64; K],
        profile: CuTrajectoryProfile,
    ) -> (CuTimingStatus, CuTrajectoryPayload<3, 512>) {
        let (chain, model) = robot();
        let env = CuCollisionEnv::EMPTY;
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        assert_eq!(
            blend_cartesian_path(
                &chain,
                &model,
                &env,
                &waypoints(points, zones),
                &CuCartesianBlendParams::default(),
                &mut path
            )
            .unwrap(),
            CuCartesianBlendStatus::Success
        );
        let mut out = CuTrajectoryPayload::<3, 512>::new();
        let status =
            parameterize_cartesian_path(&chain, &model, &env, &path, &timing(profile), &mut out)
                .unwrap();
        (status, out)
    }

    fn duration(out: &CuTrajectoryPayload<3, 512>) -> f64 {
        out.points().last().unwrap().time_from_start.get::<second>()
    }

    fn at_rest(p: &CuTrajectoryPoint<3>, x: f64, y: f64) -> bool {
        (p.positions[0].si_value() - x).abs() < 1e-9
            && (p.positions[1].si_value() - y).abs() < 1e-9
            && p.velocities.iter().all(|v| v.si_value().abs() < 1e-9)
    }

    /// Sampled velocity and finite-difference acceleration stay within limits.
    fn assert_within_limits(out: &CuTrajectoryPayload<3, 512>) {
        for w in out.points().windows(2) {
            let dt = (w[1].time_from_start - w[0].time_from_start).get::<second>();
            assert!(dt > 0.0, "times must be strictly increasing");
            for j in 0..3 {
                let v = w[1].velocities[j].si_value();
                assert!(v.abs() <= 0.4 * (1.0 + 1e-6), "velocity {v}");
                let a = (v - w[0].velocities[j].si_value()) / dt;
                assert!(a.abs() <= 1.0 * (1.0 + 1e-3), "acceleration {a}");
            }
        }
    }

    #[test]
    fn every_profile_flies_through_a_positive_zone() {
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        for profile in PROFILES {
            let (status, out) = timed(&points, [0.0, 0.1, 0.0], profile);
            assert_eq!(status, CuTimingStatus::Success, "{profile:?}");
            assert_within_limits(&out);
            assert!(at_rest(&out.points()[0], 0.0, 0.0));
            assert!(at_rest(out.points().last().unwrap(), 0.3, 0.3));
            let interior = &out.points()[1..out.len() - 1];
            assert!(
                interior
                    .iter()
                    .all(|p| p.velocities.iter().any(|v| v.si_value().abs() > 1e-6)),
                "{profile:?} must not stop before the path end"
            );
        }
    }

    #[test]
    fn every_profile_stops_exactly_at_a_zero_zone() {
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        for profile in PROFILES {
            let (status, out) = timed(&points, [0.0; 3], profile);
            assert_eq!(status, CuTimingStatus::Success, "{profile:?}");
            assert_within_limits(&out);
            assert!(out.points().iter().any(|p| at_rest(p, 0.3, 0.0)));
        }
    }

    #[test]
    fn flying_through_a_zone_is_faster_than_stopping() {
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        for profile in PROFILES {
            let (_, blended) = timed(&points, [0.0, 0.1, 0.0], profile);
            let (_, stopped) = timed(&points, [0.0; 3], profile);
            assert!(
                duration(&blended) < duration(&stopped),
                "{profile:?}: {} >= {}",
                duration(&blended),
                duration(&stopped)
            );
        }
    }

    #[test]
    fn zones_are_per_waypoint_on_one_path() {
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3), q(0.0, 0.3)];
        for profile in PROFILES {
            let (status, out) = timed(&points, [0.0, 0.1, 0.0, 0.0], profile);
            assert_eq!(status, CuTimingStatus::Success, "{profile:?}");
            assert_within_limits(&out);
            assert!(!out.points().iter().any(|p| at_rest(p, 0.3, 0.0)));
            assert!(out.points().iter().any(|p| at_rest(p, 0.3, 0.3)));
            assert!(at_rest(out.points().last().unwrap(), 0.0, 0.3));
        }
    }

    #[test]
    fn sample_cubic_is_continuous_across_stored_points() {
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let (status, out) = timed(&points, [0.0, 0.1, 0.0], CuTrajectoryProfile::Cubic);
        assert_eq!(status, CuTimingStatus::Success);
        for p in &out.points()[1..out.len() - 1] {
            let t = p.time_from_start.get::<second>();
            let mut before = (q(0.0, 0.0), [CuJointVelocity::default(); 3]);
            let mut after = before;
            out.sample_cubic(Time::new::<second>(t - 1e-7), &mut before.0, &mut before.1)
                .unwrap();
            out.sample_cubic(Time::new::<second>(t + 1e-7), &mut after.0, &mut after.1)
                .unwrap();
            for j in 0..3 {
                assert!((before.0[j].si_value() - after.0[j].si_value()).abs() < 1e-5);
                assert!((before.1[j].si_value() - after.1[j].si_value()).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn scurve_path_acceleration_is_continuous() {
        let points = [q(0.0, 0.0), q(0.3, 0.0)];
        let (status, out) = timed(&points, [0.0, 0.0], CuTrajectoryProfile::SCurve);
        assert_eq!(status, CuTimingStatus::Success);
        // Straight line: joint 0 carries the path motion, so the change of
        // its stored acceleration between knots is bounded by the jerk limit.
        for w in out.points().windows(2) {
            let dt = (w[1].time_from_start - w[0].time_from_start).get::<second>();
            let jerk = (w[1].accelerations[0].si_value() - w[0].accelerations[0].si_value()) / dt;
            assert!(jerk.abs() <= 4.0 * (1.0 + 1e-6), "jerk {jerk}");
        }
    }

    #[test]
    fn interpolated_path_is_rechecked_for_collisions() {
        let (chain, model) = robot();
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        blend_cartesian_path(
            &chain,
            &model,
            &CuCollisionEnv::EMPTY,
            &waypoints(&points, [0.0, 0.1, 0.0]),
            &CuCartesianBlendParams::default(),
            &mut path,
        )
        .unwrap();
        let obstacles = [obstacle()];
        let env = CuCollisionEnv::new(&obstacles);
        let mut out = CuTrajectoryPayload::<3, 512>::new();
        let status = parameterize_cartesian_path(
            &chain,
            &model,
            &env,
            &path,
            &timing(CuTrajectoryProfile::Cubic),
            &mut out,
        )
        .unwrap();
        assert!(matches!(status, CuTimingStatus::Collision { .. }));
        assert!(out.is_empty());
    }

    #[test]
    fn reversal_inside_a_zone_cannot_keep_moving() {
        let (chain, model) = robot();
        let env = CuCollisionEnv::EMPTY;
        let zone = CuTcpZone {
            waypoint: 1,
            center: [0.3, 0.0, 0.0].map(Length::new::<meter>),
            radius: Length::new::<meter>(0.1),
        };
        let mut path = CuCartesianBlendPath::<3, 8>::default();
        path.points[0] = CuCartesianPathPoint {
            positions: q(0.0, 0.0),
            stop: true,
            zone: None,
        };
        path.points[1] = CuCartesianPathPoint {
            positions: q(0.3, 0.0),
            stop: false,
            zone: Some(zone),
        };
        path.points[2] = CuCartesianPathPoint {
            positions: q(0.0, 0.0),
            stop: true,
            zone: None,
        };
        path.len = 3;
        let mut out = CuTrajectoryPayload::<3, 512>::new();
        let status = parameterize_cartesian_path(
            &chain,
            &model,
            &env,
            &path,
            &timing(CuTrajectoryProfile::Trapezoidal),
            &mut out,
        )
        .unwrap();
        assert_eq!(status, CuTimingStatus::NoContinuousMotion { waypoint: 1 });
        assert!(out.is_empty());
    }

    #[test]
    fn invalid_period_and_small_capacity_leave_no_output() {
        let (chain, model) = robot();
        let env = CuCollisionEnv::EMPTY;
        let points = [q(0.0, 0.0), q(0.3, 0.0), q(0.3, 0.3)];
        let mut path = CuCartesianBlendPath::<3, 256>::default();
        blend_cartesian_path(
            &chain,
            &model,
            &env,
            &waypoints(&points, [0.0, 0.1, 0.0]),
            &CuCartesianBlendParams::default(),
            &mut path,
        )
        .unwrap();
        let mut config = timing(CuTrajectoryProfile::Cubic);
        config.timing.sample_period = Time::new::<second>(0.0);
        let mut out = CuTrajectoryPayload::<3, 512>::new();
        assert_eq!(
            parameterize_cartesian_path(&chain, &model, &env, &path, &config, &mut out).unwrap(),
            CuTimingStatus::InvalidInput
        );
        assert!(out.is_empty());
        let mut small = CuTrajectoryPayload::<3, 8>::new();
        assert_eq!(
            parameterize_cartesian_path(
                &chain,
                &model,
                &env,
                &path,
                &timing(CuTrajectoryProfile::Cubic),
                &mut small
            )
            .unwrap(),
            CuTimingStatus::CapacityExceeded
        );
        assert!(small.is_empty());
    }

    #[test]
    fn rebel_flyby_is_timed_with_every_profile() {
        use cu29::units::si::angle::radian;
        use cu29::units::si::angular_velocity::radian_per_second;
        use cu29::units::si::f64::Angle;
        use cu29::units::si::f64::AngularVelocity;
        let chain = cu_kinematics::igus_rebel_chain();
        let model = cu_collision::igus_rebel_collision_model();
        let env = CuCollisionEnv::EMPTY;
        let to_q =
            |j1: f64| [j1, 0.75, -0.25, 0.0, -0.3, 0.0].map(|v| Angle::new::<radian>(v).into());
        let points = [to_q(-0.75), to_q(0.0), to_q(0.75)];
        let zones = [0.0, 0.03, 0.0].map(Length::new::<meter>);
        let waypoints: [CuCartesianWaypoint<6>; 3] =
            core::array::from_fn(|i| CuCartesianWaypoint::new(points[i], zones[i]));
        let mut blend = CuCartesianBlendParams::default();
        blend.ik.max_iterations = 256;
        let mut path = CuCartesianBlendPath::<6, 256>::default();
        assert_eq!(
            blend_cartesian_path(&chain, &model, &env, &waypoints, &blend, &mut path).unwrap(),
            CuCartesianBlendStatus::Success
        );
        let velocity = CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(1.0));
        for profile in PROFILES {
            let rev = points[0][0];
            let config = CuPathTimingParams {
                timing: CuTimingParams {
                    limits: CuJointTrajectoryLimits::new(
                        [velocity; 6],
                        [CuJointAcceleration::from_coordinate(rev, 2.0); 6],
                    )
                    .with_jerks([CuJointJerk::from_coordinate(rev, 10.0); 6]),
                    profile,
                    sample_period: Time::new::<second>(0.02),
                },
                collision_step: blend.collision_step,
            };
            let mut out = CuTrajectoryPayload::<6, 1024>::new();
            assert_eq!(
                parameterize_cartesian_path(&chain, &model, &env, &path, &config, &mut out)
                    .unwrap(),
                CuTimingStatus::Success,
                "{profile:?}"
            );
        }
    }

    #[test]
    fn output_interpolation_matches_the_validated_reconstruction() {
        let points = [q(0.0, 0.0), q(0.3, 0.0)];
        for profile in PROFILES {
            let (status, out) = timed(&points, [0.0, 0.0], profile);
            assert_eq!(status, CuTimingStatus::Success);
            let expected = if profile == CuTrajectoryProfile::SCurve {
                CuTrajectoryInterpolation::Quintic
            } else {
                CuTrajectoryInterpolation::Cubic
            };
            assert_eq!(out.interpolation, expected, "{profile:?}");
        }
    }
}
