//! Profile-selectable timing for sampled Cartesian blend paths.
//!
//! The path knots carry stop flags from the blend construction: exact-stop
//! knots (path start/end and zero-radius waypoints) receive zero velocity,
//! other knots receive shape-preserving harmonic-mean joint velocities, so
//! adjacent segments share the same velocity and motion is C1-continuous
//! through fly-by zones. Segment durations are uniformly stretched until
//! analytic velocity and acceleration extrema meet every configured joint
//! bound. Jerk is not bounded by these profiles.
//!
//! [`CuTrajectoryProfile::Cubic`] and [`CuTrajectoryProfile::MinimumJerk`]
//! honor the knot velocities and fly through zones. The rest-to-rest
//! families ([`CuTrajectoryProfile::Trapezoidal`] and
//! [`CuTrajectoryProfile::SCurve`]) take no endpoint velocity by
//! construction, so they can only stop: any positive zone rejects the job
//! with [`CuContinuousStatus::NoContinuousMotion`] naming the waypoint, and
//! an all-stop path is timed with the same rest-to-rest primitives as
//! [`parameterize_with_profile`](crate::parameterize::parameterize_with_profile).

use crate::CuTrajectoryPayload;
use crate::CuTrajectoryPoint;
use crate::cartesian_blend::CuCartesianBlendConfig;
use crate::cartesian_blend::CuCartesianBlendPath;
use crate::parameterize::CuJointTrajectoryLimits;
use crate::parameterize::CuTrajectoryProfile;
use crate::parameterize::JointPrimitive;
use crate::parameterize::joint_profile_duration;
use crate::parameterize::quintic_eval;
use crate::parameterize::scurve_joint_duration;
use crate::parameterize::stretched_eval;
use bincode::Decode;
use bincode::Encode;
use cu_collision::CuCollisionWorld;
use cu_collision::CuRobotCollisionModel;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Time;
use cu29::units::si::length::meter;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;

/// Outcome of continuous cubic timing and discrete validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuContinuousStatus {
    Success,
    InvalidInput,
    CapacityExceeded,
    Collision {
        segment: usize,
    },
    ZoneExceeded {
        waypoint: usize,
    },
    /// The joint-space path cannot maintain motion through this zone.
    NoContinuousMotion {
        waypoint: usize,
    },
    /// A non-stop path sample would force every joint to stop.
    UnexpectedStop {
        sample: usize,
    },
}

fn cubic(q0: f64, q1: f64, v0: f64, v1: f64, t: f64, u: f64) -> (f64, f64) {
    let d = q1 - q0;
    let a = v0 * t;
    let b = 3.0 * d - 2.0 * a - v1 * t;
    let c = -2.0 * d + a + v1 * t;
    (
        q0 + u * (a + u * (b + u * c)),
        (a + u * (2.0 * b + 3.0 * c * u)) / t,
    )
}
fn extrema(q0: f64, q1: f64, v0: f64, v1: f64, t: f64) -> (f64, f64) {
    let d = q1 - q0;
    let a = v0 * t;
    let b = 3.0 * d - 2.0 * a - v1 * t;
    let c = -2.0 * d + a + v1 * t;
    let mut v = libm::fabs(v0).max(libm::fabs(v1));
    if libm::fabs(c) > 1e-18 {
        let u = -b / (3.0 * c);
        if u > 0.0 && u < 1.0 {
            v = v.max(libm::fabs((a + u * (2.0 * b + 3.0 * c * u)) / t));
        }
    }
    let acc = libm::fabs(2.0 * b / (t * t)).max(libm::fabs((2.0 * b + 6.0 * c) / (t * t)));
    (v, acc)
}
fn point<const N: usize>(
    a: [CuJointPosition; N],
    b: [CuJointPosition; N],
    va: [f64; N],
    vb: [f64; N],
    duration: f64,
    u: f64,
) -> ([CuJointPosition; N], [CuJointVelocity; N]) {
    let mut q = a;
    let mut qd = [CuJointVelocity::default(); N];
    for j in 0..N {
        let (x, v) = cubic(a[j].si_value(), b[j].si_value(), va[j], vb[j], duration, u);
        q[j] = a[j].with_si_value(x);
        qd[j] = CuJointVelocity::from_coordinate(a[j], v);
    }
    (q, qd)
}

/// Conservative analytic velocity bound for a quintic Hermite segment, used
/// to size durations before output.
///
/// Writing the velocity as `dH01·(d/T) + dH10·v0 + dH11·v1`, the basis maxima
/// `|dH01| <= 1.875` and `|dH10| + |dH11| <= 1` (verified numerically over the
/// unit interval) give a peak velocity of at most
/// `1.875|d|/T + max(|v0|, |v1|)`. The `max` (not the sum) keeps the bound
/// satisfiable: knot velocities never exceed the limit, so stretching `T`
/// always succeeds.
fn quintic_vel_bound(q0: f64, q1: f64, v0: f64, v1: f64, t: f64) -> f64 {
    let d = libm::fabs(q1 - q0);
    1.875 * d / t + libm::fabs(v0).max(libm::fabs(v1))
}

/// Exact acceleration peak of a quintic Hermite segment.
///
/// The acceleration is quadratic in normalized time,
/// `a(u)·T^2 = A·u^3 + B·u^2 + C·u` with `a(0) = a(1) = 0` (zero endpoint
/// accelerations by construction), so the peak over `[0, 1]` sits at an
/// endpoint or at a root of the quadratic derivative. This is the tight bound
/// the duration loop needs: conservative estimates force runaway durations on
/// dense blend samples.
fn quintic_accel_peak(q0: f64, q1: f64, v0: f64, v1: f64, t: f64) -> f64 {
    let d = q1 - q0;
    let a = 120.0 * d - 60.0 * t * (v0 + v1);
    let b = -180.0 * d + 96.0 * t * v0 + 84.0 * t * v1;
    let c = 60.0 * d - 36.0 * t * v0 - 24.0 * t * v1;
    // |a(u)| = |A·u^3 + B·u^2 + C·u| / T^2; u = 0 evaluates to exactly 0.
    let mut peak = libm::fabs(a + b + c);
    if libm::fabs(a) > 1e-300 {
        let disc = 4.0 * b * b - 12.0 * a * c;
        if disc > 0.0 {
            let root = libm::sqrt(disc);
            let u0 = (-2.0 * b + root) / (6.0 * a);
            if u0 > 0.0 && u0 < 1.0 {
                peak = peak.max(libm::fabs(a * u0 * u0 * u0 + b * u0 * u0 + c * u0));
            }
            let u1 = (-2.0 * b - root) / (6.0 * a);
            if u1 > 0.0 && u1 < 1.0 {
                peak = peak.max(libm::fabs(a * u1 * u1 * u1 + b * u1 * u1 + c * u1));
            }
        }
    } else if libm::fabs(b) > 1e-300 {
        let u = -c / (2.0 * b);
        if u > 0.0 && u < 1.0 {
            peak = peak.max(libm::fabs(b * u * u + c * u));
        }
    }
    peak / (t * t)
}

fn quintic_point<const N: usize>(
    a: [CuJointPosition; N],
    b: [CuJointPosition; N],
    va: [f64; N],
    vb: [f64; N],
    duration: f64,
    u: f64,
) -> ([CuJointPosition; N], [CuJointVelocity; N]) {
    let mut q = a;
    let mut qd = [CuJointVelocity::default(); N];
    for j in 0..N {
        let (x, v) = quintic_eval(u, a[j].si_value(), b[j].si_value(), va[j], vb[j], duration);
        q[j] = a[j].with_si_value(x);
        qd[j] = CuJointVelocity::from_coordinate(a[j], v);
    }
    (q, qd)
}

/// Peak velocity/acceleration estimate for one C1 segment, used to size
/// collision-check subdivision and duration stretching.
///
/// Exact cubic Hermite extrema for [`CuTrajectoryProfile::Cubic`]; exact
/// acceleration peak plus a conservative velocity bound for
/// [`CuTrajectoryProfile::MinimumJerk`].
fn knot_peak(
    profile: CuTrajectoryProfile,
    q0: f64,
    q1: f64,
    v0: f64,
    v1: f64,
    t: f64,
) -> (f64, f64) {
    if profile == CuTrajectoryProfile::MinimumJerk {
        (
            quintic_vel_bound(q0, q1, v0, v1, t),
            quintic_accel_peak(q0, q1, v0, v1, t),
        )
    } else {
        extrema(q0, q1, v0, v1, t)
    }
}

/// Evaluates one blend segment at normalized time `u`.
///
/// C1 profiles interpolate the knot velocities (cubic or quintic Hermite);
/// rest-to-rest profiles evaluate the per-joint primitive stretched onto the
/// span duration. `owns` holds each joint's natural primitive duration for
/// the span and is only read when `rest_only`.
#[allow(clippy::too_many_arguments)]
fn eval_span<const N: usize, const CAP: usize>(
    profile: CuTrajectoryProfile,
    rest_only: bool,
    jerk_limited: bool,
    a: [CuJointPosition; N],
    b: [CuJointPosition; N],
    va: [f64; N],
    vb: [f64; N],
    limits: &CuJointTrajectoryLimits<N>,
    owns: &[[f64; CAP]; N],
    span: usize,
    duration: f64,
    u: f64,
) -> ([CuJointPosition; N], [CuJointVelocity; N]) {
    if rest_only {
        let mut q = a;
        let mut qd = [CuJointVelocity::default(); N];
        for j in 0..N {
            let start = a[j].si_value();
            let displacement = b[j].si_value() - start;
            let sign = if displacement >= 0.0 { 1.0 } else { -1.0 };
            let prim = JointPrimitive {
                distance: libm::fabs(displacement),
                max_vel: limits.max_velocities[j].si_value(),
                max_acc: limits.max_accelerations[j],
                max_jerk: limits.max_jerks[j],
            };
            let tau = (u * duration).clamp(0.0, duration);
            let (offset, vel) = stretched_eval(tau, duration, owns[j][span], &prim, jerk_limited);
            q[j] = a[j].with_si_value(start + sign * offset);
            qd[j] = CuJointVelocity::from_coordinate(a[j], sign * vel);
        }
        return (q, qd);
    }
    if profile == CuTrajectoryProfile::MinimumJerk {
        quintic_point(a, b, va, vb, duration, u)
    } else {
        point(a, b, va, vb, duration, u)
    }
}

/// Programmed tool-space anchors and zone radii for a checked path.
#[derive(Debug, Clone, Copy)]
pub struct CuContinuousAnchors<'a, const N: usize> {
    pub positions: &'a [[CuJointPosition; N]],
    pub zones: &'a [Length],
}

/// Timing and validation settings for continuous Cartesian blending.
#[derive(Debug, Clone, Copy)]
pub struct CuContinuousConfig<'a, const N: usize> {
    pub limits: &'a CuJointTrajectoryLimits<N>,
    pub blend: &'a CuCartesianBlendConfig,
    pub sample_period: Time,
    /// Timing profile for the blend. [`CuTrajectoryProfile::Cubic`] (the
    /// default behavior) and [`CuTrajectoryProfile::MinimumJerk`] honor the
    /// harmonic-mean knot velocities and fly through zones. The rest-to-rest
    /// families ([`CuTrajectoryProfile::Trapezoidal`],
    /// [`CuTrajectoryProfile::SCurve`]) can only stop, so any positive zone
    /// rejects the job with [`CuContinuousStatus::NoContinuousMotion`].
    pub profile: CuTrajectoryProfile,
}

/// Times a checked Cartesian blend path with the configured profile.
///
/// Exact-stop knots have zero velocity; other knots use shape-preserving
/// harmonic-mean joint velocities. With [`CuTrajectoryProfile::Cubic`] or
/// [`CuTrajectoryProfile::MinimumJerk`] adjacent segments share the knot
/// velocity and motion is C1-continuous through fly-by zones; analytic
/// velocity and acceleration extrema (exact for cubic, conservative for
/// minimum-jerk) are checked across each full piece, independently of the
/// output period. With [`CuTrajectoryProfile::Trapezoidal`] or
/// [`CuTrajectoryProfile::SCurve`] every knot must be a stop: a positive
/// zone rejects the job with [`CuContinuousStatus::NoContinuousMotion`],
/// otherwise segments use the same rest-to-rest primitives as
/// [`parameterize_with_profile`](crate::parameterize::parameterize_with_profile).
/// These profiles do not bound jerk. Discrete joint-space collision and
/// TCP-zone checks are repeated on the timed interpolation. `sample_cubic`
/// (or a cubic follower) must be used when executing a cubic or minimum-jerk
/// result; rest-to-rest results sample exactly with either follower mode.
/// The checks are sampled, not swept-volume continuous collision detection.
/// On failure `out` is empty.
pub fn parameterize_cartesian_blend<
    const N: usize,
    const M: usize,
    const O: usize,
    const CAP: usize,
    const OUT: usize,
>(
    chain: &CuKinematicChain<N>,
    model: &CuRobotCollisionModel<N, M>,
    world: Option<&CuCollisionWorld<O>>,
    anchors: CuContinuousAnchors<'_, N>,
    path: &CuCartesianBlendPath<N, CAP>,
    settings: CuContinuousConfig<'_, N>,
    out: &mut CuTrajectoryPayload<N, OUT>,
) -> CuResult<CuContinuousStatus> {
    out.clear();
    let CuContinuousAnchors {
        positions: anchors,
        zones,
    } = anchors;
    let CuContinuousConfig {
        limits,
        blend: config,
        sample_period,
        profile,
    } = settings;
    let rest_only = matches!(
        profile,
        CuTrajectoryProfile::Trapezoidal | CuTrajectoryProfile::SCurve
    );
    let jerk_limited = profile == CuTrajectoryProfile::SCurve;
    let result = (|| -> CuResult<CuContinuousStatus> {
        let count = path.len;
        let dt = sample_period.get::<second>();
        if count < 2
            || count > CAP
            || anchors.len() != zones.len()
            || !path.points[0].stop
            || !path.points[count - 1].stop
            || !dt.is_finite()
            || dt <= 0.0
            || config.collision_step.metric().validate().is_err()
        {
            return Ok(CuContinuousStatus::InvalidInput);
        }
        for j in 0..N {
            let valid_kind = matches!(
                (path.points[0].positions[j], limits.max_velocities[j]),
                (CuJointPosition::Revolute(_), CuJointVelocity::Revolute(_))
                    | (CuJointPosition::Prismatic(_), CuJointVelocity::Prismatic(_))
                    | (CuJointPosition::Fixed, CuJointVelocity::Fixed)
            );
            let v = limits.max_velocities[j].si_value();
            let a = limits.max_accelerations[j];
            if !valid_kind
                || (!matches!(path.points[0].positions[j], CuJointPosition::Fixed)
                    && (!v.is_finite() || v <= 0.0 || !a.is_finite() || a <= 0.0))
            {
                return Ok(CuContinuousStatus::InvalidInput);
            }
        }
        let mut center = [[0.0; 3]; CAP];
        if anchors.len() > CAP {
            return Ok(CuContinuousStatus::InvalidInput);
        }
        for (i, &anchor) in anchors.iter().enumerate() {
            if !zones[i].get::<meter>().is_finite() || zones[i].get::<meter>() < 0.0 {
                return Ok(CuContinuousStatus::InvalidInput);
            }
            let mut pose = Transform3D::<f64>::identity();
            chain.forward_kinematics(&anchor, &mut pose)?;
            let m = pose.to_matrix();
            center[i] = [m[0][3], m[1][3], m[2][3]];
        }
        if jerk_limited {
            for j in 0..N {
                if !matches!(path.points[0].positions[j], CuJointPosition::Fixed)
                    && (limits.max_jerks[j] <= 0.0 || limits.max_jerks[j].is_nan())
                {
                    return Ok(CuContinuousStatus::InvalidInput);
                }
            }
        }
        if rest_only {
            // Rest-to-rest profiles stop at every knot by construction, so a
            // fly-by zone is unreachable. Dense interior samples without a
            // zone are fine: spans are timed stop-to-stop below.
            for i in 1..count - 1 {
                if let Some(waypoint) = path.points[i].zone {
                    return Ok(CuContinuousStatus::NoContinuousMotion { waypoint });
                }
            }
        }
        let mut durations = [0.0_f64; CAP];
        for (i, duration_slot) in durations[..count - 1].iter_mut().enumerate() {
            let a = path.points[i].positions;
            let b = path.points[i + 1].positions;
            if !chain.is_within_limits(&a) || !chain.is_within_limits(&b) {
                return Ok(CuContinuousStatus::InvalidInput);
            }
            if rest_only {
                // Stop-to-stop spans are timed below; here only validate
                // joint-kind agreement across the dense samples.
                for j in 0..N {
                    if core::mem::discriminant(&a[j]) != core::mem::discriminant(&b[j]) {
                        return Ok(CuContinuousStatus::InvalidInput);
                    }
                }
                *duration_slot = 0.0;
                continue;
            }
            let mut t = 0.005_f64;
            for j in 0..N {
                if core::mem::discriminant(&a[j]) != core::mem::discriminant(&b[j]) {
                    return Ok(CuContinuousStatus::InvalidInput);
                }
                if matches!(a[j], CuJointPosition::Fixed) {
                    continue;
                }
                let d = libm::fabs(b[j].si_value() - a[j].si_value());
                t = t.max(1.5 * d / limits.max_velocities[j].si_value());
                t = t.max(libm::sqrt(6.0 * d / limits.max_accelerations[j]));
            }
            *duration_slot = t;
        }
        let mut velocities = [[0.0_f64; N]; CAP];
        let mut valid = false;
        // Rest-to-rest durations satisfy every bound by construction (the
        // primitives are derived from the limits and only ever stretched),
        // so no iterative validation is needed.
        if rest_only {
            valid = true;
        }
        for _ in 0..20 {
            if rest_only {
                break;
            }
            for i in 1..count - 1 {
                velocities[i] = [0.0; N];
                if path.points[i].stop {
                    continue;
                }
                for (j, value) in velocities[i].iter_mut().enumerate() {
                    let left = (path.points[i].positions[j].si_value()
                        - path.points[i - 1].positions[j].si_value())
                        / durations[i - 1];
                    let right = (path.points[i + 1].positions[j].si_value()
                        - path.points[i].positions[j].si_value())
                        / durations[i];
                    if left * right > 0.0 {
                        *value = 2.0 * left * right / (left + right);
                    }
                }
            }
            valid = true;
            for i in 0..count - 1 {
                for (j, &velocity) in velocities[i].iter().enumerate() {
                    if matches!(path.points[i].positions[j], CuJointPosition::Fixed) {
                        continue;
                    }
                    let (v, a) = if profile == CuTrajectoryProfile::MinimumJerk {
                        (
                            quintic_vel_bound(
                                path.points[i].positions[j].si_value(),
                                path.points[i + 1].positions[j].si_value(),
                                velocity,
                                velocities[i + 1][j],
                                durations[i],
                            ),
                            quintic_accel_peak(
                                path.points[i].positions[j].si_value(),
                                path.points[i + 1].positions[j].si_value(),
                                velocity,
                                velocities[i + 1][j],
                                durations[i],
                            ),
                        )
                    } else {
                        extrema(
                            path.points[i].positions[j].si_value(),
                            path.points[i + 1].positions[j].si_value(),
                            velocity,
                            velocities[i + 1][j],
                            durations[i],
                        )
                    };
                    if !v.is_finite()
                        || !a.is_finite()
                        || v > limits.max_velocities[j].si_value() * (1.0 + 1e-10)
                        || a > limits.max_accelerations[j] * (1.0 + 1e-10)
                    {
                        valid = false;
                    }
                }
            }
            if valid {
                break;
            }
            for t in &mut durations[..count - 1] {
                *t *= 2.0;
            }
        }
        if !valid {
            return Ok(CuContinuousStatus::InvalidInput);
        }
        if !rest_only {
            for (i, v) in velocities.iter().enumerate().take(count - 1).skip(1) {
                if !path.points[i].stop && v.iter().all(|speed| libm::fabs(*speed) <= 1e-12) {
                    return Ok(match path.points[i].zone {
                        Some(waypoint) => CuContinuousStatus::NoContinuousMotion { waypoint },
                        None => CuContinuousStatus::UnexpectedStop { sample: i },
                    });
                }
            }
        }
        if OUT == 0 {
            return Ok(CuContinuousStatus::CapacityExceeded);
        }
        let zero = path.points[0]
            .positions
            .map(|q| CuJointVelocity::from_coordinate(q, 0.0));
        out.push(CuTrajectoryPoint {
            time_from_start: Time::default(),
            positions: path.points[0].positions,
            velocities: zero,
        })?;
        // Evaluation spans: dense knot pairs for the C1 profiles (timed with
        // the knot velocities), stop-to-stop spans for the rest-to-rest
        // profiles (timed with synchronized primitives).
        let mut starts = [0usize; CAP];
        let mut ends = [0usize; CAP];
        let mut spans = [0.0_f64; CAP];
        let mut owns = [[0.0_f64; CAP]; N];
        let mut nspans = 0usize;
        if rest_only {
            let mut s = 0;
            while s < count - 1 {
                let mut e = s + 1;
                while e < count - 1 && !path.points[e].stop {
                    e += 1;
                }
                let a = path.points[s].positions;
                let b = path.points[e].positions;
                let mut t = 0.005_f64;
                for j in 0..N {
                    if matches!(a[j], CuJointPosition::Fixed) {
                        continue;
                    }
                    let d = libm::fabs(b[j].si_value() - a[j].si_value());
                    let own = if jerk_limited {
                        scurve_joint_duration(
                            d,
                            limits.max_velocities[j].si_value(),
                            limits.max_accelerations[j],
                            limits.max_jerks[j],
                        )
                    } else {
                        joint_profile_duration(
                            d,
                            limits.max_velocities[j].si_value(),
                            limits.max_accelerations[j],
                        )
                    };
                    owns[j][nspans] = own;
                    t = t.max(own);
                }
                starts[nspans] = s;
                ends[nspans] = e;
                spans[nspans] = t;
                nspans += 1;
                s = e;
            }
        } else {
            let mut i = 0;
            while i < count - 1 {
                starts[i] = i;
                ends[i] = i + 1;
                spans[i] = durations[i];
                i += 1;
            }
            nspans = count - 1;
        }
        let mut elapsed = 0.0;
        let mut s = 0;
        while s < nspans {
            let si = starts[s];
            let ei = ends[s];
            let a = path.points[si].positions;
            let b = path.points[ei].positions;
            let duration = spans[s];
            let output_steps = libm::ceil(duration / dt).max(1.0);
            if !output_steps.is_finite() || output_steps > OUT.saturating_sub(out.len()) as f64 {
                return Ok(CuContinuousStatus::CapacityExceeded);
            }
            let mut checks = output_steps as usize;
            let metric = config.collision_step.metric();
            for j in 0..N {
                let peak = if rest_only {
                    // Stretched primitives peak at or below the joint limit.
                    limits.max_velocities[j].si_value()
                } else {
                    let (v, _) = knot_peak(
                        profile,
                        a[j].si_value(),
                        b[j].si_value(),
                        velocities[si][j],
                        velocities[ei][j],
                        duration,
                    );
                    v
                };
                let scale = metric.scale(a[j]);
                let needed = libm::ceil(peak * duration / scale).max(1.0);
                if !needed.is_finite() || needed > 100_000.0 {
                    return Ok(CuContinuousStatus::InvalidInput);
                }
                checks = checks.max(needed as usize);
            }
            // A span carries a zone only when every knot in it agrees on one.
            // (Dense C1 spans hold two knots, so this matches the endpoint
            // rule; rest spans never carry zones.)
            let mut zone: Option<usize> = None;
            let mut uniform = true;
            let mut k = si;
            while k <= ei {
                match (zone, path.points[k].zone) {
                    (None, z) => zone = z,
                    (Some(x), Some(y)) if x == y => {}
                    _ => uniform = false,
                }
                k += 1;
            }
            if !uniform {
                zone = None;
            }
            if zone.is_some_and(|z| z >= anchors.len() || zones[z].get::<meter>() <= 0.0) {
                return Ok(CuContinuousStatus::InvalidInput);
            }
            let va = velocities[si];
            let vb = velocities[ei];
            let mut previous = a;
            for k in 1..=checks {
                let (q, _) = eval_span(
                    profile,
                    rest_only,
                    jerk_limited,
                    a,
                    b,
                    va,
                    vb,
                    limits,
                    &owns,
                    s,
                    duration,
                    k as f64 / checks as f64,
                );
                if !model.is_configuration_collision_free(chain, &q, world)?
                    || !model.is_segment_collision_free(
                        chain,
                        &previous,
                        &q,
                        config.collision_step,
                        world,
                    )?
                {
                    return Ok(CuContinuousStatus::Collision { segment: si });
                }
                if let Some(z) = zone {
                    let mut pose = Transform3D::<f64>::identity();
                    chain.forward_kinematics(&q, &mut pose)?;
                    let m = pose.to_matrix();
                    let d = [
                        m[0][3] - center[z][0],
                        m[1][3] - center[z][1],
                        m[2][3] - center[z][2],
                    ];
                    if libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) > zones[z].get::<meter>()
                    {
                        return Ok(CuContinuousStatus::ZoneExceeded { waypoint: z });
                    }
                }
                previous = q;
            }
            for k in 1..=output_steps as usize {
                let (positions, vels) = eval_span(
                    profile,
                    rest_only,
                    jerk_limited,
                    a,
                    b,
                    va,
                    vb,
                    limits,
                    &owns,
                    s,
                    duration,
                    k as f64 / output_steps,
                );
                if !model.is_configuration_collision_free(chain, &positions, world)? {
                    return Ok(CuContinuousStatus::Collision { segment: si });
                }
                if let Some(z) = zone {
                    let mut pose = Transform3D::<f64>::identity();
                    chain.forward_kinematics(&positions, &mut pose)?;
                    let m = pose.to_matrix();
                    let d = [
                        m[0][3] - center[z][0],
                        m[1][3] - center[z][1],
                        m[2][3] - center[z][2],
                    ];
                    if libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) > zones[z].get::<meter>()
                    {
                        return Ok(CuContinuousStatus::ZoneExceeded { waypoint: z });
                    }
                }
                out.push(CuTrajectoryPoint {
                    time_from_start: Time::new::<second>(
                        elapsed + duration * k as f64 / output_steps,
                    ),
                    positions,
                    velocities: vels,
                })?;
            }
            elapsed += duration;
            s += 1;
        }
        Ok(CuContinuousStatus::Success)
    })();
    if !matches!(result, Ok(CuContinuousStatus::Success)) {
        out.clear();
    }
    result
}
