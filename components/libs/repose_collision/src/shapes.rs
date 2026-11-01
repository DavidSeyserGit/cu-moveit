//! Sphere and capsule collision primitives and exact signed-distance queries.

use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Serialize;

/// 3D sphere primitive defined by a center point and radius.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuSphere {
    pub center: [Length64; 3],
    pub radius: Length64,
}

impl CuSphere {
    /// Creates a sphere, rejecting non-finite coordinates and negative or non-finite radii.
    pub fn new(center: [Length64; 3], radius: Length64) -> CuResult<Self> {
        let sphere = Self { center, radius };
        CuCollisionShape::Sphere(sphere).validate()?;
        Ok(sphere)
    }
}

/// 3D capsule (swept sphere along line segment `a` -> `b`) with radius.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCapsule {
    pub a: [Length64; 3],
    pub b: [Length64; 3],
    pub radius: Length64,
}

impl CuCapsule {
    /// Creates a capsule, rejecting non-finite endpoints and negative or non-finite radii.
    pub fn new(a: [Length64; 3], b: [Length64; 3], radius: Length64) -> CuResult<Self> {
        let capsule = Self { a, b, radius };
        CuCollisionShape::Capsule(capsule).validate()?;
        Ok(capsule)
    }
}

/// Convex collision shape primitive (`Sphere` or `Capsule`).
///
/// RON: `{"sphere": {"center": [x, y, z], "radius": r}}` or
/// `{"capsule": {"a": [x, y, z], "b": [x, y, z], "radius": r}}`, in meters.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuCollisionShape {
    Sphere(CuSphere),
    Capsule(CuCapsule),
}

impl Default for CuCollisionShape {
    fn default() -> Self {
        Self::Sphere(CuSphere::default())
    }
}

impl CuCollisionShape {
    /// Checks that all coordinates are finite and the radius is finite and non-negative.
    pub fn validate(&self) -> CuResult<()> {
        let finite3 = |p: &[Length64; 3]| p.iter().all(|v| v.get::<meter>().is_finite());
        let (finite, radius) = match self {
            Self::Sphere(s) => (finite3(&s.center), s.radius),
            Self::Capsule(c) => (finite3(&c.a) && finite3(&c.b), c.radius),
        };
        let r = radius.get::<meter>();
        if finite && r.is_finite() && r >= 0.0 {
            Ok(())
        } else {
            Err(CuError::from(
                "repose_collision: invalid or non-finite collision shape geometry",
            ))
        }
    }
}

/// A robot collision shape and the frame it is expressed in.
///
/// `frame` is a joint-frame index (`0..N`, as produced by
/// `CuKinematicChain::forward_frames`) or `None` for the robot base frame.
///
/// RON: `{"shape": {"sphere": {...}}, "frame": 2}`; omit `frame` for the base frame.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(deny_unknown_fields)]
pub struct CuLinkShape {
    /// Geometry in the local coordinates of `frame`.
    pub shape: CuCollisionShape,
    /// Joint-frame index, or `None` for the base frame.
    #[serde(default)]
    pub frame: Option<usize>,
}

impl CuLinkShape {
    /// Creates a link shape on `frame` (`None` = base frame).
    pub const fn new(shape: CuCollisionShape, frame: Option<usize>) -> Self {
        Self { shape, frame }
    }
}

/// Transforms a local `CuCollisionShape` by `frame` and writes the world-space shape into `out_world`.
pub fn transform_shape(
    frame: &Transform3D<f64>,
    local: &CuCollisionShape,
    out_world: &mut CuCollisionShape,
) {
    let mat = frame.to_matrix();
    match local {
        CuCollisionShape::Sphere(sph) => {
            let c = transform_point_raw(&mat, sph.center);
            *out_world = CuCollisionShape::Sphere(CuSphere {
                center: c,
                radius: sph.radius,
            });
        }
        CuCollisionShape::Capsule(cap) => {
            let a = transform_point_raw(&mat, cap.a);
            let b = transform_point_raw(&mat, cap.b);
            *out_world = CuCollisionShape::Capsule(CuCapsule {
                a,
                b,
                radius: cap.radius,
            });
        }
    }
}

/// Computes the exact signed surface-to-surface distance between two spheres.
///
/// Negative values indicate penetration depth.
pub fn sphere_sphere_signed_distance(a: &CuSphere, b: &CuSphere) -> Length64 {
    let pa = to_meters(a.center);
    let pb = to_meters(b.center);
    let dist = norm3(sub3(pa, pb));
    let r_sum = a.radius.get::<meter>() + b.radius.get::<meter>();
    Length64::new::<meter>(dist - r_sum)
}

/// Computes the exact signed surface-to-surface distance between a capsule and a sphere.
///
/// Negative values indicate penetration depth.
pub fn capsule_sphere_signed_distance(cap: &CuCapsule, sph: &CuSphere) -> Length64 {
    let a = to_meters(cap.a);
    let b = to_meters(cap.b);
    let p = to_meters(sph.center);
    let dist = point_segment_distance(p, a, b);
    let r_sum = cap.radius.get::<meter>() + sph.radius.get::<meter>();
    Length64::new::<meter>(dist - r_sum)
}

/// Computes the exact signed surface-to-surface distance between two capsules.
///
/// Negative values indicate penetration depth.
pub fn capsule_capsule_signed_distance(a: &CuCapsule, b: &CuCapsule) -> Length64 {
    let p1 = to_meters(a.a);
    let q1 = to_meters(a.b);
    let p2 = to_meters(b.a);
    let q2 = to_meters(b.b);
    let dist = segment_segment_distance(p1, q1, p2, q2);
    let r_sum = a.radius.get::<meter>() + b.radius.get::<meter>();
    Length64::new::<meter>(dist - r_sum)
}

/// Computes the exact signed surface-to-surface distance between any two `CuCollisionShape` primitives.
pub fn shape_signed_distance(a: &CuCollisionShape, b: &CuCollisionShape) -> Length64 {
    match (a, b) {
        (CuCollisionShape::Sphere(sa), CuCollisionShape::Sphere(sb)) => {
            sphere_sphere_signed_distance(sa, sb)
        }
        (CuCollisionShape::Capsule(ca), CuCollisionShape::Sphere(sb)) => {
            capsule_sphere_signed_distance(ca, sb)
        }
        (CuCollisionShape::Sphere(sa), CuCollisionShape::Capsule(cb)) => {
            capsule_sphere_signed_distance(cb, sa)
        }
        (CuCollisionShape::Capsule(ca), CuCollisionShape::Capsule(cb)) => {
            capsule_capsule_signed_distance(ca, cb)
        }
    }
}

/// Returns `true` if the signed distance between `a` and `b` is strictly less than `safety_margin`.
pub fn shapes_violate_margin(
    a: &CuCollisionShape,
    b: &CuCollisionShape,
    safety_margin: Length64,
) -> bool {
    shape_signed_distance(a, b).get::<meter>() < safety_margin.get::<meter>()
}

fn transform_point_raw(mat: &[[f64; 4]; 4], pt: [Length64; 3]) -> [Length64; 3] {
    let x = pt[0].get::<meter>();
    let y = pt[1].get::<meter>();
    let z = pt[2].get::<meter>();
    [
        Length64::new::<meter>(mat[0][0] * x + mat[0][1] * y + mat[0][2] * z + mat[0][3]),
        Length64::new::<meter>(mat[1][0] * x + mat[1][1] * y + mat[1][2] * z + mat[1][3]),
        Length64::new::<meter>(mat[2][0] * x + mat[2][1] * y + mat[2][2] * z + mat[2][3]),
    ]
}

fn to_meters(pt: [Length64; 3]) -> [f64; 3] {
    [
        pt[0].get::<meter>(),
        pt[1].get::<meter>(),
        pt[2].get::<meter>(),
    ]
}

const fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

const fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm3(v: [f64; 3]) -> f64 {
    libm::sqrt(dot3(v, v))
}

fn point_segment_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub3(b, a);
    let ap = sub3(p, a);
    let len_sq = dot3(ab, ab);
    if len_sq <= 1e-18 {
        return norm3(ap);
    }
    let t = (dot3(ap, ab) / len_sq).clamp(0.0, 1.0);
    let closest = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    norm3(sub3(p, closest))
}

/// Exact distance between two 3D line segments `p1..q1` and `p2..q2` (Ericson, Real-Time Collision Detection).
fn segment_segment_distance(p1: [f64; 3], q1: [f64; 3], p2: [f64; 3], q2: [f64; 3]) -> f64 {
    let d1 = sub3(q1, p1);
    let d2 = sub3(q2, p2);
    let r = sub3(p1, p2);
    let a = dot3(d1, d1);
    let e = dot3(d2, d2);
    let f = dot3(d2, r);

    let eps = 1e-18;
    if a <= eps && e <= eps {
        return norm3(r);
    }
    if a <= eps {
        return point_segment_distance(p1, p2, q2);
    }
    if e <= eps {
        return point_segment_distance(p2, p1, q1);
    }

    let c = dot3(d1, r);
    let b = dot3(d1, d2);
    let denom = a * e - b * b;

    let mut s = if denom != 0.0 {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    let c1 = [p1[0] + s * d1[0], p1[1] + s * d1[1], p1[2] + s * d1[2]];
    let c2 = [p2[0] + t * d2[0], p2[1] + t * d2[1], p2[2] + t * d2[2]];
    norm3(sub3(c1, c2))
}

/// Builds a sphere from meter values without validation (crate-internal presets and tests).
pub(crate) fn sphere_m(center: [f64; 3], radius: f64) -> CuSphere {
    CuSphere {
        center: center.map(Length64::new::<meter>),
        radius: Length64::new::<meter>(radius),
    }
}

/// Builds a capsule from meter values without validation (crate-internal presets and tests).
pub(crate) fn capsule_m(a: [f64; 3], b: [f64; 3], radius: f64) -> CuCapsule {
    CuCapsule {
        a: a.map(Length64::new::<meter>),
        b: b.map(Length64::new::<meter>),
        radius: Length64::new::<meter>(radius),
    }
}
