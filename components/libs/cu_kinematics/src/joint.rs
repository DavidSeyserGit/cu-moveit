//! Typed coordinates and bounds for serial revolute, prismatic, and fixed joints.

use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use serde::Deserialize;
use serde::Serialize;

/// A joint coordinate carries the physical unit of its joint.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuJointPosition {
    Revolute(Angle),
    Prismatic(Length),
    Fixed,
}

impl Default for CuJointPosition {
    fn default() -> Self {
        Self::Revolute(Angle::default())
    }
}

impl From<Angle> for CuJointPosition {
    fn from(value: Angle) -> Self {
        Self::Revolute(value)
    }
}

impl From<Length> for CuJointPosition {
    fn from(value: Length) -> Self {
        Self::Prismatic(value)
    }
}

impl CuJointPosition {
    /// Extracts an angle, rejecting linear and fixed coordinates.
    pub fn angle(self) -> CuResult<Angle> {
        match self {
            Self::Revolute(v) => Ok(v),
            _ => Err(CuError::from("Expected revolute coordinate")),
        }
    }
    /// Extracts a displacement, rejecting angular and fixed coordinates.
    pub fn displacement(self) -> CuResult<Length> {
        match self {
            Self::Prismatic(v) => Ok(v),
            _ => Err(CuError::from("Expected prismatic coordinate")),
        }
    }
    #[doc(hidden)]
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<radian>(),
            Self::Prismatic(v) => v.get::<meter>(),
            Self::Fixed => 0.0,
        }
    }
    #[doc(hidden)]
    pub fn with_si_value(self, value: f64) -> Self {
        match self {
            Self::Revolute(_) => Self::Revolute(Angle::new::<radian>(value)),
            Self::Prismatic(_) => Self::Prismatic(Length::new::<meter>(value)),
            Self::Fixed => Self::Fixed,
        }
    }
    /// Interpolates between coordinates of the same joint type.
    pub fn interpolate(self, other: Self, fraction: f64) -> CuResult<Self> {
        if core::mem::discriminant(&self) != core::mem::discriminant(&other)
            || !fraction.is_finite()
        {
            return Err(CuError::from(
                "Incompatible joint coordinates or interpolation fraction",
            ));
        }
        Ok(self.with_si_value(self.si_value() + fraction * (other.si_value() - self.si_value())))
    }
}

/// A joint velocity carries angular or linear velocity units.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuJointVelocity {
    Revolute(AngularVelocity),
    Prismatic(Velocity),
    Fixed,
}
impl Default for CuJointVelocity {
    fn default() -> Self {
        Self::Revolute(AngularVelocity::default())
    }
}
impl CuJointVelocity {
    #[doc(hidden)]
    pub fn from_coordinate(position: CuJointPosition, value: f64) -> Self {
        match position {
            CuJointPosition::Revolute(_) => {
                Self::Revolute(AngularVelocity::new::<radian_per_second>(value))
            }
            CuJointPosition::Prismatic(_) => {
                Self::Prismatic(Velocity::new::<meter_per_second>(value))
            }
            CuJointPosition::Fixed => Self::Fixed,
        }
    }
    #[doc(hidden)]
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<radian_per_second>(),
            Self::Prismatic(v) => v.get::<meter_per_second>(),
            Self::Fixed => 0.0,
        }
    }
}

/// Position and velocity bounds, with units determined by joint type.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuJointLimits {
    Revolute {
        lower: Angle,
        upper: Angle,
        max_velocity: AngularVelocity,
    },
    Prismatic {
        lower: Length,
        upper: Length,
        max_velocity: Velocity,
    },
    /// Unbounded rotation; finite bounds returned by `lower`/`upper` are a sampling interval.
    Continuous { max_velocity: AngularVelocity },
    #[default]
    Fixed,
}
impl CuJointLimits {
    /// Creates angular bounds from radians and radians per second.
    pub fn from_radians(lower: f64, upper: f64, max_velocity: f64) -> Self {
        Self::Revolute {
            lower: Angle::new::<radian>(lower),
            upper: Angle::new::<radian>(upper),
            max_velocity: AngularVelocity::new::<radian_per_second>(max_velocity),
        }
    }
    /// Creates linear bounds from meters and meters per second.
    pub fn from_meters(lower: f64, upper: f64, max_velocity: f64) -> Self {
        Self::Prismatic {
            lower: Length::new::<meter>(lower),
            upper: Length::new::<meter>(upper),
            max_velocity: Velocity::new::<meter_per_second>(max_velocity),
        }
    }
    pub fn lower(self) -> CuJointPosition {
        match self {
            Self::Revolute { lower, .. } => lower.into(),
            Self::Prismatic { lower, .. } => lower.into(),
            Self::Continuous { .. } => Angle::new::<radian>(-core::f64::consts::PI).into(),
            Self::Fixed => CuJointPosition::Fixed,
        }
    }
    pub fn upper(self) -> CuJointPosition {
        match self {
            Self::Revolute { upper, .. } => upper.into(),
            Self::Prismatic { upper, .. } => upper.into(),
            Self::Continuous { .. } => Angle::new::<radian>(core::f64::consts::PI).into(),
            Self::Fixed => CuJointPosition::Fixed,
        }
    }
    pub fn max_velocity(self) -> CuJointVelocity {
        match self {
            Self::Revolute { max_velocity, .. } => CuJointVelocity::Revolute(max_velocity),
            Self::Prismatic { max_velocity, .. } => CuJointVelocity::Prismatic(max_velocity),
            Self::Continuous { max_velocity } => CuJointVelocity::Revolute(max_velocity),
            Self::Fixed => CuJointVelocity::Fixed,
        }
    }
    pub fn validate(self, q: CuJointPosition) -> CuResult<()> {
        if core::mem::discriminant(&q) != core::mem::discriminant(&self.lower())
            || !q.si_value().is_finite()
        {
            return Err(CuError::from(
                "Joint coordinate has incorrect units or is non-finite",
            ));
        }
        let lo = self.lower().si_value();
        let hi = self.upper().si_value();
        let velocity = self.max_velocity().si_value();
        if !lo.is_finite() || !hi.is_finite() || lo > hi || !velocity.is_finite() || velocity < 0.0
        {
            return Err(CuError::from("Invalid joint limits"));
        }
        Ok(())
    }
    pub fn contains(self, q: CuJointPosition) -> bool {
        self.validate(q).is_ok()
            && (matches!(self, Self::Continuous { .. })
                || (q.si_value() >= self.lower().si_value()
                    && q.si_value() <= self.upper().si_value()))
    }
    pub fn clamp(self, q: CuJointPosition) -> CuResult<CuJointPosition> {
        self.validate(q)?;
        if matches!(self, Self::Continuous { .. }) {
            return Ok(q);
        }
        Ok(q.with_si_value(
            q.si_value()
                .clamp(self.lower().si_value(), self.upper().si_value()),
        ))
    }
}

/// Physical scaling used to compare angular and linear coordinates without mixing units.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuJointMetric {
    pub angular_scale: Angle,
    pub linear_scale: Length,
}
impl Default for CuJointMetric {
    fn default() -> Self {
        Self {
            angular_scale: Angle::new::<radian>(1.0),
            linear_scale: Length::new::<meter>(1.0),
        }
    }
}
impl CuJointMetric {
    pub fn validate(self) -> CuResult<()> {
        let a = self.angular_scale.get::<radian>();
        let l = self.linear_scale.get::<meter>();
        if !a.is_finite() || a <= 0.0 || !l.is_finite() || l <= 0.0 {
            return Err(CuError::from(
                "Joint metric scales must be positive and finite",
            ));
        }
        Ok(())
    }
    /// Returns a dimensionless joint coordinate for numerical algorithms.
    pub fn normalized(self, q: CuJointPosition) -> f64 {
        q.si_value() / self.scale(q)
    }
    #[doc(hidden)]
    pub fn scale(self, q: CuJointPosition) -> f64 {
        match q {
            CuJointPosition::Revolute(_) => self.angular_scale.get::<radian>(),
            CuJointPosition::Prismatic(_) => self.linear_scale.get::<meter>(),
            CuJointPosition::Fixed => 1.0,
        }
    }
}

/// Per-joint sampling resolution for discrete collision checks.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuJointResolution {
    pub angular: Angle,
    pub linear: Length,
}
impl Default for CuJointResolution {
    fn default() -> Self {
        Self {
            angular: Angle::new::<radian>(0.08),
            linear: Length::new::<meter>(0.01),
        }
    }
}
impl From<Angle> for CuJointResolution {
    fn from(angular: Angle) -> Self {
        Self {
            angular,
            ..Self::default()
        }
    }
}
impl CuJointResolution {
    pub fn metric(self) -> CuJointMetric {
        CuJointMetric {
            angular_scale: self.angular,
            linear_scale: self.linear,
        }
    }
}
