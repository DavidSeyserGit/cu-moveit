//! Typed coordinates and bounds for serial revolute, prismatic, and fixed joints.

use bincode::Decode;
use bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_acceleration::radian_per_second_squared;
use cu29::units::si::angular_jerk::radian_per_second_cubed;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Acceleration;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularAcceleration;
use cu29::units::si::f64::AngularJerk;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Jerk;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::jerk::meter_per_second_cubed;
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
    /// Returns the coordinate in SI base units (radians or meters; `0.0` for fixed joints).
    ///
    /// Intended for numerical kernels that operate on dimensionless arrays; convert back with
    /// [`Self::with_si_value`] so the unit tag is never lost.
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<radian>(),
            Self::Prismatic(v) => v.get::<meter>(),
            Self::Fixed => 0.0,
        }
    }
    /// Returns a coordinate of the same joint kind holding `value` in SI base units
    /// (radians or meters). Fixed coordinates stay fixed.
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
    /// Creates a velocity of the same joint kind as `position` from an SI value
    /// (rad/s or m/s). Fixed joints yield [`CuJointVelocity::Fixed`].
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
    /// Returns the velocity in SI base units (rad/s or m/s; `0.0` for fixed joints).
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
    /// Creates bounded revolute limits.
    pub const fn revolute(lower: Angle, upper: Angle, max_velocity: AngularVelocity) -> Self {
        Self::Revolute {
            lower,
            upper,
            max_velocity,
        }
    }
    /// Creates bounded prismatic limits.
    pub const fn prismatic(lower: Length, upper: Length, max_velocity: Velocity) -> Self {
        Self::Prismatic {
            lower,
            upper,
            max_velocity,
        }
    }
    /// Lower position bound (`-pi` sampling bound for continuous joints).
    pub fn lower(self) -> CuJointPosition {
        match self {
            Self::Revolute { lower, .. } => lower.into(),
            Self::Prismatic { lower, .. } => lower.into(),
            Self::Continuous { .. } => Angle::new::<radian>(-core::f64::consts::PI).into(),
            Self::Fixed => CuJointPosition::Fixed,
        }
    }
    /// Upper position bound (`+pi` sampling bound for continuous joints).
    pub fn upper(self) -> CuJointPosition {
        match self {
            Self::Revolute { upper, .. } => upper.into(),
            Self::Prismatic { upper, .. } => upper.into(),
            Self::Continuous { .. } => Angle::new::<radian>(core::f64::consts::PI).into(),
            Self::Fixed => CuJointPosition::Fixed,
        }
    }
    /// Maximum absolute joint velocity.
    pub fn max_velocity(self) -> CuJointVelocity {
        match self {
            Self::Revolute { max_velocity, .. } => CuJointVelocity::Revolute(max_velocity),
            Self::Prismatic { max_velocity, .. } => CuJointVelocity::Prismatic(max_velocity),
            Self::Continuous { max_velocity } => CuJointVelocity::Revolute(max_velocity),
            Self::Fixed => CuJointVelocity::Fixed,
        }
    }
    /// Checks that `q` has this joint's unit kind and that the limits themselves are well formed.
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
    /// Returns `true` if `q` is valid and inside the bounds (always inside for continuous joints).
    pub fn contains(self, q: CuJointPosition) -> bool {
        self.validate(q).is_ok()
            && (matches!(self, Self::Continuous { .. })
                || (q.si_value() >= self.lower().si_value()
                    && q.si_value() <= self.upper().si_value()))
    }
    /// Clamps `q` to the bounds (continuous joints are returned unchanged).
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

/// A joint acceleration carries angular (rad/s²) or linear (m/s²) acceleration units.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuJointAcceleration {
    Revolute(AngularAcceleration),
    Prismatic(Acceleration),
    Fixed,
}
impl Default for CuJointAcceleration {
    fn default() -> Self {
        Self::Revolute(AngularAcceleration::default())
    }
}
impl From<AngularAcceleration> for CuJointAcceleration {
    fn from(value: AngularAcceleration) -> Self {
        Self::Revolute(value)
    }
}
impl From<Acceleration> for CuJointAcceleration {
    fn from(value: Acceleration) -> Self {
        Self::Prismatic(value)
    }
}
impl CuJointAcceleration {
    /// Creates a value of the same joint kind as `position` from an SI value
    /// (rad/s² or m/s²). Fixed joints yield `Fixed`.
    pub fn from_coordinate(position: CuJointPosition, value: f64) -> Self {
        match position {
            CuJointPosition::Revolute(_) => {
                Self::Revolute(AngularAcceleration::new::<radian_per_second_squared>(value))
            }
            CuJointPosition::Prismatic(_) => {
                Self::Prismatic(Acceleration::new::<meter_per_second_squared>(value))
            }
            CuJointPosition::Fixed => Self::Fixed,
        }
    }
    /// Returns the value in SI base units (rad/s² or m/s²; `0.0` for fixed joints).
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<radian_per_second_squared>(),
            Self::Prismatic(v) => v.get::<meter_per_second_squared>(),
            Self::Fixed => 0.0,
        }
    }
}

/// A joint jerk carries angular (rad/s³) or linear (m/s³) jerk units.
///
/// An unbounded jerk (for example a jerk limit that should not constrain) is expressed as
/// `f64::INFINITY` inside the unit value, e.g.
/// `CuJointJerk::Revolute(AngularJerk::new::<radian_per_second_cubed>(f64::INFINITY))`;
/// consumers may alternatively use `Option<CuJointJerk>` at the use site.
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuJointJerk {
    Revolute(AngularJerk),
    Prismatic(Jerk),
    Fixed,
}
impl Default for CuJointJerk {
    fn default() -> Self {
        Self::Revolute(AngularJerk::default())
    }
}
impl From<AngularJerk> for CuJointJerk {
    fn from(value: AngularJerk) -> Self {
        Self::Revolute(value)
    }
}
impl From<Jerk> for CuJointJerk {
    fn from(value: Jerk) -> Self {
        Self::Prismatic(value)
    }
}
impl CuJointJerk {
    /// Creates a value of the same joint kind as `position` from an SI value
    /// (rad/s³ or m/s³). Fixed joints yield `Fixed`.
    pub fn from_coordinate(position: CuJointPosition, value: f64) -> Self {
        match position {
            CuJointPosition::Revolute(_) => {
                Self::Revolute(AngularJerk::new::<radian_per_second_cubed>(value))
            }
            CuJointPosition::Prismatic(_) => {
                Self::Prismatic(Jerk::new::<meter_per_second_cubed>(value))
            }
            CuJointPosition::Fixed => Self::Fixed,
        }
    }
    /// Returns the value in SI base units (rad/s³ or m/s³; `0.0` for fixed joints).
    pub fn si_value(self) -> f64 {
        match self {
            Self::Revolute(v) => v.get::<radian_per_second_cubed>(),
            Self::Prismatic(v) => v.get::<meter_per_second_cubed>(),
            Self::Fixed => 0.0,
        }
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
    /// Checks that both scales are positive and finite.
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
    /// Returns the SI scale (radians or meters) that normalizes coordinates of `q`'s joint kind;
    /// `1.0` for fixed joints.
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
    /// Uses the resolution as a joint-space metric.
    pub fn metric(self) -> CuJointMetric {
        CuJointMetric {
            angular_scale: self.angular,
            linear_scale: self.linear,
        }
    }
}
