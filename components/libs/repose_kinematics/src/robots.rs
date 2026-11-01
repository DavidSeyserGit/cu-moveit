//! Kinematic chain presets for standard manipulators and the RON chain source.

use crate::chain::CuJointAxis;
use crate::chain::CuJointSpec;
use crate::chain::CuKinematicChain;
use crate::joint::CuJointLimits;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::Serializer;

fn m(v: f64) -> Length {
    Length::new::<meter>(v)
}

fn rad(v: f64) -> Angle {
    Angle::new::<radian>(v)
}

fn rebel_joint(xyz: [f64; 3], rpy_z: f64, axis: CuJointAxis, lo: f64, hi: f64) -> CuJointSpec {
    CuJointSpec::revolute(
        [m(xyz[0]), m(xyz[1]), m(xyz[2])],
        [rad(0.0), rad(0.0), rad(rpy_z)],
        axis,
        CuJointLimits::revolute(
            rad(lo),
            rad(hi),
            AngularVelocity::new::<radian_per_second>(2.16),
        ),
    )
}

/// Returns the 6-DOF kinematic chain for the **igus ReBeL** cobot.
///
/// Link origins, axes, and joint limits are derived directly from `igus_rebel.urdf`
/// (combining fixed motor-housing offsets with their corresponding revolute joints).
/// In RON the same chain is selected with `"chain": "igus_rebel"`.
pub fn igus_rebel_chain() -> CuKinematicChain<6> {
    CuKinematicChain::new([
        // J_motor1 (xyz: [0, 0, 0.1462], rpy: [0, 0, pi]) + joint1 (axis: +Z)
        rebel_joint(
            [0.0, 0.0, 0.1462],
            core::f64::consts::PI,
            CuJointAxis::PosZ,
            -3.1241,
            3.1241,
        ),
        // J_motor2 (xyz: [0, -0.0225, 0.106]) + joint2 (xyz: [0, 0.049, 0], axis: -Y)
        rebel_joint(
            [0.0, 0.0265, 0.106],
            0.0,
            CuJointAxis::NegY,
            -1.3500,
            2.4435,
        ),
        // J_motor3 (xyz: [0, -0.049, 0.24152]) + joint3 (xyz: [0, 0.0225, 0], axis: -Y)
        rebel_joint(
            [0.0, -0.0265, 0.24152],
            0.0,
            CuJointAxis::NegY,
            -1.39626,
            2.4000,
        ),
        // J_motor4 (xyz: [0.0013449814287804, 0, 0.105479407401356]) + joint4 (xyz: [0, 0, 0.052], axis: +Z)
        rebel_joint(
            [0.0013449814287804, 0.0, 0.157479407401356],
            0.0,
            CuJointAxis::PosZ,
            -3.12414,
            3.12414,
        ),
        // J_motor5 (xyz: [0, -0.0182, 0.142]) + joint5 (xyz: [0, 0.0182, 0], axis: -Y)
        rebel_joint([0.0, 0.0, 0.142], 0.0, CuJointAxis::NegY, -1.65806, 1.65806),
        // joint6 (xyz: [0, 0, 0.0768], axis: +Z)
        rebel_joint(
            [0.0, 0.0, 0.0768],
            0.0,
            CuJointAxis::PosZ,
            -3.12414,
            3.12414,
        ),
    ])
}

/// Named robot chain presets, selected in RON by their snake_case name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuRobotPreset {
    /// 6-DOF igus ReBeL ([`igus_rebel_chain`]); RON name `"igus_rebel"`.
    IgusRebel,
}

impl CuRobotPreset {
    /// RON name of the preset.
    pub const fn name(self) -> &'static str {
        match self {
            Self::IgusRebel => "igus_rebel",
        }
    }

    /// Number of joints of the preset chain.
    pub const fn dof(self) -> usize {
        match self {
            Self::IgusRebel => 6,
        }
    }

    /// Builds the preset chain, failing when its joint count differs from `N`.
    pub fn chain<const N: usize>(self) -> CuResult<CuKinematicChain<N>> {
        let joints = match self {
            Self::IgusRebel => *igus_rebel_chain().joints(),
        };
        let joints: [CuJointSpec; N] = joints.as_slice().try_into().map_err(|_| {
            CuError::from(alloc::format!(
                "Robot preset `{}` has {} joints but the task expects N = {N}",
                self.name(),
                self.dof()
            ))
        })?;
        Ok(CuKinematicChain::new(joints))
    }
}

/// Source of a robot chain in RON: a preset name or an inline chain.
///
/// ```ron
/// "chain": "igus_rebel"
/// "chain": {"joints": [ /* N CuJointSpec */ ], "tool_offset": /* optional */}
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CuChainSpec<const N: usize> {
    /// A named preset; its joint count is checked against `N` by [`Self::resolve`].
    Preset(CuRobotPreset),
    /// An explicit chain.
    Inline(CuKinematicChain<N>),
}

impl<const N: usize> From<CuKinematicChain<N>> for CuChainSpec<N> {
    fn from(chain: CuKinematicChain<N>) -> Self {
        Self::Inline(chain)
    }
}

impl<const N: usize> From<CuRobotPreset> for CuChainSpec<N> {
    fn from(preset: CuRobotPreset) -> Self {
        Self::Preset(preset)
    }
}

impl<const N: usize> CuChainSpec<N> {
    /// Builds and validates the chain.
    pub fn resolve(&self) -> CuResult<CuKinematicChain<N>> {
        let chain = match *self {
            Self::Preset(preset) => preset.chain()?,
            Self::Inline(chain) => chain,
        };
        chain.validate()?;
        Ok(chain)
    }
}

impl<const N: usize> Serialize for CuChainSpec<N> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Self::Preset(preset) => preset.serialize(serializer),
            Self::Inline(chain) => chain.serialize(serializer),
        }
    }
}

impl<'de, const N: usize> Deserialize<'de> for CuChainSpec<N> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use serde::de::IntoDeserializer;
        use serde::de::MapAccess;
        use serde::de::Visitor;
        use serde::de::value::MapAccessDeserializer;

        struct SpecVisitor<const M: usize>;
        impl<'de, const M: usize> Visitor<'de> for SpecVisitor<M> {
            type Value = CuChainSpec<M>;
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(
                    f,
                    "a robot preset name (\"igus_rebel\") or an inline chain {{\"joints\": [...]}}"
                )
            }
            fn visit_str<E: serde::de::Error>(self, v: &str) -> Result<Self::Value, E> {
                CuRobotPreset::deserialize(v.into_deserializer()).map(CuChainSpec::Preset)
            }
            fn visit_map<A: MapAccess<'de>>(self, map: A) -> Result<Self::Value, A::Error> {
                CuKinematicChain::deserialize(MapAccessDeserializer::new(map))
                    .map(CuChainSpec::Inline)
            }
        }
        deserializer.deserialize_any(SpecVisitor::<N>)
    }
}
