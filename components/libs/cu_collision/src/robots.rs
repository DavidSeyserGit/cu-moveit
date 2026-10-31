//! Preconfigured collision models and the RON collision-model spec.

use crate::model::CuCollisionModelParams;
use crate::model::CuRobotCollisionModel;
use crate::shapes::CuCollisionShape;
use crate::shapes::CuLinkShape;
use crate::shapes::capsule_m;
use crate::shapes::sphere_m;
use cu_kinematics::CuRobotPreset;
use cu29::prelude::*;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::Serializer;

/// Link shapes of the igus ReBeL, one per joint frame `0..6`.
fn igus_rebel_link_shapes() -> [CuLinkShape; 6] {
    let capsule = |a, b, r| CuCollisionShape::Capsule(capsule_m(a, b, r));
    let shapes = [
        // Link 0 (shoulder column around joint 1 frame at z = 0.1462 m)
        capsule([0.0, 0.0, -0.10], [0.0, 0.0, 0.04], 0.052),
        // Link 1 (upper arm from joint 2 at z = 0.2522 m toward joint 3)
        capsule([0.0, 0.0, 0.02], [0.0, 0.0, 0.20], 0.045),
        // Link 2 (elbow housing from joint 3 toward joint 4)
        capsule([0.0, 0.0, 0.02], [0.0, 0.0, 0.12], 0.042),
        // Link 3 (forearm tube from joint 4 toward joint 5)
        capsule([0.0, 0.0, 0.02], [0.0, 0.0, 0.11], 0.038),
        // Link 4 (wrist housing from joint 5 toward joint 6)
        capsule([0.0, 0.0, 0.01], [0.0, 0.0, 0.05], 0.034),
        // Link 5 (tool flange sphere at joint 6)
        CuCollisionShape::Sphere(sphere_m([0.0, 0.0, 0.02], 0.032)),
    ];
    let mut frame = 0;
    shapes.map(|shape| {
        frame += 1;
        CuLinkShape::new(shape, Some(frame - 1))
    })
}

fn igus_rebel_params() -> CuCollisionModelParams {
    CuCollisionModelParams {
        safety_margin: Length64::new::<meter>(0.005),
        skip_adjacent_links: true,
    }
}

/// Returns the 6-link capsule/sphere collision model for the **igus ReBeL** 6-DOF cobot.
///
/// Each shape is expressed in the local coordinate frame of its corresponding joint frame `0..6`
/// produced by `cu_kinematics::igus_rebel_chain().forward_frames(...)`. Safety margin 5 mm;
/// adjacent links are not checked against each other.
pub fn igus_rebel_collision_model() -> CuRobotCollisionModel<6> {
    // Constant, known-valid inputs: skip the fallible validation path.
    CuRobotCollisionModel::assemble(&igus_rebel_link_shapes(), igus_rebel_params())
}

/// Source of a robot collision model in RON: a preset name or an inline model.
///
/// ```ron
/// "collision_model": "igus_rebel"
/// "collision_model": {
///     "shapes": [
///         {"shape": {"capsule": {"a": [0.0, 0.0, 0.0], "b": [0.0, 0.0, 0.2], "radius": 0.05}}, "frame": 0},
///         {"shape": {"sphere": {"center": [0.0, 0.0, 0.02], "radius": 0.03}}, "frame": 1},
///     ],
///     "safety_margin": 0.005,       // optional, default 0.0
///     "skip_adjacent_links": true,  // optional, default true
/// }
/// ```
///
/// An inline model lists exactly `M` shapes; omit `frame` for a base-frame shape. Use
/// [`CuRobotCollisionModel::new`] for fewer than `M` shapes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CuCollisionModelSpec<const M: usize> {
    /// A named preset; its shape and joint counts are checked by [`Self::resolve`].
    Preset(CuRobotPreset),
    /// Explicit shapes and parameters.
    Inline {
        /// One shape per model slot.
        shapes: [CuLinkShape; M],
        /// Model parameters.
        params: CuCollisionModelParams,
    },
}

impl<const M: usize> From<CuRobotPreset> for CuCollisionModelSpec<M> {
    fn from(preset: CuRobotPreset) -> Self {
        Self::Preset(preset)
    }
}

impl<const M: usize> CuCollisionModelSpec<M> {
    /// Builds and validates the model for an `N`-joint chain.
    pub fn resolve<const N: usize>(&self) -> CuResult<CuRobotCollisionModel<N, M>> {
        match self {
            Self::Preset(CuRobotPreset::IgusRebel) => {
                if N != CuRobotPreset::IgusRebel.dof() {
                    return Err(CuError::from(
                        "cu_collision: preset `igus_rebel` collision model needs N = 6",
                    ));
                }
                CuRobotCollisionModel::new(&igus_rebel_link_shapes(), igus_rebel_params())
            }
            Self::Inline { shapes, params } => CuRobotCollisionModel::new(shapes, *params),
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct InlineModel<const M: usize> {
    #[serde(with = "crate::serde_arrays::link_shape_array")]
    shapes: [CuLinkShape; M],
    #[serde(default = "default_safety_margin")]
    safety_margin: Length64,
    #[serde(default = "default_skip_adjacent_links")]
    skip_adjacent_links: bool,
}

fn default_safety_margin() -> Length64 {
    CuCollisionModelParams::default().safety_margin
}

fn default_skip_adjacent_links() -> bool {
    CuCollisionModelParams::default().skip_adjacent_links
}

impl<const M: usize> Serialize for CuCollisionModelSpec<M> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Self::Preset(preset) => preset.serialize(serializer),
            Self::Inline { shapes, params } => InlineModel {
                shapes: *shapes,
                safety_margin: params.safety_margin,
                skip_adjacent_links: params.skip_adjacent_links,
            }
            .serialize(serializer),
        }
    }
}

impl<'de, const M: usize> Deserialize<'de> for CuCollisionModelSpec<M> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use serde::de::IntoDeserializer;
        use serde::de::MapAccess;
        use serde::de::Visitor;
        use serde::de::value::MapAccessDeserializer;

        struct SpecVisitor<const K: usize>;
        impl<'de, const K: usize> Visitor<'de> for SpecVisitor<K> {
            type Value = CuCollisionModelSpec<K>;
            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(
                    f,
                    "a robot preset name (\"igus_rebel\") or an inline collision model {{\"shapes\": [...]}}"
                )
            }
            fn visit_str<E: serde::de::Error>(self, v: &str) -> Result<Self::Value, E> {
                CuRobotPreset::deserialize(v.into_deserializer()).map(CuCollisionModelSpec::Preset)
            }
            fn visit_map<A: MapAccess<'de>>(self, map: A) -> Result<Self::Value, A::Error> {
                let inline = InlineModel::<K>::deserialize(MapAccessDeserializer::new(map))?;
                Ok(CuCollisionModelSpec::Inline {
                    shapes: inline.shapes,
                    params: CuCollisionModelParams {
                        safety_margin: inline.safety_margin,
                        skip_adjacent_links: inline.skip_adjacent_links,
                    },
                })
            }
        }
        deserializer.deserialize_any(SpecVisitor::<M>)
    }
}
