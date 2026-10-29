//! Copper `CuTask` wrapper for real-time forward kinematics evaluation.

use crate::chain::CuKinematicChain;
use crate::robots::igus_rebel_chain;
use bincode::Decode;
use bincode::Encode;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::f64::Angle as Angle64;
use serde::Deserialize;
use serde::Serialize;

/// Fixed-DOF joint position payload (`CuMsgPayload`-compliant).
#[derive(Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPositionsPayload<const N: usize> {
    #[serde(with = "joint_array_serde")]
    pub positions: [Angle64; N],
}

impl<const N: usize> Default for CuJointPositionsPayload<N> {
    fn default() -> Self {
        Self {
            positions: [Angle64::default(); N],
        }
    }
}

/// Stateless `CuTask` computing end-effector `Transform3D<f64>` from joint angles.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuForwardKinematicsTask<const N: usize> {
    #[reflect(ignore)]
    chain: CuKinematicChain<N>,
}

impl<const N: usize> CuForwardKinematicsTask<N> {
    /// Creates a forward kinematics task from an explicit `CuKinematicChain<N>`.
    pub const fn new_with_chain(chain: CuKinematicChain<N>) -> Self {
        Self { chain }
    }

    /// Borrows the underlying kinematic chain.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }
}

impl<const N: usize> Freezable for CuForwardKinematicsTask<N> {}

impl CuTask for CuForwardKinematicsTask<6> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuJointPositionsPayload<6>);
    type Output<'m> = output_msg!(Transform3D<f64>);

    fn new(_cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::new_with_chain(igus_rebel_chain()))
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let Some(joint_msg) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let mut pose = Transform3D::<f64>::identity();
        self.chain
            .forward_kinematics(&joint_msg.positions, &mut pose)?;
        output.set_payload(pose);
        Ok(())
    }
}

mod joint_array_serde {
    use cu29::units::si::f64::Angle as Angle64;
    use serde::Deserializer;
    use serde::Serializer;
    use serde::de::Error as DeError;
    use serde::de::SeqAccess;
    use serde::de::Visitor;
    use serde::ser::SerializeTuple;

    pub fn serialize<S, const N: usize>(
        values: &[Angle64; N],
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut seq = serializer.serialize_tuple(N)?;
        let mut i = 0;
        while i < N {
            seq.serialize_element(&values[i])?;
            i += 1;
        }
        seq.end()
    }

    pub fn deserialize<'de, D, const N: usize>(deserializer: D) -> Result<[Angle64; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ArrayVisitor<const M: usize>;

        impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
            type Value = [Angle64; M];

            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(formatter, "an array of {M} Angle64 values")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut out = [Angle64::default(); M];
                let mut i = 0;
                while i < M {
                    out[i] = seq
                        .next_element()?
                        .ok_or_else(|| DeError::invalid_length(i, &self))?;
                    i += 1;
                }
                Ok(out)
            }
        }

        deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
    }
}
