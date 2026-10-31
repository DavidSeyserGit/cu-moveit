//! Serde helpers for const-generic fixed-size arrays.
//!
//! This is the one shared set of fixed-array serde helpers for the workspace. Use a module with
//! `#[serde(with = "cu_kinematics::serde_arrays::<module>")]` on a `[T; N]` field; each module
//! provides `serialize` and `deserialize`, encodes the array as a tuple of exactly `N` elements,
//! and rejects sequences of any other length.
//!
//! ```rust
//! use cu_kinematics::CuJointPosition;
//! use serde::{Deserialize, Serialize};
//!
//! #[derive(Serialize, Deserialize)]
//! struct Goal<const N: usize> {
//!     #[serde(with = "cu_kinematics::serde_arrays::position_array")]
//!     positions: [CuJointPosition; N],
//! }
//! ```

macro_rules! impl_const_array_serde {
    ($mod_name:ident, $elem_ty:ty, $label:literal) => {
        #[doc = concat!("Serde `with` module for `[", $label, "; N]` arrays.")]
        pub mod $mod_name {
            use serde::Deserializer;
            use serde::Serializer;
            use serde::de::Error as DeError;
            use serde::de::SeqAccess;
            use serde::de::Visitor;
            use serde::ser::SerializeTuple;

            /// Serializes the array as an `N`-element tuple.
            pub fn serialize<S, const N: usize>(
                values: &[$elem_ty; N],
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

            /// Deserializes exactly `N` elements, rejecting shorter or longer sequences.
            pub fn deserialize<'de, D, const N: usize>(
                deserializer: D,
            ) -> Result<[$elem_ty; N], D::Error>
            where
                D: Deserializer<'de>,
            {
                struct ArrayVisitor<const M: usize>;

                impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
                    type Value = [$elem_ty; M];

                    fn expecting(
                        &self,
                        formatter: &mut core::fmt::Formatter<'_>,
                    ) -> core::fmt::Result {
                        write!(formatter, "an array of {M} {} values", $label)
                    }

                    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
                    where
                        A: SeqAccess<'de>,
                    {
                        let mut out = [<$elem_ty>::default(); M];
                        let mut i = 0;
                        while i < M {
                            out[i] = seq
                                .next_element()?
                                .ok_or_else(|| DeError::invalid_length(i, &self))?;
                            i += 1;
                        }
                        if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                            return Err(DeError::invalid_length(M + 1, &self));
                        }
                        Ok(out)
                    }
                }

                deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
            }
        }
    };
}

impl_const_array_serde!(joint_spec_array, crate::chain::CuJointSpec, "CuJointSpec");
impl_const_array_serde!(angle_array, cu29::units::si::f64::Angle, "Angle64");
impl_const_array_serde!(
    angular_velocity_array,
    cu29::units::si::f64::AngularVelocity,
    "AngularVelocity64"
);

impl_const_array_serde!(
    position_array,
    crate::joint::CuJointPosition,
    "CuJointPosition"
);
impl_const_array_serde!(
    velocity_array,
    crate::joint::CuJointVelocity,
    "CuJointVelocity"
);
impl_const_array_serde!(
    acceleration_array,
    crate::joint::CuJointAcceleration,
    "CuJointAcceleration"
);
impl_const_array_serde!(jerk_array, crate::joint::CuJointJerk, "CuJointJerk");
