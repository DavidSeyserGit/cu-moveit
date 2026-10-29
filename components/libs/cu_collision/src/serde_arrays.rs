//! Serde helpers for const-generic fixed-size arrays.

macro_rules! impl_const_array_serde {
    ($mod_name:ident, $elem_ty:ty, $label:literal) => {
        pub mod $mod_name {
            use serde::Deserializer;
            use serde::Serializer;
            use serde::de::Error as DeError;
            use serde::de::SeqAccess;
            use serde::de::Visitor;
            use serde::ser::SerializeTuple;

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
                        Ok(out)
                    }
                }

                deserializer.deserialize_tuple(N, ArrayVisitor::<N>)
            }
        }
    };
}

impl_const_array_serde!(
    shape_array,
    crate::shapes::CuCollisionShape,
    "collision shape"
);

impl_const_array_serde!(
    collision_contact_array,
    crate::model::CuCollisionContact,
    "collision contact"
);
impl_const_array_serde!(
    collision_shape_array,
    crate::shapes::CuCollisionShape,
    "collision shape"
);
