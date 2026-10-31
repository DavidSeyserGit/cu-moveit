//! Crate-private serde helpers for const-generic fixed-size arrays of this crate's types.
//!
//! The shared helpers for kinematics types live in `cu_kinematics::serde_arrays`; its macro is not
//! exported, so this crate keeps one local copy of the same pattern for its own element types.

macro_rules! impl_const_array_serde {
    ($mod_name:ident, $elem_ty:ty, $label:literal) => {
        pub(crate) mod $mod_name {
            use serde::Deserializer;
            use serde::Serializer;
            use serde::de::Error as DeError;
            use serde::de::SeqAccess;
            use serde::de::Visitor;
            use serde::ser::SerializeTuple;

            pub(crate) fn serialize<S, const N: usize>(
                values: &[$elem_ty; N],
                serializer: S,
            ) -> Result<S::Ok, S::Error>
            where
                S: Serializer,
            {
                let mut seq = serializer.serialize_tuple(N)?;
                for value in values {
                    seq.serialize_element(value)?;
                }
                seq.end()
            }

            pub(crate) fn deserialize<'de, D, const N: usize>(
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

impl_const_array_serde!(
    contact_array,
    crate::model::CuCollisionContact,
    "collision contact"
);
impl_const_array_serde!(link_shape_array, crate::shapes::CuLinkShape, "link shape");
impl_const_array_serde!(
    obstacle_array,
    crate::scene::CuSceneObstacle,
    "scene obstacle"
);
impl_const_array_serde!(
    attached_array,
    crate::scene::CuAttachedShape,
    "attached shape"
);
impl_const_array_serde!(
    exclusion_array,
    crate::scene_exclusions::CuSceneExclusion,
    "scene exclusion"
);
