//! Serde support for const-generic arrays of [`crate::CuJointEffort`]
//! (serde's built-in array impls stop at small fixed sizes).

pub(crate) mod effort_array {
    use crate::CuJointEffort;
    use core::fmt;
    use serde::de::{Error as DeError, SeqAccess, Visitor};
    use serde::ser::SerializeTuple;
    use serde::{Deserializer, Serializer};

    pub fn serialize<S, const N: usize>(
        value: &[CuJointEffort; N],
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut tuple = serializer.serialize_tuple(N)?;
        for item in value {
            tuple.serialize_element(item)?;
        }
        tuple.end()
    }

    pub fn deserialize<'de, D, const N: usize>(
        deserializer: D,
    ) -> Result<[CuJointEffort; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ArrayVisitor<const M: usize>;
        impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
            type Value = [CuJointEffort; M];
            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(formatter, "an array of {M} joint effort values")
            }
            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut out = [CuJointEffort::default(); M];
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
