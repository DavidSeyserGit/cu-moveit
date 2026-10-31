//! Serde support for const-generic `bool` arrays (serde's built-in array
//! impls stop at 32 elements).

pub(crate) mod bool_array {
    use core::fmt;
    use serde::de::{Error as DeError, SeqAccess, Visitor};
    use serde::ser::SerializeTuple;
    use serde::{Deserializer, Serializer};

    pub(crate) fn serialize<S, const N: usize>(
        value: &[bool; N],
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

    pub(crate) fn deserialize<'de, D, const N: usize>(
        deserializer: D,
    ) -> Result<[bool; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ArrayVisitor<const M: usize>;
        impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
            type Value = [bool; M];
            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(formatter, "an array of {M} booleans")
            }
            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut out = [false; M];
                for (i, slot) in out.iter_mut().enumerate() {
                    *slot = seq
                        .next_element()?
                        .ok_or_else(|| DeError::invalid_length(i, &self))?;
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
