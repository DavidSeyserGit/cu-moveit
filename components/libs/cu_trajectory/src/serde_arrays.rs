//! Serde helpers for const-generic fixed-size trajectory point arrays.

use crate::CuTrajectoryPoint;
use serde::Deserializer;
use serde::Serializer;
use serde::de::Error as DeError;
use serde::de::SeqAccess;
use serde::de::Visitor;
use serde::ser::SerializeTuple;

/// Fixed-size array serialization for plain `f64` limit entries.
pub mod f64_array {
    use super::*;

    pub fn serialize<S, const N: usize>(values: &[f64; N], serializer: S) -> Result<S::Ok, S::Error>
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

    pub fn deserialize<'de, D, const N: usize>(deserializer: D) -> Result<[f64; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ArrayVisitor<const M: usize>;

        impl<'de, const M: usize> Visitor<'de> for ArrayVisitor<M> {
            type Value = [f64; M];

            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(formatter, "an array of {M} f64 values")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut out = [0.0; M];
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

/// Fixed-size array serialization for [`CuTrajectoryPoint`] waypoints.
pub mod trajectory_point_array {
    use super::*;

    pub fn serialize<S, const N: usize, const M: usize>(
        values: &[CuTrajectoryPoint<N>; M],
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut seq = serializer.serialize_tuple(M)?;
        let mut i = 0;
        while i < M {
            seq.serialize_element(&values[i])?;
            i += 1;
        }
        seq.end()
    }

    pub fn deserialize<'de, D, const N: usize, const M: usize>(
        deserializer: D,
    ) -> Result<[CuTrajectoryPoint<N>; M], D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ArrayVisitor<const P: usize, const Q: usize>;

        impl<'de, const P: usize, const Q: usize> Visitor<'de> for ArrayVisitor<P, Q> {
            type Value = [CuTrajectoryPoint<P>; Q];

            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(formatter, "an array of {Q} CuTrajectoryPoint<{P}> values")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut out = core::array::from_fn(|_| CuTrajectoryPoint::<P>::default());
                let mut i = 0;
                while i < Q {
                    out[i] = seq
                        .next_element()?
                        .ok_or_else(|| DeError::invalid_length(i, &self))?;
                    i += 1;
                }
                if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                    return Err(DeError::invalid_length(Q + 1, &self));
                }
                Ok(out)
            }
        }

        deserializer.deserialize_tuple(M, ArrayVisitor::<N, M>)
    }
}
