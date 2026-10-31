//! Crate-private serde helpers for arrays `cu_kinematics::serde_arrays` does not cover.

use crate::CuTrajectoryPoint;
use serde::Deserializer;
use serde::Serializer;
use serde::de::Error as DeError;
use serde::de::SeqAccess;
use serde::de::Visitor;
use serde::ser::SerializeTuple;

/// Serialization for an optional per-joint jerk array (`None` = unbounded).
///
/// RON accepts the bare array (`"max_jerks": [{"revolute": 10.0}]`) or omits
/// the key; arrays are exactly `N` long.
pub(crate) mod option_jerk_array {
    use cu_kinematics::CuJointJerk;
    use serde::Deserialize;
    use serde::Serialize;

    #[derive(Serialize, Deserialize)]
    #[serde(transparent)]
    struct Jerks<const N: usize>(
        #[serde(with = "cu_kinematics::serde_arrays::jerk_array")] [CuJointJerk; N],
    );

    pub(crate) fn serialize<S, const N: usize>(
        values: &Option<[CuJointJerk; N]>,
        serializer: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: super::Serializer,
    {
        values.map(Jerks).serialize(serializer)
    }

    pub(crate) fn deserialize<'de, D, const N: usize>(
        deserializer: D,
    ) -> Result<Option<[CuJointJerk; N]>, D::Error>
    where
        D: super::Deserializer<'de>,
    {
        Ok(Option::<Jerks<N>>::deserialize(deserializer)?.map(|jerks| jerks.0))
    }
}

/// Fixed-size array serialization for [`CuTrajectoryPoint`] waypoints.
pub(crate) mod trajectory_point_array {
    use super::*;

    pub(crate) fn serialize<S, const N: usize, const M: usize>(
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

    pub(crate) fn deserialize<'de, D, const N: usize, const M: usize>(
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
