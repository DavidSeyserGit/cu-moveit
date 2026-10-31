//! Deterministic, allocation-free capsule and sphere robot collision checking for `copper-rs`.
//!
//! - [`CuRobotCollisionModel`]: robot link shapes ([`CuLinkShape`]) on joint frames, built with
//!   [`CuRobotCollisionModel::new`] and [`CuCollisionModelParams`], or from RON via
//!   [`CuCollisionModelSpec`].
//! - [`CuCollisionEnv`]: borrowed view of obstacles, attached shapes, and exclusions that every
//!   check takes; [`CuPlanningScene::collision_env`] provides one without copying.
//! - [`CuCollisionCheckTask`]: the `CuTask` wrapper, configured by [`CuCollisionCheckParams`].
//!
//! # Example
//! ```rust
//! use cu_collision::{CuCollisionEnv, CuCollisionReport, CuCollisionShape, CuSphere};
//! use cu_collision::igus_rebel_collision_model;
//! use cu_kinematics::igus_rebel_chain;
//! use cu29::units::si::angle::radian;
//! use cu29::units::si::f64::{Angle, Length};
//! use cu29::units::si::length::meter;
//!
//! let chain = igus_rebel_chain();
//! let model = igus_rebel_collision_model();
//! let q_home = [Angle::new::<radian>(0.0).into(); 6];
//! let mut report = CuCollisionReport::<8>::default();
//! model.check_configuration(&chain, &q_home, &CuCollisionEnv::EMPTY, &mut report).unwrap();
//! assert!(report.is_collision_free());
//!
//! let m = Length::new::<meter>;
//! let obstacles = [CuCollisionShape::Sphere(CuSphere::new([m(0.0), m(0.0), m(0.87)], m(0.08)).unwrap())];
//! model.check_configuration(&chain, &q_home, &CuCollisionEnv::new(&obstacles), &mut report).unwrap();
//! assert!(!report.is_collision_free());
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod env;
mod model;
mod robots;
mod scene;
mod scene_exclusions;
mod serde_arrays;
mod shapes;
mod task;

pub use env::CuCollisionEnv;
pub use model::CuCollisionContact;
pub use model::CuCollisionModelParams;
pub use model::CuCollisionReport;
pub use model::CuContactKind;
pub use model::CuRobotCollisionModel;
pub use robots::CuCollisionModelSpec;
pub use robots::igus_rebel_collision_model;
pub use scene::CuAttachedShape;
pub use scene::CuAttachedUpdate;
pub use scene::CuObjectId;
pub use scene::CuPlanningScene;
pub use scene::CuSceneError;
pub use scene::CuSceneObstacle;
pub use scene::CuSceneRevision;
pub use scene::DEFAULT_MAX_ATTACHED;
pub use scene::DEFAULT_MAX_OBSTACLES;
pub use scene_exclusions::CuSceneExclusion;
pub use scene_exclusions::CuSceneExclusionPartner;
pub use scene_exclusions::CuSceneExclusions;
pub use scene_exclusions::DEFAULT_MAX_EXCLUSIONS;
pub use shapes::CuCapsule;
pub use shapes::CuCollisionShape;
pub use shapes::CuLinkShape;
pub use shapes::CuSphere;
pub use shapes::capsule_capsule_signed_distance;
pub use shapes::capsule_sphere_signed_distance;
pub use shapes::shape_signed_distance;
pub use shapes::shapes_violate_margin;
pub use shapes::sphere_sphere_signed_distance;
pub use shapes::transform_shape;
pub use task::CuCollisionCheckParams;
pub use task::CuCollisionCheckTask;
pub use task::DEFAULT_MAX_CONTACTS;

#[cfg(test)]
mod tests {
    use crate::CuAttachedShape;
    use crate::CuAttachedUpdate;
    use crate::CuCollisionCheckTask;
    use crate::CuCollisionEnv;
    use crate::CuCollisionModelParams;
    use crate::CuCollisionModelSpec;
    use crate::CuCollisionReport;
    use crate::CuCollisionShape;
    use crate::CuContactKind;
    use crate::CuLinkShape;
    use crate::CuObjectId;
    use crate::CuPlanningScene;
    use crate::CuRobotCollisionModel;
    use crate::CuSceneExclusion;
    use crate::CuSceneExclusions;
    use crate::CuSphere;
    use crate::capsule_capsule_signed_distance;
    use crate::capsule_sphere_signed_distance;
    use crate::igus_rebel_collision_model;
    use crate::shapes::capsule_m;
    use crate::shapes::sphere_m;
    use crate::sphere_sphere_signed_distance;
    use cu_kinematics::CuJointPositionsPayload;
    use cu_kinematics::CuRobotPreset;
    use cu_kinematics::igus_rebel_chain;
    use cu29::prelude::*;
    use cu29::units::si::angle::radian;
    use cu29::units::si::f64::Angle as Angle64;
    use cu29::units::si::f64::Length as Length64;
    use cu29::units::si::length::meter;

    fn sphere(center: [f64; 3], radius: f64) -> CuCollisionShape {
        CuCollisionShape::Sphere(sphere_m(center, radius))
    }

    fn params(margin_m: f64, skip_adjacent_links: bool) -> CuCollisionModelParams {
        CuCollisionModelParams {
            safety_margin: Length64::new::<meter>(margin_m),
            skip_adjacent_links,
        }
    }

    #[test]
    fn attached_collision_model_snapshot_preserves_geometry_and_pairs() {
        let shape = sphere([0.0; 3], 0.1);
        let mut model = CuRobotCollisionModel::<2, 3>::new(
            &[
                CuLinkShape::new(shape, None),
                CuLinkShape::new(shape, Some(0)),
                CuLinkShape::new(shape, Some(1)),
            ],
            params(0.005, false),
        )
        .unwrap();
        model.set_pair_enabled(0, 2, false).unwrap();
        let mut buffer = [0_u8; 2048];
        let len =
            bincode::encode_into_slice(model, &mut buffer, bincode::config::standard()).unwrap();
        let (restored, consumed): (CuRobotCollisionModel<2, 3>, usize) =
            bincode::decode_from_slice(&buffer[..len], bincode::config::standard()).unwrap();
        assert_eq!(consumed, len);
        assert_eq!(restored, model);
    }

    #[test]
    fn model_constructor_validates_and_skips_pairs_uniformly() {
        let shape = sphere([0.0; 3], 0.1);
        let link = |frame| CuLinkShape::new(shape, frame);
        let shapes = [link(Some(0)), link(Some(1)), link(Some(2)), link(Some(2))];
        let model = CuRobotCollisionModel::<3, 4>::new(&shapes, params(0.0, true)).unwrap();
        assert!(!model.is_pair_enabled(0, 1)); // adjacent
        assert!(model.is_pair_enabled(0, 2));
        assert!(!model.is_pair_enabled(2, 3)); // same rigid body
        let model = CuRobotCollisionModel::<3, 4>::new(&shapes, params(0.0, false)).unwrap();
        assert!(model.is_pair_enabled(0, 1));
        assert!(!model.is_pair_enabled(2, 3));
        // Fewer shapes than capacity.
        let model = CuRobotCollisionModel::<3, 4>::new(&shapes[..2], params(0.0, false)).unwrap();
        assert_eq!(model.shape_count(), 2);
        // Invalid inputs.
        assert!(CuRobotCollisionModel::<3, 1>::new(&shapes, params(0.0, true)).is_err());
        assert!(CuRobotCollisionModel::<2, 4>::new(&shapes, params(0.0, true)).is_err());
        assert!(CuRobotCollisionModel::<3, 4>::new(&shapes, params(-1.0, true)).is_err());
        assert!(
            CuRobotCollisionModel::<3, 4>::new(
                &[
                    link(None),
                    CuLinkShape::new(sphere([f64::NAN; 3], 0.1), None)
                ],
                params(0.0, true)
            )
            .is_err()
        );
    }

    #[test]
    fn collision_model_spec_resolves_presets_and_inline_models() {
        let preset: CuCollisionModelSpec<6> = CuRobotPreset::IgusRebel.into();
        assert_eq!(preset.resolve::<6>().unwrap(), igus_rebel_collision_model());
        assert!(preset.resolve::<5>().is_err());
        fn parse<const M: usize>(text: &str) -> CuResult<CuCollisionModelSpec<M>> {
            // Same path as a node config.
            let cfg: ComponentConfig = ron::from_str(&alloc::format!(
                "#![enable(unwrap_newtypes)]\n{{\"collision_model\": {text}}}"
            ))
            .map_err(|e| CuError::from(alloc::format!("{e}")))?;
            let params: crate::CuCollisionCheckParams<1, M> = cfg.deserialize_into()?;
            params
                .collision_model
                .ok_or_else(|| CuError::from("missing"))
        }
        let spec: CuCollisionModelSpec<2> = parse(
            r#"{"shapes": [{"shape": {"sphere": {"center": [0.0, 0.0, 0.0], "radius": 0.1}}, "frame": 0}, {"shape": {"capsule": {"a": [0.0, 0.0, 0.0], "b": [0.0, 0.0, 1.0], "radius": 0.1}}}], "skip_adjacent_links": false}"#,
        )
        .unwrap();
        let model = spec.resolve::<1>().unwrap();
        assert_eq!(model.link_shapes()[1].frame, None);
        assert_eq!(model.safety_margin().get::<meter>(), 0.0);
        assert!(parse::<2>(r#"{"link_shapes": [], "shapes": []}"#).is_err());
        let spec: CuCollisionModelSpec<6> = parse(r#""igus_rebel""#).unwrap();
        assert_eq!(spec, preset);
    }

    #[test]
    fn test_sphere_and_capsule_signed_distances() {
        let s1 = sphere_m([0.0, 0.0, 0.0], 0.1);
        let s2 = sphere_m([0.3, 0.0, 0.0], 0.05);
        let d_ss = sphere_sphere_signed_distance(&s1, &s2).get::<meter>();
        assert!((d_ss - 0.15).abs() < 1e-12);

        let c1 = capsule_m([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.1);
        let s3 = sphere_m([0.25, 0.0, 0.5], 0.05);
        let d_cs = capsule_sphere_signed_distance(&c1, &s3).get::<meter>();
        assert!((d_cs - 0.10).abs() < 1e-12);

        // Crossing capsules (perpendicular skew segments at z=0.5, separated by 0.2 in X)
        let c2 = capsule_m([0.2, -0.5, 0.5], [0.2, 0.5, 0.5], 0.12);
        let d_cc = capsule_capsule_signed_distance(&c1, &c2).get::<meter>();
        // Segment distance is 0.2, sum of radii is 0.22 -> penetration of -0.02 m
        assert!((d_cc - (-0.02)).abs() < 1e-12);

        let m = Length64::new::<meter>;
        assert!(CuSphere::new([m(0.0); 3], m(-0.1)).is_err());
        assert!(CuSphere::new([m(f64::INFINITY), m(0.0), m(0.0)], m(0.1)).is_err());
        assert_eq!(
            CuSphere::new([m(0.0); 3], m(0.1)).unwrap(),
            sphere_m([0.0; 3], 0.1)
        );
    }

    #[test]
    fn test_igus_rebel_home_is_collision_free_and_folded_self_collides() {
        let chain = igus_rebel_chain();
        let model = igus_rebel_collision_model();

        let q_home = [Angle64::new::<radian>(0.0).into(); 6];
        let mut report = CuCollisionReport::<16>::default();
        model
            .check_configuration(&chain, &q_home, &CuCollisionEnv::EMPTY, &mut report)
            .unwrap();
        assert!(report.is_collision_free());
        assert!(report.min_signed_distance().get::<meter>() > 0.01);

        // Fold upper arm and forearm sharply back into the shoulder/base column
        let q_folded = [
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(1.25).into(),
            Angle64::new::<radian>(2.35).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(1.4).into(),
            Angle64::new::<radian>(0.0).into(),
        ];
        model
            .check_configuration(&chain, &q_folded, &CuCollisionEnv::EMPTY, &mut report)
            .unwrap();
        assert!(!report.is_collision_free());
        assert_eq!(report.as_slice()[0].kind, CuContactKind::SelfCollision);
    }

    #[test]
    fn test_environment_collision_and_segment_interpolation() {
        let chain = igus_rebel_chain();
        let model = igus_rebel_collision_model();

        let q_start = [Angle64::new::<radian>(0.0).into(); 6];
        let q_end = [
            Angle64::new::<radian>(0.8).into(),
            Angle64::new::<radian>(0.5).into(),
            Angle64::new::<radian>(-0.4).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
            Angle64::new::<radian>(0.0).into(),
        ];

        assert!(
            model
                .is_segment_collision_free(
                    &chain,
                    &q_start,
                    &q_end,
                    Angle64::new::<radian>(0.05),
                    &CuCollisionEnv::EMPTY,
                )
                .unwrap()
        );

        // Place an obstacle sphere right on the end-effector at q_home (z ≈ 0.87 m)
        let obstacles = [sphere([0.0, 0.0, 0.87], 0.08)];
        let env = CuCollisionEnv::new(&obstacles);

        assert!(
            !model
                .is_configuration_collision_free(&chain, &q_start, &env)
                .unwrap()
        );
        assert!(
            !model
                .is_segment_collision_free(
                    &chain,
                    &q_start,
                    &q_end,
                    Angle64::new::<radian>(0.05),
                    &env,
                )
                .unwrap()
        );
    }

    #[test]
    fn attached_shapes_and_exclusions_from_scene() {
        let chain = igus_rebel_chain();
        let model = igus_rebel_collision_model();
        let q = [Angle64::new::<radian>(0.0).into(); 6];
        let mut scene = CuPlanningScene::<4, 4>::new();
        // Gripper overlapping the flange shape (robot shape 5) on frame 5.
        let gripper = scene
            .add_attached(Some(5), sphere([0.0, 0.0, 0.05], 0.04), None)
            .unwrap();
        let mut report = CuCollisionReport::<8>::default();
        model
            .check_configuration(&chain, &q, &scene.collision_env(), &mut report)
            .unwrap();
        assert!(!report.is_collision_free());
        assert_eq!(report.as_slice()[0].link_index, 6);
        assert!(report.as_slice().iter().any(|c| c.partner_index == 5));

        let mut exclusions = CuSceneExclusions::<4>::new();
        exclusions.add(CuSceneExclusion::robot(gripper, 4)).unwrap();
        exclusions.add(CuSceneExclusion::robot(gripper, 5)).unwrap();
        exclusions.validate(&scene, model.shape_count()).unwrap();
        let env = scene.collision_env().with_exclusions(&exclusions);
        assert!(
            model
                .is_configuration_collision_free(&chain, &q, &env)
                .unwrap()
        );

        // Unknown IDs fail and leave the report untouched.
        let mut bad = CuSceneExclusions::<1>::new();
        bad.add(CuSceneExclusion::robot(CuObjectId::new(99), 0))
            .unwrap();
        let before = report.clone();
        let env = scene.collision_env().with_exclusions(&bad);
        assert!(
            model
                .check_configuration(&chain, &q, &env, &mut report)
                .is_err()
        );
        assert_eq!(report, before);

        // Same-frame attached shapes are one rigid body; attach/detach keep IDs.
        let box_id = scene
            .add_obstacle(sphere([0.5, 0.0, 0.0], 0.05), None)
            .unwrap();
        scene
            .attach(box_id, Some(5), sphere([0.0, 0.0, 0.05], 0.04))
            .unwrap();
        assert_eq!(scene.attached_shapes().len(), 2);
        let env = scene.collision_env().with_exclusions(&exclusions);
        model
            .check_configuration(&chain, &q, &env, &mut report)
            .unwrap();
        // Only the box (attached index 1) against robot shapes; never box vs. gripper.
        assert!(!report.is_collision_free());
        assert!(
            report
                .as_slice()
                .iter()
                .all(|c| c.link_index == 7 && c.partner_index < 6)
        );
        scene
            .update_attached(box_id, CuAttachedUpdate::new().with_frame(Some(4)))
            .unwrap();
        assert_eq!(scene.attached_shape(box_id).unwrap().frame, Some(4));
        scene.detach(box_id, sphere([0.5, 0.0, 0.0], 0.05)).unwrap();
        assert_eq!(scene.obstacle_ids(), &[box_id]);
        let attached = CuAttachedShape::new(gripper, Some(5), sphere([0.0; 3], 0.01)).unwrap();
        assert_eq!(attached.frame, Some(5));
    }

    #[test]
    fn test_collision_check_task() {
        let (ctx, _mock) = CuContext::new_mock_clock();
        let mut task = CuCollisionCheckTask::<6, 6, 8, 8, 8>::from_params(
            igus_rebel_chain(),
            igus_rebel_collision_model(),
        )
        .unwrap();

        let joints_msg = CuMsg::new(Some(CuJointPositionsPayload {
            positions: [Angle64::new::<radian>(0.0).into(); 6],
        }));
        let scene_msg = CuMsg::new(Some(CuPlanningScene::<8, 8>::new()));
        let exclusions_msg = CuMsg::new(Some(CuSceneExclusions::new()));
        let mut out_msg = CuMsg::new(None);

        task.process(
            &ctx,
            &(&joints_msg, &scene_msg, &exclusions_msg),
            &mut out_msg,
        )
        .unwrap();
        let report = out_msg.payload().unwrap();
        assert!(report.is_collision_free());
    }
}
