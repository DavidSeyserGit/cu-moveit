# Migrating to 0.5

0.5 simplifies the public API across the workspace following
[`api-conventions.md`](api-conventions.md). No capability was removed; each
entry below shows where it moved.

Common to all crates:

- Import from the crate root (`cu_kinematics::CuJointLimits`); modules are private.
- A task's RON `config:` block is one params struct, read with
  `deserialize_into`. Unknown keys are startup errors. Keys have no unit
  suffixes and values are SI.
- `new_with_*` constructors are now `from_params(...) -> CuResult<Self>`.

## cu-kinematics

| Before | After |
|---|---|
| `CuIkConfig`, `CuDiffIkConfig` | `CuIkParams`, `CuDiffIkParams` (+ `validate()`); `CuIkParams.solver: CuIkSolver` declares the algorithm (`damped_least_squares` default, `pseudoinverse`) |
| `CuIkConfig::from_config`, keys `ik_config` / flat `*_m`, `*_rad` | node key `"ik": {...}` / `"diff_ik": {...}` |
| `CuKinematicChain::new_with_tool(j, t)` | `CuKinematicChain::new(j).with_tool_offset(t)` |
| `CuKinematicChain::from_config(cfg)` | `CuChainSpec<N>::resolve()`; RON `"chain": "igus_rebel"` or `{"joints": [...]}` |
| `CuJointSpec::revolute(xyz_m: [f64;3], rpy_rad: [f64;3], axis, limits)` | `CuJointSpec::revolute([Length;3], [Angle;3], axis, limits)` (same for `prismatic`) |
| `CuJointLimits::from_radians(lo, hi, v)` | `CuJointLimits::revolute(Angle, Angle, AngularVelocity)` |
| `CuJointLimits::from_meters(lo, hi, v)` | `CuJointLimits::prismatic(Length, Length, Velocity)` |
| `CuTwist6D::from_base_mps_radps(l, a)` / `from_body_mps_radps` | `CuTwist6D::new([Velocity;3], [AngularVelocity;3], CuTwistFrame::Base / Body)` |
| `CuForwardKinematicsTask::new_with_chain(chain)` | `from_params(chain)` |
| `CuInverseKinematicsTask::new_with_chain(chain, cfg)`, `.config()` | `from_params(chain, CuIkParams)`, `.params()` |
| `CuServoTask::new_with_chain(chain, cfg)`, `.config()` | `from_params(chain, CuDiffIkParams)`, `.params()` |
| — | `CuRobotPreset`, `CuChainSpec<N>`, `CuJointAcceleration`, `CuJointJerk`, `serde_arrays::{acceleration_array, jerk_array}` |
| `CuJointStatePayload`, `CuRobotStatus` in `cu-controller` | defined in `cu-kinematics` (still re-exported by `cu-controller`) |

Task RON:

```ron
// CuInverseKinematicsTask<6>
config: {"chain": "igus_rebel", "ik": {"max_iterations": 128}}
```

## cu-io

| Before | After |
|---|---|
| `CuGripperWiring` | `CuGripperParams` (serde, `validate::<IN, OUT>()`) |
| `CuGripperWiring::single_acting(i)` | `CuGripperParams::single_acting(i)` |
| `settle_ns: u64`, `motion_timeout_ns: u64` | `settle: Time`, `motion_timeout: Time` |
| `CuGripperTask::new_with_wiring(w)` | `CuGripperTask::from_params(p)`, `.params()` |

```ron
config: {"close_output": 0, "open_output": 1, "settle": 0.3, "motion_timeout": 2.0}
```

## cu-collision

Modules are private (`cu_collision::scene_query::*`, `cu_collision::model::*` etc. are gone);
import everything from the crate root.

| Before | After |
|---|---|
| `CuSphere::from_meters(c, r)` / `CuCapsule::from_meters(a, b, r)` | `CuSphere::new([Length;3], Length) -> CuResult` / `CuCapsule::new(a, b, radius) -> CuResult` |
| — | `CuCollisionShape::validate()`, `CuLinkShape { shape, frame: Option<usize> }` |
| `CuRobotCollisionModel::new(shapes, margin)` | `new(&[CuLinkShape::new(s, Some(i)), ...], CuCollisionModelParams { safety_margin, skip_adjacent_links: true })?` |
| `CuRobotCollisionModel::new_attached(shapes, frames, len, margin)` | `new(&link_shapes[..len], CuCollisionModelParams { safety_margin, skip_adjacent_links: false })?` |
| `CuRobotCollisionModel::from_config(cfg)` | `CuCollisionModelSpec<M>::resolve::<N>()`; RON `"collision_model": "igus_rebel"` or `{"shapes": [...]}` |
| `model.link_shapes() -> &[CuCollisionShape; M]` | `model.link_shapes() -> &[CuLinkShape]` (active only; `.shape`, `.frame`) |
| `model.set_safety_margin(m)` | returns `CuResult<()>` (rejects negative/non-finite) |
| `CuCollisionWorld<O>` (`new`, `add_obstacle`, `as_slice`) | `CuCollisionEnv::new(&[CuCollisionShape])` (borrowed, zero-copy) |
| `world: Option<&CuCollisionWorld<O>>` arg of `check_frames` / `check_configuration` / `is_configuration_collision_free` / `is_segment_collision_free` | `env: &CuCollisionEnv<'_>`; `None` → `&CuCollisionEnv::EMPTY` |
| `model.check_frames(..)` (infallible) | returns `CuResult<()>` (env validation; report untouched on error) |
| `scene.write_collision_world(&mut world)` | `scene.collision_env()` |
| `scene_query::{check_attached_shapes, check_attached_configuration, check_attached_world_shapes, attached_world_shapes_collision_free, transform_attached_shapes}` | the normal model checks with `scene.collision_env().with_exclusions(&excl)` (or `CuCollisionEnv::new(obs).with_attached(&attached)`); one call covers robot, obstacles, and attached shapes |
| `CuSceneQueryExclusions`, `CuSceneQueryExclusion`, `CuQueryPartnerKind`, `CuSceneExclusions::resolve(scene, n, &mut out)` | removed; ID-based `CuSceneExclusions` is used directly. Up-front check: `exclusions.validate(&scene, model.shape_count())` |
| `report.len`, `report.total_contacts`, `report.min_signed_distance`, `report.contacts` (pub fields) | `report.as_slice()`, `report.total_contacts()`, `report.min_signed_distance()` |
| `CuObjectId(pub u32)`, `CuSceneRevision(pub u64)` | private field; `CuObjectId::new` / `as_u32`, `CuSceneRevision::new` / `as_u64` |
| `CuSceneObstacle::new` (unchecked) / `try_new` | `CuSceneObstacle::new(id, shape) -> CuResult` |
| `CuAttachedShape { link_index: usize, .. }`, `new` / `try_new` / `new_with_transform` / `try_new_with_transform` | `CuAttachedShape { frame: Option<usize>, .. }`, `new(id, frame, shape) -> CuResult`, `.with_transform(&T) -> CuResult<Self>` |
| `scene.add_obstacle(shape)` / `add_obstacle_with_id(id, shape)` | `scene.add_obstacle(shape, None)` / `add_obstacle(shape, Some(id))` → `CuResult<CuObjectId>` |
| `scene.add_attached_shape(link, shape)` / `add_attached_shape_with_id(id, link, shape)` | `scene.add_attached(Some(link), shape, None / Some(id))` |
| `update_attached_shape(id, s)` / `update_attached_link(id, l)` / `update_attached(id, l, s)` | `update_attached(id, CuAttachedUpdate::new().with_shape(s).with_frame(Some(l)))` |
| `remove_attached_shape(id)` / `clear_attached_shapes()` | `remove_attached(id)` / `clear_attached()` |
| `attach_obstacle(id, link, local)` / `detach_shape(id, world)` | `attach(id, Some(link), local)` / `detach(id, world)` |
| `scene.obstacles() -> &[CuSceneObstacle]` | `obstacles()` iterator of `CuSceneObstacle`; slices `obstacle_shapes()`, `obstacle_ids()` |
| `obstacle_count()`, `attached_count()`, `total_object_count()`, `is_obstacle_full()`, `is_attached_full()`, `obstacle_capacity()`, `attached_capacity()` | `obstacle_shapes().len()`, `attached_shapes().len()`, `CuPlanningScene::<O, A>::OBSTACLE_CAPACITY` / `ATTACHED_CAPACITY` |
| `scene.world_attached_shapes(frames, out, &mut len)` | `let len = scene.world_attached_shapes(frames, out)?;` |
| `find_obstacle_index`, `find_attached_index`, `validate_invariants`, `next_id` | crate-private (invariants hold after every mutation and decode) |
| `DEFAULT_MAX_SCENE_EXCLUSIONS` | `DEFAULT_MAX_EXCLUSIONS` (also exported: `DEFAULT_MAX_OBSTACLES`, `DEFAULT_MAX_ATTACHED`, `DEFAULT_MAX_CONTACTS`) |
| `CuCollisionCheckTask<N, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, MAX_EXCL, M>` | `CuCollisionCheckTask<N, M = N, MAX_OBS, MAX_ATTACHED, MAX_CONTACTS, MAX_EXCL>` |
| `CuCollisionCheckTask::new_with_model(chain, model)` | `from_params(chain, model) -> CuResult<Self>` |
| task keys `chain` + `collision_model` + `safety_margin_m` | `CuCollisionCheckParams { chain, collision_model }`; margin lives in the model spec |

Adjacent-link skipping is now explicit and identical on every path: old `from_config` skipped
adjacent frames, `new_attached` did not. Shapes on the same frame are never checked.

```ron
// CuCollisionCheckTask<6>
config: {"chain": "igus_rebel", "collision_model": "igus_rebel"}
// inline model (exactly M shapes; omit "frame" for the base frame)
"collision_model": {
    "shapes": [
        {"shape": {"capsule": {"a": [0.0, 0.0, -0.10], "b": [0.0, 0.0, 0.04], "radius": 0.052}}, "frame": 0},
        {"shape": {"sphere": {"center": [0.0, 0.0, 0.02], "radius": 0.032}}, "frame": 1},
    ],
    "safety_margin": 0.005,       // default 0.0
    "skip_adjacent_links": true,  // default true
}
```

Old `{"link_shapes": [...], "frame_indices": [...]}` becomes `"shapes"` with one
`{"shape", "frame"}` entry per old index (old default: shape `i` on frame `i`).

## cu-robot-description

| Before | After |
|---|---|
| `from_urdf(xml, base, tip, margin: Length)` | `from_urdf(xml, CuUrdfImport::new(base_link, tip_link).with_collision(CuCollisionModelParams { .. }))` |
| `from_urdf_srdf(urdf, srdf, group, margin: Length)` | `from_urdf_srdf(urdf, srdf, CuSrdfImport::new(group).with_collision(CuCollisionModelParams { .. }))` |
| margin only; adjacent links never skipped | `CuCollisionModelParams { safety_margin, skip_adjacent_links }`; import default is margin `0` and `skip_adjacent_links: false` (unchanged behavior; SRDF `disable_collisions` disables pairs) |
| `robot.chain`, `robot.collision_model` (pub fields) | `robot.chain()`, `robot.collision_model()` (`&`, both `Copy`), or `robot.into_parts() -> (chain, model)` |

```rust,ignore
let robot = CuRobotDescription::<6, 24>::from_urdf_srdf(
    &urdf_xml,
    &srdf_xml,
    CuSrdfImport::new("manipulator").with_collision(CuCollisionModelParams {
        safety_margin: Length::new::<meter>(0.005),
        skip_adjacent_links: false,
    }),
)?;
let (chain, model) = robot.into_parts();
```

## cu-trajectory

Modules are private (`cu_trajectory::parameterize::*`, `cu_trajectory::path_timing::*`,
`cu_trajectory::cartesian_blend::*`, `cu_trajectory::quintic::*`, `cu_trajectory::serde_arrays`
are gone); import everything from the crate root.

| Before | After |
|---|---|
| `parameterize_trajectory(wp, &limits, dt, &mut out)` | `parameterize(wp, &CuTimingParams { limits, sample_period: dt, ..Default::default() }, CuTimingOverrides::default(), &mut out)` |
| `parameterize_with_profile(wp, &limits, profile, &timing, dt, &mut out)` | `parameterize(wp, &CuTimingParams { limits, profile, sample_period: dt }, overrides, &mut out)` |
| `CuTrajectoryTiming<'a, N>` (`new()`) | `CuTimingOverrides<'a, N>` (`default()`); same `segment_durations` / `waypoint_velocities` fields |
| `CuParameterizeStatus`, `CuPathTimingStatus` | one `CuTimingStatus` (`Success`, `EmptyPath`, `InvalidInput`, `CapacityExceeded`, `Collision`, `ZoneExceeded`, `NoContinuousMotion`, `UnexpectedStop`) |
| `CuParameterizeStatus::InvalidLimits` | `CuTimingStatus::InvalidInput` |
| `CuJointTrajectoryLimits::max_accelerations: [f64; N]` | `[CuJointAcceleration; N]` |
| `max_jerks: [f64; N]` (`f64::INFINITY` = unbounded) | `max_jerks: Option<[CuJointJerk; N]>` (`None` = unbounded; an `INFINITY` entry still unbounds one joint) |
| `CuJointTrajectoryLimits::new(v, [f64])`, `new_with_jerks(v, a, j)`, `.with_max_jerks(j)` | `CuJointTrajectoryLimits::new(v, a)` + `.with_jerks(j)`; `.validate()` |
| `CuTrajectoryPoint::accelerations: [f64; N]` | `[CuJointAcceleration; N]` |
| — | `CuTrajectoryPayload::interpolation: CuTrajectoryInterpolation` (default `Linear`; `parameterize` sets `Cubic`, Cartesian S-curve sets `Quintic`, other Cartesian profiles `Cubic`) |
| `payload.sample(t, ..)` (always linear) | `payload.sample_linear(t, ..)`; `payload.sample(t, ..)` now dispatches on `payload.interpolation`; `sample_with(interp, t, ..)` for an explicit choice; `sample_cubic` / `sample_quintic` unchanged |
| `quintic::quintic_limit_ratio(&traj, &limits)` | `traj.quintic_limit_ratio(&limits)` |
| `quintic::stretch_time(&mut traj, k)` | `traj.stretch_time(k)` |
| `CuFollowTrajectoryTask::new()` | `CuFollowTrajectoryTask::default()` |
| `CuFollowTrajectoryTask::new_with_interpolation(i)` | `from_params(CuFollowTrajectoryParams { interpolation: Some(i) })?`, `.params()` |
| follower default interpolation `Linear` | `None` = the payload's own interpolation (hand-built payloads are still `Linear`) |
| `CuPathTimingConfig { limits, profile, sample_period, collision_step }` | `CuPathTimingParams { timing: CuTimingParams { limits, profile, sample_period }, collision_step }` (+ `validate()`) |
| `parameterize_cartesian_path(chain, model, world: Option<&CuCollisionWorld<O>>, path, &cfg, out)` | `parameterize_cartesian_path(chain, model, &env, path, &params, out)`; `None` → `&CuCollisionEnv::EMPTY` |
| `blend_cartesian_path(chain, model, world, &waypoints, zones: &[Length], &cfg, out)` | `blend_cartesian_path(chain, model, &env, &[CuCartesianWaypoint::new(q, zone), ...], &params, out)` |
| `CuCartesianBlendConfig { .., ik: CuIkConfig }` | `CuCartesianBlendParams { .., ik: CuIkParams }` (serde default / deny unknown, `validate()`; invalid params → `InvalidInput`) |

Limits and timing now reject mismatched acceleration (and, for the S-curve, jerk) kinds the
same way velocity kinds were already rejected.

```ron
// CuFollowTrajectoryTask<6, 1024>
config: {}                          // sample each trajectory with its own interpolation
config: {"interpolation": "cubic"}  // explicit override (old behaviour of the key)

// CuTimingParams<N> (e.g. the timing block of a planner-to-trajectory task)
{
    "limits": {
        "max_velocities": [{"revolute": 0.8}, {"revolute": 0.8}],
        "max_accelerations": [{"revolute": 2.0}, {"revolute": 2.0}], // was [2.0, 2.0]
        "max_jerks": [{"revolute": 10.0}, {"revolute": 10.0}],      // optional; omit = unbounded
    },
    "profile": "s_curve",   // default "trapezoidal"
    "sample_period": 0.02,  // was "sample_period_s"; default 0.02
}
```

## cu-controller

The `cu_kinematics` re-exports are gone (this supersedes the "still re-exported
by `cu-controller`" note above): RON connections must name
`cu_kinematics::CuJointStatePayload<6>`, Rust code imports
`cu_kinematics::{CuJointStatePayload, CuRobotStatus}`.

| Before | After |
|---|---|
| keys `mode`, `tracking_error_limit_rad`, `state_timeout_ns`, `max_joint_speed_si`, `max_joint_acceleration_si`, `effort_kp`, `effort_kv` read one by one | one `CuControllerParams` (`deserialize_into`, unknown keys rejected, `validate()`) |
| `tracking_error_limit_rad: f64` (compared in SI per joint kind: rad or m) | `tracking_error_limit: f64`, same semantics (dimensionless SI magnitude per joint kind) |
| `state_timeout_ns: u64` (0 rejected) | `state_timeout: Option<Time>` (seconds in RON; non-positive rejected) |
| `max_joint_speed_si` + `max_joint_acceleration_si` (both-or-neither, runtime check) | `rate_limit: Option<CuRateLimit { max_speed, max_acceleration }>` (same per-joint-kind SI scalars; still requires `mode: position`) |
| `effort_kp`, `effort_kv` | `effort_gains: CuEffortGains { kp, kv }` (defaults 20.0 / 0.8) |
| `CuControllerTask::new_with_limit(limit)` (unvalidated) | `CuControllerTask::from_params(CuControllerParams { tracking_error_limit: limit, ..Default::default() })?` |
| `set_effort_gains(kp, kv)` (silently ignored invalid input) | `set_effort_gains(CuEffortGains { kp, kv }) -> CuResult<()>` |
| — | `CuControllerTask::params()` |
| `CuJointEffort::si_value` (`#[doc(hidden)]`) | documented `pub` |
| `cu_controller::serde_arrays` (private module, `pub` item) | `pub(crate)` |

`CuJointCommandPayload` is unchanged (including `max_tracking_error`).

```ron
config: {
    "mode": "position",
    "tracking_error_limit": 0.5,
    "state_timeout": 0.2,
    "rate_limit": {"max_speed": 1.5, "max_acceleration": 6.0},
    "effort_gains": {"kp": 20.0, "kv": 0.8},
}
```

## cu-mujoco-sim

| Before | After |
|---|---|
| `DEFAULT_HOST`, `DEFAULT_PORT`, `DEFAULT_READ_TIMEOUT_MS` | `CuMujocoSimParams::default()` (`127.0.0.1`, `8765`, 0.25 s) |
| keys `host`, `port`, `read_timeout_ms` (read one by one) | one `CuMujocoSimParams { host, port, read_timeout: Time }` (`deserialize_into`, unknown keys rejected, `validate()`) |
| `bridge.host()`, `.port()`, `.read_timeout_ms()` | `bridge.params().host`, `.port`, `.read_timeout` |
| — | `CuMujocoSimBridge::from_params(params)` |
| `pub fn encode_command_line(..) -> String`, `pub fn decode_state_line` | `pub(crate)`; encoding returns `CuResult<String>` instead of panicking |

```ron
config: {"host": "127.0.0.1", "port": 8765, "read_timeout": 0.25}
```

## cu-motion-planning

Modules are private; import everything from the crate root.

| Before | After |
|---|---|
| `CuJointDirectPlanner` | `CuJointDirect`; every planner (`CuJointAStar`, `CuJointPrm`, `CuJointRrtConnect`, `CuJointStomp`, `CuJointPathOptimizer`, `CuJointDirect`) now also has a `CuJoint<Algo>Task<N, M = N>` alias |
| `CuSceneValidity`, `cu_motion_planning::joint_distance` (module paths) | crate-root `CuSceneValidity` / `joint_distance` (modules are private) |
| `CuJointOptimizerParams` | `CuJointPathOptimizerParams` |
| planner task built ad hoc per planner | `CuJointPlannerTask::from_planner(planner, base, refine, seed)?` or `::from_params(CuJointPlannerParams { chain, collision_model, params, base_budget, refine_budget }, seed)?` |
| node config parsed per planner | one `CuJointPlannerParams` node config (`"chain"`, `"collision_model"`, `"params"`, `"base_budget"`, `"refine_budget"`) |
| `CuJointPlanRequest` / `CuJointPlanPath` also reachable via `cu-joint-rrt-star` | only `cu_motion_planning::{CuJointPlanRequest, CuJointPlanPath}` |
| — | `CuJointGoalTask` (IK goal task), `CuPlanToTrajectoryTask` (plan → timed trajectory), capacity consts (`ASTAR_MAX_NODES`, `PRM_MAX_NODES`, `RRT_CONNECT_MAX_NODES`, `STOMP_MAX_ROLLOUTS`) |

```ron
// CuJointRrtStarTask<6>
config: {"chain": "igus_rebel", "collision_model": "igus_rebel", "params": {"step_size": 0.3}}
```

## cu-joint-rrt-star

| Before | After |
|---|---|
| `CuJointRrtParams` | `CuJointRrtStarParams` (+ `validate()`) |
| `MAX_NODES` | `RRT_STAR_MAX_NODES` |
| `CuJointRrtStar::new(chain, model, CuJointRrtParams)` (infallible) | `CuJointPlanner::new(chain, model, CuJointRrtStarParams) -> CuResult<Self>` (trait in scope) |
| `CuJointRrtStarTask::new_with_planner(planner, 64, 32, 7)` | `CuJointRrtStarTask::from_planner(planner, 64, 32, 7)?` |
| own `CuJointPlanRequest` / `CuJointPlanPath` types | `cu_motion_planning::CuJointPlanRequest` / `CuJointPlanPath` |
