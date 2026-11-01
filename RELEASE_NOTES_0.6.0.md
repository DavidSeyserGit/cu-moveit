# cu-moveit 0.6.0

Runtime reflection works again, and the MuJoCo example is now a
RON-only graph of library components.

## Breaking

- `CuJointPlanner` now requires `TypePath`. Out-of-tree planners add one line:
  `cu_motion_planning::impl_planner_type_path!(MyPlanner, "my_crate::MyPlanner");`
  (for a `MyPlanner<N, M>` planner). Built-in planners already do this.

## Fixed

- The workspace builds with `cu29/reflect` (real `bevy_reflect`). Every
  `#[reflect(type_path = false)]` opt-out was removed: bevy requires `TypePath`
  for every reflected type, so const-generic payloads, params, and model types
  failed to compile with reflection on. `just lint` and `just test` now also
  build with `cu29/reflect`, so this cannot regress silently.

## Added

- `examples/mujoco_rebel`: goal → anytime RRT* → timing → follower →
  controller → MuJoCo, configured entirely in `copperconfig.ron` with the
  `igus_rebel` chain and collision presets. Only the headless result oracle is
  application code. The sidecar ignores non-protocol clients such as local
  HTTP port probes.

## Docs

- README robot-configuration and URDF/SRDF snippets use the 0.5+ API.
- `docs/api-conventions.md`: never use `type_path = false`.
