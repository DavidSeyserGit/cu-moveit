# cu-moveit 0.4.0

Planner-agnostic motion planning: sampling-based and non-sampling planners
share one contract and one Copper task.

## Added

- `cu-motion-planning` (new crate):
  - `CuJointPlanner`: the trait every planner implements — `begin` a job
    from a request and seed, `step` a bounded work quantum, `write_best`
    into a caller-owned path, report `quality`. `CuPlannerKind` tags the
    family (sampling, optimization, search, direct).
  - `CuJointPlanRequest<N>` / `CuJointPlanPath<N>`: planner-neutral
    payloads. The path carries an explicit `CuPlanStatus` (`searching`,
    `improving`, `converged`, `exhausted`, `invalid_request`,
    `invalid_start`, `invalid_goal`, `capacity_exceeded`); failures are
    published with their reason instead of being dropped.
  - `CuJointPlannerTask<P, N>`: the generic anytime task for any planner,
    owning base/refine budgets (`base_budget`, `refine_budget`), monotone
    publishing, job seeding, and replay state.
  - `CuSceneValidity`: shared scene-aware configuration, segment, and
    clearance queries (obstacles, attached objects, exclusions).
  - Non-sampling planners:
    - `CuJointDirectPlanner`: the straight joint line or a definite
      `exhausted`, in one step.
    - `CuJointPathOptimizer`: deterministic CHOMP-style optimization of a
      fixed waypoint path for smoothness and clearance, improving each
      quantum.

## Changed

- `cu-joint-rrt-star` is the sampling-based implementation of
  `CuJointPlanner`; `CuJointRrtStarTask` is now
  `CuJointPlannerTask<CuJointRrtStar<N, M>, N>`. Its collision queries use
  `CuSceneValidity`.

## Breaking

- Request/path payloads and capacity constants moved to
  `cu-motion-planning` (still re-exported by `cu-joint-rrt-star`).
- `CuJointPlanPath::iterations` is now `effort`; the path gains `status`.
- `CuJointRrtStar::new(chain, model, params)` no longer starts a job; call
  `begin(request, seed)`.
- `CuJointRrtStarTask::new_with_model(..)` is replaced by
  `new_with_planner(CuJointRrtStar::new(..), base_budget, refine_budget, seed)`.
- RON keys `base_iterations` / `block_iterations` are now `base_budget` /
  `refine_budget`.

Workspace version 0.4.0, one shared version as usual; consume via git
dependency on the `v0.4.0` tag.
