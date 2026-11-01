# API conventions

These rules apply to every crate in the workspace, in addition to the house
rules in `AGENTS.md`. The goal is one obvious way to do each thing, with every
capability still reachable and adjustable by parameters.

## Configuration: one struct, the same in Rust and RON

- Every configurable algorithm, task, or bridge has exactly one parameter
  struct named `Cu<Thing>Params`:

  ```rust
  #[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
  #[serde(default, deny_unknown_fields)]
  pub struct CuThingParams { /* fields named after what they are */ }
  impl Default for CuThingParams { /* the only place defaults live */ }
  impl CuThingParams { pub fn validate(&self) -> CuResult<()> { … } }
  ```

- A task's node `config:` block **is** its params struct. `CuTask::new` reads it
  with one call, then validates:

  ```rust
  let params = cfg.map(ComponentConfig::deserialize_into::<CuThingParams>)
      .transpose()?.unwrap_or_default();
  params.validate()?;
  ```

  No key-by-key `cfg.get(...)` parsing, no second spelling of a field.
  `deny_unknown_fields` turns a typo into a startup error instead of a silent
  default.
- Field and RON key names carry no unit suffix (`sample_period`, not
  `sample_period_s`). The type carries the unit, and RON values are SI.
- "Both or neither" settings are one `Option<SubParams>`, not two optionals
  checked at runtime.
- Required inputs that have no sensible default (a robot chain, a goal) are
  fields without a usable default and are rejected by `validate()`.

## Constructors

- Runtime: `CuTask::new(cfg, resources)` only.
- Programmatic: one `pub fn from_params(required…, params: CuThingParams) -> CuResult<Self>`
  that validates. Required inputs come first, then params.
- Plain data types: one validating `new(...)` (returning `CuResult` when it can
  fail). Optional parts use `with_*(self, …) -> Self` builders.
- No `new_with_*`, `try_new`, or `from_meters`-style variants.

## Units

- Public lengths, angles, velocities, accelerations, jerks, and times use
  `cu29::units` types or the joint-typed enums from `cu-kinematics`
  (`CuJointPosition`, `CuJointVelocity`, `CuJointAcceleration`, `CuJointJerk`).
- Dimensionless values (gains, weights, probabilities, metric-scaled
  joint-space distances) stay `f64` and say "dimensionless" in their docs.

## Robot models

- Chains and collision models are given in RON either as a named preset or
  inline: `"chain": "igus_rebel"` or `"chain": {"joints": [...]}`.

## Types and generics

- Never use `#[reflect(type_path = false)]`, despite the generic-struct advice in
  the upstream Copper skills. Every reflected type keeps `TypePath`: tasks so RON
  can name `cu_crate::CuThingTask<6, 1024>` directly (applications never wrap
  library tasks), and payloads/params because real `bevy_reflect` (the
  `cu29/reflect` feature) requires it. Generic type parameters that are not
  reflected get a manual `TypePath` impl (see
  `cu_motion_planning::impl_planner_type_path!`). `just lint` and `just test`
  build with `cu29/reflect` to keep this working.
- Const generics users change most come first; everything else has a default.
  Names: `N` joints, `M` collision shapes, `MAX_*` capacities.
- Each planner `CuJoint<Algo>` has `CuJoint<Algo>Params` and a task alias
  `CuJoint<Algo>Task<N, M = N>`.

## Visibility

- Modules are private; `lib.rs` re-exports the public items, so each item has
  exactly one import path. A crate never re-exports another crate's items.
- Serde helpers, index lookups, and invariant checks are `pub(crate)`. The one
  shared set of fixed-array serde helpers lives in `cu-kinematics`.
- No `#[doc(hidden)] pub`.

## Hot-path functions

- Inputs as one borrowed view struct, outputs as `&mut` (scratch and report).
  No functions that need `#[allow(clippy::too_many_arguments)]`.
- One status enum per problem; no two enums with the same outcomes.

## Preserving capability

A breaking change is acceptable; removing a capability is not. Every change
records how the old use is expressed afterward, and every existing test keeps
its assertion, rewritten against the new API if needed.
