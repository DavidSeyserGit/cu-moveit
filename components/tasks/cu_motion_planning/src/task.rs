//! The one Copper task that runs any [`CuJointPlanner`].

use crate::CuJointPlanPath;
use crate::CuJointPlanRequest;
use crate::CuJointPlanner;
use crate::CuPlanStatus;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_collision::CuCollisionModelSpec;
use cu_kinematics::CuChainSpec;
use cu_rng::prelude::*;
use cu29::cutask_anytime::AnytimeStatus;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use cu29::resources;
use serde::Deserialize;
use serde::Serialize;

/// Resources of [`CuJointPlannerTask`]: an owned `rng` that seeds jobs.
pub mod planner_resources {
    use super::*;
    resources!({ rng => Owned<CuRng> });
}

/// Compact progress snapshot for remote inspection.
#[derive(Default, Debug, Reflect)]
pub struct CuJointPlannerDebugState {
    pub status: CuPlanStatus,
    pub effort: u32,
    pub published_cost: f64,
    pub published_quality: Quality,
}

/// Node config of [`CuJointPlannerTask`] (and every `CuJoint<Planner>Task`
/// alias): robot models, the planner's own parameters, and work budgets.
///
/// `P` is the planner's [`CuJointPlanner::Params`].
///
/// ```ron
/// // CuJointRrtStarTask<6> (cu-joint-rrt-star)
/// config: {
///     "chain": "igus_rebel",            // or {"joints": [...]}
///     "collision_model": "igus_rebel",  // or {"shapes": [...], "safety_margin": 0.005}
///     "params": {"step_size": 0.3, "max_nodes": 200}, // planner params; omit = defaults
///     "base_budget": 64,                // optional; default CuJointPlanner::DEFAULT_BUDGETS
///     "refine_budget": 32,              // optional
/// }
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
// Field-level defaults: serde does not support container `default` on const-generic structs.
#[serde(
    deny_unknown_fields,
    bound(
        serialize = "P: Serialize",
        deserialize = "P: Deserialize<'de> + Default"
    )
)]
pub struct CuJointPlannerParams<P, const N: usize, const M: usize = N> {
    /// Required robot chain: a preset name (`"igus_rebel"`) or an inline chain.
    #[serde(default)]
    pub chain: Option<CuChainSpec<N>>,
    /// Required collision model: a preset name (`"igus_rebel"`) or an inline model.
    #[serde(default)]
    pub collision_model: Option<CuCollisionModelSpec<M>>,
    /// The planner's parameters (validated by [`CuJointPlanner::new`]).
    #[serde(default)]
    pub params: P,
    /// Work units of `base()`; `None` uses the planner's default.
    #[serde(default)]
    pub base_budget: Option<u32>,
    /// Work units of each `refine()`; `None` uses the planner's default.
    #[serde(default)]
    pub refine_budget: Option<u32>,
}

impl<P: Default, const N: usize, const M: usize> Default for CuJointPlannerParams<P, N, M> {
    fn default() -> Self {
        Self {
            chain: None,
            collision_model: None,
            params: P::default(),
            base_budget: None,
            refine_budget: None,
        }
    }
}

impl<P, const N: usize, const M: usize> CuJointPlannerParams<P, N, M> {
    /// Checks that the chain and collision model are present and valid and
    /// that explicit budgets are positive. Planner parameters are validated
    /// by [`CuJointPlanner::new`].
    pub fn validate(&self) -> CuResult<()> {
        self.resolve_models()?;
        if self.base_budget == Some(0) || self.refine_budget == Some(0) {
            return Err(CuError::from(
                "cu-motion-planning: 'base_budget' and 'refine_budget' must be positive",
            ));
        }
        Ok(())
    }

    fn resolve_models(
        &self,
    ) -> CuResult<(
        cu_kinematics::CuKinematicChain<N>,
        cu_collision::CuRobotCollisionModel<N, M>,
    )> {
        let chain = self
            .chain
            .as_ref()
            .ok_or_else(|| CuError::from("cu-motion-planning: missing required 'chain'"))?
            .resolve()?;
        let model = self
            .collision_model
            .as_ref()
            .ok_or_else(|| CuError::from("cu-motion-planning: missing required 'collision_model'"))?
            .resolve::<N>()?;
        Ok((chain, model))
    }
}

/// Anytime planning task (`CuAnytimeTask`) over any [`CuJointPlanner`].
///
/// `base()` starts a job from the request and runs `base_budget` work units;
/// each `refine()` runs `refine_budget` more. The output only changes when
/// the path cost strictly improves, so the published quality is monotone.
/// Failures (invalid start or goal, invalid request, exhausted search,
/// capacity) are published as a [`CuJointPlanPath`] without waypoints whose
/// `status` names the reason, and the job aborts.
///
/// Node config: [`CuJointPlannerParams`]. The job seed comes from the task's
/// `rng` resource; deterministic planners ignore it. Each planner has a task
/// alias (`CuJointRrtConnectTask<N, M = N>`, ...), so RON names it directly.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuJointPlannerTask<P, const N: usize, const M: usize = N> {
    #[reflect(ignore)]
    planner: P,
    base_budget: u32,
    refine_budget: u32,
    base_seed: u64,
    job_counter: u64,
    #[reflect(ignore)]
    status: CuPlanStatus,
    #[reflect(ignore)]
    effort: u32,
    published_cost: f64,
    published_quality: Quality,
}

impl<P: CuJointPlanner<N, M>, const N: usize, const M: usize> CuJointPlannerTask<P, N, M> {
    /// Wraps `planner` with explicit positive work budgets and a base seed.
    pub fn from_planner(
        planner: P,
        base_budget: u32,
        refine_budget: u32,
        base_seed: u64,
    ) -> CuResult<Self> {
        if base_budget == 0 || refine_budget == 0 {
            return Err(CuError::from(
                "cu-motion-planning: 'base_budget' and 'refine_budget' must be positive",
            ));
        }
        Ok(Self {
            planner,
            base_budget,
            refine_budget,
            base_seed,
            job_counter: 0,
            status: CuPlanStatus::default(),
            effort: 0,
            published_cost: f64::INFINITY,
            published_quality: quality_from_f32(0.0),
        })
    }

    /// Builds the planner from `params` (robot models, planner parameters,
    /// budgets with per-planner defaults) and wraps it with `base_seed`.
    pub fn from_params(
        params: CuJointPlannerParams<P::Params, N, M>,
        base_seed: u64,
    ) -> CuResult<Self> {
        params.validate()?;
        let (chain, model) = params.resolve_models()?;
        let planner = P::new(chain, model, params.params)?;
        let (base, refine) = P::DEFAULT_BUDGETS;
        Self::from_planner(
            planner,
            params.base_budget.unwrap_or(base),
            params.refine_budget.unwrap_or(refine),
            base_seed,
        )
    }

    /// The wrapped planner.
    pub const fn planner(&self) -> &P {
        &self.planner
    }

    /// Status of the current job.
    pub const fn status(&self) -> CuPlanStatus {
        self.status
    }

    /// Compact snapshot of the current job's progress.
    pub fn debug_state(&self) -> CuJointPlannerDebugState {
        CuJointPlannerDebugState {
            status: self.status,
            effort: self.effort,
            published_cost: self.published_cost,
            published_quality: self.published_quality,
        }
    }

    fn reset_published(&mut self, output: &mut CuMsg<CuJointPlanPath<N>>) {
        output.clear_payload();
        self.published_cost = f64::INFINITY;
        self.published_quality = quality_from_f32(0.0);
    }

    /// Publishes an improved path or a final failure; returns the anytime status.
    fn publish(&mut self, output: &mut CuMsg<CuJointPlanPath<N>>) -> AnytimeStatus<Quality> {
        let mut path = CuJointPlanPath::<N>::default();
        let status = self.planner.write_best(&mut path);
        self.status = status;
        self.effort = path.effort;
        if status.has_path() {
            if path.cost < self.published_cost {
                self.published_cost = path.cost;
                self.published_quality = self.planner.quality();
                output.set_payload(path);
            } else if let Some(published) = output.payload_mut() {
                // Same path, but the job may have just converged.
                published.status = status;
                published.effort = path.effort;
            }
            return if status == CuPlanStatus::Converged {
                AnytimeStatus::Converged(self.published_quality)
            } else {
                AnytimeStatus::Improved(self.published_quality)
            };
        }
        if status.is_final() {
            self.published_cost = f64::INFINITY;
            self.published_quality = quality_from_f32(0.0);
            output.set_payload(path);
            return AnytimeStatus::Aborted;
        }
        AnytimeStatus::Improved(self.published_quality)
    }
}

impl<P, const N: usize, const M: usize> Freezable for CuJointPlannerTask<P, N, M> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.base_seed, encoder)?;
        Encode::encode(&self.job_counter, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.base_seed = Decode::decode(decoder)?;
        self.job_counter = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<P, const N: usize, const M: usize> CuAnytimeTask for CuJointPlannerTask<P, N, M>
where
    P: CuJointPlanner<N, M> + Send + Sync + 'static,
{
    type Input<'m> = input_msg!(CuJointPlanRequest<N>);
    type Output<'m> = output_msg!(CuJointPlanPath<N>);
    type Resources<'r> = planner_resources::Resources;
    type Quality = Quality;

    fn register_debug_state_types(registry: &mut TypeRegistry) {
        registry.register::<CuJointPlannerDebugState>();
    }

    fn debug_state_type_path() -> &'static str {
        CuJointPlannerDebugState::type_path()
    }

    fn with_debug_state<R>(&self, f: impl FnOnce(&dyn bevy_reflect::Reflect) -> R) -> R {
        f(&self.debug_state())
    }

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let params = config
            .map(ComponentConfig::deserialize_into::<CuJointPlannerParams<P::Params, N, M>>)
            .transpose()?
            .unwrap_or_default();
        let Owned(mut rng) = resources.rng;
        Self::from_params(params, rng.random::<u64>())
    }

    fn base<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        output.tov = input.tov;
        self.reset_published(output);
        let Some(request) = input.payload() else {
            self.status = CuPlanStatus::default();
            return Ok(AnytimeStatus::Aborted);
        };
        self.job_counter = self.job_counter.wrapping_add(1);
        let seed = self.base_seed.wrapping_add(self.job_counter);
        let status = self.planner.begin(request, seed)?;
        if !status.is_final() {
            self.planner.step(self.base_budget)?;
        }
        Ok(self.publish(output))
    }

    fn refine<'o>(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        if self.status.is_final() {
            return Ok(if self.status.has_path() {
                AnytimeStatus::Converged(self.published_quality)
            } else {
                AnytimeStatus::Aborted
            });
        }
        self.planner.step(self.refine_budget)?;
        Ok(self.publish(output))
    }
}
