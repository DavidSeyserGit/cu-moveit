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
use cu_rng::prelude::*;
use cu29::cutask_anytime::AnytimeStatus;
use cu29::cutask_anytime::CuAnytimeTask;
use cu29::cutask_anytime::Quality;
use cu29::cutask_anytime::quality_from_f32;
use cu29::prelude::*;
use cu29::resources;

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

/// Anytime planning task (`CuAnytimeTask`) over any [`CuJointPlanner`].
///
/// `base()` starts a job from the request and runs `base_budget` work units;
/// each `refine()` runs `refine_budget` more. The output only changes when
/// the path cost strictly improves, so the published quality is monotone.
/// Failures (invalid start or goal, invalid request, exhausted search,
/// capacity) are published as a [`CuJointPlanPath`] without waypoints whose
/// `status` names the reason, and the job aborts.
///
/// Config keys: `base_budget` (`u32`, default 64), `refine_budget` (`u32`,
/// default 32), plus the planner's own keys. The job seed comes from the
/// task's `rng` resource; deterministic planners ignore it.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuJointPlannerTask<P, const N: usize> {
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

impl<P: CuJointPlanner<N>, const N: usize> CuJointPlannerTask<P, N> {
    /// Wraps `planner` with explicit work budgets and base seed.
    pub fn new_with_planner(
        planner: P,
        base_budget: u32,
        refine_budget: u32,
        base_seed: u64,
    ) -> Self {
        Self {
            planner,
            base_budget,
            refine_budget,
            base_seed,
            job_counter: 0,
            status: CuPlanStatus::default(),
            effort: 0,
            published_cost: f64::INFINITY,
            published_quality: quality_from_f32(0.0),
        }
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

impl<P, const N: usize> Freezable for CuJointPlannerTask<P, N> {
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

impl<P, const N: usize> CuAnytimeTask for CuJointPlannerTask<P, N>
where
    P: CuJointPlanner<N> + Send + Sync + 'static,
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
        let planner = P::from_config(config)?;
        let mut base_budget = 64u32;
        let mut refine_budget = 32u32;
        if let Some(cfg) = config {
            if let Some(v) = cfg.get::<u32>("base_budget")? {
                base_budget = v;
            }
            if let Some(v) = cfg.get::<u32>("refine_budget")? {
                refine_budget = v;
            }
        }
        if base_budget == 0 || refine_budget == 0 {
            return Err(CuError::from(
                "cu-motion-planning: 'base_budget' and 'refine_budget' must be positive",
            ));
        }
        let Owned(mut rng) = resources.rng;
        Ok(Self::new_with_planner(
            planner,
            base_budget,
            refine_budget,
            rng.random::<u64>(),
        ))
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
