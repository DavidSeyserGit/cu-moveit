//! Headless test oracle: reports whether the arm reached the goal.

use cu_controller::CuControllerStatus;
use cu_controller::CuJointCommandPayload;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointStatePayload;
use cu29::bincode::Decode;
use cu29::bincode::Encode;
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::DecodeError;
use cu29::bincode::error::EncodeError;
use cu29::prelude::*;
use serde::Deserialize;

/// Consecutive controller-fault cycles tolerated (1 s at 50 Hz): the runtime
/// delivers empty state while the bridge connects.
const MAX_FAULT_CYCLES: u32 = 50;

#[derive(Deserialize)]
struct Goal(#[serde(with = "cu_kinematics::serde_arrays::position_array")] [CuJointPosition; 6]);

/// Writes `result.json` and exits once the controller holds within
/// `tolerance` of `goal`, faults persistently, or `max_cycles` elapse.
#[derive(Debug, Reflect)]
pub struct GoalObserver {
    #[reflect(ignore)]
    goal: [CuJointPosition; 6],
    tolerance: f64,
    max_cycles: u32,
    cycles: u32,
    fault_cycles: u32,
}

impl Freezable for GoalObserver {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.cycles, encoder)?;
        Encode::encode(&self.fault_cycles, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.cycles = Decode::decode(decoder)?;
        self.fault_cycles = Decode::decode(decoder)?;
        Ok(())
    }
}

impl GoalObserver {
    fn finish(&self, outcome: &str, error_rad: Option<f64>) -> ! {
        let error = error_rad.map_or(String::new(), |e| format!(",\"final_error_rad\":{e:.6}"));
        let success = outcome == "reached";
        let body = format!(
            "{{\"success\":{success},\"outcome\":\"{outcome}\",\"cycles\":{}{error}}}",
            self.cycles
        );
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("result.json");
        if let Err(e) = std::fs::write(path, format!("{body}\n")) {
            eprintln!("sim-demo: cannot write result.json: {e}");
            std::process::exit(1);
        }
        eprintln!("sim-demo: {body}");
        std::process::exit(if success { 0 } else { 1 });
    }
}

impl CuSinkTask for GoalObserver {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuJointStatePayload<6>, CuJointCommandPayload<6>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self> {
        let cfg = cfg.ok_or_else(|| CuError::from("observer: missing config"))?;
        let Goal(goal) = cfg
            .get_value::<Goal>("goal")?
            .ok_or_else(|| CuError::from("observer: missing 'goal'"))?;
        Ok(Self {
            goal,
            tolerance: cfg.get::<f64>("tolerance")?.unwrap_or(0.02),
            max_cycles: cfg.get::<u32>("max_cycles")?.unwrap_or(3000),
            cycles: 0,
            fault_cycles: 0,
        })
    }

    fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        let (state, command) = *input;
        self.cycles += 1;
        let error_rad = state.payload().map(|state| {
            self.goal
                .iter()
                .zip(&state.positions)
                .map(|(goal, measured)| (goal.si_value() - measured.si_value()).abs())
                .fold(0.0, f64::max)
        });
        let status = command.payload().map(|command| command.status);
        if status == Some(CuControllerStatus::Holding)
            && error_rad.is_some_and(|e| e < self.tolerance)
        {
            self.finish("reached", error_rad);
        }
        self.fault_cycles = match status {
            Some(CuControllerStatus::Fault) => self.fault_cycles + 1,
            _ => 0,
        };
        if self.fault_cycles >= MAX_FAULT_CYCLES {
            self.finish("controller_fault", error_rad);
        }
        if self.cycles >= self.max_cycles {
            self.finish("timeout", error_rad);
        }
        Ok(())
    }
}
