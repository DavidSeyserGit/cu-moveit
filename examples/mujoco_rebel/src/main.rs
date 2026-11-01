//! Closed-loop MuJoCo demo: plan to a joint goal, time and follow the path,
//! and drive the simulated igus ReBeL. The graph is entirely in
//! `copperconfig.ron`; only the headless test oracle ([`observer`]) is
//! application code.

mod observer;

use cu29::prelude::*;

#[copper_runtime(config = "copperconfig.ron")]
struct SimDemo {}

fn main() -> CuResult<()> {
    let log_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("sim_demo.copper");
    let app = SimDemo::builder().with_log_path(log_path, None)?.build()?;
    app.run_until_shutdown().map_err(|fault| fault.error)?;
    Ok(())
}
