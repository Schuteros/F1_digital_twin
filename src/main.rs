pub mod physics;

use physics::config::load_run;
use physics::simulation::start_simulation;
use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

fn main() -> ExitCode {
    // 1. Load the config from the run file given as the first argument
    let Some(run_path) = std::env::args().nth(1) else {
        eprintln!("Usage: cargo run -- <run file>, e.g. cargo run -- data/runs/baseline.toml");
        return ExitCode::FAILURE;
    };

    let mut config = match load_run(Path::new(&run_path)) {
        Ok(config) => config,
        Err(message) => {
            eprintln!("Error: {}", message);
            return ExitCode::FAILURE;
        }
    };

    println!("Launching vehicle dynamics simulation...");

    // 2. Run the simulation end-to-end
    let start = Instant::now();
    let final_state = start_simulation(&mut config, false, false);
    let duration = start.elapsed();

    println!("Simulation finished in: {:?}", duration);

    // 3. Inspect the final output
    println!("\nSimulation Complete!");
    println!("Final Time    : {:.3} s", final_state.time);
    println!("Final Speed   : {:.2} m/s", final_state.car.speed);
    println!("Final Distance: {:.2} m", final_state.car.distance);

    ExitCode::SUCCESS
}
