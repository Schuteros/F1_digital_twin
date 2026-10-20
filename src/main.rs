pub mod physics;

use physics::simulation::{SimulationConfig, start_simulation};
use std::time::Instant;

fn main() {
    // 1. Load default config (or tweak specific values)
    let config = SimulationConfig {
        end_time: 100000.0,
        ..SimulationConfig::default()
    };

    println!("Launching vehicle dynamics simulation...");

    // 2. Run the simulation end-to-end
    let start = Instant::now();
    let final_state = start_simulation(&config, false, false);
    let duration = start.elapsed();

    println!("Simulation finished in: {:?}", duration);

    // 3. Inspect the final output
    println!("\nSimulation Complete!");
    println!("Final Time    : {:.3} s", final_state.time);
    println!("Final Speed   : {:.2} m/s", final_state.car.speed);
    println!("Final Distance: {:.2} m", final_state.car.distance);
}
