pub mod physics;

use physics::simulation::{start_simulation, SimulationConfig};

fn main() {
    // 1. Load default config (or tweak specific values)
    let config = SimulationConfig::default();

    println!("Launching vehicle dynamics simulation...");

    // 2. Run the simulation end-to-end
    let final_state = start_simulation(&config, false, true);

    // 3. Inspect the final output
    println!("\nSimulation Complete!");
    println!("Final Time    : {:.3} s", final_state.current_time);
    println!("Final Speed   : {:.2} m/s", final_state.current_state.speed);
    println!("Final Distance: {:.2} m", final_state.current_state.distance);
}