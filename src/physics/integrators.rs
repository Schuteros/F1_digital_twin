//! This module is concerned with integrating velocity and displacement using simulation runner and initial simulation config every time step
//!
//! Underlying science behind this is explained in the docs/1_Acceleration_and_Velocity_Integration.MD


use super::SimulationRunner;


/// Function calculates velocity for the current time step by integrating acceleration
pub fn calculate_velocity(variables: &mut SimulationRunner) {
    let current_state = &mut variables.current_state;
    let dt = variables.simulation_config.time_step;

    current_state.velocity += current_state.acceleration * dt;
}


/// Function calculates displacement for the current time step by integrating velocity
pub fn calculate_displacement(variables: &mut SimulationRunner) {
    let current_state = &mut variables.current_state;
    let dt = variables.simulation_config.time_step;

    current_state.displacement += current_state.velocity * dt;
}


// inside physics/integrators.rs

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::states::CarState;
    use crate::physics::{SimulationConfig, CarModel, TyreModel};

    #[test]
    fn test_velocity_and_displacement_integration() {
        // 1. Define the base Tyres
        let tyres = TyreModel {
            mu_static_friction: 1.2,
            mu_rolling_friction: 0.02,
            mu_breakaway_friction: 0.8,
        };

        // 2. Define the Car Model (Pack the Tyres inside)
        let car = CarModel {
            power: 745.0, // kW
            mass: 798.0,  // kg
            max_torque: 950.0, // Nm
            tyre_model: tyres,
        };

        // 3. Define the Initial State of the car
        let initial_car_state = CarState {
            velocity: 10.0,     // m/s
            acceleration: 5.0,  // m/s^2
            displacement: 0.0,  // m
            force: 100.0, // N, random force number, not needed for the test
        };

        // 4. Define the Simulation Config (Pack the Car and Initial State inside)
        let config = SimulationConfig {
            initial_state: initial_car_state,
            car_model: car,
            start_time: 0.0,          // s
            end_time: 10.0,           // s
            start_displacement: 0.0,  // m
            start_velocity: 10.0,     // m/s
            start_acceleration: 5.0,  // m/s^2
            time_step: 0.1,           // dt = 0.1 seconds
        };

        // 5. Initialize the Active Runner
        // We clone or copy the initial state into the 'current_state' slot
        // so the runner has a live state to modify during the race distance.
        let mut runner = SimulationRunner {
            current_state: config.initial_state.clone(),
            simulation_config: config,
            current_time: 0.0, // s
        };

        // --- EXECUTE MATH TESTS ---

        // Test Velocity Integration
        // Math: 10.0 + (5.0 * 0.1) = 10.5
        calculate_velocity(&mut runner);
        let v_diff = (runner.current_state.velocity - 10.5).abs();
        assert!(v_diff < 1e-6, "Velocity calculation failed! Got: {}", runner.current_state.velocity);

        // Test Displacement Integration
        // Math: 0.0 + (10.5 * 0.1) = 1.05
        calculate_displacement(&mut runner);
        let d_diff = (runner.current_state.displacement - 1.05).abs();
        assert!(d_diff < 1e-6, "Displacement calculation failed! Got: {}", runner.current_state.displacement);
    }
}