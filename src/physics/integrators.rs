//! This module is concerned with integrating velocity and displacement using simulation runner and initial simulation config every time step
//!
//! Underlying science behind this is explained in the docs/1_Acceleration_and_Velocity_Integration.MD


use super::SimulationRunner;


/// Function calculates velocity for the current time step by integrating acceleration
pub fn calculate_velocity(variables: &mut SimulationRunner) {
    variables.current_state.velocity = variables.current_state.velocity + variables.current_state.acceleration * variables.simulation_config.time_step;
}


/// Function calculates displacement for the current time step by integrating velocity
pub fn calculate_displacement(variables: &mut SimulationRunner) {
    variables.current_state.displacement = variables.current_state.displacement + variables.current_state.velocity * variables.simulation_config.time_step;
}