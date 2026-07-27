pub(crate) use crate::physics::{CarModel, EnvironmentModel, SimulationConfig, SimulationState};
use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::states::CarState;

/// Calculates vehicle acceleration given the current car model, state, and environment.
pub fn get_vehicle_acceleration(
    car_model: &CarModel,
    car_state: &CarState,
    environment_model: &EnvironmentModel,
) -> f64 {
    let force = calculate_net_force(
        car_model.mass_distribution.total_mass,
        car_model.mass_distribution.rear_axle_mass, // or driven axle mass
        car_state,
        &car_model.powertrain_model,
        &car_model.tyre_model,
        environment_model,
        &car_model.aero_model,
    );

    calculate_acceleration(force, car_model.mass_distribution.total_mass)
}


/// Calculates vehicle speed given the current car model, state, environment model and simulation_config.
fn get_vehicle_speed(car_model: &CarModel,
                     car_state: &CarState,
                     environment_model: &EnvironmentModel,
                     simulation_config: &SimulationConfig,
) -> f64 {

    let acceleration = get_vehicle_acceleration(car_model, car_state, environment_model);

    let speed = euler(car_state.speed, acceleration, simulation_config.time_step);

    speed
}

/// Calculates vehicle distance given the current car model, state, environment model and simulation_config.
fn get_vehicle_distance(car_state: &CarState,
                        simulation_config: &SimulationConfig,
                        speed: f64,
) -> f64 {

    let distance = euler(car_state.distance, speed, simulation_config.time_step);

    distance
}

/// Simulation step updates simulation runner
fn simulation_step(simulation_state: &mut SimulationState, simulation_config: &SimulationConfig) {
    simulation_state.current_state.speed = get_vehicle_speed(&simulation_config.car_model,
                                  &simulation_state.current_state,
                                  &simulation_config.environment_model,
                                  &simulation_config);

    simulation_state.current_state.distance = get_vehicle_distance(&simulation_state.current_state,
                                                                   &simulation_config,
                                                                   simulation_state.current_state.speed);

    simulation_state.current_time += simulation_config.time_step;
}

/// Function to start simulation and iterates trough simulation steps
pub fn start_simulation(
    simulation_config: &SimulationConfig,
    telemetry: bool,
    verbose: bool,
) -> SimulationState {
    let mut simulation_state = SimulationState::new(simulation_config);

    if verbose {
        println!(
            "Start: Speed {:.2} m/s | Distance {:.2} m",
            simulation_state.current_state.speed,
            simulation_state.current_state.distance
        );
    }

    while simulation_state.current_time < simulation_config.end_time {
        if telemetry {
            // Telemetry recording logic here
        }

        simulation_step(&mut simulation_state, simulation_config);
    }

    if verbose {
        println!(
            "Final: Speed {:.2} m/s | Distance {:.2} m",
            simulation_state.current_state.speed,
            simulation_state.current_state.distance
        );
    }

    simulation_state
}


#[cfg(test)]
mod tests {
    use crate::physics::{SimulationConfig, SimulationState};
    use crate::physics::simulation::{get_vehicle_acceleration, get_vehicle_distance, get_vehicle_speed, simulation_step};


    #[test]
    fn test_get_vehicle_acceleration() {
        let simulation_config = SimulationConfig::default();


        let acceleration = get_vehicle_acceleration(&simulation_config.car_model, &simulation_config.initial_state, &simulation_config.environment_model);

        // from forces.rs and knowing total mass expected value:
        // 2002.839286 N / ( 8000.0 / 9.81 ) = 2.455981674
        assert!((acceleration - 2.455981674).abs() < 1e-3);
    }

    #[test]
    fn test_get_vehicle_speed() {
        let simulation_config = SimulationConfig::default();

        let simulation_state = SimulationState::new(&simulation_config);

        let speed = get_vehicle_speed(&simulation_config.car_model,
                                      &simulation_state.current_state,
                                      &simulation_config.environment_model,
                                      &simulation_config,
        );

        // from previous test results we know:
        // expected value: speed = 10 + 2.455981674 * 0.001 =   10.0024559817
        assert!((speed - 10.0024559817).abs() < 1e-4);
    }

    #[test]
    fn test_get_vehicle_distance() {
        let simulation_config = SimulationConfig::default();

        let simulation_state = SimulationState::new(&simulation_config);

        let speed = 10.0024559817;

        let distance = get_vehicle_distance(
        &simulation_state.current_state,
        &simulation_config,
        speed);

        // From previous test results:
        // Expected value: distance = 100 + 10.0024559817 * 0.001 = 100.010002456
        assert!((distance - 100.010002456).abs() < 1e-4);
    }

    #[test]
    fn test_simulation_step() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);

        simulation_step(&mut simulation_state, &simulation_config);


        // Expected value from previous results: speed = 10.0024559817
        assert!((simulation_state.current_state.speed - 10.0024559817).abs() < 1e-4, "Speed didn't update correctly");

        // Expected value from previous results: distance = 100.010002456
        assert!((simulation_state.current_state.distance - 100.010002456).abs() < 1e-4, "Distance didn't update correctly");
    }
}
