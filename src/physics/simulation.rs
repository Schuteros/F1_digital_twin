use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::powertrain::{calculate_transmission_input_revs, select_best_gear};
use crate::physics::states::CarState;
pub(crate) use crate::physics::{CarModel, EnvironmentModel, SimulationConfig, SimulationState};

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
fn get_vehicle_speed(
    car_model: &CarModel,
    car_state: &CarState,
    environment_model: &EnvironmentModel,
    simulation_config: &SimulationConfig,
) -> f64 {
    let acceleration = get_vehicle_acceleration(car_model, car_state, environment_model);

    let speed = euler(car_state.speed, acceleration, simulation_config.time_step);

    speed
}

/// Calculates vehicle distance given the current car model, state, environment model and simulation_config.
fn get_vehicle_distance(
    car_state: &CarState,
    simulation_config: &SimulationConfig,
    speed: f64,
) -> f64 {
    let distance = euler(car_state.distance, speed, simulation_config.time_step);

    distance
}

/// Simulation step updates simulation runner
fn simulation_step(simulation_state: &mut SimulationState, simulation_config: &SimulationConfig) {
    simulation_state.current_state.current_revs = calculate_transmission_input_revs(
        &simulation_config.car_model.powertrain_model,
        simulation_config.car_model.tyre_model.wheel_radius,
        &simulation_state.current_state,
    );
    simulation_state.current_state.current_gear = select_best_gear(
        &simulation_config.car_model.powertrain_model,
        simulation_state.current_state.current_revs,
        simulation_state.current_state.current_gear,
    );
    simulation_state.current_state.speed = get_vehicle_speed(
        &simulation_config.car_model,
        &simulation_state.current_state,
        &simulation_config.environment_model,
        &simulation_config,
    );

    simulation_state.current_state.distance = get_vehicle_distance(
        &simulation_state.current_state,
        &simulation_config,
        simulation_state.current_state.speed,
    );

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
            "Speed {:.2} m/s | Distance {:.2} m",
            simulation_state.current_state.speed, simulation_state.current_state.distance
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
            simulation_state.current_state.speed, simulation_state.current_state.distance
        );
    }

    simulation_state
}

#[cfg(test)]
mod tests {
    use crate::physics::simulation::{
        get_vehicle_acceleration, get_vehicle_distance, get_vehicle_speed, simulation_step,
        start_simulation,
    };
    use crate::physics::{SimulationConfig, SimulationState};

    #[test]
    fn test_get_vehicle_acceleration() {
        let mut simulation_config = SimulationConfig::default();

        simulation_config.initial_state.current_gear = 4;
        let acceleration = get_vehicle_acceleration(
            &simulation_config.car_model,
            &simulation_config.initial_state,
            &simulation_config.environment_model,
        );

        // from forces.rs and knowing total mass expected value:
        // 2383.7917 / ( 8000.0 / 9.81 ) = 2.92312457212
        assert!(
            (acceleration - 2.92312457212).abs() < 1e-3,
            "Expected value: 2.92312457212, calculated value: {}",
            acceleration
        );
    }

    #[test]
    fn test_get_vehicle_speed() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.current_state.current_gear = 4;

        let speed = get_vehicle_speed(
            &simulation_config.car_model,
            &simulation_state.current_state,
            &simulation_config.environment_model,
            &simulation_config,
        );

        // from previous test results we know:
        // expected value: speed = 10 + 2.92312457212 * 0.001 =    10.0029231246
        assert!(
            (speed - 10.0029231246).abs() < 1e-4,
            "Expected value: 10.0029231246 Calculated value: {}",
            speed
        );
    }

    #[test]
    fn test_get_vehicle_distance() {
        let simulation_config = SimulationConfig::default();

        let simulation_state = SimulationState::new(&simulation_config);

        let speed = 10.0029231246;

        let distance =
            get_vehicle_distance(&simulation_state.current_state, &simulation_config, speed);

        // From previous test results:
        // Expected value: distance = 100 + 10.0029231246 * 0.001 = 100.010002923
        assert!((distance - 100.010002923).abs() < 1e-4);
    }

    #[test]
    fn test_simulation_step() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.current_state.current_gear = 4;

        simulation_step(&mut simulation_state, &simulation_config);

        // Expected value from the calculated results with correct gear: speed = 10.00374062453125
        assert!(
            (simulation_state.current_state.speed - 10.00374062453125).abs() < 1e-4,
            "Speed didn't update correctly, expected: calculated: {}",
            simulation_state.current_state.speed
        );

        // Expected value from previous results: distance = 100 + 10.00374062453125 * 0.001 =
        assert!(
            (simulation_state.current_state.distance - 100.010003741).abs() < 1e-4,
            "Distance didn't update correctly"
        );
    }

    #[test]
    fn test_simulation_convergence() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.end_time = 5.0;
        simulation_config.time_step = 0.1; // Start at 0.1s (50 steps) instead of 1.0s (5 steps)

        let mut dist_a = start_simulation(&simulation_config, false, false)
            .current_state
            .distance;
        let mut prev_diff = f64::MAX;

        for _ in 0..5 {
            simulation_config.time_step /= 5.0;
            let dist_b = start_simulation(&simulation_config, false, false)
                .current_state
                .distance;

            let current_diff = (dist_a - dist_b).abs();
            println!(
                "dt: {:.5} | Current Diff: {:.6} | Prev Diff: {:.6}",
                simulation_config.time_step, current_diff, prev_diff
            );

            assert!(
                current_diff < prev_diff,
                "Simulation failed to converge! Current diff ({}) >= Prev diff ({})",
                current_diff,
                prev_diff
            );

            prev_diff = current_diff;
            dist_a = dist_b;
        }
    }

    #[test]
    fn test_simulation_speed_increase() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.end_time = 1.0;

        let mut speed_slower = start_simulation(&simulation_config, false, false)
            .current_state
            .speed;
        let mut speed_faster: f64;

        for _ in 0..10 {
            simulation_config.time_step += 0.5;
            speed_faster = start_simulation(&simulation_config, false, false)
                .current_state
                .speed;
            assert!(speed_faster > speed_slower, "Speed didn't increase!");
            speed_slower = speed_faster;
        }
    }
}
