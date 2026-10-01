use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::powertrain::{calculate_transmission_input_revs, select_best_gear};
use crate::physics::states::CarState;
use crate::physics::track::{find_active_braking_zone, is_braking_zone};
pub(crate) use crate::physics::{Car, Environment, SimulationConfig, SimulationState};

/// Calculates car acceleration given the current car model, state, and environment.
pub fn get_car_acceleration(
    car: &Car,
    car_state: &CarState,
    environment: &Environment,
) -> f64 {
    let force = calculate_net_force(car, environment, car_state);

    calculate_acceleration(force, car.mass.total)
}

/// Calculates car speed given the current car model, state, environment model and simulation_config.
fn get_car_speed(
    car: &Car,
    car_state: &CarState,
    environment: &Environment,
    simulation_config: &SimulationConfig,
) -> f64 {
    let acceleration = get_car_acceleration(car, car_state, environment);

    let speed = euler(car_state.speed, acceleration, simulation_config.time_step);

    speed
}

/// Calculates car distance given the current car model, state, environment model and simulation_config.
fn get_car_distance(
    car_state: &CarState,
    simulation_config: &SimulationConfig,
    speed: f64,
) -> f64 {
    let distance = euler(car_state.distance, speed, simulation_config.time_step);

    distance
}

/// Checks if the car has stopped and won't continue to drive
fn is_car_stopped(car_state: &CarState) -> bool {
    car_state.braking && car_state.speed == 0.0
}

/// Calculates how many time steps are needed to reach end_time from start_time
fn calculate_total_steps(simulation_config: &SimulationConfig) -> u64 {
    let span = simulation_config.end_time - simulation_config.start_time;
    if span <= 0.0 || simulation_config.time_step <= 0.0 {
        return 0;
    }

    let steps = span / simulation_config.time_step;
    let rounded = steps.round();

    // Treat values within floating point noise of a whole number as that whole number
    if (steps - rounded).abs() < 1e-9 * rounded.max(1.0) {
        rounded as u64
    } else {
        steps.ceil() as u64
    }
}

/// Simulation step updates simulation runner
fn simulation_step(simulation_state: &mut SimulationState, simulation_config: &SimulationConfig) {
    simulation_state.car.active_braking_zone = find_active_braking_zone(
        simulation_state.car.distance,
        &simulation_config.track.braking_zones,
        simulation_state.car.active_braking_zone,
    );

    simulation_state.car.braking = is_braking_zone(
        &simulation_config.track.braking_zones,
        simulation_state.car.distance,
        simulation_state.car.active_braking_zone,
    );

    simulation_state.car.revs_hz = calculate_transmission_input_revs(
        &simulation_config.car.powertrain,
        simulation_config.car.tyre.wheel_radius,
        &simulation_state.car,
    );
    simulation_state.car.gear = select_best_gear(
        &simulation_config.car.powertrain,
        simulation_state.car.revs_hz,
        simulation_state.car.gear,
    );

    simulation_state.car.speed = get_car_speed(
        &simulation_config.car,
        &simulation_state.car,
        &simulation_config.environment,
        &simulation_config,
    );

    if simulation_state.car.speed <= 0.0 && simulation_state.car.braking {
        simulation_state.car.speed = 0.0;
    }

    simulation_state.car.distance = get_car_distance(
        &simulation_state.car,
        &simulation_config,
        simulation_state.car.speed,
    );

    simulation_state.time += simulation_config.time_step;
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
            simulation_state.car.speed, simulation_state.car.distance
        );
    }

    // Step count is computed up front: accumulating `time += time_step` drifts in
    // floating point and can run one extra step (e.g. 5.0 s / 0.004 s gives 1251 steps, not 1250).
    let total_steps = calculate_total_steps(simulation_config);
    let mut step: u64 = 0;

    while step < total_steps && !is_car_stopped(&simulation_state.car) {
        if telemetry {
            // Telemetry recording logic here
        }

        simulation_step(&mut simulation_state, simulation_config);
        step += 1;
        simulation_state.time =
            simulation_config.start_time + step as f64 * simulation_config.time_step;
    }

    if verbose {
        println!(
            "Final: Speed {:.2} m/s | Distance {:.2} m",
            simulation_state.car.speed, simulation_state.car.distance
        );
    }

    simulation_state
}

#[cfg(test)]
mod tests {
    use crate::physics::simulation::{
        calculate_total_steps, get_car_acceleration, get_car_distance, get_car_speed,
        simulation_step, start_simulation,
    };
    use crate::physics::{SimulationConfig, SimulationState};

    #[test]
    fn test_get_car_acceleration() {
        let mut simulation_config = SimulationConfig::default();

        simulation_config.initial_car.gear = 4;
        let acceleration = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
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
    fn test_get_car_speed() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.car.gear = 4;

        let speed = get_car_speed(
            &simulation_config.car,
            &simulation_state.car,
            &simulation_config.environment,
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
    fn test_get_car_distance() {
        let simulation_config = SimulationConfig::default();

        let simulation_state = SimulationState::new(&simulation_config);

        let speed = 10.0029231246;

        let distance =
            get_car_distance(&simulation_state.car, &simulation_config, speed);

        // From previous test results:
        // Expected value: distance = 100 + 10.0029231246 * 0.001 = 100.010002923
        assert!((distance - 100.010002923).abs() < 1e-4);
    }

    #[test]
    fn test_simulation_step() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.car.gear = 4;

        simulation_step(&mut simulation_state, &simulation_config);

        // Expected value from the calculated results with correct gear: speed = 10.00374062453125
        assert!(
            (simulation_state.car.speed - 10.00374062453125).abs() < 1e-4,
            "Speed didn't update correctly, expected: calculated: {}",
            simulation_state.car.speed
        );

        // Expected value from previous results: distance = 100 + 10.00374062453125 * 0.001 =
        assert!(
            (simulation_state.car.distance - 100.010003741).abs() < 1e-4,
            "Distance didn't update correctly"
        );
    }

    #[test]
    fn test_simulation_convergence() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.end_time = 5.0;
        simulation_config.time_step = 0.1; // Start at 0.1s (50 steps) instead of 1.0s (5 steps)

        let mut dist_a = start_simulation(&simulation_config, false, false)
            .car
            .distance;
        let mut prev_diff = f64::MAX;

        for _ in 0..5 {
            simulation_config.time_step /= 5.0;
            let dist_b = start_simulation(&simulation_config, false, false)
                .car
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
            .car
            .speed;
        let mut speed_faster: f64;

        for _ in 0..10 {
            simulation_config.time_step += 0.5;
            speed_faster = start_simulation(&simulation_config, false, false)
                .car
                .speed;
            assert!(speed_faster > speed_slower, "Speed didn't increase!");
            speed_slower = speed_faster;
        }
    }

    #[test]
    fn test_straight_line_braking() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(0.0, 500.0)];

        let mut simulation_state = SimulationState::new(&simulation_config);

        let mut last_speed = simulation_state.car.speed;

        for _ in 0..10 {
            simulation_step(& mut simulation_state, &simulation_config );
            assert!(last_speed > simulation_state.car.speed);
            last_speed = simulation_state.car.speed;
        }
    }

    #[test]
    fn test_straight_line_stopping() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(0.0, 1000.0)];
        simulation_config.end_time = 10.0;

        let end_state = start_simulation(&simulation_config, false, false);
        assert_eq!(end_state.car.speed, 0.0);
    }

    #[test]
    fn test_calculate_total_steps() {
        let mut simulation_config = SimulationConfig::default();

        simulation_config.end_time = 5.0;
        simulation_config.time_step = 0.1 / 25.0;
        // Accumulating 0.004 s in a loop gives 1251 steps, expected exactly 5.0 / 0.004 = 1250
        assert_eq!(calculate_total_steps(&simulation_config), 1250);

        simulation_config.end_time = 1.0;
        simulation_config.time_step = 1.5;
        // Partial last step is still executed: ceil(1.0 / 1.5) = 1
        assert_eq!(calculate_total_steps(&simulation_config), 1);

        simulation_config.time_step = 0.0;
        // Invalid time step must not loop forever
        assert_eq!(calculate_total_steps(&simulation_config), 0);
    }
}
