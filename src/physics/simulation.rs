use crate::physics::car::Car;
use crate::physics::environment::Environment;
use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::loads::calculate_axle_loads;
use crate::physics::powertrain::{
    Drivetrain, calculate_transmission_input_revs, select_best_gear,
};
use crate::physics::states::CarState;
use crate::physics::track::{Track, find_active_braking_zone, is_braking_zone};

/// Holds all the values needed for the simulation
pub(crate) struct SimulationState {
    pub(crate) car: CarState,
    pub(crate) time: f64, // Seconds (s)
}

/// Holds all the initial values to start simulation
pub struct SimulationConfig {
    /// Initial state of the car at the start of the simulation
    pub initial_car: CarState,

    /// Car model containing all the specs and assumptions of the car
    pub car: Car,

    /// Contains the starting conditions of the environment
    pub environment: Environment,

    /// Contains the track configuration
    pub track: Track,

    /// Start time of the simulation in seconds, s
    /// **Range:**
    /// * min: f64::MIN
    /// * max: <end_time
    pub start_time: f64, // Seconds (s)

    /// End time of the simulation in seconds, s
    /// **Range:**
    /// * min: >start_time
    /// * max: f64::MAX
    pub end_time: f64, // Seconds (s)

    /// Time step of the simulation in seconds, s
    /// * Smaller value increases accuracy of the simulation, but increases computation time linearly
    /// * Larger value decreases computation time linearly, but at cost of reduced accuracy of the simulation
    pub time_step: f64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            initial_car: CarState::default(),
            car: Car::default(),
            environment: Environment::default(),
            track: Track::default(),
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
        }
    }
}

/// Allowed difference between mass.front + mass.rear and mass.total in kilograms, kg
const MASS_SUM_TOLERANCE: f64 = 1e-6;

impl SimulationConfig {
    /// Checks the config for values the physics model can't handle.
    /// Returns a message describing the first invalid value found.
    pub fn validate(&self) -> Result<(), String> {
        let front_bias = self.car.brakes.front_bias;
        if !(0.0..=1.0).contains(&front_bias) {
            return Err(format!(
                "brakes.front_bias must be in 0.0..=1.0, got {}",
                front_bias
            ));
        }

        if let Drivetrain::AllWheelDrive { front_torque_split } = self.car.powertrain.drivetrain
            && !(0.0..=1.0).contains(&front_torque_split)
        {
            return Err(format!(
                "AllWheelDrive front_torque_split must be in 0.0..=1.0, got {}",
                front_torque_split
            ));
        }

        let wheelbase = self.car.geometry.wheelbase;
        if !wheelbase.is_finite() || wheelbase <= 0.0 {
            return Err(format!("geometry.wheelbase must be > 0 m, got {}", wheelbase));
        }

        let cog_height = self.car.geometry.cog_height;
        if !cog_height.is_finite() || cog_height < 0.0 {
            return Err(format!("geometry.cog_height must be >= 0 m, got {}", cog_height));
        }

        if !self.time_step.is_finite() || self.time_step <= 0.0 {
            return Err(format!("time_step must be > 0 s, got {}", self.time_step));
        }

        let mass = &self.car.mass;
        let mass_difference = (mass.front + mass.rear - mass.total).abs();
        if mass_difference.is_nan() || mass_difference > MASS_SUM_TOLERANCE {
            return Err(format!(
                "mass.front + mass.rear must equal mass.total, got {} + {} != {}",
                mass.front, mass.rear, mass.total
            ));
        }

        Ok(())
    }
}

impl SimulationState {
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            car: config.initial_car.clone(),
            time: config.start_time,
        }
    }
}

/// Calculates car acceleration in m/s^2 given the current car model, state, and environment.
///
/// Axle loads are built from `car_state.acceleration`, the acceleration of the previous step
/// (lagged load transfer, docs/10 section 10.6), which breaks the circular dependency between
/// acceleration and load transfer.
pub fn get_car_acceleration(
    car: &Car,
    car_state: &CarState,
    environment: &Environment,
) -> f64 {
    let axle_loads = calculate_axle_loads(
        &car.mass,
        &car.geometry,
        environment.g_acceleration,
        car_state.acceleration,
    );
    let force = calculate_net_force(car, environment, car_state, &axle_loads);

    calculate_acceleration(force, car.mass.total)
}

/// Calculates car speed given the current car state, simulation_config and acceleration of this step.
fn get_car_speed(
    car_state: &CarState,
    simulation_config: &SimulationConfig,
    acceleration: f64,
) -> f64 {
    euler(car_state.speed, acceleration, simulation_config.time_step)
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

    // Acceleration is computed once per step from the previous step's acceleration (load
    // transfer), then stored so the next step can use it.
    simulation_state.car.acceleration = get_car_acceleration(
        &simulation_config.car,
        &simulation_state.car,
        &simulation_config.environment,
    );

    simulation_state.car.speed = get_car_speed(
        &simulation_state.car,
        simulation_config,
        simulation_state.car.acceleration,
    );

    if simulation_state.car.speed <= 0.0 && simulation_state.car.braking {
        // Car has stopped: it is no longer decelerating, so no load transfer either
        simulation_state.car.speed = 0.0;
        simulation_state.car.acceleration = 0.0;
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
    // An invalid config is a setup error, not a runtime condition, so fail loudly.
    // Callers who want to handle it gracefully can call `validate()` themselves first.
    if let Err(message) = simulation_config.validate() {
        panic!("Invalid simulation config: {}", message);
    }

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
        SimulationConfig, SimulationState, calculate_total_steps, get_car_acceleration,
        get_car_distance, get_car_speed, simulation_step, start_simulation,
    };
    use crate::physics::powertrain::Drivetrain;

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
    fn test_get_car_acceleration_uses_stored_acceleration() {
        let mut simulation_config = SimulationConfig::default();
        // 1st gear at 10 m/s: powertrain force = min(800_000 / 10, 800 * 3 * 1.5 / 0.36) = 10000 N,
        // so the rear (driven) axle grip is the limit
        simulation_config.initial_car.gear = 1;

        let acceleration_without_transfer = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
        );

        // Previous acceleration 0 m/s^2 -> static rear load 4000 N -> traction 0.9 * 4000 N = 3600 N
        // Net force = 3600 N - 282.875 N = 3317.125 N
        // Acceleration = 3317.125 N / 815.494393476 kg = 4.067623 m/s^2
        assert!(
            (acceleration_without_transfer - 4.067623).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_without_transfer
        );

        simulation_config.initial_car.acceleration = 5.0;
        let acceleration_with_transfer = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
        );

        // Previous acceleration 5 m/s^2 -> load transfer = 815.494393476 kg * 5 m/s^2 * 0.3 m / 3.4 m = 359.7769 N
        // Rear load = 4000 N + 359.7769 N = 4359.7769 N -> traction = 0.9 * 4359.7769 N = 3923.7992 N
        // Net force = 3923.7992 N - 282.875 N = 3640.9242 N
        // Acceleration = 3640.9242 N / 815.494393476 kg = 4.464749 m/s^2
        assert!(
            (acceleration_with_transfer - 4.464749).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_with_transfer
        );
    }

    #[test]
    fn test_get_car_speed() {
        let simulation_config = SimulationConfig::default();

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.car.gear = 4;

        let acceleration = get_car_acceleration(
            &simulation_config.car,
            &simulation_state.car,
            &simulation_config.environment,
        );
        let speed = get_car_speed(&simulation_state.car, &simulation_config, acceleration);

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
        // This test checks the time stepping only, so load transfer is switched off (CoG on the
        // ground reduces the model to static axle loads). With load transfer, the first step of a
        // 0.5 s+ time step uses a lagged acceleration of 0 and misses the rear load gain the
        // 1 ms run gets, so the coarse run ends slower (the lag error from docs/10 section 10.6).
        simulation_config.car.geometry.cog_height = 0.0;

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
        // Stopped car is no longer decelerating
        assert_eq!(end_state.car.acceleration, 0.0);
    }

    #[test]
    fn test_validate_accepts_default() {
        assert_eq!(SimulationConfig::default().validate(), Ok(()));
    }

    #[test]
    fn test_validate_rejects_bad_configs() {
        // Each case breaks exactly one value of an otherwise valid config
        let cases: Vec<(&str, fn(&mut SimulationConfig))> = vec![
            ("front_bias < 0", |c| c.car.brakes.front_bias = -0.1),
            ("front_bias > 1", |c| c.car.brakes.front_bias = 1.1),
            ("front_bias NaN", |c| c.car.brakes.front_bias = f64::NAN),
            ("AWD split < 0", |c| {
                c.car.powertrain.drivetrain = Drivetrain::AllWheelDrive {
                    front_torque_split: -0.2,
                }
            }),
            ("AWD split > 1", |c| {
                c.car.powertrain.drivetrain = Drivetrain::AllWheelDrive {
                    front_torque_split: 1.5,
                }
            }),
            ("wheelbase = 0", |c| c.car.geometry.wheelbase = 0.0),
            ("wheelbase < 0", |c| c.car.geometry.wheelbase = -3.4),
            ("cog_height < 0", |c| c.car.geometry.cog_height = -0.1),
            ("time_step = 0", |c| c.time_step = 0.0),
            ("time_step < 0", |c| c.time_step = -0.001),
            ("front + rear != total", |c| c.car.mass.front += 10.0),
        ];

        for (name, break_config) in cases {
            let mut simulation_config = SimulationConfig::default();
            break_config(&mut simulation_config);

            assert!(
                simulation_config.validate().is_err(),
                "validate() accepted bad config: {}",
                name
            );
        }
    }

    #[test]
    fn test_validate_accepts_edge_values() {
        let mut simulation_config = SimulationConfig::default();
        // Bias and split edges are valid: all braking / drive on one axle
        simulation_config.car.brakes.front_bias = 1.0;
        simulation_config.car.powertrain.drivetrain = Drivetrain::AllWheelDrive {
            front_torque_split: 0.0,
        };
        // CoG on the ground is valid: no load transfer
        simulation_config.car.geometry.cog_height = 0.0;

        assert_eq!(simulation_config.validate(), Ok(()));
    }

    #[test]
    #[should_panic(expected = "Invalid simulation config")]
    fn test_start_simulation_panics_on_invalid_config() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.car.geometry.wheelbase = 0.0;

        start_simulation(&simulation_config, false, false);
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
