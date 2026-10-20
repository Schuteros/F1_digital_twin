use crate::physics::car::Car;
use crate::physics::chassis::{
    calculate_body_accelerations_from_state, calculate_tyre_loads, sum_axle_loads,
    update_suspension_positions, update_suspension_velocities,
};
use crate::physics::environment::Environment;
use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::powertrain::{Drivetrain, calculate_transmission_input_revs, select_best_gear};
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
            return Err(format!(
                "geometry.wheelbase must be > 0 m, got {}",
                wheelbase
            ));
        }

        let cog_height = self.car.geometry.cog_height;
        if !cog_height.is_finite() || cog_height < 0.0 {
            return Err(format!(
                "geometry.cog_height must be >= 0 m, got {}",
                cog_height
            ));
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
            car: config.initial_car,
            time: config.start_time,
        }
    }
}

/// Calculates car acceleration in m/s^2 given the current car model, state, and environment.
///
/// Axle loads are the corner tyre loads from the suspension state at the start of the step
/// (docs/11 section 11.8, steps 1 to 4), so there is no circular dependency between
/// acceleration and load transfer.
pub fn get_car_acceleration(car: &Car, car_state: &CarState, environment: &Environment) -> f64 {
    let tyre_loads = calculate_tyre_loads(car, &car_state.suspension, environment.g_acceleration);
    let axle_loads = sum_axle_loads(&tyre_loads);
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
fn get_car_distance(car_state: &CarState, simulation_config: &SimulationConfig, speed: f64) -> f64 {
    euler(car_state.distance, speed, simulation_config.time_step)
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

    // docs/11 section 11.8, steps 1 to 4: tyre loads from the suspension state, then a_x
    simulation_state.car.acceleration = get_car_acceleration(
        &simulation_config.car,
        &simulation_state.car,
        &simulation_config.environment,
    );

    // Step 5: body accelerations from the same suspension state and this step's a_x
    let body_accelerations = calculate_body_accelerations_from_state(
        &simulation_config.car,
        &simulation_state.car.suspension,
        simulation_config.environment.g_acceleration,
        simulation_state.car.acceleration,
    );

    // Step 6: update velocities
    simulation_state.car.speed = get_car_speed(
        &simulation_state.car,
        simulation_config,
        simulation_state.car.acceleration,
    );

    if simulation_state.car.speed <= 0.0 && simulation_state.car.braking {
        // Car has stopped: it is no longer decelerating. From the next step a_x = 0
        // (net force is 0 at standstill), and the body settles back to static.
        simulation_state.car.speed = 0.0;
        simulation_state.car.acceleration = 0.0;
    }

    simulation_state.car.suspension = update_suspension_velocities(
        &simulation_state.car.suspension,
        &body_accelerations,
        simulation_config.time_step,
    );

    // Step 7: update positions with the new velocities (semi-implicit Euler)
    simulation_state.car.distance = get_car_distance(
        &simulation_state.car,
        simulation_config,
        simulation_state.car.speed,
    );

    simulation_state.car.suspension = update_suspension_positions(
        &simulation_state.car.suspension,
        simulation_config.time_step,
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
    use crate::physics::chassis::{SuspensionState, calculate_tyre_loads, sum_axle_loads};
    use crate::physics::powertrain::Drivetrain;
    use crate::physics::simulation::{
        SimulationConfig, SimulationState, calculate_total_steps, get_car_acceleration,
        get_car_distance, get_car_speed, simulation_step, start_simulation,
    };

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
    fn test_get_car_acceleration_uses_suspension_state() {
        let mut simulation_config = SimulationConfig::default();
        // 1st gear at 10 m/s: powertrain force = min(800_000 / 10, 800 * 3 * 1.5 / 0.36) = 10000 N,
        // so the rear (driven) axle grip is the limit
        simulation_config.initial_car.gear = 1;

        let acceleration_static = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
        );

        // Suspension at static -> static rear load 4000 N -> traction 0.9 * 4000 N = 3600 N
        // Net force = 3600 N - 282.875 N = 3317.125 N
        // Acceleration = 3317.125 N / 815.494393476 kg = 4.067623 m/s^2
        assert!(
            (acceleration_static - 4.067623).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_static
        );

        // The stored acceleration is an output only, it no longer moves load (no lag)
        simulation_config.initial_car.acceleration = 5.0;
        let acceleration_with_stored = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
        );
        assert_eq!(acceleration_with_stored, acceleration_static);

        // Nose up (squat) by 1 mrad: s_r = 0 - 1.7 m * (-0.001) = +1.7 mm per rear corner
        // Rear corner force = 130 000 N/m * 0.0017 m = 221 N -> rear load = 4000 N + 2 * 221 N = 4442 N
        // Traction = 0.9 * 4442 N = 3997.8 N, front + rear stays 8000 N so the losses stay 282.875 N
        // Net force = 3997.8 N - 282.875 N = 3714.925 N
        // Acceleration = 3714.925 N / 815.494393476 kg = 4.555423 m/s^2
        simulation_config.initial_car.suspension.pitch = -0.001;
        let acceleration_squat = get_car_acceleration(
            &simulation_config.car,
            &simulation_config.initial_car,
            &simulation_config.environment,
        );
        assert!(
            (acceleration_squat - 4.555423).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_squat
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

        let distance = get_car_distance(&simulation_state.car, &simulation_config, speed);

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
        let mut simulation_config = SimulationConfig {
            end_time: 5.0,
            // Start at 0.01 s (500 steps): 0.1 s and 0.02 s break the suspension stability limit
            // (omega_max = 114 rad/s with the bump stops, docs/11 section 11.8)
            time_step: 0.01,
            ..SimulationConfig::default()
        };

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
        let mut simulation_config = SimulationConfig {
            end_time: 1.0,
            ..SimulationConfig::default()
        };
        // This test checks the time stepping only, so load transfer is switched off (CoG on the
        // ground: no pitch moment, the suspension stays at static and the axle loads are static).
        simulation_config.car.geometry.cog_height = 0.0;

        let mut speed_slower = start_simulation(&simulation_config, false, false).car.speed;
        let mut speed_faster: f64;

        for _ in 0..10 {
            simulation_config.time_step += 0.5;
            speed_faster = start_simulation(&simulation_config, false, false).car.speed;
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
            simulation_step(&mut simulation_state, &simulation_config);
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
    fn test_car_launches_from_standstill() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.distance = 0.0;
        simulation_config.initial_car.gear = 1;
        simulation_config.end_time = 1.0;

        let end_state = start_simulation(&simulation_config, false, false);

        // Net force at rest = 3600 N - 280 N = 3320 N > 0 (see forces.rs), so the car must move off.
        // With the old mu_breakaway = 0.7 (5600 N > 3600 N) it stayed at 0 m/s forever.
        assert!(
            end_state.car.speed > 0.0,
            "Car didn't launch, speed: {}",
            end_state.car.speed
        );
        assert!(end_state.car.distance > 0.0);
    }

    #[test]
    fn test_car_never_moves_backwards() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.distance = 0.0;
        simulation_config.initial_car.gear = 1;
        // Launch, accelerate for 50 m, then brake until stopped
        simulation_config.track.braking_zones = vec![(50.0, 1000.0)];

        let mut simulation_state = SimulationState::new(&simulation_config);
        let mut last_distance = simulation_state.car.distance;
        let mut braked = false;

        for _ in 0..20_000 {
            simulation_step(&mut simulation_state, &simulation_config);

            assert!(
                simulation_state.car.speed >= 0.0,
                "Negative speed: {}",
                simulation_state.car.speed
            );
            assert!(
                simulation_state.car.distance >= last_distance,
                "Car moved backwards: {} -> {}",
                last_distance,
                simulation_state.car.distance
            );
            last_distance = simulation_state.car.distance;
            braked |= simulation_state.car.braking;

            if simulation_state.car.braking && simulation_state.car.speed == 0.0 {
                break;
            }
        }

        // The scenario must actually reach the braking zone and stop, otherwise it proves nothing
        assert!(braked, "Car never reached the braking zone");
        assert_eq!(simulation_state.car.speed, 0.0);
        assert_eq!(simulation_state.car.acceleration, 0.0);
    }

    #[test]
    fn test_validate_accepts_default() {
        assert_eq!(SimulationConfig::default().validate(), Ok(()));
    }

    #[test]
    fn test_validate_rejects_bad_configs() {
        // Each case breaks exactly one value of an otherwise valid config
        type BreakConfig = fn(&mut SimulationConfig);
        let cases: Vec<(&str, BreakConfig)> = vec![
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
    fn test_braking_dives_and_settles_after_stop() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(0.0, 1000.0)];
        let g_acceleration = simulation_config.environment.g_acceleration;

        let mut simulation_state = SimulationState::new(&simulation_config);

        // Brake for 0.3 s: about two pitch periods, mostly settled (zeta = 0.75)
        for _ in 0..300 {
            simulation_step(&mut simulation_state, &simulation_config);
        }
        let suspension = simulation_state.car.suspension;
        let loads = sum_axle_loads(&calculate_tyre_loads(
            &simulation_config.car,
            &suspension,
            g_acceleration,
        ));
        // Nose down and load moved to the front. The total stays m * g = 8000 N: the symmetric
        // car keeps heave at exactly 0, so F_f = -F_r (1e-6 N covers rounding)
        assert!(suspension.pitch > 0.0);
        assert!(loads.front > 4000.0 + 500.0, "{:?}", loads);
        assert!((loads.total() - 8000.0).abs() < 1e-6, "{:?}", loads);

        // Brake until stopped
        let mut steps = 0;
        while !(simulation_state.car.braking && simulation_state.car.speed == 0.0) {
            simulation_step(&mut simulation_state, &simulation_config);
            steps += 1;
            assert!(steps < 20_000, "Car never stopped");
        }
        assert!(simulation_state.car.suspension.pitch > 0.0);

        // Stopped: a_x = 0, the body keeps integrating and settles back to static.
        // 3 s: decay exp(-0.7455 * 38.77 * 3) = exp(-87), so only rounding is left
        for _ in 0..3_000 {
            simulation_step(&mut simulation_state, &simulation_config);
            assert_eq!(simulation_state.car.acceleration, 0.0);
            assert_eq!(simulation_state.car.speed, 0.0);
        }
        let suspension = simulation_state.car.suspension;
        assert!(suspension.pitch.abs() < 1e-12, "{:?}", suspension);
        assert!(suspension.pitch_rate.abs() < 1e-12, "{:?}", suspension);
        assert!(suspension.heave.abs() < 1e-12, "{:?}", suspension);
        let loads = sum_axle_loads(&calculate_tyre_loads(
            &simulation_config.car,
            &suspension,
            g_acceleration,
        ));
        assert!((loads.front - 4000.0).abs() < 1e-6);
        assert!((loads.rear - 4000.0).abs() < 1e-6);
    }

    #[test]
    fn test_acceleration_squats() {
        // Accelerating from 10 m/s in 1st gear: load moves to the rear, nose up
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.gear = 1;
        let mut simulation_state = SimulationState::new(&simulation_config);
        assert_eq!(simulation_state.car.suspension, SuspensionState::default());

        for _ in 0..300 {
            simulation_step(&mut simulation_state, &simulation_config);
        }

        let loads = sum_axle_loads(&calculate_tyre_loads(
            &simulation_config.car,
            &simulation_state.car.suspension,
            simulation_config.environment.g_acceleration,
        ));
        assert!(simulation_state.car.suspension.pitch < 0.0);
        assert!(loads.rear > loads.front, "{:?}", loads);
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
        let mut simulation_config = SimulationConfig {
            end_time: 5.0,
            time_step: 0.1 / 25.0,
            ..SimulationConfig::default()
        };

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
