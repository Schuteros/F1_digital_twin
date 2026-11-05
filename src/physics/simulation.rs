use crate::physics::aero::{AeroLoads, calculate_aero_loads};
use crate::physics::car::Car;
use crate::physics::car_state_log::CarStateLog;
use crate::physics::chassis::calculate_axle_distances;
use crate::physics::environment::Environment;
use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::loads::AxleLoads;
use crate::physics::powertrain::{
    Drivetrain, Powertrain, calculate_engine_revs_from_wheel_speed, count_down_shift_time,
    fill_powertrain_tables, select_gear,
};
use crate::physics::states::CarState;
use crate::physics::suspension::{
    CornerForces, CornerSuspension, StaticCarValues, calculate_axle_forces,
    calculate_body_accelerations, calculate_corner_forces_from_state, calculate_corner_tyre_loads,
    calculate_natural_frequencies, calculate_static_car_values, sum_axle_loads,
    update_suspension_positions, update_suspension_velocities,
};
use crate::physics::track::{Track, find_active_braking_zone, is_braking_zone};

/// Allowed difference between mass.front + mass.rear and mass.total in kilograms, kg
const MASS_SUM_TOLERANCE: f64 = 1e-6;

/// Semi-implicit Euler is only stable for omega_max * time_step below this (docs/11 section 11.8)
const STABILITY_LIMIT: f64 = 2.0;

/// Holds all the initial values to start simulation
#[derive(Debug, PartialEq)]
pub struct SimulationConfig {
    /// Initial state of the car at the start of the simulation
    pub initial_car_state: CarState,

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

    /// Values that stay the same for the whole run (axle distances, static corner loads, aero
    /// points). Zero in Default and in a loaded config, filled by `prepare`.
    pub(crate) static_values: StaticCarValues,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            initial_car_state: CarState::default(),
            car: Car::default(),
            environment: Environment::default(),
            track: Track::default(),
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
            static_values: StaticCarValues::default(),
        }
    }
}

/// Holds all the values needed for the simulation
#[derive(Debug, PartialEq)]
pub(crate) struct SimulationState {
    pub(crate) car: CarState,
    pub(crate) time: f64, // Seconds (s)
}

impl SimulationState {
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            car: config.initial_car_state,
            time: config.start_time,
        }
    }
}

/// Loads on the car from the state at the start of a step. They are calculated once per step
/// and used by both the longitudinal acceleration and the body accelerations.
#[derive(Debug, Clone, Copy)]
pub(crate) struct StepLoads {
    /// Clamped suspension force of each corner on top of its static load in Newtons, N
    pub corner_forces: CornerForces,
    /// Axle normal loads (sum of the corner tyre loads) in Newtons, N
    pub axle_loads: AxleLoads,
    /// Drag and downforce at the speed the step starts from in Newtons, N
    pub aero_loads: AeroLoads,
}

/// Function to start simulation and iterates trough simulation steps.
///
/// Fills the powertrain shift tables in `simulation_config` first (`SimulationConfig::prepare`).
/// With a `log`, the car state is written to it at the start and after logged steps.
pub fn start_simulation(
    simulation_config: &mut SimulationConfig,
    mut log: Option<&mut CarStateLog>,
    verbose: bool,
) -> SimulationState {
    // An invalid config is a setup error, not a runtime condition, so fail loudly.
    // Callers who want to handle it gracefully can call `prepare()` themselves first.
    if let Err(message) = simulation_config.prepare() {
        panic!("Invalid simulation config: {}", message);
    }
    let simulation_config = &*simulation_config;

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

    if let Some(log) = log.as_deref_mut() {
        log.record(step, &simulation_state, false);
    }

    while step < total_steps && !is_car_stopped(&simulation_state.car) {
        simulation_step(&mut simulation_state, simulation_config);
        step += 1;
        simulation_state.time =
            simulation_config.start_time + step as f64 * simulation_config.time_step;

        if let Some(log) = log.as_deref_mut() {
            let last = step == total_steps || is_car_stopped(&simulation_state.car);
            log.record(step, &simulation_state, last);
        }
    }

    if verbose {
        println!(
            "Final: Speed {:.2} m/s | Distance {:.2} m",
            simulation_state.car.speed, simulation_state.car.distance
        );
    }

    simulation_state
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

    let powertrain = &simulation_config.car.powertrain;

    // docs/12 section 12.10, steps 1 and 2: no new shift while one is in progress. A gear
    // change starts the shift timer, and the step that starts the shift is its first step
    // without drive, so 5 ms at 1 ms steps is exactly 5 steps without drive.
    if simulation_state.car.shift_time_left <= 0.0 {
        let gear = select_gear(
            powertrain,
            simulation_state.car.gear,
            simulation_state.car.speed,
        );

        if gear != simulation_state.car.gear {
            simulation_state.car.gear = gear;
            simulation_state.car.shift_time_left = powertrain.shift_time;
        }
    }

    // Step 3: locked engine speed for the current gear (docs/12, equation 12.2)
    simulation_state.car.revs_hz = calculate_engine_revs_from_wheel_speed(
        powertrain,
        simulation_config.car.tyre.wheel_radius,
        simulation_state.car.gear,
        simulation_state.car.speed,
    );

    // Steps 4 and 5 (docs/12 section 12.10) with docs/11 section 11.8, steps 1 to 4: corner
    // forces and tyre loads from the suspension state and aero loads from the speed at the start
    // of the step (docs/13 section 13.5), then a_x. The powertrain force is the full-throttle
    // force in the gear (12.10), 0 while shift_time_left > 0, capped at the tyre grip limit.
    let step_loads = calculate_step_loads(simulation_config, &simulation_state.car);
    simulation_state.car.acceleration =
        get_car_acceleration(simulation_config, &simulation_state.car, &step_loads);

    // The drive for this step is decided, so count the shift timer down (docs/12 section 12.8)
    if simulation_state.car.shift_time_left > 0.0 {
        simulation_state.car.shift_time_left = count_down_shift_time(
            simulation_state.car.shift_time_left,
            simulation_config.time_step,
        );
    }

    // Step 5: body accelerations from the same corner forces and aero loads and this step's a_x
    let car = &simulation_config.car;
    let static_values = &simulation_config.static_values;
    let body_accelerations = calculate_body_accelerations(
        car.mass.total,
        car.suspension.pitch_inertia,
        car.geometry.cog_height,
        &static_values.axle_distances,
        simulation_state.car.acceleration,
        &calculate_axle_forces(&step_loads.corner_forces),
        &step_loads.aero_loads,
        &static_values.aero_points,
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

/// Checks if the car has stopped and won't continue to drive
fn is_car_stopped(car_state: &CarState) -> bool {
    car_state.braking && car_state.speed == 0.0
}

/// Calculates the loads on the car from its state at the start of a step: corner forces and
/// tyre loads from the suspension state (docs/11 section 11.8, steps 1 to 3) and the aero
/// loads from the speed (docs/13 section 13.5). Needs `prepare` to have run.
pub(crate) fn calculate_step_loads(
    simulation_config: &SimulationConfig,
    car_state: &CarState,
) -> StepLoads {
    let static_values = &simulation_config.static_values;
    let corner_forces = calculate_corner_forces_from_state(
        &simulation_config.car.suspension,
        static_values,
        &car_state.suspension,
    );
    let tyre_loads =
        calculate_corner_tyre_loads(&static_values.static_corner_loads, &corner_forces);

    StepLoads {
        corner_forces,
        axle_loads: sum_axle_loads(&tyre_loads),
        aero_loads: calculate_aero_loads(
            &simulation_config.car.aero,
            simulation_config.environment.air_density,
            car_state.speed,
        ),
    }
}

/// Calculates car acceleration a_x in m/s^2 from the loads at the start of the step.
///
/// The axle loads come from the suspension state, not from a_x (docs/11 section 11.8, steps
/// 1 to 4), so there is no circular dependency between acceleration and load transfer.
pub(crate) fn get_car_acceleration(
    simulation_config: &SimulationConfig,
    car_state: &CarState,
    step_loads: &StepLoads,
) -> f64 {
    let car = &simulation_config.car;
    let force = calculate_net_force(
        car,
        car_state,
        &step_loads.axle_loads,
        &step_loads.aero_loads,
    );

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

impl SimulationConfig {
    /// Validates the config, then fills everything that follows from it:
    /// * `static_values`: axle distances, static corner loads and aero points,
    /// * the powertrain values that only depend on car data: `peak_torque`, `clutch_lock_speeds`
    ///   (docs/12, equation 12.8) and `upshift_speeds` (docs/12 section 12.6),
    /// * the calculated part of `initial_car_state` (`calculate_initial_car_state`).
    ///
    /// Runs before the simulation (docs/12 section 12.10, before the simulation). Running it
    /// again gives the same result.
    pub fn prepare(&mut self) -> Result<(), String> {
        self.validate()?;

        self.static_values =
            calculate_static_car_values(&self.car, self.environment.g_acceleration);
        fill_powertrain_tables(&mut self.car.powertrain, self.car.tyre.wheel_radius);

        // The acceleration needs the values above, so they are filled first
        self.initial_car_state = calculate_initial_car_state(self);

        Ok(())
    }

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

        validate_powertrain(&self.car.powertrain)?;

        // The shift tables and the wheel force divide by the wheel radius (docs/12, equation 12.3)
        let wheel_radius = self.car.tyre.wheel_radius;
        if !wheel_radius.is_finite() || wheel_radius <= 0.0 {
            return Err(format!(
                "tyre.wheel_radius must be > 0 m, got {}",
                wheel_radius
            ));
        }

        let top_gear = self.car.powertrain.gear_ratios.len();
        let initial_gear = self.initial_car_state.gear as usize;
        if !(1..=top_gear).contains(&initial_gear) {
            return Err(format!(
                "initial_car_state.gear must be in 1..={}, got {}",
                top_gear, initial_gear
            ));
        }

        let initial_car_state = &self.initial_car_state;
        if !initial_car_state.speed.is_finite() || initial_car_state.speed < 0.0 {
            return Err(format!(
                "initial_car_state.speed must be >= 0 m/s, got {}",
                initial_car_state.speed
            ));
        }

        if !initial_car_state.shift_time_left.is_finite() || initial_car_state.shift_time_left < 0.0
        {
            return Err(format!(
                "initial_car_state.shift_time_left must be >= 0 s, got {}",
                initial_car_state.shift_time_left
            ));
        }

        let initial_suspension = &initial_car_state.suspension;
        for (field, value) in [
            ("distance", initial_car_state.distance),
            ("suspension.heave", initial_suspension.heave),
            (
                "suspension.heave_velocity",
                initial_suspension.heave_velocity,
            ),
            ("suspension.pitch", initial_suspension.pitch),
            ("suspension.pitch_rate", initial_suspension.pitch_rate),
        ] {
            if !value.is_finite() {
                return Err(format!(
                    "initial_car_state.{} must be finite, got {}",
                    field, value
                ));
            }
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

        for (index, &(start, end)) in self.track.braking_zones.iter().enumerate() {
            if !start.is_finite() || !end.is_finite() {
                return Err(format!(
                    "track.braking_zones[{}] must be finite, got {} m to {} m",
                    index, start, end
                ));
            }

            if end < start {
                return Err(format!(
                    "track.braking_zones[{}] end must be >= start, got {} m < {} m",
                    index, end, start
                ));
            }
        }

        let mass = &self.car.mass;
        let mass_difference = (mass.front + mass.rear - mass.total).abs();
        if mass_difference.is_nan() || mass_difference > MASS_SUM_TOLERANCE {
            return Err(format!(
                "mass.front + mass.rear must equal mass.total, got {} + {} != {}",
                mass.front, mass.rear, mass.total
            ));
        }

        let aero = &self.car.aero;
        for (field, value) in [
            ("downforce_coefficient", aero.downforce_coefficient),
            (
                "downforce_centre_of_pressure_from_front_axle",
                aero.downforce_centre_of_pressure_from_front_axle,
            ),
            (
                "drag_centre_of_pressure_height",
                aero.drag_centre_of_pressure_height,
            ),
        ] {
            // Any finite value is allowed: C_down < 0 is lift, and the centre of pressure may
            // lie outside the wheelbase (docs/13 section 13.5)
            if !value.is_finite() {
                return Err(format!("aero.{} must be finite, got {}", field, value));
            }
        }

        let suspension = &self.car.suspension;
        for (name, corner) in [
            ("front_left", &suspension.front_left),
            ("front_right", &suspension.front_right),
            ("rear_left", &suspension.rear_left),
            ("rear_right", &suspension.rear_right),
        ] {
            validate_corner_suspension(name, corner)?;
        }

        if !suspension.pitch_inertia.is_finite() || suspension.pitch_inertia <= 0.0 {
            return Err(format!(
                "suspension.pitch_inertia must be > 0 kg·m^2, got {}",
                suspension.pitch_inertia
            ));
        }

        for (field, ride_height) in [
            (
                "static_ride_height_front",
                suspension.static_ride_height_front,
            ),
            (
                "static_ride_height_rear",
                suspension.static_ride_height_rear,
            ),
        ] {
            if !ride_height.is_finite() {
                return Err(format!(
                    "suspension.{} must be finite, got {}",
                    field, ride_height
                ));
            }
        }

        // Time step stability (docs/11 section 11.8) with the uncoupled frequencies (11.17),
        // using the stiffest rate of each corner (spring + end stop)
        let distances = calculate_axle_distances(mass, &self.car.geometry);
        let frequencies = calculate_natural_frequencies(
            mass.total,
            suspension.pitch_inertia,
            &distances,
            calculate_stiffest_corner_rate(&suspension.front_left)
                + calculate_stiffest_corner_rate(&suspension.front_right),
            calculate_stiffest_corner_rate(&suspension.rear_left)
                + calculate_stiffest_corner_rate(&suspension.rear_right),
        );
        let frequency_max = frequencies.heave.max(frequencies.pitch);
        let stability = frequency_max * self.time_step;
        if stability.is_nan() || stability >= STABILITY_LIMIT {
            return Err(format!(
                "time_step must satisfy omega_max * time_step < {}, got {} rad/s * {} s = {}",
                STABILITY_LIMIT, frequency_max, self.time_step, stability
            ));
        }

        Ok(())
    }
}

/// Calculates the parts of the starting car state that follow from the starting conditions
/// (speed, distance, gear, shift time left, suspension state) and the config, the same way a
/// simulation step does:
/// * `active_braking_zone` and `braking` from the distance and the track,
/// * `revs_hz`, the locked engine speed in the starting gear (docs/12, equation 12.2),
/// * `acceleration`, a_x in the starting gear before any gear change (docs/11 section 11.8).
///
/// Needs the powertrain tables filled. The first simulation step recalculates all of these,
/// so they don't change the results, they only make the starting state consistent.
fn calculate_initial_car_state(simulation_config: &SimulationConfig) -> CarState {
    let braking_zones = &simulation_config.track.braking_zones;
    let mut car_state = simulation_config.initial_car_state;

    car_state.active_braking_zone = find_active_braking_zone(car_state.distance, braking_zones, 0);
    car_state.braking = is_braking_zone(
        braking_zones,
        car_state.distance,
        car_state.active_braking_zone,
    );
    car_state.revs_hz = calculate_engine_revs_from_wheel_speed(
        &simulation_config.car.powertrain,
        simulation_config.car.tyre.wheel_radius,
        car_state.gear,
        car_state.speed,
    );
    let step_loads = calculate_step_loads(simulation_config, &car_state);
    car_state.acceleration = get_car_acceleration(simulation_config, &car_state, &step_loads);

    car_state
}

/// Checks one suspension corner. `name` is the corner's field name, used in the error message.
fn validate_corner_suspension(name: &str, corner: &CornerSuspension) -> Result<(), String> {
    if !corner.spring_rate.is_finite() || corner.spring_rate <= 0.0 {
        return Err(format!(
            "suspension.{}.spring_rate must be > 0 N/m, got {}",
            name, corner.spring_rate
        ));
    }

    if !corner.damping.is_finite() || corner.damping < 0.0 {
        return Err(format!(
            "suspension.{}.damping must be >= 0 N·s/m, got {}",
            name, corner.damping
        ));
    }

    for (field, gap) in [
        ("bump_gap", corner.bump_gap),
        ("droop_gap", corner.droop_gap),
    ] {
        if !gap.is_finite() || gap < 0.0 {
            return Err(format!(
                "suspension.{}.{} must be >= 0 m, got {}",
                name, field, gap
            ));
        }
    }

    for (field, rate) in [
        ("bump_rate", corner.bump_rate),
        ("droop_rate", corner.droop_rate),
    ] {
        if !rate.is_finite() || rate < 0.0 {
            return Err(format!(
                "suspension.{}.{} must be >= 0 N/m, got {}",
                name, field, rate
            ));
        }
    }

    Ok(())
}

/// Checks the torque curve, rev range, gearing and shift data (docs/12 sections 12.2 and 12.9)
fn validate_powertrain(powertrain: &Powertrain) -> Result<(), String> {
    let curve = &powertrain.torque_curve;
    if curve.len() < 2 {
        return Err(format!(
            "powertrain.torque_curve must have at least 2 points, got {}",
            curve.len()
        ));
    }

    for (index, &(revs_hz, torque)) in curve.iter().enumerate() {
        if !revs_hz.is_finite() {
            return Err(format!(
                "powertrain.torque_curve[{}] engine speed must be finite, got {}",
                index, revs_hz
            ));
        }

        if !torque.is_finite() || torque < 0.0 {
            return Err(format!(
                "powertrain.torque_curve[{}] torque must be >= 0 N·m, got {}",
                index, torque
            ));
        }
    }

    for (index, pair) in curve.windows(2).enumerate() {
        if pair[1].0 <= pair[0].0 {
            return Err(format!(
                "powertrain.torque_curve engine speeds must be strictly increasing, got {} Hz at [{}] after {} Hz",
                pair[1].0,
                index + 1,
                pair[0].0
            ));
        }
    }

    for (field, revs) in [
        ("min_revs", powertrain.min_revs),
        ("max_revs", powertrain.max_revs),
    ] {
        if !revs.is_finite() {
            return Err(format!("powertrain.{} must be finite, got {}", field, revs));
        }
    }

    if powertrain.min_revs >= powertrain.max_revs {
        return Err(format!(
            "powertrain.min_revs must be < powertrain.max_revs, got {} Hz >= {} Hz",
            powertrain.min_revs, powertrain.max_revs
        ));
    }

    let first_revs = curve[0].0;
    let last_revs = curve[curve.len() - 1].0;
    if first_revs > powertrain.min_revs || last_revs < powertrain.max_revs {
        return Err(format!(
            "powertrain.torque_curve must cover min_revs to max_revs ({} Hz to {} Hz), got {} Hz to {} Hz",
            powertrain.min_revs, powertrain.max_revs, first_revs, last_revs
        ));
    }

    let gear_ratios = &powertrain.gear_ratios;
    if gear_ratios.is_empty() {
        return Err("powertrain.gear_ratios must not be empty".to_string());
    }

    for (index, &gear_ratio) in gear_ratios.iter().enumerate() {
        if !gear_ratio.is_finite() || gear_ratio <= 0.0 {
            return Err(format!(
                "powertrain.gear_ratios[{}] must be > 0, got {}",
                index, gear_ratio
            ));
        }
    }

    for (index, pair) in gear_ratios.windows(2).enumerate() {
        if pair[1] >= pair[0] {
            return Err(format!(
                "powertrain.gear_ratios must be strictly decreasing, got {} at [{}] after {}",
                pair[1],
                index + 1,
                pair[0]
            ));
        }
    }

    if !powertrain.primary_ratio.is_finite() || powertrain.primary_ratio <= 0.0 {
        return Err(format!(
            "powertrain.primary_ratio must be > 0, got {}",
            powertrain.primary_ratio
        ));
    }

    if !powertrain.final_drive.is_finite() || powertrain.final_drive <= 0.0 {
        return Err(format!(
            "powertrain.final_drive must be > 0, got {}",
            powertrain.final_drive
        ));
    }

    if !powertrain.shift_time.is_finite() || powertrain.shift_time < 0.0 {
        return Err(format!(
            "powertrain.shift_time must be >= 0 s, got {}",
            powertrain.shift_time
        ));
    }

    if !powertrain.downshift_hysteresis.is_finite() || powertrain.downshift_hysteresis < 0.0 {
        return Err(format!(
            "powertrain.downshift_hysteresis must be >= 0 m/s, got {}",
            powertrain.downshift_hysteresis
        ));
    }

    Ok(())
}

/// Stiffest wheel rate a corner can have in Newtons per meter, N/m: spring + the stiffer end stop
fn calculate_stiffest_corner_rate(corner: &CornerSuspension) -> f64 {
    corner.spring_rate + corner.bump_rate.max(corner.droop_rate)
}

#[cfg(test)]
mod tests {
    use crate::physics::aero::calculate_aero_loads;
    use crate::physics::aero::calculate_air_drag;
    use crate::physics::powertrain::{
        Drivetrain, calculate_engine_revs_from_wheel_speed, calculate_engine_torque,
        calculate_peak_torque, calculate_powertrain_force, calculate_total_ratio,
    };
    use crate::physics::simulation::{
        SimulationConfig, SimulationState, calculate_step_loads, calculate_total_steps,
        get_car_acceleration, get_car_distance, get_car_speed, simulation_step, start_simulation,
    };
    use crate::physics::states::CarState;
    use crate::physics::suspension::{SuspensionState, calculate_tyre_loads, sum_axle_loads};
    use crate::physics::track::Track;
    use crate::physics::tyres::{calculate_current_tyre_friction, calculate_split_force_limit};

    /// Largest rounding error in Newtons, N, when a step's powertrain force is recovered from
    /// a_x * m + losses: dividing by the mass and multiplying back on forces up to ~2e4 N rounds
    /// at ~1e-11 N, so 1e-6 N is far above rounding and far below any real force
    const FORCE_ROUNDING: f64 = 1e-6;

    /// Returns the config with the powertrain shift tables filled (docs/12 section 12.10)
    fn prepared(mut simulation_config: SimulationConfig) -> SimulationConfig {
        simulation_config.prepare().unwrap();
        simulation_config
    }

    /// Default config from rest at 0 m, in 1st gear, with no braking zones
    fn standing_start_config() -> SimulationConfig {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car_state.speed = 0.0;
        simulation_config.initial_car_state.distance = 0.0;
        simulation_config.initial_car_state.gear = 1;
        simulation_config.track.braking_zones = vec![];
        prepared(simulation_config)
    }

    /// Powertrain force at the tyres in Newtons, N, that a step applied: the net force
    /// a_x * m plus the losses (tyre friction and air drag) at the state the step started from
    fn applied_powertrain_force(
        simulation_config: &SimulationConfig,
        before: &CarState,
        acceleration: f64,
    ) -> f64 {
        let car = &simulation_config.car;
        let environment = &simulation_config.environment;
        let axle_loads = sum_axle_loads(&calculate_tyre_loads(
            car,
            &before.suspension,
            environment.g_acceleration,
        ));
        let losses = calculate_current_tyre_friction(
            axle_loads.total(),
            car.tyre.mu_rolling,
            car.tyre.mu_breakaway,
            before.speed,
        ) + calculate_air_drag(
            environment.air_density,
            car.aero.drag_coefficient,
            car.aero.frontal_area,
            before.speed,
        );

        acceleration * car.mass.total + losses
    }

    /// Tyre grip limit of the driven axles in Newtons, N, at the state a step started from
    fn grip_limit(simulation_config: &SimulationConfig, before: &CarState) -> f64 {
        let car = &simulation_config.car;
        let axle_loads = sum_axle_loads(&calculate_tyre_loads(
            car,
            &before.suspension,
            simulation_config.environment.g_acceleration,
        ));

        calculate_split_force_limit(
            &axle_loads,
            car.tyre.mu_static,
            car.powertrain.drivetrain.front_share(),
        )
    }

    /// Gear the upshift table gives for `speed` in meters per second, m/s: 1 + the number of
    /// upshift speeds at or below it (docs/12 section 12.6)
    fn table_gear(simulation_config: &SimulationConfig, speed: f64) -> u8 {
        let upshifts_passed = simulation_config
            .car
            .powertrain
            .upshift_speeds
            .iter()
            .filter(|&&upshift_speed| speed >= upshift_speed)
            .count();
        1 + upshifts_passed as u8
    }

    /// a_x from the state, the same way a simulation step calculates it
    fn acceleration(simulation_config: &SimulationConfig, car_state: &CarState) -> f64 {
        let step_loads = calculate_step_loads(simulation_config, car_state);
        get_car_acceleration(simulation_config, car_state, &step_loads)
    }

    #[test]
    fn test_get_car_acceleration() {
        let mut simulation_config = prepared(SimulationConfig::default());

        simulation_config.initial_car_state.gear = 4;
        let acceleration = acceleration(&simulation_config, &simulation_config.initial_car_state);

        // from forces.rs: gear 4 at 10 m/s slips and gives 8687.5 N, the RWD tyres limit it to
        // 3600 N, net force = 3600 - 282.875 = 3317.125 N. Knowing total mass expected value:
        // 3317.125 / 815.494393476 = 4.06762453125 m/s^2 (1e-9: rounding of one division)
        assert!(
            (acceleration - 4.06762453125).abs() < 1e-9,
            "Expected value: 4.06762453125, calculated value: {}",
            acceleration
        );
    }

    #[test]
    fn test_get_car_acceleration_uses_suspension_state() {
        let mut simulation_config = prepared(SimulationConfig::default());
        // 1st gear at 10 m/s, below the 18.514 m/s lock speed: the clutch slips and the
        // powertrain force = 375 * 5.43 * 3.0 / 0.36 = 16968.75 N, so the rear (driven) axle
        // grip is the limit
        simulation_config.initial_car_state.gear = 1;

        let acceleration_static =
            acceleration(&simulation_config, &simulation_config.initial_car_state);

        // Suspension at static -> static rear load 4000 N -> traction 0.9 * 4000 N = 3600 N
        // Net force = 3600 N - 282.875 N = 3317.125 N
        // Acceleration = 3317.125 N / 815.494393476 kg = 4.067623 m/s^2
        assert!(
            (acceleration_static - 4.067623).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_static
        );

        // The stored acceleration is an output only, it no longer moves load (no lag)
        simulation_config.initial_car_state.acceleration = 5.0;
        let acceleration_with_stored =
            acceleration(&simulation_config, &simulation_config.initial_car_state);
        assert_eq!(acceleration_with_stored, acceleration_static);

        // Nose up (squat) by 1 mrad: s_r = 0 - 1.7 m * (-0.001) = +1.7 mm per rear corner
        // Rear corner force = 130 000 N/m * 0.0017 m = 221 N -> rear load = 4000 N + 2 * 221 N = 4442 N
        // Traction = 0.9 * 4442 N = 3997.8 N, front + rear stays 8000 N so the losses stay 282.875 N
        // Net force = 3997.8 N - 282.875 N = 3714.925 N
        // Acceleration = 3714.925 N / 815.494393476 kg = 4.555423 m/s^2
        simulation_config.initial_car_state.suspension.pitch = -0.001;
        let acceleration_squat =
            acceleration(&simulation_config, &simulation_config.initial_car_state);
        assert!(
            (acceleration_squat - 4.555423).abs() < 1e-3,
            "Calculated value: {}",
            acceleration_squat
        );
    }

    #[test]
    fn test_get_car_speed() {
        let simulation_config = prepared(SimulationConfig::default());

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.car.gear = 4;

        let acceleration = acceleration(&simulation_config, &simulation_state.car);
        let speed = get_car_speed(&simulation_state.car, &simulation_config, acceleration);

        // from previous test results we know:
        // expected value: speed = 10 + 4.06762453125 * 0.001 = 10.00406762453125
        assert!(
            (speed - 10.00406762453125).abs() < 1e-4,
            "Expected value: 10.00406762453125 Calculated value: {}",
            speed
        );
    }

    #[test]
    fn test_get_car_distance() {
        let simulation_config = SimulationConfig::default();

        let simulation_state = SimulationState::new(&simulation_config);

        let speed = 10.00406762453125;

        let distance = get_car_distance(&simulation_state.car, &simulation_config, speed);

        // From previous test results:
        // Expected value: distance = 100 + 10.00406762453125 * 0.001 = 100.01000406762
        assert!((distance - 100.01000406762).abs() < 1e-4);
    }

    #[test]
    fn test_simulation_step() {
        let simulation_config = prepared(SimulationConfig::default());

        let mut simulation_state = SimulationState::new(&simulation_config);
        simulation_state.car.gear = 4;

        simulation_step(&mut simulation_state, &simulation_config);

        // 4th gear at 10 m/s is too high: 10 < 46.38 - 0.5 m/s (docs/12, equation 12.14), so
        // the step shifts down to 3rd and starts the 5 ms shift timer (one step counted down)
        assert_eq!(simulation_state.car.gear, 3);
        // 0.005 - 0.001 s, 1e-15 s covers the rounding of one subtraction
        assert!((simulation_state.car.shift_time_left - 0.004).abs() < 1e-15);

        // No drive during the shift, only losses: a = -282.875 / 815.494393476 = -0.34687546875
        // Expected value: speed = 10 - 0.34687546875 * 0.001 = 9.99965312453125
        assert!(
            (simulation_state.car.speed - 9.99965312453125).abs() < 1e-4,
            "Speed didn't update correctly, expected: 9.99965312453125 calculated: {}",
            simulation_state.car.speed
        );

        // Expected value from previous results: distance = 100 + 9.99965312453125 * 0.001 = 100.00999965
        assert!(
            (simulation_state.car.distance - 100.00999965).abs() < 1e-4,
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

        let mut dist_a = start_simulation(&mut simulation_config, None, false)
            .car
            .distance;
        let mut prev_diff = f64::MAX;

        for _ in 0..5 {
            simulation_config.time_step /= 5.0;
            let dist_b = start_simulation(&mut simulation_config, None, false)
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
        // Same for the aero loads (docs/13): no downforce, drag acting at ground level
        simulation_config.car.aero.downforce_coefficient = 0.0;
        simulation_config.car.aero.drag_centre_of_pressure_height = 0.0;
        // Time steps up to 5.5 s need a suspension that passes the stability limit
        // (docs/11 section 11.8): 1 N/m springs and no end stops give
        // omega_pitch = sqrt(4 * 1 N/m * 1.7^2 m^2 / 1000 kg·m^2) = 0.108 rad/s -> 0.59 at 5.5 s < 2
        for corner in [
            &mut simulation_config.car.suspension.front_left,
            &mut simulation_config.car.suspension.front_right,
            &mut simulation_config.car.suspension.rear_left,
            &mut simulation_config.car.suspension.rear_right,
        ] {
            corner.spring_rate = 1.0;
            corner.bump_rate = 0.0;
            corner.droop_rate = 0.0;
        }

        let mut speed_slower = start_simulation(&mut simulation_config, None, false)
            .car
            .speed;
        let mut speed_faster: f64;

        for _ in 0..10 {
            simulation_config.time_step += 0.5;
            speed_faster = start_simulation(&mut simulation_config, None, false)
                .car
                .speed;
            assert!(speed_faster > speed_slower, "Speed didn't increase!");
            speed_slower = speed_faster;
        }
    }

    #[test]
    fn test_straight_line_braking() {
        let mut simulation_config = prepared(SimulationConfig::default());
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

        let end_state = start_simulation(&mut simulation_config, None, false);
        assert_eq!(end_state.car.speed, 0.0);
        // Stopped car is no longer decelerating
        assert_eq!(end_state.car.acceleration, 0.0);
    }

    #[test]
    fn test_car_launches_from_standstill() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car_state.speed = 0.0;
        simulation_config.initial_car_state.distance = 0.0;
        simulation_config.initial_car_state.gear = 1;
        simulation_config.end_time = 1.0;

        let end_state = start_simulation(&mut simulation_config, None, false);

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
        simulation_config.initial_car_state.speed = 0.0;
        simulation_config.initial_car_state.distance = 0.0;
        simulation_config.initial_car_state.gear = 1;
        // Launch, accelerate for 50 m, then brake until stopped
        simulation_config.track.braking_zones = vec![(50.0, 1000.0)];
        let simulation_config = prepared(simulation_config);

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
    fn test_full_run_upshifts_without_hunting() {
        // From rest with no braking zones the car should go 1 -> 8, one gear at a time, each
        // upshift once, at the table speed (docs/12 section 12.6), never over-revving
        // (docs/12 section 12.7), with exactly 5 steps without drive per shift (5 ms at 1 ms
        // steps, docs/12 section 12.8)
        let simulation_config = standing_start_config();
        let powertrain = &simulation_config.car.powertrain;
        let wheel_radius = simulation_config.car.tyre.wheel_radius;
        let top_gear = powertrain.gear_ratios.len() as u8;
        let mut simulation_state = SimulationState::new(&simulation_config);

        let mut upshifts: Vec<u8> = Vec::new();
        // Speed the previous step chose its gear at, m/s
        let mut last_checked_speed = simulation_state.car.speed;
        // Steps without drive since the last upshift, None before the first one
        let mut steps_without_drive: Option<usize> = None;
        let mut steps_in_top_gear = 0;

        // 1 s in top gear is enough to see the last shift through; 200 s is a safety stop
        for _ in 0..200_000 {
            let before = simulation_state.car;
            simulation_step(&mut simulation_state, &simulation_config);
            let after = simulation_state.car;

            assert!(!after.braking);
            assert!(
                after.gear >= before.gear,
                "Downshift {} -> {} at {} m/s",
                before.gear,
                after.gear,
                before.speed
            );

            let shifted = after.gear != before.gear;
            if shifted {
                assert_eq!(after.gear, before.gear + 1, "Skipped a gear");
                assert!(!upshifts.contains(&before.gear), "Repeated upshift");
                upshifts.push(before.gear);

                // The first step to see the speed at or above the table speed shifts, so the shift
                // is within one step of travel of it
                let upshift_speed = powertrain.upshift_speeds[before.gear as usize - 1];
                assert!(
                    last_checked_speed < upshift_speed && upshift_speed <= before.speed,
                    "Upshift {} -> {} at {} m/s, table speed {} m/s, previous step {} m/s",
                    before.gear,
                    after.gear,
                    before.speed,
                    upshift_speed,
                    last_checked_speed
                );
            }
            if before.shift_time_left <= 0.0 {
                last_checked_speed = before.speed;
            }

            // Drive this step: 0 during a shift, otherwise the powertrain force capped at the grip
            let applied_force =
                applied_powertrain_force(&simulation_config, &before, after.acceleration);
            let best_force =
                calculate_powertrain_force(powertrain, wheel_radius, after.gear, before.speed);
            let shifting = shifted || before.shift_time_left > 0.0;
            let expected_force = if shifting {
                0.0
            } else {
                best_force.min(grip_limit(&simulation_config, &before))
            };
            assert!(
                (applied_force - expected_force).abs() < FORCE_ROUNDING,
                "Gear {} at {} m/s: applied {} N, expected {} N",
                after.gear,
                before.speed,
                applied_force,
                expected_force
            );

            // Exactly 5 steps without drive after each upshift, the 6th has drive again
            if shifted {
                steps_without_drive = Some(0);
            }
            if let Some(count) = steps_without_drive {
                if shifting {
                    steps_without_drive = Some(count + 1);
                } else {
                    assert_eq!(
                        count, 5,
                        "Steps without drive after upshift to {}",
                        after.gear
                    );
                    assert!(applied_force > 0.0);
                    steps_without_drive = None;
                }
            }

            // Rev limit: the locked engine speed never passes max_revs below top gear. In top
            // gear the limiter cuts the force to 0 instead.
            if after.gear < top_gear {
                assert!(
                    after.revs_hz <= powertrain.max_revs,
                    "Over-rev in gear {}: {} Hz",
                    after.gear,
                    after.revs_hz
                );
            } else {
                if after.revs_hz > powertrain.max_revs {
                    assert_eq!(best_force, 0.0);
                }
                steps_in_top_gear += 1;
                if steps_in_top_gear > 1000 {
                    break;
                }
            }
        }

        assert_eq!(upshifts, vec![1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(simulation_state.car.gear, top_gear);
        assert_eq!(steps_without_drive, None, "Last shift never finished");
    }

    #[test]
    fn test_launch_clutch_slip_grip_limited() {
        // From rest in 1st, the clutch slips below the lock speed 18.514 m/s (docs/12 section
        // 12.5): the engine sits at peak torque, 375 * 16.29 / 0.36 = 16968.75 N, well above
        // the RWD grip (~3600 N), so the force at the tyres is the grip limit
        let simulation_config = standing_start_config();
        let powertrain = &simulation_config.car.powertrain;
        let wheel_radius = simulation_config.car.tyre.wheel_radius;
        let peak = calculate_peak_torque(powertrain);
        let clutch_lock_speed = powertrain.clutch_lock_speeds[0];
        // Rounded to 3 decimals (+-0.0005), so 0.001 covers it
        assert!((clutch_lock_speed - 18.514).abs() < 1e-3);

        let mut simulation_state = SimulationState::new(&simulation_config);
        let mut slipping_steps = 0;
        let mut locked_steps = 0;

        while simulation_state.car.speed < powertrain.upshift_speeds[0] {
            let before = simulation_state.car;
            simulation_step(&mut simulation_state, &simulation_config);
            let after = simulation_state.car;
            assert_eq!(after.gear, 1);

            let best_force = calculate_powertrain_force(powertrain, wheel_radius, 1, before.speed);
            if before.speed < clutch_lock_speed {
                // Slipping: the locked engine speed is below peak torque speed, the engine is
                // held at peak torque instead (1e-9 relative: rounding only)
                slipping_steps += 1;
                assert!(after.revs_hz < peak.revs_hz);
                assert!((best_force - 16968.75).abs() < 1e-9 * 16968.75);
            } else {
                // Locked: the engine follows the gearing (docs/12, equation 12.3)
                locked_steps += 1;
                let locked_force = calculate_engine_torque(powertrain, after.revs_hz)
                    * calculate_total_ratio(powertrain, 1)
                    / wheel_radius;
                assert_eq!(best_force, locked_force);
            }

            // Grip-limited in both phases with the default data
            let grip = grip_limit(&simulation_config, &before);
            assert!(
                best_force > grip,
                "Not grip-limited at {} m/s",
                before.speed
            );
            let applied_force =
                applied_powertrain_force(&simulation_config, &before, after.acceleration);
            assert!(
                (applied_force - grip).abs() < FORCE_ROUNDING,
                "At {} m/s: applied {} N, grip {} N",
                before.speed,
                applied_force,
                grip
            );
        }

        assert!(slipping_steps > 0 && locked_steps > 0);
    }

    #[test]
    fn test_downshift_while_braking() {
        // Brake from 80 m/s in top gear for 215 m (exits at about 42.0 m/s, between the 2 -> 3
        // and 3 -> 4 upshift speeds 38.37 and 46.38 m/s). The car steps down one gear at a time
        // inside the zone, at least one shift_time (5 steps) apart, and leaves it in the gear the
        // table gives, so there is no shift right after the zone (docs/12 section 12.7)
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car_state.speed = 80.0;
        simulation_config.initial_car_state.distance = 0.0;
        simulation_config.initial_car_state.gear = 8;
        simulation_config.track.braking_zones = vec![(0.0, 215.0)];
        let simulation_config = prepared(simulation_config);
        let mut simulation_state = SimulationState::new(&simulation_config);

        let mut shift_steps: Vec<usize> = Vec::new();
        let mut step = 0;
        loop {
            let before = simulation_state.car;
            simulation_step(&mut simulation_state, &simulation_config);
            if !simulation_state.car.braking {
                // This step left the zone, rewind it so the checks below see the exit state
                simulation_state.car = before;
                break;
            }

            let gear_after = simulation_state.car.gear;
            if gear_after != before.gear {
                assert_eq!(gear_after, before.gear - 1, "Not one gear at a time");
                shift_steps.push(step);
            }
            step += 1;
        }
        assert!(!shift_steps.is_empty(), "No downshift while braking");
        for pair in shift_steps.windows(2) {
            assert!(
                pair[1] - pair[0] >= 5,
                "Downshifts less than one shift_time apart"
            );
        }

        // The exit speed must be clear of the hysteresis bands, so the table gear is unambiguous
        let exit_speed = simulation_state.car.speed;
        for &upshift_speed in &simulation_config.car.powertrain.upshift_speeds {
            assert!(
                (exit_speed - upshift_speed).abs() > 1.0,
                "Exit speed {} m/s too close to {} m/s",
                exit_speed,
                upshift_speed
            );
        }
        let target_gear = table_gear(&simulation_config, exit_speed);
        assert!(
            target_gear < 7,
            "Exit speed {} m/s too high to test",
            exit_speed
        );
        assert_eq!(simulation_state.car.gear, target_gear);
        assert_eq!(shift_steps.len(), (8 - target_gear) as usize);

        // Already in the right gear, so no shift right after the zone
        for _ in 0..100 {
            simulation_step(&mut simulation_state, &simulation_config);
            assert!(!simulation_state.car.braking);
            assert_eq!(simulation_state.car.gear, target_gear);
        }
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
            ("spring_rate = 0", |c| {
                c.car.suspension.front_left.spring_rate = 0.0
            }),
            ("spring_rate < 0", |c| {
                c.car.suspension.rear_right.spring_rate = -1.0
            }),
            ("spring_rate NaN", |c| {
                c.car.suspension.front_right.spring_rate = f64::NAN
            }),
            ("damping < 0", |c| c.car.suspension.rear_left.damping = -1.0),
            ("downforce coefficient NaN", |c| {
                c.car.aero.downforce_coefficient = f64::NAN
            }),
            ("centre of pressure infinite", |c| {
                c.car.aero.downforce_centre_of_pressure_from_front_axle = f64::INFINITY
            }),
            ("drag height NaN", |c| {
                c.car.aero.drag_centre_of_pressure_height = f64::NAN
            }),
            ("damping infinite", |c| {
                c.car.suspension.rear_left.damping = f64::INFINITY
            }),
            ("bump_gap < 0", |c| {
                c.car.suspension.front_left.bump_gap = -0.001
            }),
            ("droop_gap < 0", |c| {
                c.car.suspension.rear_right.droop_gap = -0.001
            }),
            ("bump_gap NaN", |c| {
                c.car.suspension.front_right.bump_gap = f64::NAN
            }),
            ("bump_rate < 0", |c| {
                c.car.suspension.front_left.bump_rate = -1.0
            }),
            ("droop_rate < 0", |c| {
                c.car.suspension.rear_left.droop_rate = -1.0
            }),
            ("pitch_inertia = 0", |c| {
                c.car.suspension.pitch_inertia = 0.0
            }),
            ("pitch_inertia < 0", |c| {
                c.car.suspension.pitch_inertia = -1000.0
            }),
            ("ride height NaN", |c| {
                c.car.suspension.static_ride_height_front = f64::NAN
            }),
            ("ride height infinite", |c| {
                c.car.suspension.static_ride_height_rear = f64::INFINITY
            }),
            // omega_max = 114.29 rad/s (pitch, spring + bump stop) -> dt limit = 2 / 114.29 = 17.5 ms
            ("time_step unstable", |c| c.time_step = 0.018),
            ("time_step unstable, stiff droop", |c| {
                c.time_step = 0.005;
                c.car.suspension.rear_left.droop_rate = 1e8;
            }),
            // Torque curve (docs/12 section 12.2)
            ("torque_curve 1 point", |c| {
                c.car.powertrain.torque_curve = vec![(66.67, 300.0)]
            }),
            ("torque_curve empty", |c| {
                c.car.powertrain.torque_curve = vec![]
            }),
            ("torque_curve repeated engine speed", |c| {
                c.car.powertrain.torque_curve[2].0 = 100.0
            }),
            ("torque_curve decreasing engine speed", |c| {
                c.car.powertrain.torque_curve[2].0 = 90.0
            }),
            ("torque_curve engine speed NaN", |c| {
                c.car.powertrain.torque_curve[3].0 = f64::NAN
            }),
            ("torque_curve torque < 0", |c| {
                c.car.powertrain.torque_curve[1].1 = -1.0
            }),
            ("torque_curve torque NaN", |c| {
                c.car.powertrain.torque_curve[1].1 = f64::NAN
            }),
            ("torque_curve torque infinite", |c| {
                c.car.powertrain.torque_curve[4].1 = f64::INFINITY
            }),
            ("torque_curve starts above min_revs", |c| {
                c.car.powertrain.torque_curve[0].0 = 70.0
            }),
            ("torque_curve ends below max_revs", |c| {
                c.car.powertrain.torque_curve[5].0 = 210.0
            }),
            // Rev range
            ("min_revs = max_revs", |c| {
                c.car.powertrain.min_revs = 216.67
            }),
            ("min_revs > max_revs", |c| c.car.powertrain.min_revs = 300.0),
            ("min_revs NaN", |c| c.car.powertrain.min_revs = f64::NAN),
            ("max_revs infinite", |c| {
                c.car.powertrain.max_revs = f64::INFINITY
            }),
            // Gearing
            ("gear_ratios empty", |c| {
                c.car.powertrain.gear_ratios = vec![]
            }),
            ("gear ratio = 0", |c| c.car.powertrain.gear_ratios[7] = 0.0),
            ("gear ratio < 0", |c| {
                c.car.powertrain.gear_ratios[7] = -1.65
            }),
            ("gear ratio NaN", |c| {
                c.car.powertrain.gear_ratios[3] = f64::NAN
            }),
            ("gear ratios repeated", |c| {
                c.car.powertrain.gear_ratios[2] = 4.02
            }),
            ("gear ratios increasing", |c| {
                c.car.powertrain.gear_ratios[2] = 5.0
            }),
            ("primary_ratio = 0", |c| {
                c.car.powertrain.primary_ratio = 0.0
            }),
            ("primary_ratio NaN", |c| {
                c.car.powertrain.primary_ratio = f64::NAN
            }),
            ("final_drive = 0", |c| c.car.powertrain.final_drive = 0.0),
            ("final_drive < 0", |c| c.car.powertrain.final_drive = -3.0),
            ("final_drive infinite", |c| {
                c.car.powertrain.final_drive = f64::INFINITY
            }),
            // Shifting
            ("shift_time < 0", |c| c.car.powertrain.shift_time = -0.001),
            ("shift_time NaN", |c| c.car.powertrain.shift_time = f64::NAN),
            ("downshift_hysteresis < 0", |c| {
                c.car.powertrain.downshift_hysteresis = -0.5
            }),
            ("downshift_hysteresis infinite", |c| {
                c.car.powertrain.downshift_hysteresis = f64::INFINITY
            }),
            // Wheel radius and starting gear (the shift tables need both)
            ("wheel_radius = 0", |c| c.car.tyre.wheel_radius = 0.0),
            ("wheel_radius NaN", |c| c.car.tyre.wheel_radius = f64::NAN),
            ("initial gear 0", |c| c.initial_car_state.gear = 0),
            ("initial gear above top gear", |c| {
                c.initial_car_state.gear = 9
            }),
            // Other starting conditions
            ("initial speed < 0", |c| c.initial_car_state.speed = -1.0),
            ("initial speed NaN", |c| {
                c.initial_car_state.speed = f64::NAN
            }),
            ("initial distance infinite", |c| {
                c.initial_car_state.distance = f64::INFINITY
            }),
            ("initial shift_time_left < 0", |c| {
                c.initial_car_state.shift_time_left = -0.001
            }),
            ("initial heave NaN", |c| {
                c.initial_car_state.suspension.heave = f64::NAN
            }),
            ("initial heave_velocity infinite", |c| {
                c.initial_car_state.suspension.heave_velocity = f64::INFINITY
            }),
            ("initial pitch NaN", |c| {
                c.initial_car_state.suspension.pitch = f64::NAN
            }),
            ("initial pitch_rate infinite", |c| {
                c.initial_car_state.suspension.pitch_rate = f64::NEG_INFINITY
            }),
            // Braking zones
            ("braking zone end < start", |c| {
                c.track.braking_zones = vec![(900.0, 1000.0), (1200.0, 1100.0)]
            }),
            ("braking zone start NaN", |c| {
                c.track.braking_zones = vec![(f64::NAN, 1000.0)]
            }),
            ("braking zone end infinite", |c| {
                c.track.braking_zones = vec![(900.0, f64::INFINITY)]
            }),
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
        // Instant shifts and no hysteresis are valid
        simulation_config.car.powertrain.shift_time = 0.0;
        simulation_config.car.powertrain.downshift_hysteresis = 0.0;
        // A curve reaching past the rev range is valid, and a single gear is valid
        simulation_config.car.powertrain.torque_curve =
            vec![(50.0, 300.0), (133.33, 375.0), (250.0, 200.0)];
        simulation_config.car.powertrain.gear_ratios = vec![5.43];
        // Touching zones and a zero length zone are valid
        simulation_config.track.braking_zones =
            vec![(200.0, 300.0), (300.0, 400.0), (450.0, 450.0)];

        assert_eq!(simulation_config.validate(), Ok(()));

        // No braking zones is valid
        simulation_config.track.braking_zones = vec![];
        assert_eq!(simulation_config.validate(), Ok(()));
    }

    #[test]
    fn test_validate_names_field_and_value() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.car.suspension.rear_left.damping = -5.0;
        assert_eq!(
            simulation_config.validate(),
            Err("suspension.rear_left.damping must be >= 0 N·s/m, got -5".to_string())
        );

        let simulation_config = SimulationConfig {
            time_step: 0.02,
            ..SimulationConfig::default()
        };
        let message = simulation_config.validate().unwrap_err();
        assert!(message.starts_with("time_step must satisfy omega_max * time_step < 2"));
        assert!(message.contains("0.02"), "{}", message);

        let mut simulation_config = SimulationConfig::default();
        simulation_config.car.powertrain.shift_time = -0.001;
        assert_eq!(
            simulation_config.validate(),
            Err("powertrain.shift_time must be >= 0 s, got -0.001".to_string())
        );

        let mut simulation_config = SimulationConfig::default();
        simulation_config.car.powertrain.gear_ratios[2] = 5.0;
        assert_eq!(
            simulation_config.validate(),
            Err(
                "powertrain.gear_ratios must be strictly decreasing, got 5 at [2] after 4.02"
                    .to_string()
            )
        );

        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(900.0, 1000.0), (1200.0, 1100.0)];
        assert_eq!(
            simulation_config.validate(),
            Err("track.braking_zones[1] end must be >= start, got 1100 m < 1200 m".to_string())
        );
    }

    #[test]
    fn test_validate_accepts_stable_time_step() {
        // Just under the limit: 2 / 114.29 rad/s = 17.5 ms
        let simulation_config = SimulationConfig {
            time_step: 0.017,
            ..SimulationConfig::default()
        };
        assert_eq!(simulation_config.validate(), Ok(()));
    }

    #[test]
    fn test_prepare_fills_shift_tables() {
        let mut simulation_config = SimulationConfig::default();
        // Empty in Default
        assert!(
            simulation_config
                .car
                .powertrain
                .clutch_lock_speeds
                .is_empty()
        );
        assert!(simulation_config.car.powertrain.upshift_speeds.is_empty());

        simulation_config.prepare().unwrap();

        // One clutch lock speed per gear, one upshift speed per gear except the top gear
        assert_eq!(simulation_config.car.powertrain.clutch_lock_speeds.len(), 8);
        assert_eq!(simulation_config.car.powertrain.upshift_speeds.len(), 7);
    }

    #[test]
    fn test_prepare_fills_initial_car_state() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(100.0, 200.0), (300.0, 400.0)];
        simulation_config.initial_car_state.distance = 250.0;
        simulation_config.initial_car_state.speed = 20.0;
        simulation_config.initial_car_state.gear = 3;

        simulation_config.prepare().unwrap();
        let initial_car_state = simulation_config.initial_car_state;

        // Past the first zone, before the second: the second zone is next, not braking
        assert_eq!(initial_car_state.active_braking_zone, 1);
        assert!(!initial_car_state.braking);
        // Locked engine speed in 3rd gear (docs/12, equation 12.2), not the default 0.5 Hz
        assert_eq!(
            initial_car_state.revs_hz,
            calculate_engine_revs_from_wheel_speed(
                &simulation_config.car.powertrain,
                0.36,
                3,
                20.0
            )
        );
        // a_x from the starting state, the same calculation the first step makes
        assert_eq!(
            initial_car_state.acceleration,
            acceleration(&simulation_config, &initial_car_state)
        );
        assert!(initial_car_state.acceleration > 0.0);

        // Preparing again gives the same config
        let mut prepared_twice = prepared(SimulationConfig {
            track: Track {
                braking_zones: vec![(100.0, 200.0), (300.0, 400.0)],
            },
            initial_car_state: CarState {
                distance: 250.0,
                speed: 20.0,
                gear: 3,
                ..CarState::default()
            },
            ..SimulationConfig::default()
        });
        prepared_twice.prepare().unwrap();
        assert_eq!(prepared_twice, simulation_config);

        // Inside a zone: braking
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car_state.distance = 950.0;
        simulation_config.prepare().unwrap();
        assert_eq!(simulation_config.initial_car_state.active_braking_zone, 0);
        assert!(simulation_config.initial_car_state.braking);
        assert!(simulation_config.initial_car_state.acceleration < 0.0);
    }

    #[test]
    fn test_prepare_validates_first() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.car.powertrain.gear_ratios = vec![];

        assert!(simulation_config.prepare().is_err());
        // Nothing is computed from an invalid config
        assert!(
            simulation_config
                .car
                .powertrain
                .clutch_lock_speeds
                .is_empty()
        );
        assert!(simulation_config.car.powertrain.upshift_speeds.is_empty());
    }

    #[test]
    fn test_downforce_lowers_car_and_adds_tyre_load() {
        // docs/13 section 13.7, test 7: accelerate with the default (placeholder) aero until
        // 50 m/s. Downforce there: 0.5 * 1.225 * 1.0 * 2.0 * 50^2 = 3062.5 N
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![];
        let simulation_config = prepared(simulation_config);
        let g_acceleration = simulation_config.environment.g_acceleration;
        let weight = simulation_config.car.mass.total * g_acceleration;

        let mut simulation_state = SimulationState::new(&simulation_config);
        let mut steps = 0;
        while simulation_state.car.speed < 50.0 {
            simulation_step(&mut simulation_state, &simulation_config);
            steps += 1;
            assert!(steps < 60_000, "Car never reached 50 m/s");
        }

        let suspension = simulation_state.car.suspension;
        let loads = sum_axle_loads(&calculate_tyre_loads(
            &simulation_config.car,
            &suspension,
            g_acceleration,
        ));
        let downforce = calculate_aero_loads(
            &simulation_config.car.aero,
            simulation_config.environment.air_density,
            simulation_state.car.speed,
        )
        .downforce;

        // The springs carry the downforce: the body has moved down and the tyres carry
        // about weight + downforce (the body lags slightly behind the rising downforce)
        assert!(suspension.heave > 0.0, "{:?}", suspension);
        assert!(
            (loads.total() - (weight + downforce)).abs() < 0.05 * downforce,
            "{:?}, expected about {} N",
            loads,
            weight + downforce
        );
    }

    #[test]
    fn test_braking_dives_and_settles_after_stop() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.track.braking_zones = vec![(0.0, 1000.0)];
        // This test checks the braking dive only, so the aero loads are switched off
        // (docs/13): downforce would heave the body and the drag would pitch it
        simulation_config.car.aero.downforce_coefficient = 0.0;
        simulation_config.car.aero.drag_centre_of_pressure_height = 0.0;
        let simulation_config = prepared(simulation_config);
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
        // car keeps heave at exactly 0, so dF_f = -dF_r (1e-6 N covers rounding)
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
        simulation_config.initial_car_state.gear = 1;
        let simulation_config = prepared(simulation_config);
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

        start_simulation(&mut simulation_config, None, false);
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
