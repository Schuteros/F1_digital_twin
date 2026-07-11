use crate::physics::states::CarState;

pub mod integrators;
pub mod states;


/// Holds all the values needed for the simulation
pub struct SimulationRunner {
    simulation_config: SimulationConfig,
    current_state: CarState,
    current_time: f64, // Seconds (s)

}


/// Holds all the initial values to start simulation
pub struct SimulationConfig {

    /// Initial state of the car at the start of the simulation
    initial_state: CarState,

    /// Car model containing all the specs and assumptions of the car
    car_model: CarModel,

    /// Start time of the simulation in seconds, s
    /// **Range:**
    /// * min: f64::MIN
    /// * max: <end_time
    start_time: f64, // Seconds (s)

    /// End time of the simulation in seconds, s
    /// **Range:**
    /// * min: >start_time
    /// * max: f64::MAX
    end_time: f64, // Seconds (s)

    /// Start displacement of the simulation in meters, m
    start_displacement: f64,

    /// Start velocity of the simulation in meters per second, m/s
    start_velocity: f64,

    /// Start acceleration of the simulation in meters per second squared, m/s^2
    start_acceleration: f64,

    /// Time step of the simulation in seconds, s
    /// * Smaller value increases accuracy of the simulation, but increases computation time linearly
    /// * Larger value decreases computation time linearly, but at cost of reduced accuracy of the simulation
    time_step: f64,
}


/// Contains all the variables that define the car needed to be simulated
pub struct CarModel {

    /// Power from the powertrain that is delivered to the wheels in kilowatts, kW
    power: f64,

    /// Mass of the car in kilograms, kg
    mass: f64,

    /// Maximum torque on the wheels delivered from the powertrain in Newton meters, Nm
    max_torque: f64,

    /// Defines the tires used to simulate the car
    tyre_model: TyreModel,
}


/// Model defines the tires used to simulate the car
pub struct TyreModel {

    /// mu static friction which limits traction of the wheel
    mu_static_friction: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    mu_rolling_friction: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    mu_breakaway_friction: f64,
}