use crate::physics::forces::calculate_net_force;
use crate::physics::states::CarState;

mod integrators;
mod states;
mod powertrain;
mod forces;
mod tires;
mod aero;
mod simulation;

/// Holds all the values needed for the simulation
pub(crate) struct SimulationRunner {
    current_state: CarState,
    simulation_config: SimulationConfig,
    current_time: f64, // Seconds (s)

}


/// Holds all the initial values to start simulation
pub(crate) struct SimulationConfig {

    /// Initial state of the car at the start of the simulation
    initial_state: CarState,

    /// Car model containing all the specs and assumptions of the car
    car_model: CarModel,

    /// Contains the starting conditions of the environment
    environment_model: EnvironmentModel,

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

    /// Time step of the simulation in seconds, s
    /// * Smaller value increases accuracy of the simulation, but increases computation time linearly
    /// * Larger value decreases computation time linearly, but at cost of reduced accuracy of the simulation
    time_step: f64,
}

pub(crate) struct EnvironmentModel {
    air_density: f64,
    g_acceleration: f64,
}


/// Contains all the variables that define the car needed to be simulated
pub(crate) struct CarModel {
    /// Contains data about mass distribution
    mass_distribution: MassDistribution,

    /// Defines the powertrain used to simulate the car
    powertrain_model: PowertrainModel,

    /// Defines the tires used to simulate the car
    tyre_model: TyreModel,

    /// Defines the aerodynamic model of the car
    aero_model: AeroModel,
}


/// Contains all the info about the aerodynamics of the car
pub(crate) struct AeroModel {
    /// drag coefficient of the car
    drag_coefficient: f64,

    /// frontal area of the car
    frontal_area: f64,
}

/// Contains mass distribution data
pub(crate) struct MassDistribution {
    total_mass: f64,
    rear_axle_mass: f64,
    front_axle_mass: f64,
}


/// Contains all the variables that define the powertrain
pub(crate) struct PowertrainModel {
    /// Power from the powertrain that is delivered to the wheels in Watts, W
    power: f64,
    /// Maximum torque on the wheels delivered from the powertrain in Newton meters, Nm
    max_torque: f64,
}


/// Model defines the tires used to simulate the car
pub(crate) struct TyreModel {

    /// mu static friction which limits traction of the wheel
    mu_static_friction: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    mu_rolling_friction: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    mu_breakaway_friction: f64,

    /// wheel radius
    wheel_radius: f64,
}



#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::states::CarState;

}