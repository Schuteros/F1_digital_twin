use crate::physics::states::CarState;

mod integrators;
mod states;
mod powertrain;
mod forces;
mod tires;
mod aero;
mod simulation;

/// Holds all the values needed for the simulation
pub(crate) struct SimulationState {
    current_state: CarState,
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


impl Default for EnvironmentModel {
    fn default() -> Self {
        Self {
            air_density: 1.225,
            g_acceleration: 9.81,
        }
    }
}

impl Default for TyreModel {
    fn default() -> Self {
        Self {
            mu_static_friction: 0.9,
            mu_rolling_friction: 0.03,
            mu_breakaway_friction: 0.7,
            wheel_radius: 0.35,
        }
    }
}

impl Default for AeroModel {
    fn default() -> Self {
        Self {
            drag_coefficient: 0.35,
            frontal_area: 2.0,
        }
    }
}

impl Default for PowertrainModel {
    fn default() -> Self {
        Self {
            power: 800_000.0,
            max_torque: 800.0,
        }
    }
}

impl Default for CarState {
    fn default() -> Self {
        Self {
            speed: 0.0,
            distance: 0.0,
        }
    }
}



#[cfg(test)]
mod tests {

}