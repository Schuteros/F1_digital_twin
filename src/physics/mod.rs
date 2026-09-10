use crate::physics::states::CarState;

mod aero;
mod forces;
mod integrators;
mod powertrain;
pub(crate) mod simulation;
mod states;
mod tires;
pub mod track;
pub mod braking;

/// Holds all the values needed for the simulation
pub(crate) struct SimulationState {
    pub(crate) current_state: CarState,
    pub(crate) current_time: f64, // Seconds (s)
}

/// Holds all the initial values to start simulation
pub struct SimulationConfig {
    /// Initial state of the car at the start of the simulation
    pub initial_state: CarState,

    /// Car model containing all the specs and assumptions of the car
    pub car_model: CarModel,

    /// Contains the starting conditions of the environment
    pub environment_model: EnvironmentModel,

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

pub struct EnvironmentModel {
    pub air_density: f64,
    pub g_acceleration: f64,
}

/// Contains all the variables that define the car needed to be simulated
pub struct CarModel {
    /// Contains data about mass distribution
    pub mass_distribution: MassDistribution,

    /// Defines the powertrain used to simulate the car
    pub powertrain_model: PowertrainModel,

    /// Defines the tires used to simulate the car
    pub tyre_model: TyreModel,

    /// Defines the aerodynamic model of the car
    pub aero_model: AeroModel,
}

/// Contains all the info about the aerodynamics of the car
pub struct AeroModel {
    /// drag coefficient of the car
    pub drag_coefficient: f64,

    /// frontal area of the car
    pub frontal_area: f64,
}

/// Contains mass distribution data
pub struct MassDistribution {
    pub total_mass: f64,
    pub rear_axle_mass: f64,
    pub front_axle_mass: f64,
}

/// Contains all the variables that define the powertrain
pub struct PowertrainModel {
    /// Power from the powertrain that is delivered to the wheels in Watts, W
    pub power: f64,
    /// Maximum torque on the wheels delivered from the powertrain in Newton meters, Nm
    pub max_torque: f64,
    /// Max rotational speed of the engine in Hertz, Hz
    pub max_revs: f64,
    /// Min rotational speed of the engine in Hertz, Hz
    pub min_revs: f64,
    /// Gear ratios from largest to smallest
    pub gear_ratios: Vec<f64>,
    /// Final drive ratio
    pub final_drive: f64,
    /// Shift up margin in Hertz, Hz
    pub shift_up_margin: f64,
    /// Shift down margin in Hertz, Hz
    pub shift_down_margin: f64,
}

/// Model defines the tires used to simulate the car
pub struct TyreModel {
    /// mu static friction which limits traction of the wheel
    pub mu_static_friction: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    pub mu_rolling_friction: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    pub mu_breakaway_friction: f64,

    /// wheel radius
    pub wheel_radius: f64,
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
            wheel_radius: 0.36,
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
            max_revs: 133.33,
            min_revs: 13.33,
            gear_ratios: vec![3.0, 2.0, 1.0, 0.8, 0.5],
            final_drive: 1.5,
            shift_up_margin: 0.5,
            shift_down_margin: 0.5,
        }
    }
}

impl Default for CarState {
    fn default() -> Self {
        Self {
            speed: 10.0,
            distance: 100.0,
            current_gear: 1,
            current_revs: 0.5,
            active_braking_zone: 0,
        }
    }
}

impl Default for CarModel {
    fn default() -> Self {
        let total_mass = 815.494393476;
        let driven_axle_mass = 407.747196738;

        Self {
            mass_distribution: MassDistribution {
                total_mass,
                rear_axle_mass: driven_axle_mass,
                front_axle_mass: total_mass - driven_axle_mass,
            },
            powertrain_model: PowertrainModel::default(),
            tyre_model: TyreModel::default(),
            aero_model: AeroModel::default(),
        }
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            initial_state: CarState::default(),
            car_model: CarModel::default(),
            environment_model: EnvironmentModel::default(),
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
        }
    }
}

impl SimulationState {
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            current_state: config.initial_state.clone(),
            current_time: config.start_time,
        }
    }
}

#[cfg(test)]
mod tests {}
