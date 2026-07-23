use crate::physics::forces::calculate_net_force;
use crate::physics::states::CarState;

mod integrators;
mod states;
mod powertrain;
mod forces;
mod tires;
mod aero;

/// Holds all the values needed for the simulation
pub(crate) struct SimulationRunner {
    simulation_config: SimulationConfig,
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


impl SimulationRunner {
    /// Orchestrates the single-step lifecycle of the simulation frame.
    /// Maps domain metrics to the underlying mathematical integrators.
    pub fn step_lifecycle(&mut self) {
        let dt = self.simulation_config.time_step;
        let car_model = &self.simulation_config.car_model;
        //let normal_force = calculate
        //let net_force = calculate_net_force()
    }
}


pub(crate) fn calculate_acceleration(force: f64, mass: f64) -> f64 {
    force / mass
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::states::CarState;

    #[test]
    fn test_simulation_step_lifecycle() {
        let tyre_model = TyreModel {
            mu_static_friction: 1.2,
            mu_rolling_friction: 0.02,
            mu_breakaway_friction: 0.8,
            wheel_radius: 0.36
        };

        let mass_distribution = MassDistribution {
            total_mass: 1200.0,
            rear_axle_mass: 600.0,
            front_axle_mass: 600.0,
        };

        let powertrain_model = PowertrainModel {
            power: 750_000.0,
            max_torque: 650.0,
        };

        let aero_model = AeroModel {
            drag_coefficient: 0.35,
            frontal_area: 1.6,
        };

        let car = CarModel {
            mass_distribution,
            powertrain_model,
            tyre_model,
            aero_model,
        };

        let config = SimulationConfig {
            initial_state: CarState {
                speed: 10.0,
                distance: 0.0,
            },
            environment_model: EnvironmentModel {
                air_density: 1.225,
                g_acceleration: 9.81,
            },
            car_model: car,
            start_time: 0.0,
            end_time: 10.0,
            start_displacement: 0.0,
            start_velocity: 10.0,
            start_acceleration: 5.0,
            time_step: 0.1,
        };

        let mut runner = SimulationRunner {
            current_state: config.initial_state.clone(),
            simulation_config: config,
            current_time: 0.0,
        };

        // Act: Run a single lifecycle tick
        runner.step_lifecycle();

        // Assert
        assert!((runner.current_state.speed - 10.5).abs() < 1e-6);
        assert!((runner.current_state.distance - 1.05).abs() < 1e-6);
        assert!((runner.current_time - 0.1).abs() < 1e-6);
    }
}