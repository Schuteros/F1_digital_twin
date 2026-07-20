use crate::physics::states::CarState;

mod integrators;
mod states;
mod powertrain;
mod forces;
mod tires;

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
pub(crate) struct CarModel {

    /// Power from the powertrain that is delivered to the wheels in Watts, W
    power: f64,

    /// Mass of the car in kilograms, kg
    mass: f64,

    /// Maximum torque on the wheels delivered from the powertrain in Newton meters, Nm
    max_torque: f64,

    /// Defines the tires used to simulate the car
    tyre_model: TyreModel,
}


/// Model defines the tires used to simulate the car
pub(crate) struct TyreModel {

    /// mu static friction which limits traction of the wheel
    mu_static_friction: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    mu_rolling_friction: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    mu_breakaway_friction: f64,
}


impl SimulationRunner {
    /// Orchestrates the single-step lifecycle of the simulation frame.
    /// Maps domain metrics to the underlying mathematical integrators.
    pub fn step_lifecycle(&mut self) {
        let dt = self.simulation_config.time_step;

        // 1. Calculate new velocity from current acceleration
        // Maps domain (velocity/acceleration) to generic math (integrators::euler)
        self.current_state.velocity = integrators::euler(
            self.current_state.velocity,
            self.current_state.acceleration,
            dt,
        );

        // 2. Calculate new displacement from the updated velocity
        self.current_state.displacement = integrators::euler(
            self.current_state.displacement,
            self.current_state.velocity,
            dt,
        );

        // 3. Advance global simulation clock
        self.current_time += dt;
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::states::CarState;

    #[test]
    fn test_simulation_step_lifecycle() {
        let tyres = TyreModel {
            mu_static_friction: 1.2,
            mu_rolling_friction: 0.02,
            mu_breakaway_friction: 0.8,
        };

        let car = CarModel {
            power: 745.0,
            mass: 798.0,
            max_torque: 950.0,
            tyre_model: tyres,
        };

        let config = SimulationConfig {
            initial_state: CarState {
                velocity: 10.0,
                acceleration: 5.0,
                displacement: 0.0,
                force: 100.0,
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
        assert!((runner.current_state.velocity - 10.5).abs() < 1e-6);
        assert!((runner.current_state.displacement - 1.05).abs() < 1e-6);
        assert!((runner.current_time - 0.1).abs() < 1e-6);
    }
}