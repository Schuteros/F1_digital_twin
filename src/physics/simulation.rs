use crate::physics::{CarModel, EnvironmentModel, SimulationConfig};
use crate::physics::forces::{calculate_acceleration, calculate_net_force};
use crate::physics::integrators::euler;
use crate::physics::states::CarState;

/// Calculates vehicle acceleration given the current car model, state, and environment.
pub fn get_vehicle_acceleration(
    car_model: &CarModel,
    car_state: &CarState,
    environment_model: &EnvironmentModel,
) -> f64 {
    let force = calculate_net_force(
        car_model.mass_distribution.total_mass,
        car_model.mass_distribution.rear_axle_mass, // or driven axle mass
        car_state,
        &car_model.powertrain_model,
        &car_model.tyre_model,
        environment_model,
        &car_model.aero_model,
    );

    calculate_acceleration(force, car_model.mass_distribution.total_mass)
}


/// Calculates vehicle speed given the current car model, state, environment model and simulation_config.
fn get_vehicle_speed(car_model: &CarModel,
                     car_state: &CarState,
                     environment_model: &EnvironmentModel,
                     simulation_config: &SimulationConfig,
) -> f64 {

    let acceleration = get_vehicle_acceleration(car_model, car_state, environment_model);

    let speed = euler(car_state.speed, acceleration, simulation_config.time_step);

    speed
}

/// Calculates vehicle distance given the current car model, state, environment model and simulation_config.
fn get_vehicle_distance(car_state: &CarState,
                        simulation_config: &SimulationConfig,
                        speed: f64,
) -> f64 {

    let distance = euler(car_state.distance, speed, simulation_config.time_step);

    distance
}

/// Simulation step updates simulation runner
fn simulation_step() {

}


#[cfg(test)]
mod tests {
    use crate::physics::states::CarState;
    use crate::physics::{AeroModel, CarModel, EnvironmentModel, MassDistribution, PowertrainModel, SimulationConfig, SimulationRunner, TyreModel};
    use crate::physics::simulation::{get_vehicle_acceleration, get_vehicle_distance, get_vehicle_speed};


    #[test]
    fn test_get_vehicle_acceleration() {
        let total_mass: f64 = 8000.0 / 9.81;
        let driven_axle_mass: f64 = 4000.0 / 9.81;

        let tyre_model = TyreModel {
            mu_static_friction: 0.9,
            mu_rolling_friction: 0.03,
            mu_breakaway_friction: 0.7,
            wheel_radius: 0.35,
        };

        let car_state = CarState {
            speed: 10.0,
            distance: 100.0,
        };

        let environment_model = EnvironmentModel {
            air_density: 1.225,
            g_acceleration: 9.81,
        };

        let aero_model = AeroModel {
            drag_coefficient: 0.35,
            frontal_area: 2.0,
        };

        let powertrain_model = PowertrainModel {
            power: 800_000.0,
            max_torque: 800.0,
        };

        let simulation_config = SimulationConfig {
            initial_state: car_state,
            car_model: CarModel {
                mass_distribution: MassDistribution {
                    total_mass,
                    rear_axle_mass: driven_axle_mass,
                    front_axle_mass: total_mass - driven_axle_mass,
                },
                powertrain_model,
                tyre_model,
                aero_model,
            },
            environment_model,
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
        };


        let acceleration = get_vehicle_acceleration(&simulation_config.car_model, &simulation_config.initial_state, &simulation_config.environment_model);

        // from forces.rs and knowing total mass expected value:
        // 2002.839286 N / ( 8000.0 / 9.81 ) = 2.455981674
        assert!((acceleration - 2.455981674).abs() < 1e-3);
    }

    #[test]
    fn test_get_vehicle_speed() {
        let total_mass: f64 = 8000.0 / 9.81;
        let driven_axle_mass: f64 = 4000.0 / 9.81;

        let tyre_model = TyreModel {
            mu_static_friction: 0.9,
            mu_rolling_friction: 0.03,
            mu_breakaway_friction: 0.7,
            wheel_radius: 0.35,
        };

        let car_state = CarState {
            speed: 10.0,
            distance: 100.0,
        };

        let environment_model = EnvironmentModel {
            air_density: 1.225,
            g_acceleration: 9.81,
        };

        let aero_model = AeroModel {
            drag_coefficient: 0.35,
            frontal_area: 2.0,
        };

        let powertrain_model = PowertrainModel {
            power: 800_000.0,
            max_torque: 800.0,
        };

        let simulation_config = SimulationConfig {
            initial_state: car_state,
            car_model: CarModel {
                mass_distribution: MassDistribution {
                    total_mass,
                    rear_axle_mass: driven_axle_mass,
                    front_axle_mass: total_mass - driven_axle_mass,
                },
                powertrain_model,
                tyre_model,
                aero_model,
            },
            environment_model,
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
        };



        let simulation_runner = SimulationRunner {
            current_state: CarState::clone(&simulation_config.initial_state),
            simulation_config,
            current_time: 0.0,
        };

        let speed = get_vehicle_speed(&simulation_runner.simulation_config.car_model,
                                      &simulation_runner.current_state,
                                      &simulation_runner.simulation_config.environment_model,
                                      &simulation_runner.simulation_config,
        );

        // from previous test results we know:
        // expected value: speed = 10 + 2.455981674 * 0.001 =   10.0024559817
        assert!((speed - 10.0024559817).abs() < 1e-4);
    }

    #[test]
    fn test_get_vehicle_distance() {
        let total_mass: f64 = 8000.0 / 9.81;
        let driven_axle_mass: f64 = 4000.0 / 9.81;

        let tyre_model = TyreModel {
            mu_static_friction: 0.9,
            mu_rolling_friction: 0.03,
            mu_breakaway_friction: 0.7,
            wheel_radius: 0.35,
        };

        let car_state = CarState {
            speed: 10.0,
            distance: 100.0,
        };

        let environment_model = EnvironmentModel {
            air_density: 1.225,
            g_acceleration: 9.81,
        };

        let aero_model = AeroModel {
            drag_coefficient: 0.35,
            frontal_area: 2.0,
        };

        let powertrain_model = PowertrainModel {
            power: 800_000.0,
            max_torque: 800.0,
        };

        let simulation_config = SimulationConfig {
            initial_state: car_state,
            car_model: CarModel {
                mass_distribution: MassDistribution {
                    total_mass,
                    rear_axle_mass: driven_axle_mass,
                    front_axle_mass: total_mass - driven_axle_mass,
                },
                powertrain_model,
                tyre_model,
                aero_model,
            },
            environment_model,
            start_time: 0.0,
            end_time: 1.0,
            time_step: 0.001,
        };



        let simulation_runner = SimulationRunner {
            current_state: CarState::clone(&simulation_config.initial_state),
            simulation_config,
            current_time: 0.0,
        };

        let speed = 10.0024559817;

        let distance = get_vehicle_distance(
        &simulation_runner.current_state,
        &simulation_runner.simulation_config,
        speed);

        // From previous test results:
        // Expected value: distance = 100 + 10.0024559817 * 0.001 = 100.010002456
        assert!((distance - 100.010002456).abs() < 1e-4);
    }
}
