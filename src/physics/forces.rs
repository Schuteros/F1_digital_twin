use crate::physics::aero::calculate_air_drag;
use crate::physics::braking::calculate_brake_force;
use crate::physics::car::Car;
use crate::physics::environment::Environment;
use crate::physics::loads::AxleLoads;
use crate::physics::powertrain::{Powertrain, calculate_force_from_powertrain};
use crate::physics::states::CarState;
use crate::physics::tyres::{Tyre, calculate_current_tyre_friction, calculate_split_force_limit};

/// Calculates powertrain force in Newtons, N, limited by the grip of the driven axles.
/// The powertrain force is split between the axles by the drivetrain, and the first
/// driven axle to reach its traction limit caps the total (ideal traction control).
fn calculate_powertrain_force_tyre_traction_limited(
    axle_loads: &AxleLoads,
    car_state: &CarState,
    powertrain: &Powertrain,
    tyre: &Tyre,
) -> f64 {
    let powertrain_force =
        calculate_force_from_powertrain(powertrain, tyre.wheel_radius, car_state);
    let traction_force = calculate_split_force_limit(
        axle_loads,
        tyre.mu_static,
        powertrain.drivetrain.front_share(),
    );

    powertrain_force.min(traction_force)
}

/// Calculates resistive losses in Newtons, N: rolling (or breakaway) tyre friction and air drag.
/// Brake force is not included, it is added separately in the braking branch of the net force.
fn calculate_force_losses(
    car: &Car,
    environment: &Environment,
    car_state: &CarState,
    axle_loads: &AxleLoads,
) -> f64 {
    let friction_loss = calculate_current_tyre_friction(
        axle_loads.total(),
        car.tyre.mu_rolling,
        car.tyre.mu_breakaway,
        car_state.speed,
    );

    let air_drag_loss = calculate_air_drag(
        environment.air_density,
        car.aero.drag_coefficient,
        car.aero.frontal_area,
        car_state.speed,
    );

    friction_loss + air_drag_loss
}

pub fn calculate_normal_force(mass: f64, g_acceleration: f64) -> f64 {
    mass * g_acceleration
}

/// Calculates the net longitudinal force on the car in Newtons, N, positive forward.
///
/// `axle_loads` are the axle normal forces for this step; in the simulation they are built
/// from the previous step's acceleration (lagged load transfer, docs/10 section 10.6).
pub fn calculate_net_force(
    car: &Car,
    environment: &Environment,
    car_state: &CarState,
    axle_loads: &AxleLoads,
) -> f64 {
    let force_losses = calculate_force_losses(car, environment, car_state, axle_loads);

    if car_state.braking {
        if car_state.speed == 0.0 {
            0.0
        } else {
            let brake_force =
                calculate_brake_force(axle_loads, car.tyre.mu_static, car.brakes.front_bias);

            -(force_losses + brake_force)
        }
    } else {
        let powertrain_force = calculate_powertrain_force_tyre_traction_limited(
            axle_loads,
            car_state,
            &car.powertrain,
            &car.tyre,
        );

        let net_force = powertrain_force - force_losses;

        // At standstill, breakaway friction can only resist motion, never push the car backwards.
        // While moving, a negative net force is real deceleration (e.g. drag above terminal speed).
        if car_state.speed == 0.0 && net_force < 0.0 {
            0.0
        } else {
            net_force
        }
    }
}

pub(crate) fn calculate_acceleration(force: f64, mass: f64) -> f64 {
    force / mass
}

#[cfg(test)]
mod tests {
    use crate::physics::forces::{
        calculate_acceleration, calculate_force_losses, calculate_net_force,
        calculate_normal_force, calculate_powertrain_force_tyre_traction_limited,
    };
    use crate::physics::loads::AxleLoads;
    use crate::physics::powertrain::{Drivetrain, Powertrain};
    use crate::physics::simulation::SimulationConfig;
    use crate::physics::states::CarState;
    use crate::physics::tyres::Tyre;

    /// Default car without load transfer: 407.747196738 kg * 9.81 m/s^2 = 4000 N per axle
    fn default_static_axle_loads() -> AxleLoads {
        AxleLoads {
            front: 4000.0, // Newtons (N)
            rear: 4000.0,  // Newtons (N)
        }
    }

    #[test]
    fn test_calculate_powertrain_force_tyre_traction_limited() {
        let car_state = CarState {
            gear: 4,
            ..CarState::default()
        };
        let powertrain = Powertrain::default();

        let tyre = Tyre::default();

        let powertrain_force_tyre_traction_limited =
            calculate_powertrain_force_tyre_traction_limited(
                &default_static_axle_loads(),
                &car_state,
                &powertrain,
                &tyre,
            );

        // Car traction (RWD) = 4000 N * 0.9 = 3600 N
        // Powertrain force power limited = 800_000 W / 10 m/s = 80000 N
        // Powertrain force torque limited = 800 * 0.8 * 1.5 / 0.36 =  2666.6667 N
        // As Powertrain force torque limited is smallest force and smaller than traction, then expected value: 2666.6667 N
        assert!(
            (powertrain_force_tyre_traction_limited - 2666.6667).abs() < 1e-3,
            "Expected force: 2666.6667 Calculated force: {}",
            powertrain_force_tyre_traction_limited
        );
    }

    #[test]
    fn test_traction_limit_follows_drivetrain() {
        let car_state = CarState {
            speed: 0.0, // meters per second (m/s)
            gear: 1,
            ..CarState::default()
        };
        let tyre = Tyre::default();
        let axle_loads = AxleLoads {
            front: 5000.0, // Newtons (N)
            rear: 3000.0,  // Newtons (N)
        };
        let mut powertrain = Powertrain {
            drivetrain: Drivetrain::RearWheelDrive,
            ..Powertrain::default()
        };

        // Powertrain force at standstill in 1st gear = 800 * 3 * 1.5 / 0.36 = 10000 N,
        // so in every case below the tyres are the limit

        // RWD: 0.9 * 3000 N = 2700 N
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads, &car_state, &powertrain, &tyre,
        );
        assert!((force - 2700.0).abs() < 1e-9, "RWD force: {}", force);

        // FWD: 0.9 * 5000 N = 4500 N
        powertrain.drivetrain = Drivetrain::FrontWheelDrive;
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads, &car_state, &powertrain, &tyre,
        );
        assert!((force - 4500.0).abs() < 1e-9, "FWD force: {}", force);

        // AWD 60:40: min(4500 N / 0.6, 2700 N / 0.4) = min(7500 N, 6750 N) = 6750 N
        powertrain.drivetrain = Drivetrain::AllWheelDrive {
            front_torque_split: 0.6,
        };
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads, &car_state, &powertrain, &tyre,
        );
        assert!((force - 6750.0).abs() < 1e-9, "AWD force: {}", force);
    }

    #[test]
    fn test_calculate_force_losses() {
        // Total normal force = 4000 N + 4000 N = 8000 N
        let axle_loads = AxleLoads {
            front: 4000.0, // Newtons (N)
            rear: 4000.0,  // Newtons (N)
        };

        let simulation_config = SimulationConfig::default();

        let force_losses = calculate_force_losses(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &axle_loads,
        );

        // As the car is moving there is rolling friction = 8000 N * 0.03 = 240 N
        // Air drag = 0.5 * 1.225 * 0.35 * 2 * 10 * 10 * (+1) = 42.875 N
        // Total losses  = 240 N + 42.875 N = 282.875
        assert!((force_losses - 282.875).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_normal_force() {
        let mass: f64 = 1000.0;
        let g_acceleration: f64 = 9.81;

        let normal_force = calculate_normal_force(mass, g_acceleration);

        // expected value: normal force = 1000 kg * 9.81 m/s^2 = 9810 N
        assert!((normal_force - 9810.0).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_net_force() {
        let mut simulation_config = SimulationConfig::default();

        simulation_config.initial_car.gear = 4;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Using previous values calculated:
        // Net force = 2666.6667 - 282.875 =  2383.7917 N
        assert!(
            (net_force - 2383.7917).abs() < 1e-3,
            "Expected value: {}, calculated value: {}",
            2383.7917,
            net_force
        );
    }

    #[test]
    fn test_calculate_acceleration() {
        let mass: f64 = 1000.0;
        let force: f64 = 1000.0;

        let acceleration = calculate_acceleration(force, mass);

        // expected value: acceleration = 1000 / 1000 = 1 m/s^2
        assert!((acceleration - 1.0).abs() < 1e-3);
    }

    #[test]
    fn straight_line_braking() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.braking = true;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Brake force with 0.575 front bias on static loads = 6260.8696 N (see braking.rs)
        // Net force = -282.875 N - 6260.8696 N = -6543.7446 N
        assert!((net_force - (-6543.7446)).abs() < 1e-4);
    }

    #[test]
    fn test_net_force_braking_at_standstill() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.braking = true;
        simulation_config.initial_car.speed = 0.0;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // A stopped car stays stopped: brakes and friction can't push it backwards, expected value: 0 N
        assert_eq!(net_force, 0.0);
    }

    #[test]
    fn test_net_force_coasting_above_terminal_speed() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 80.0;
        simulation_config.initial_car.gear = 5;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Powertrain force torque limited = 800 * 0.5 * 1.5 / 0.36 = 1666.6667 N
        // Losses = 240 N + 0.5 * 1.225 * 0.35 * 2 * 80 * 80 = 240 + 2744 = 2984 N
        // Net force = 1666.6667 - 2984 = -1317.3333 N (car must slow down)
        assert!(
            (net_force - (-1317.3333)).abs() < 1e-3,
            "Expected value: -1317.3333, calculated value: {}",
            net_force
        );
    }

    #[test]
    fn test_net_force_launch_from_standstill() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.gear = 1;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Powertrain force in 1st gear = 800 * 3 * 1.5 / 0.36 = 10000 N
        // Traction limited force (RWD) = 0.9 * 4000 N = 3600 N
        // Breakaway friction = 8000 N * 0.035 = 280 N, air drag at 0 m/s = 0 N
        // Net force = 3600 N - 280 N = 3320 N
        assert!(
            (net_force - 3320.0).abs() < 1e-9,
            "Expected value: 3320, calculated value: {}",
            net_force
        );
    }

    #[test]
    fn test_net_force_standstill_not_negative() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.gear = 1;
        // Breakaway friction set explicitly higher than the traction limit
        simulation_config.car.tyre.mu_breakaway = 0.7;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Traction limited force = 3600 N, breakaway friction = 8000 * 0.7 = 5600 N
        // Breakaway friction can't push the car backwards, so expected value: 0 N
        assert_eq!(net_force, 0.0);
    }
}
