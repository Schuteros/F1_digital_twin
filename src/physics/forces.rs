use crate::physics::aero::calculate_air_drag;
use crate::physics::braking::calculate_brake_force;
use crate::physics::car::Car;
use crate::physics::environment::Environment;
use crate::physics::loads::AxleLoads;
use crate::physics::powertrain::{Powertrain, calculate_best_force};
use crate::physics::states::CarState;
use crate::physics::tyres::{Tyre, calculate_current_tyre_friction, calculate_split_force_limit};

/// Calculates powertrain force in Newtons, N, limited by the grip of the driven axles.
/// The powertrain force is split between the axles by the drivetrain, and the first
/// driven axle to reach its traction limit caps the total (ideal traction control).
///
/// The powertrain force is the best force in the current gear (docs/12, equation 12.10),
/// or 0 while a gear shift is in progress (docs/12 section 12.8).
fn calculate_powertrain_force_tyre_traction_limited(
    axle_loads: &AxleLoads,
    car_state: &CarState,
    powertrain: &Powertrain,
    tyre: &Tyre,
) -> f64 {
    let powertrain_force = if car_state.shift_time_left > 0.0 {
        0.0
    } else {
        calculate_best_force(
            powertrain,
            tyre.wheel_radius,
            car_state.gear,
            car_state.speed,
        )
    };
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
/// `axle_loads` are the axle normal forces for this step; in the simulation they are the
/// corner tyre loads from the suspension state (docs/11 section 11.8).
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

    /// Default config with the powertrain shift tables filled (docs/12 section 12.10)
    fn prepared_config() -> SimulationConfig {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.prepare().unwrap();
        simulation_config
    }

    /// Default powertrain with the shift tables filled for the default tyre
    fn prepared_powertrain() -> Powertrain {
        prepared_config().car.powertrain
    }

    #[test]
    fn test_calculate_powertrain_force_tyre_traction_limited() {
        let car_state = CarState {
            gear: 4,
            ..CarState::default()
        };
        let powertrain = prepared_powertrain();

        let tyre = Tyre::default();
        // Heavy rear axle so the tyres are not the limit: RWD traction = 0.9 * 20000 N = 18000 N
        let axle_loads = AxleLoads {
            front: 4000.0,  // Newtons (N)
            rear: 20_000.0, // Newtons (N)
        };

        let powertrain_force_tyre_traction_limited =
            calculate_powertrain_force_tyre_traction_limited(
                &axle_loads,
                &car_state,
                &powertrain,
                &tyre,
            );

        // Gear 4 at 10 m/s is below its clutch lock speed (36.16 m/s), so the clutch slips:
        // powertrain force = 375 * 2.78 * 3.0 / 0.36 = 8687.5 N (docs/12, equation 12.10)
        // As the powertrain force is smaller than traction, expected value: 8687.5 N
        // (1e-9 relative: rounding only)
        assert!(
            (powertrain_force_tyre_traction_limited - 8687.5).abs() < 1e-9 * 8687.5,
            "Expected force: 8687.5 Calculated force: {}",
            powertrain_force_tyre_traction_limited
        );
    }

    #[test]
    fn test_powertrain_force_zero_while_shifting() {
        let car_state = CarState {
            gear: 4,
            shift_time_left: 0.003,
            ..CarState::default()
        };

        let force = calculate_powertrain_force_tyre_traction_limited(
            &default_static_axle_loads(),
            &car_state,
            &prepared_powertrain(),
            &Tyre::default(),
        );

        // No torque reaches the wheels during a shift (docs/12 section 12.8)
        assert_eq!(force, 0.0);
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
            ..prepared_powertrain()
        };

        // Powertrain force at standstill in 1st gear (clutch slipping, docs/12 equation 12.10)
        // = 375 * 5.43 * 3.0 / 0.36 = 16968.75 N, so in every case below the tyres are the limit

        // RWD: 0.9 * 3000 N = 2700 N
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads,
            &car_state,
            &powertrain,
            &tyre,
        );
        assert!((force - 2700.0).abs() < 1e-9, "RWD force: {}", force);

        // FWD: 0.9 * 5000 N = 4500 N
        powertrain.drivetrain = Drivetrain::FrontWheelDrive;
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads,
            &car_state,
            &powertrain,
            &tyre,
        );
        assert!((force - 4500.0).abs() < 1e-9, "FWD force: {}", force);

        // AWD 60:40: min(4500 N / 0.6, 2700 N / 0.4) = min(7500 N, 6750 N) = 6750 N
        powertrain.drivetrain = Drivetrain::AllWheelDrive {
            front_torque_split: 0.6,
        };
        let force = calculate_powertrain_force_tyre_traction_limited(
            &axle_loads,
            &car_state,
            &powertrain,
            &tyre,
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
        let mut simulation_config = prepared_config();

        simulation_config.initial_car.gear = 4;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Powertrain force in gear 4 at 10 m/s = 8687.5 N (slipping, see above), above the
        // RWD traction 0.9 * 4000 N = 3600 N, so the tyres limit it to 3600 N
        // Net force = 3600 - 282.875 = 3317.125 N (1e-9 N: a single subtraction of exact values)
        assert!(
            (net_force - 3317.125).abs() < 1e-9,
            "Expected value: {}, calculated value: {}",
            3317.125,
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
        let mut simulation_config = prepared_config();
        simulation_config.initial_car.speed = 95.0;
        simulation_config.initial_car.gear = 8;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Gear 8 at 95 m/s, clutch locked: n = 95 * 4.95 / (2 * PI * 0.36) = 207.896 Hz
        // T = 318 + (250 - 318) * (207.896 - 200) / 16.67 = 285.790 N·m
        // Powertrain force = 285.790 * 4.95 / 0.36 = 3929.61 N, tyre limit 3600 N -> 3600 N
        // Losses = 240 N + 0.5 * 1.225 * 0.35 * 2 * 95 * 95 = 240 + 3869.46875 = 4109.46875 N
        // Net force = 3600 - 4109.46875 = -509.46875 N (car must slow down)
        // 1e-9 N: sums of values that are exact in binary plus one rounded drag product
        assert!(
            (net_force - (-509.46875)).abs() < 1e-9,
            "Expected value: -509.46875, calculated value: {}",
            net_force
        );
    }

    #[test]
    fn test_net_force_launch_from_standstill() {
        let mut simulation_config = prepared_config();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.gear = 1;

        let net_force = calculate_net_force(
            &simulation_config.car,
            &simulation_config.environment,
            &simulation_config.initial_car,
            &default_static_axle_loads(),
        );

        // Powertrain force in 1st gear, clutch slipping = 375 * 5.43 * 3.0 / 0.36 = 16968.75 N
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
        let mut simulation_config = prepared_config();
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
