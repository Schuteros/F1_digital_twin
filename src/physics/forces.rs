use crate::physics::aero::calculate_air_drag;
use crate::physics::braking::calculate_brake_force;
use crate::physics::powertrain::calculate_force_from_powertrain;
use crate::physics::states::CarState;
use crate::physics::tyres::{calculate_current_tyre_friction, calculate_force_static_friction};
use crate::physics::{Aero, Environment, Mass, Powertrain, Tyre};

fn calculate_powertrain_force_tyre_traction_limited(
    driven_axle_normal_force: f64,
    car_state: &CarState,
    powertrain: &Powertrain,
    tyre: &Tyre,
) -> f64 {
    let powertrain_force =
        calculate_force_from_powertrain(powertrain, tyre.wheel_radius, car_state);
    let traction_force =
        calculate_force_static_friction(driven_axle_normal_force, tyre.mu_static);

    powertrain_force.min(traction_force)
}

fn calculate_force_losses(
    total_normal_force: f64,
    tyre: &Tyre,
    car_state: &CarState,
    environment: &Environment,
    aero: &Aero,
    mass: &Mass,
) -> f64 {
    let friction_loss = calculate_current_tyre_friction(
        total_normal_force,
        tyre.mu_rolling,
        tyre.mu_breakaway,
        car_state.speed,
    );

    let air_drag_loss = calculate_air_drag(
        environment.air_density,
        aero.drag_coefficient,
        aero.frontal_area,
        car_state.speed,
    );

    let mut brake_force = 0.0;

    if car_state.braking {
        brake_force = calculate_brake_force(
            mass,
            tyre.mu_static,
            environment.g_acceleration,
        );
    }

    friction_loss + air_drag_loss + brake_force
}

pub fn calculate_normal_force(mass: f64, g_acceleration: f64) -> f64 {
    mass * g_acceleration
}

pub fn calculate_net_force(
    mass: &Mass,
    driven_axle_mass: f64,
    car_state: &CarState,
    powertrain: &Powertrain,
    tyre: &Tyre,
    environment: &Environment,
    aero: &Aero,
) -> f64 {
    let driven_axle_normal_force =
        calculate_normal_force(driven_axle_mass, environment.g_acceleration);
    let total_normal_force = calculate_normal_force(
        mass.total,
        environment.g_acceleration,
    );

    let force_losses = calculate_force_losses(
        total_normal_force,
        tyre,
        car_state,
        environment,
        aero,
        mass,
    );

    if car_state.braking {
        if car_state.speed == 0.0 {
            0.0
        } else {
            -force_losses
        }
    } else {
        let powertrain_force = calculate_powertrain_force_tyre_traction_limited(
            driven_axle_normal_force,
            car_state,
            powertrain,
            tyre,
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
    use crate::physics::states::CarState;
    use crate::physics::{Powertrain, SimulationConfig, Tyre};

    #[test]
    fn test_calculate_powertrain_force_tyre_traction_limited() {
        let mut car_state = CarState::default();
        car_state.gear = 4;
        let powertrain = Powertrain::default();

        let tyre = Tyre::default();

        let driven_axle_normal_force: f64 = 4000.0;

        let powertrain_force_tyre_traction_limited =
            calculate_powertrain_force_tyre_traction_limited(
                driven_axle_normal_force,
                &car_state,
                &powertrain,
                &tyre,
            );

        // Car traction = 4000 N * 0.9 = 3600 N
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
    fn test_calculate_force_losses() {
        let total_normal_force: f64 = 8000.0;

        let simulation_config = SimulationConfig::default();

        let force_losses = calculate_force_losses(
            total_normal_force,
            &simulation_config.car.tyre,
            &simulation_config.initial_car,
            &simulation_config.environment,
            &simulation_config.car.aero,
            &simulation_config.car.mass,
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

        let driven_axle_mass: f64 = simulation_config.car.mass.rear;
        simulation_config.initial_car.gear = 4;

        let net_force = calculate_net_force(
            &simulation_config.car.mass,
            driven_axle_mass,
            &simulation_config.initial_car,
            &simulation_config.car.powertrain,
            &simulation_config.car.tyre,
            &simulation_config.environment,
            &simulation_config.car.aero,
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
            &simulation_config.car.mass,
            simulation_config.car.mass.rear,
            &simulation_config.initial_car,
            &simulation_config.car.powertrain,
            &simulation_config.car.tyre,
            &simulation_config.environment,
            &simulation_config.car.aero,
        );

        // From previous calculations: net force = -282.875 - 7200 = -7482.875 N
        assert!((net_force - (-7482.875)).abs() < 1e-4);
    }

    #[test]
    fn test_net_force_coasting_above_terminal_speed() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 80.0;
        simulation_config.initial_car.gear = 5;

        let net_force = calculate_net_force(
            &simulation_config.car.mass,
            simulation_config.car.mass.rear,
            &simulation_config.initial_car,
            &simulation_config.car.powertrain,
            &simulation_config.car.tyre,
            &simulation_config.environment,
            &simulation_config.car.aero,
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
    fn test_net_force_standstill_not_negative() {
        let mut simulation_config = SimulationConfig::default();
        simulation_config.initial_car.speed = 0.0;
        simulation_config.initial_car.gear = 1;

        let net_force = calculate_net_force(
            &simulation_config.car.mass,
            simulation_config.car.mass.rear,
            &simulation_config.initial_car,
            &simulation_config.car.powertrain,
            &simulation_config.car.tyre,
            &simulation_config.environment,
            &simulation_config.car.aero,
        );

        // Traction limited force = 3600 N, breakaway friction = 8000 * 0.7 = 5600 N
        // Breakaway friction can't push the car backwards, so expected value: 0 N
        assert_eq!(net_force, 0.0);
    }
}
