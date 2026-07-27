use crate::physics::aero::calculate_air_drag;
use crate::physics::powertrain::calculate_force_from_powertrain;
use crate::physics::tires::{calculate_current_tire_friction, calculate_force_static_friction};
use crate::physics::states::CarState;
use crate::physics::{AeroModel, EnvironmentModel, PowertrainModel, TyreModel};

fn calculate_powertrain_force_tire_traction_limited(
    driven_axle_normal_force: f64,
    car_state: &CarState,
    powertrain_model: &PowertrainModel,
    tyre_model: &TyreModel
) -> f64 {
    let powertrain_force = calculate_force_from_powertrain(
        powertrain_model.power,
        car_state.speed,
        powertrain_model.max_torque,
        tyre_model.wheel_radius
    );
    let traction_force = calculate_force_static_friction(driven_axle_normal_force, tyre_model.mu_static_friction);

    powertrain_force.min(traction_force)
}

fn calculate_force_losses(
    total_normal_force: f64,
    tyre_model: &TyreModel,
    car_state: &CarState,
    environment_model: &EnvironmentModel,
    aero_model: &AeroModel
) -> f64 {

    let friction_loss = calculate_current_tire_friction(
        total_normal_force,
        tyre_model.mu_rolling_friction,
        tyre_model.mu_breakaway_friction,
        car_state.speed
    );

    let air_drag_loss = calculate_air_drag(
        environment_model.air_density,
        aero_model.drag_coefficient,
        aero_model.frontal_area,
        car_state.speed
    );

    friction_loss + air_drag_loss
}

pub fn calculate_normal_force(mass: f64, g_acceleration: f64) -> f64 {
    mass * g_acceleration
}

pub fn calculate_net_force(
    total_mass: f64,
    driven_axle_mass: f64,
    car_state: &CarState,
    powertrain_model: &PowertrainModel,
    tyre_model: &TyreModel,
    environment_model: &EnvironmentModel,
    aero_model: &AeroModel,
) -> f64 {
    let driven_axle_normal_force = calculate_normal_force(driven_axle_mass, environment_model.g_acceleration);
    let total_normal_force = calculate_normal_force(total_mass, environment_model.g_acceleration);

    let powertrain_force = calculate_powertrain_force_tire_traction_limited(
        driven_axle_normal_force,
        car_state,
        powertrain_model,
        tyre_model
    );
    let force_losses = calculate_force_losses(
        total_normal_force,
        tyre_model,
        car_state,
        environment_model,
        aero_model
    );

    powertrain_force - force_losses
}


pub(crate) fn calculate_acceleration(force: f64, mass: f64) -> f64 {
    force / mass
}


#[cfg(test)]
mod tests {
    use crate::physics::{AeroModel, EnvironmentModel, PowertrainModel, TyreModel};
    use crate::physics::forces::{calculate_acceleration, calculate_force_losses, calculate_net_force, calculate_normal_force, calculate_powertrain_force_tire_traction_limited};
    use crate::physics::states::CarState;

    #[test]
    fn test_calculate_powertrain_force_tire_traction_limited() {
        let car_state = CarState {
            speed: 10.0,
            distance: 100.0,
        };

        let powertrain_model = PowertrainModel {
            power: 800_000.0,
            max_torque: 800.0,
        };

        let tyre_model = TyreModel {
            mu_static_friction: 0.9,
            mu_rolling_friction: 0.03,
            mu_breakaway_friction: 0.7,
            wheel_radius: 0.35,
        };

        let driven_axle_normal_force: f64 = 4000.0;

        let powertrain_force_tire_traction_limited = calculate_powertrain_force_tire_traction_limited(
            driven_axle_normal_force,
            &car_state,
            &powertrain_model,
            &tyre_model
        );

        // Car traction = 4000 N * 0.9 = 3600 N
        // Powertrain force power limited = 800_000 W / 10 m/s = 80000 N
        // Powertrain force torque limited = 800 Nm / 0.35 m = 2285.714286 N
        // As Powertrain force torque limited is smallest force and smaller than traction, then expected value: 2285.714286
        assert!((powertrain_force_tire_traction_limited - 2285.714286).abs() < 1e3);
    }

    #[test]
    fn test_calculate_force_losses() {
        let total_normal_force: f64 = 8000.0;

        let tyre_model = TyreModel::default();

        let car_state = CarState::default();

        let environment_model = EnvironmentModel::default();

        let aero_model = AeroModel::default();


        let force_losses = calculate_force_losses(total_normal_force, &tyre_model, &car_state, &environment_model, &aero_model);

        // As the car is moving there is rolling friction = 8000 N * 0.03 = 240 N
        // Air drag = 0.5 * 1.225 * 0.35 * 2 * 10 * 10 * (+1) = 42.875 N
        // Total losses  = 240 N + 42.875 N = 282.875
        assert!((force_losses - 282.875).abs() < 1e3);
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
        let total_mass: f64 = 8000.0 / 9.81;
        let driven_axle_mass: f64 = 4000.0 / 9.81;

        let tyre_model = TyreModel::default();

        let car_state = CarState::default();

        let environment_model = EnvironmentModel::default();

        let aero_model = AeroModel::default();

        let powertrain_model = PowertrainModel::default();

        let net_force = calculate_net_force(total_mass, driven_axle_mass, &car_state, &powertrain_model, &tyre_model, &environment_model, &aero_model);

        // Using previous values calculated:
        // Net force = 2285.714286 - 282.875 = 2002.839286 N
        assert!((net_force - 2002.839286).abs() < 1e-3);
    }


    #[test]
    fn test_calculate_acceleration() {
        let mass: f64 = 1000.0;
        let force: f64 = 1000.0;

        let acceleration = calculate_acceleration(force, mass);

        // expected value: acceleration = 1000 / 1000 = 1 m/s^2
        assert!((acceleration - 1.0).abs() < 1e-3);
    }
}