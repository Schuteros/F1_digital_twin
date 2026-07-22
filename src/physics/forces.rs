use crate::physics::aero::calculate_air_drag;
use crate::physics::powertrain::calculate_force_from_powertrain;
use crate::physics::tires::{calculate_current_tire_friction, calculate_force_static_friction};
use crate::physics::states::CarState;
use crate::physics::{AeroModel, EnvironmentModel, PowertrainModel, TyreModel};

fn calculate_powertrain_force_tire_traction_limited(
    normal_force: f64,
    car_state: &CarState,
    powertrain_model: &PowertrainModel,
    tyre_model: &TyreModel
) -> f64 {
    let powertrain_force = calculate_force_from_powertrain(
        powertrain_model.power,
        car_state.velocity,
        powertrain_model.max_torque,
        tyre_model.wheel_radius
    );
    let traction_force = calculate_force_static_friction(normal_force, tyre_model.mu_static_friction);

    powertrain_force.min(traction_force)
}

fn calculate_force_losses(
    normal_force: f64,
    tyre_model: &TyreModel,
    car_state: &CarState,
    environment_model: &EnvironmentModel,
    aero_model: &AeroModel
) -> f64 {
    let friction_loss = calculate_current_tire_friction(
        normal_force,
        tyre_model.mu_rolling_friction,
        tyre_model.mu_breakaway_friction,
        car_state.velocity
    );
    let air_drag_loss = calculate_air_drag(
        environment_model.air_density,
        aero_model.drag_coefficient,
        aero_model.frontal_area,
        car_state.velocity
    );

    friction_loss + air_drag_loss
}

pub fn calculate_net_force(
    normal_force: f64,
    car_state: &CarState,
    powertrain_model: &PowertrainModel,
    tyre_model: &TyreModel,
    environment_model: &EnvironmentModel,
    aero_model: &AeroModel,
) -> f64 {
    let powertrain_force = calculate_powertrain_force_tire_traction_limited(
        normal_force,
        car_state,
        powertrain_model,
        tyre_model
    );
    let force_losses = calculate_force_losses(
        normal_force,
        tyre_model,
        car_state,
        environment_model,
        aero_model
    );

    powertrain_force - force_losses
}