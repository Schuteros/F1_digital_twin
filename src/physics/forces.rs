use crate::physics::powertrain::{calculate_force_from_powertrain};
use crate::physics::tires::calculate_force_static_friction;

fn calculate_powertrain_force_tire_traction_limited(normal_force: f64,
                                               mu_static_friction: f64,
                                               powertrain_power: f64,
                                               vehicle_speed: f64,
                                               powertrain_max_torque: f64,
                                               wheel_radius: f64) -> f64 {
    let powertrain_force = calculate_force_from_powertrain(powertrain_power, vehicle_speed, powertrain_max_torque, wheel_radius); // Newtons (N)
    let traction_force = calculate_force_static_friction(normal_force, mu_static_friction);

    if traction_force >= powertrain_force {
        powertrain_force
    } else {
        traction_force
    }
}


