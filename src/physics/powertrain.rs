//! This module contains calculations related to powertrain
//!
//! Underlying science equation are found in this documentation:
//! [Powertrain Engine Documentation](../../docs/2_Powertrain_Force_Dynamics.MD)


/// Calculates force from powertrain that is limited by the power
pub(crate) fn calculate_force_power_limited(powertrain_power: f64, vehicle_speed: f64) -> f64 {
    let force: f64 = powertrain_power / vehicle_speed; // Newtons (N)

    force
}


/// Calculates force from powertrain that is limited by the max torque
pub(crate) fn calculate_force_max_torque_limited(powertrain_max_torque: f64, wheel_radius: f64, ) ->  f64 {
    let force: f64 = powertrain_max_torque / wheel_radius; // Newtons (N)

    force
}


/// Calculates force from powertrain by comparing force limited by torque and power
pub(crate) fn calculate_force_from_powertrain(powertrain_power: f64, vehicle_speed: f64, powertrain_max_torque: f64, wheel_radius: f64) -> f64 {
    let force_max_torque_limited: f64 = calculate_force_max_torque_limited(powertrain_max_torque, wheel_radius);
    let force_power_limited = calculate_force_power_limited(powertrain_power, vehicle_speed);

    if force_power_limited > force_max_torque_limited {
        force_max_torque_limited
    } else {
        force_power_limited
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_powertrain_force_max_torque_limited() {

        let powertrain_max_torque: f64 = 950.0; // Newton meters (Nm)
        let wheel_radius: f64 = 0.36; // meters (m)

        let force = calculate_force_max_torque_limited(powertrain_max_torque, wheel_radius);

        // Expected: 950 / 0.36 = 2638.8888 N
        assert!((force - 2638.8888).abs() < 1e-3, "Torque limit guard failed at 0 m/s!");
    }

    #[test]
    fn test_powertrain_force_power_limited() {
        let powertrain_max_power: f64 = 750_000.0; // 750 kW
        let vehicle_speed: f64 = 100.0; // 100 m/s

        let force: f64 = calculate_force_power_limited(powertrain_max_power, vehicle_speed);

        // expected 750_000 / 100 = 7500.0000 N
        assert!((force - 7500.0000).abs() < 1e-3, "Car's power limit is incorrect!");
    }

    #[test]
    fn test_force_from_powertrain_standstill() {
        let powertrain_max_power: f64 = 750_000.0; // 750 kW
        let vehicle_speed: f64 = 0.0; // 100 m/s
        let powertrain_max_torque: f64 = 950.0; // Newton meters (Nm)
        let wheel_radius: f64 = 0.36; // meters (m)

        let force: f64 = calculate_force_from_powertrain(powertrain_max_power, vehicle_speed, powertrain_max_torque, wheel_radius);

        // Expected: 950 / 0.36 = 2638.8888 N as the Force limited by power goes to infinity when the car is standstill
        assert!((force - 2638.8888).abs() < 1e-3, "Torque limit guard failed at 0 m/s!");
    }


    #[test]
    fn test_force_from_powertrain_moving() {
        let powertrain_max_power: f64 = 750_000.0; // 750 kW
        let vehicle_speed: f64 = 100.0; // 100 m/s
        let powertrain_max_torque: f64 = 950.0; // Newton meters (Nm)
        let wheel_radius: f64 = 0.36; // meters (m)

        let force: f64 = calculate_force_from_powertrain(powertrain_max_power, vehicle_speed, powertrain_max_torque, wheel_radius);

        // Force limited by max torque: 950 / 0.36 = 2638.8888 N
        // Force limited by power: 750_000 / 100 = 7500.0000 N
        // Expected: 950 / 0.36 = 2638.8888 N
        assert!((force - 2638.8888).abs() < 1e-3, "Torque limit guard failed at 0 m/s!");
    }

}