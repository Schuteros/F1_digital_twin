//! This module contains calculations related to powertrain dynamics.
//!
//! Underlying science equations are found in this documentation:
//! [Powertrain Engine Documentation](../../docs/02_Powertrain_Force_Dynamics.MD)
//! [Clutch Dynamics Documentation](../../docs/08_clutch_dynamics.MD)

use crate::physics::PowertrainModel;
use crate::physics::states::CarState;
use std::f64::consts::PI;

/// Calculates force from powertrain that is limited by power output
pub(crate) fn calculate_force_power_limited(powertrain_power: f64, vehicle_speed: f64) -> f64 {
    powertrain_power / vehicle_speed
}

/// Calculates force from powertrain that is limited by maximum torque
pub(crate) fn calculate_force_max_torque_limited(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    current_gear: u8,
) -> f64 {
    if wheel_radius <= 0.0 || current_gear == 0 {
        return 0.0;
    }

    let gear_index = (current_gear - 1) as usize;
    (powertrain_model.max_torque / wheel_radius)
        * powertrain_model.gear_ratios[gear_index]
        * powertrain_model.final_drive
}

/// Calculates transmission input shaft rotational speed (Hz) if locked
pub(crate) fn calculate_transmission_input_revs(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    if wheel_radius <= 0.0 || car_state.current_gear == 0 {
        return 0.0;
    }

    let gear_index = (car_state.current_gear - 1) as usize;
    let gear_ratio = powertrain_model.gear_ratios[gear_index];

    (car_state.speed * powertrain_model.final_drive * gear_ratio) / (2.0 * PI * wheel_radius)
}

/// Selects needed gear in locked clutch
pub(crate) fn select_best_gear(
    powertrain_model: &PowertrainModel,
    current_revs: f64,
    current_gear: u8,
) -> u8 {
    let max_gears = powertrain_model.gear_ratios.len() as u8;

    let near_redline =
        (powertrain_model.max_revs - current_revs) < powertrain_model.shift_up_margin;
    let near_stall =
        (current_revs - powertrain_model.min_revs) < powertrain_model.shift_down_margin;

    if near_redline && current_gear < max_gears {
        current_gear + 1
    } else if near_stall && current_gear > 1 {
        current_gear - 1
    } else {
        current_gear
    }
}

/// Checks if clutch friction/slip dynamics are active (stall prevention or launch)
pub(crate) fn check_clutch_need(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    car_state: &CarState,
) -> bool {
    let transmission_input_revs =
        calculate_transmission_input_revs(powertrain_model, wheel_radius, car_state);
    transmission_input_revs < powertrain_model.min_revs
}

/// Calculates mechanical power (Watts) from rotational speed (Hz) and torque (N·m)
#[allow(dead_code)]
pub(crate) fn calculate_output_shaft_power(
    powertrain_max_torque: f64,
    output_shaft_revs: f64,
) -> f64 {
    2.0 * PI * output_shaft_revs * powertrain_max_torque
}

/// Calculates clutch output shaft power-limited force
#[allow(dead_code)]
pub(crate) fn calculate_force_output_shaft_power_limited(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    let output_shaft_revs =
        calculate_transmission_input_revs(powertrain_model, wheel_radius, car_state);
    let output_shaft_power =
        calculate_output_shaft_power(powertrain_model.max_torque, output_shaft_revs);

    calculate_force_power_limited(output_shaft_power, car_state.speed)
}

/// Calculates clutch-limited force during slip (strictly torque-limited by clutch capacity)
#[allow(dead_code)]
pub(crate) fn calculate_force_clutch_limited(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    calculate_force_max_torque_limited(powertrain_model, wheel_radius, car_state.current_gear)
}

/// Calculates force from powertrain across both Slipping and Locked states
pub(crate) fn calculate_force_from_powertrain(
    powertrain_model: &PowertrainModel,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    let force_torque_limited =
        calculate_force_max_torque_limited(powertrain_model, wheel_radius, car_state.current_gear);

    if check_clutch_need(powertrain_model, wheel_radius, car_state) {
        // SLIPPING / LAUNCH REGIME: Strictly torque-limited across the clutch friction
        force_torque_limited
    } else {
        // LOCKED REGIME: Capped by the engine's power limit (P / v) as speed builds
        let force_power_limited =
            calculate_force_power_limited(powertrain_model.power, car_state.speed);
        force_power_limited.min(force_torque_limited)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_powertrain_force_max_torque_limited() {
        let wheel_radius: f64 = 0.36; // meters (m)
        let powertrain_model = PowertrainModel::default();
        let current_gear: u8 = 4;

        let force =
            calculate_force_max_torque_limited(&powertrain_model, wheel_radius, current_gear);

        // Expected: 800 * 0.8 * 1.5 / 0.36 =  2666.6667 N
        assert!(
            (force - 2666.6667).abs() < 1e-3,
            "Torque limit guard failed at 0 m/s!"
        );
    }

    #[test]
    fn test_powertrain_force_power_limited() {
        let powertrain_max_power: f64 = 750_000.0; // 750 kW
        let vehicle_speed: f64 = 100.0; // 100 m/s

        let force: f64 = calculate_force_power_limited(powertrain_max_power, vehicle_speed);

        // expected 750_000 / 100 = 7500.0000 N
        assert!(
            (force - 7500.0000).abs() < 1e-3,
            "Car's power limit is incorrect!"
        );
    }

    #[test]
    fn test_force_from_powertrain_standstill() {
        let powertrain_model = PowertrainModel::default();
        let wheel_radius: f64 = 0.36;
        let mut car_state: CarState = Default::default();
        car_state.speed = 0.0;
        car_state.current_gear = 1;

        let force: f64 =
            calculate_force_from_powertrain(&powertrain_model, wheel_radius, &car_state);

        // Expected: 800 * 3 * 1.5 / 0.36 = 10000 N as the Force limited by power goes to infinity when the car is standstill
        assert!(
            (force - 10000.0).abs() < 1e-3,
            "Torque limit guard failed at 0 m/s!"
        );
    }

    #[test]
    fn test_force_from_powertrain_moving() {
        let powertrain_model = PowertrainModel::default();
        let mut car_state: CarState = Default::default();
        car_state.current_gear = 4;
        let wheel_radius: f64 = 0.36; // meters (m)

        let force: f64 =
            calculate_force_from_powertrain(&powertrain_model, wheel_radius, &car_state);

        // Force limited by max torque: 800 * 0.8 * 1.5 / 0.36 =  2666,66667 N
        // Force limited by power: 800_000 / 10 = 80000.0000 N
        // Expected: 800 * 3 * 1.5 / 0.36 =  2666,66667 N
        assert!((force - 2666.66667).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_transmission_input_revs() {
        let powertrain_model = PowertrainModel::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let transmission_input_revs =
            calculate_transmission_input_revs(&powertrain_model, wheel_radius, &car_state);

        // Expected value: 10 * 1.5 * 3 / (2 * PI * 0.36) =  19.8944
        assert!((transmission_input_revs - 19.8944).abs() < 1e-3);
    }

    #[test]
    fn test_select_best_gear_from_lower_gear() {
        let powertrain_model = PowertrainModel::default();
        let current_revs: f64 = 12.5;
        let current_gear: u8 = 2;

        let best_gear = select_best_gear(&powertrain_model, current_revs, current_gear);

        // As the revs are lower than min revs by more than 0.5: 13.33 - 0.5 > 12.5, then best gear should be shifted down to 1st, it is then 1st gear.
        assert_eq!(best_gear, 1);
    }

    #[test]
    fn test_check_clutch_need() {
        let powertrain_model = PowertrainModel::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let clutch_need = check_clutch_need(&powertrain_model, wheel_radius, &car_state);

        assert_eq!(clutch_need, false);
    }

    #[test]
    fn test_calculate_output_shaft_power() {
        let powertrain_max_torque = 800.0;
        let output_shaft_revs: f64 = 0.5;

        let output_shaft_power =
            calculate_output_shaft_power(powertrain_max_torque, output_shaft_revs);

        // Expected value: 2* PI * 800 * 0.5 = 2513.2741
        assert!((output_shaft_power - 2513.2741).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_force_clutch_limited() {
        let powertrain_model = PowertrainModel::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let force = calculate_force_clutch_limited(&powertrain_model, wheel_radius, &car_state);

        //As the speed is high enough, then the clutch is engaged, so that means that max engine torque is the same as clutch output torque, so it is same as moving car force: 10'000 N
        assert!(
            (force - 10_000.0).abs() < 1e-3,
            "Test failed, actual value {} but test value {}",
            force,
            10_000.0
        );
    }
}
