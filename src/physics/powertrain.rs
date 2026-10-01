//! This module contains calculations related to powertrain dynamics.
//!
//! Underlying science equations are found in this documentation:
//! [Powertrain Engine Documentation](../../docs/02_Powertrain_Force_Dynamics.MD)
//! [Clutch Dynamics Documentation](../../docs/08_clutch_dynamics.MD)

use crate::physics::states::CarState;
use std::f64::consts::PI;

/// Contains all the variables that define the powertrain
pub struct Powertrain {
    /// Power from the powertrain that is delivered to the wheels in Watts, W
    pub power: f64,
    /// Maximum torque on the wheels delivered from the powertrain in Newton meters, Nm
    pub max_torque: f64,
    /// Max rotational speed of the engine in Hertz, Hz
    pub max_revs: f64,
    /// Min rotational speed of the engine in Hertz, Hz
    pub min_revs: f64,
    /// Gear ratios from largest to smallest
    pub gear_ratios: Vec<f64>,
    /// Final drive ratio
    pub final_drive: f64,
    /// Shift up margin in Hertz, Hz
    pub shift_up_margin: f64,
    /// Shift down margin in Hertz, Hz
    pub shift_down_margin: f64,
    /// Which axles the powertrain drives
    pub drivetrain: Drivetrain,
}

/// Defines which axles receive the powertrain force
// Full names are kept for readability. FWD / AWD are config options, main.rs only uses the default.
#[allow(clippy::enum_variant_names, dead_code)]
pub enum Drivetrain {
    /// All powertrain force goes to the front axle
    FrontWheelDrive,
    /// All powertrain force goes to the rear axle
    RearWheelDrive,
    /// Powertrain force is split between both axles
    AllWheelDrive {
        /// Fraction of the powertrain force sent to the front axle, dimensionless, 0.0..=1.0
        /// (rear gets 1 - front_torque_split)
        front_torque_split: f64,
    },
}

impl Drivetrain {
    /// Fraction of the powertrain force delivered to the front axle, dimensionless, 0.0..=1.0
    pub(crate) fn front_share(&self) -> f64 {
        match self {
            Drivetrain::FrontWheelDrive => 1.0,
            Drivetrain::RearWheelDrive => 0.0,
            Drivetrain::AllWheelDrive { front_torque_split } => *front_torque_split,
        }
    }
}

impl Default for Powertrain {
    fn default() -> Self {
        Self {
            power: 800_000.0,
            max_torque: 800.0,
            max_revs: 133.33,
            min_revs: 13.33,
            gear_ratios: vec![3.0, 2.0, 1.0, 0.8, 0.5],
            final_drive: 1.5,
            shift_up_margin: 0.5,
            shift_down_margin: 0.5,
            drivetrain: Drivetrain::RearWheelDrive,
        }
    }
}

/// Calculates force from powertrain that is limited by power output
pub(crate) fn calculate_force_power_limited(powertrain_power: f64, car_speed: f64) -> f64 {
    powertrain_power / car_speed
}

/// Calculates force from powertrain that is limited by maximum torque
pub(crate) fn calculate_force_max_torque_limited(
    powertrain: &Powertrain,
    wheel_radius: f64,
    gear: u8,
) -> f64 {
    if wheel_radius <= 0.0 || gear == 0 {
        return 0.0;
    }

    let gear_index = (gear - 1) as usize;
    (powertrain.max_torque / wheel_radius)
        * powertrain.gear_ratios[gear_index]
        * powertrain.final_drive
}

/// Calculates transmission input shaft rotational speed (Hz) if locked
pub(crate) fn calculate_transmission_input_revs(
    powertrain: &Powertrain,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    if wheel_radius <= 0.0 || car_state.gear == 0 {
        return 0.0;
    }

    let gear_index = (car_state.gear - 1) as usize;
    let gear_ratio = powertrain.gear_ratios[gear_index];

    (car_state.speed * powertrain.final_drive * gear_ratio) / (2.0 * PI * wheel_radius)
}

/// Selects needed gear in locked clutch
pub(crate) fn select_best_gear(
    powertrain: &Powertrain,
    revs_hz: f64,
    gear: u8,
) -> u8 {
    let max_gears = powertrain.gear_ratios.len() as u8;

    let near_redline =
        (powertrain.max_revs - revs_hz) < powertrain.shift_up_margin;
    let near_stall =
        (revs_hz - powertrain.min_revs) < powertrain.shift_down_margin;

    if near_redline && gear < max_gears {
        gear + 1
    } else if near_stall && gear > 1 {
        gear - 1
    } else {
        gear
    }
}

/// Checks if clutch friction/slip dynamics are active (stall prevention or launch)
pub(crate) fn check_clutch_need(
    powertrain: &Powertrain,
    wheel_radius: f64,
    car_state: &CarState,
) -> bool {
    let transmission_input_revs =
        calculate_transmission_input_revs(powertrain, wheel_radius, car_state);
    transmission_input_revs < powertrain.min_revs
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
    powertrain: &Powertrain,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    let output_shaft_revs =
        calculate_transmission_input_revs(powertrain, wheel_radius, car_state);
    let output_shaft_power =
        calculate_output_shaft_power(powertrain.max_torque, output_shaft_revs);

    calculate_force_power_limited(output_shaft_power, car_state.speed)
}

/// Calculates clutch-limited force during slip (strictly torque-limited by clutch capacity)
#[allow(dead_code)]
pub(crate) fn calculate_force_clutch_limited(
    powertrain: &Powertrain,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    calculate_force_max_torque_limited(powertrain, wheel_radius, car_state.gear)
}

/// Calculates force from powertrain across both Slipping and Locked states
pub(crate) fn calculate_force_from_powertrain(
    powertrain: &Powertrain,
    wheel_radius: f64,
    car_state: &CarState,
) -> f64 {
    let force_torque_limited =
        calculate_force_max_torque_limited(powertrain, wheel_radius, car_state.gear);

    if check_clutch_need(powertrain, wheel_radius, car_state) {
        // SLIPPING / LAUNCH REGIME: Strictly torque-limited across the clutch friction
        force_torque_limited
    } else {
        // LOCKED REGIME: Capped by the engine's power limit (P / v) as speed builds
        let force_power_limited =
            calculate_force_power_limited(powertrain.power, car_state.speed);
        force_power_limited.min(force_torque_limited)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_powertrain_force_max_torque_limited() {
        let wheel_radius: f64 = 0.36; // meters (m)
        let powertrain = Powertrain::default();
        let gear: u8 = 4;

        let force =
            calculate_force_max_torque_limited(&powertrain, wheel_radius, gear);

        // Expected: 800 * 0.8 * 1.5 / 0.36 =  2666.6667 N
        assert!(
            (force - 2666.6667).abs() < 1e-3,
            "Torque limit guard failed at 0 m/s!"
        );
    }

    #[test]
    fn test_powertrain_force_power_limited() {
        let powertrain_max_power: f64 = 750_000.0; // 750 kW
        let car_speed: f64 = 100.0; // 100 m/s

        let force: f64 = calculate_force_power_limited(powertrain_max_power, car_speed);

        // expected 750_000 / 100 = 7500.0000 N
        assert!(
            (force - 7500.0000).abs() < 1e-3,
            "Car's power limit is incorrect!"
        );
    }

    #[test]
    fn test_force_from_powertrain_standstill() {
        let powertrain = Powertrain::default();
        let wheel_radius: f64 = 0.36;
        let car_state = CarState {
            speed: 0.0,
            gear: 1,
            ..CarState::default()
        };

        let force: f64 =
            calculate_force_from_powertrain(&powertrain, wheel_radius, &car_state);

        // Expected: 800 * 3 * 1.5 / 0.36 = 10000 N as the Force limited by power goes to infinity when the car is standstill
        assert!(
            (force - 10000.0).abs() < 1e-3,
            "Torque limit guard failed at 0 m/s!"
        );
    }

    #[test]
    fn test_force_from_powertrain_moving() {
        let powertrain = Powertrain::default();
        let car_state = CarState {
            gear: 4,
            ..CarState::default()
        };
        let wheel_radius: f64 = 0.36; // meters (m)

        let force: f64 =
            calculate_force_from_powertrain(&powertrain, wheel_radius, &car_state);

        // Force limited by max torque: 800 * 0.8 * 1.5 / 0.36 =  2666,66667 N
        // Force limited by power: 800_000 / 10 = 80000.0000 N
        // Expected: 800 * 3 * 1.5 / 0.36 =  2666,66667 N
        assert!((force - 2666.66667).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_transmission_input_revs() {
        let powertrain = Powertrain::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let transmission_input_revs =
            calculate_transmission_input_revs(&powertrain, wheel_radius, &car_state);

        // Expected value: 10 * 1.5 * 3 / (2 * PI * 0.36) =  19.8944
        assert!((transmission_input_revs - 19.8944).abs() < 1e-3);
    }

    #[test]
    fn test_select_best_gear_from_lower_gear() {
        let powertrain = Powertrain::default();
        let revs_hz: f64 = 12.5;
        let gear: u8 = 2;

        let best_gear = select_best_gear(&powertrain, revs_hz, gear);

        // As the revs are lower than min revs by more than 0.5: 13.33 - 0.5 > 12.5, then best gear should be shifted down to 1st, it is then 1st gear.
        assert_eq!(best_gear, 1);
    }

    #[test]
    fn test_check_clutch_need() {
        let powertrain = Powertrain::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let clutch_need = check_clutch_need(&powertrain, wheel_radius, &car_state);

        assert!(!clutch_need);
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
        let powertrain = Powertrain::default();
        let wheel_radius: f64 = 0.36;
        let car_state: CarState = CarState::default();

        let force = calculate_force_clutch_limited(&powertrain, wheel_radius, &car_state);

        //As the speed is high enough, then the clutch is engaged, so that means that max engine torque is the same as clutch output torque, so it is same as moving car force: 10'000 N
        assert!(
            (force - 10_000.0).abs() < 1e-3,
            "Test failed, actual value {} but test value {}",
            force,
            10_000.0
        );
    }

    #[test]
    fn test_drivetrain_front_share() {
        // FWD sends everything to the front axle
        assert_eq!(Drivetrain::FrontWheelDrive.front_share(), 1.0);
        // RWD sends nothing to the front axle
        assert_eq!(Drivetrain::RearWheelDrive.front_share(), 0.0);
        // AWD sends the configured split to the front axle
        let all_wheel_drive = Drivetrain::AllWheelDrive {
            front_torque_split: 0.35,
        };
        assert_eq!(all_wheel_drive.front_share(), 0.35);
    }
}
