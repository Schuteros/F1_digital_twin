//! This module contains calculations related to powertrain dynamics.
//!
//! Underlying science equations are found in this documentation:
//! [Powertrain Engine Documentation](../../docs/02_Powertrain_Force_Dynamics.MD)
//! [Clutch Dynamics Documentation](../../docs/08_Clutch_Dynamics.MD)
//! [Torque Curve and Shifting Documentation](../../docs/12_Torque_Curve_and_Shifting.MD)

use serde::Deserialize;
use std::f64::consts::PI;

/// Speed step of the upshift scan in meters per second, m/s (docs/12 section 12.6)
const UPSHIFT_SCAN_STEP: f64 = 0.01;

/// A shift timer left within this fraction of a time step of 0 is floating point noise and
/// counts as 0, so 5 ms at 1 ms steps lasts exactly 5 steps, not 6 (docs/12 section 12.8)
const SHIFT_TIMER_TOLERANCE: f64 = 1e-9;

/// Contains all the variables that define the powertrain
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Powertrain {
    /// Full-throttle torque curve as points of (engine speed in Hertz, Hz, torque in Newton
    /// meters, N·m), sorted by engine speed (docs/12 section 12.2)
    #[serde(rename = "torque_curve_Hz_N_m")]
    pub torque_curve: Vec<(f64, f64)>,
    /// Max rotational speed of the engine (rev limit) in Hertz, Hz
    #[serde(rename = "max_revs_Hz")]
    pub max_revs: f64,
    /// Min rotational speed of the engine before stalling in Hertz, Hz
    #[serde(rename = "min_revs_Hz")]
    pub min_revs: f64,
    /// Gear ratios from largest to smallest, dimensionless
    pub gear_ratios: Vec<f64>,
    /// Final drive ratio, dimensionless
    pub final_drive: f64,
    /// Time without drive during a gear shift in seconds, s (docs/12 section 12.8)
    #[serde(rename = "shift_time_s")]
    pub shift_time: f64,
    /// Downshift hysteresis in meters per second, m/s (docs/12, equation 12.14)
    #[serde(rename = "downshift_hysteresis_m_per_s")]
    pub downshift_hysteresis: f64,
    /// Optimal upshift speed from each gear to the next in meters per second, m/s, one per gear
    /// except the top gear (docs/12 section 12.6). Empty in Default and in a loaded config,
    /// filled by `SimulationConfig::prepare` before the simulation starts.
    #[serde(skip)]
    pub upshift_speeds: Vec<f64>,
    /// Speed above which the clutch stays locked in each gear in meters per second, m/s, one
    /// per gear (docs/12, equation 12.8). Empty in Default and in a loaded config, filled by
    /// `SimulationConfig::prepare` before the simulation starts.
    #[serde(skip)]
    pub clutch_lock_speeds: Vec<f64>,
    /// Which axles the powertrain drives
    pub drivetrain: Drivetrain,
}

/// Defines which axles receive the powertrain force
// Full names are kept for readability.
#[allow(clippy::enum_variant_names)]
#[derive(Debug, PartialEq, Deserialize)]
#[serde(from = "DrivetrainFile")]
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

/// `Drivetrain` as it is written in a car file: a table with `type` and, for all wheel drive,
/// `front_torque_split`.
///
/// Serde ignores extra keys on unit variants of a tagged enum, even with `deny_unknown_fields`,
/// so the file variants are empty structs. That way a `front_torque_split` on a rear wheel
/// drive car is an error instead of a silently ignored key.
#[allow(clippy::enum_variant_names)]
#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum DrivetrainFile {
    /// All powertrain force goes to the front axle
    FrontWheelDrive {},
    /// All powertrain force goes to the rear axle
    RearWheelDrive {},
    /// Powertrain force is split between both axles
    AllWheelDrive {
        /// Fraction of the powertrain force sent to the front axle, dimensionless, 0.0..=1.0
        front_torque_split: f64,
    },
}

impl From<DrivetrainFile> for Drivetrain {
    /// Converts the file form into the `Drivetrain` used by the simulation
    fn from(file: DrivetrainFile) -> Self {
        match file {
            DrivetrainFile::FrontWheelDrive {} => Drivetrain::FrontWheelDrive,
            DrivetrainFile::RearWheelDrive {} => Drivetrain::RearWheelDrive,
            DrivetrainFile::AllWheelDrive { front_torque_split } => {
                Drivetrain::AllWheelDrive { front_torque_split }
            }
        }
    }
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
            // Estimates for a 2026-style F1 engine without the MGU-K (docs/12 section 12.9).
            // Placeholder values: replace with sourced ones.
            torque_curve: vec![
                (66.67, 300.0),
                (100.0, 360.0),
                (133.33, 375.0),
                (175.0, 364.0),
                (200.0, 318.0),
                (216.67, 250.0),
            ],
            max_revs: 216.67,
            min_revs: 66.67,
            gear_ratios: vec![5.43, 4.02, 3.29, 2.78, 2.41, 2.13, 1.87, 1.65],
            final_drive: 3.0,
            shift_time: 0.005,
            downshift_hysteresis: 0.5,
            upshift_speeds: Vec::new(),
            clutch_lock_speeds: Vec::new(),
            drivetrain: Drivetrain::RearWheelDrive,
        }
    }
}

/// Peak of the full-throttle torque curve (docs/12 section 12.1)
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PeakTorque {
    /// Highest torque on the curve (T_pk) in Newton meters, N·m
    pub torque: f64,
    /// Engine speed where the peak torque occurs (n_pk) in Hertz, Hz
    pub revs_hz: f64,
}

/// Calculates the total ratio from the engine to the wheel, G_k = i_k * i_f, dimensionless
/// (docs/12 section 12.1). `gear` starts at 1.
pub(crate) fn calculate_total_ratio(powertrain: &Powertrain, gear: u8) -> f64 {
    powertrain.gear_ratios[(gear - 1) as usize] * powertrain.final_drive
}

/// Calculates full-throttle engine torque in Newton meters, N·m, at `revs_hz` in Hertz, Hz,
/// by linear interpolation between the torque curve points (docs/12, equation 12.1).
///
/// Below the first point the first point's torque is used (the clutch slips there anyway,
/// docs/12 section 12.5). Above `max_revs` the rev limiter cuts the engine, so the torque is 0.
pub(crate) fn calculate_engine_torque(powertrain: &Powertrain, revs_hz: f64) -> f64 {
    if revs_hz > powertrain.max_revs {
        return 0.0;
    }

    let curve = &powertrain.torque_curve;
    match curve
        .iter()
        .position(|&(point_revs, _)| point_revs >= revs_hz)
    {
        Some(0) => curve[0].1,
        Some(index) => {
            let (revs_above, torque_above) = curve[index];
            if revs_above == revs_hz {
                // Exactly on a point: return its torque without interpolation rounding
                return torque_above;
            }

            let (revs_below, torque_below) = curve[index - 1];
            torque_below
                + (torque_above - torque_below) * (revs_hz - revs_below) / (revs_above - revs_below)
        }
        // Past the last point. A validated curve reaches max_revs, so this only guards bad data.
        None => curve.last().map_or(0.0, |&(_, torque)| torque),
    }
}

/// Finds the peak torque (T_pk) and its engine speed (n_pk) on the torque curve
/// (docs/12 section 12.1).
///
/// The curve is linear between points, so the peak is always on a point. Points above
/// `max_revs` are skipped, because the rev limiter cuts the engine there (docs/12, equation 12.1).
/// If two points share the peak, the lower engine speed is used.
pub(crate) fn calculate_peak_torque(powertrain: &Powertrain) -> PeakTorque {
    let mut peak = PeakTorque {
        torque: 0.0,
        revs_hz: 0.0,
    };

    for &(revs_hz, torque) in &powertrain.torque_curve {
        if revs_hz <= powertrain.max_revs && torque > peak.torque {
            peak = PeakTorque { torque, revs_hz };
        }
    }

    peak
}

/// Calculates transmission input shaft rotational speed in Hertz, Hz, if the clutch is locked:
/// the engine speed n_k that `gear` forces at `speed` in meters per second, m/s
/// (docs/12, equation 12.2)
pub(crate) fn calculate_transmission_input_revs(
    powertrain: &Powertrain,
    wheel_radius: f64,
    gear: u8,
    speed: f64,
) -> f64 {
    if wheel_radius <= 0.0 || gear == 0 {
        return 0.0;
    }

    speed * calculate_total_ratio(powertrain, gear) / (2.0 * PI * wheel_radius)
}

/// Calculates the speed in meters per second, m/s, at which `gear` reaches the rev limit
/// (docs/12, equation 12.13)
pub(crate) fn calculate_limiter_speed(powertrain: &Powertrain, wheel_radius: f64, gear: u8) -> f64 {
    2.0 * PI * wheel_radius * powertrain.max_revs / calculate_total_ratio(powertrain, gear)
}

/// Calculates the best powertrain force at the contact patch in Newtons, N, in `gear` at
/// `speed` in meters per second, m/s (docs/12, equation 12.10):
/// * below the clutch lock speed the clutch slips with the engine at peak torque (12.7),
/// * above it the clutch is locked and the engine follows the gearing (12.3),
/// * above the rev limit the force is 0.
///
/// Needs `clutch_lock_speeds` filled (`SimulationConfig::prepare`). The tyre grip limit is
/// applied separately (forces.rs).
pub(crate) fn calculate_best_force(
    powertrain: &Powertrain,
    wheel_radius: f64,
    gear: u8,
    speed: f64,
) -> f64 {
    if wheel_radius <= 0.0 || gear == 0 {
        return 0.0;
    }

    let locked_revs = calculate_transmission_input_revs(powertrain, wheel_radius, gear, speed);
    if locked_revs > powertrain.max_revs {
        return 0.0;
    }

    let clutch_lock_speed = powertrain.clutch_lock_speeds[(gear - 1) as usize];
    let engine_torque = if speed < clutch_lock_speed {
        calculate_peak_torque(powertrain).torque
    } else {
        calculate_engine_torque(powertrain, locked_revs)
    };

    engine_torque * calculate_total_ratio(powertrain, gear) / wheel_radius
}

/// Calculates the clutch lock speed of every gear in meters per second, m/s: the speed where
/// the locked engine speed reaches max(n_pk, min_revs) (docs/12, equation 12.8)
pub(crate) fn calculate_clutch_lock_speeds(powertrain: &Powertrain, wheel_radius: f64) -> Vec<f64> {
    let lock_revs = calculate_peak_torque(powertrain)
        .revs_hz
        .max(powertrain.min_revs);

    powertrain
        .gear_ratios
        .iter()
        .map(|gear_ratio| {
            2.0 * PI * wheel_radius * lock_revs / (gear_ratio * powertrain.final_drive)
        })
        .collect()
}

/// Calculates the optimal upshift speed of every gear except the top gear in meters per
/// second, m/s, by the scan in docs/12 section 12.6.
///
/// Needs `clutch_lock_speeds` filled first, the scan starts there and uses the best force (12.10).
pub(crate) fn calculate_upshift_speeds(powertrain: &Powertrain, wheel_radius: f64) -> Vec<f64> {
    let top_gear = powertrain.gear_ratios.len() as u8;

    (1..top_gear)
        .map(|gear| calculate_upshift_speed(powertrain, wheel_radius, gear))
        .collect()
}

/// Scans for the upshift speed from `gear` to `gear + 1` in meters per second, m/s
/// (docs/12 section 12.6): from the clutch lock speed of `gear` up to its limiter speed, the
/// first speed where `gear + 1` gives at least as much force (12.11). If there is none, the
/// limiter speed (12.13).
fn calculate_upshift_speed(powertrain: &Powertrain, wheel_radius: f64, gear: u8) -> f64 {
    let start_speed = powertrain.clutch_lock_speeds[(gear - 1) as usize];
    let limiter_speed = calculate_limiter_speed(powertrain, wheel_radius, gear);

    // Speed = start + step count * scan step, so the 0.01 m/s steps don't add up rounding error
    let mut step_count: u32 = 0;
    loop {
        let speed = start_speed + step_count as f64 * UPSHIFT_SCAN_STEP;
        if speed > limiter_speed {
            return limiter_speed;
        }

        let force_current = calculate_best_force(powertrain, wheel_radius, gear, speed);
        let force_next = calculate_best_force(powertrain, wheel_radius, gear + 1, speed);
        if force_next >= force_current {
            return speed;
        }

        step_count += 1;
    }
}

/// Selects the gear for this step, at most one gear away from `gear` (docs/12 sections 12.6
/// and 12.7):
/// * upshift when `speed` >= upshift_speeds[k] (12.11),
/// * downshift only when not braking and `speed` < upshift_speeds[k - 1] - downshift_hysteresis
///   (12.14).
///
/// `speed` is in meters per second, m/s. Needs `upshift_speeds` filled (`SimulationConfig::prepare`).
pub(crate) fn select_gear(powertrain: &Powertrain, gear: u8, speed: f64, braking: bool) -> u8 {
    if gear == 0 {
        return gear;
    }

    let gear_index = (gear - 1) as usize;
    let top_gear = powertrain.gear_ratios.len() as u8;

    if gear < top_gear && speed >= powertrain.upshift_speeds[gear_index] {
        gear + 1
    } else if !braking
        && gear > 1
        && speed < powertrain.upshift_speeds[gear_index - 1] - powertrain.downshift_hysteresis
    {
        gear - 1
    } else {
        gear
    }
}

/// Counts the shift timer down by one `time_step`, both in seconds, s (docs/12 section 12.8).
///
/// Never goes below 0. A remainder within floating point noise of 0 counts as 0, so a shift
/// that is a whole number of steps lasts exactly that many steps. Otherwise it lasts until the
/// next whole step (5.4 ms at 1 ms steps is 6 steps), so the drive is cut for at least `shift_time`.
pub(crate) fn count_down_shift_time(shift_time_left: f64, time_step: f64) -> f64 {
    let remaining = shift_time_left - time_step;

    if remaining <= SHIFT_TIMER_TOLERANCE * time_step {
        0.0
    } else {
        remaining
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Wheel radius of the default tyre in meters, m
    const WHEEL_RADIUS: f64 = 0.36;

    /// Default powertrain with both shift tables filled for the default 0.36 m wheel
    fn prepared_powertrain() -> Powertrain {
        let mut powertrain = Powertrain::default();
        powertrain.clutch_lock_speeds = calculate_clutch_lock_speeds(&powertrain, WHEEL_RADIUS);
        powertrain.upshift_speeds = calculate_upshift_speeds(&powertrain, WHEEL_RADIUS);
        powertrain
    }

    #[test]
    fn test_engine_torque_interpolation() {
        let powertrain = Powertrain::default();

        // Exact curve points return their own torque, no interpolation rounding
        for &(revs_hz, torque) in &powertrain.torque_curve {
            assert_eq!(calculate_engine_torque(&powertrain, revs_hz), torque);
        }

        // 150 Hz lies between (133.33, 375) and (175, 364):
        // 375 + (364 - 375) * (150 - 133.33) / (175 - 133.33) = 375 - 11 * 16.67 / 41.67
        // = 370.599472 N·m. 1e-6 N·m covers rounding of a few operations on ~400 N·m.
        let torque = calculate_engine_torque(&powertrain, 150.0);
        assert!((torque - 370.599472).abs() < 1e-6, "Torque: {}", torque);

        // Below the first point (66.67 Hz): the first point's torque, 300 N·m
        assert_eq!(calculate_engine_torque(&powertrain, 50.0), 300.0);

        // Above max_revs (216.67 Hz) the rev limiter cuts the engine: 0 N·m
        assert_eq!(calculate_engine_torque(&powertrain, 220.0), 0.0);
    }

    #[test]
    fn test_peak_torque() {
        // The highest point of the default curve is (133.33 Hz, 375 N·m)
        let peak = calculate_peak_torque(&Powertrain::default());

        assert_eq!(
            peak,
            PeakTorque {
                torque: 375.0,
                revs_hz: 133.33
            }
        );
    }

    #[test]
    fn test_calculate_transmission_input_revs() {
        let powertrain = Powertrain::default();

        let transmission_input_revs =
            calculate_transmission_input_revs(&powertrain, WHEEL_RADIUS, 1, 10.0);

        // Expected value: 10 * 5.43 * 3.0 / (2 * PI * 0.36) = 162.9 / 2.2619467 = 72.017612 Hz
        // 1e-6 Hz covers rounding of a few operations on ~100 Hz
        assert!(
            (transmission_input_revs - 72.017612).abs() < 1e-6,
            "Revs: {}",
            transmission_input_revs
        );

        // No gear, no input speed
        assert_eq!(
            calculate_transmission_input_revs(&powertrain, WHEEL_RADIUS, 0, 10.0),
            0.0
        );
    }

    #[test]
    fn test_clutch_lock_speeds() {
        let powertrain = Powertrain::default();

        let clutch_lock_speeds = calculate_clutch_lock_speeds(&powertrain, WHEEL_RADIUS);

        // One per gear
        assert_eq!(clutch_lock_speeds.len(), 8);
        // max(n_pk, n_min) = max(133.33, 66.67) = 133.33 Hz
        // Gear 1: 2 * PI * 0.36 * 133.33 / (5.43 * 3.0) = 301.5875 / 16.29 = 18.5135 m/s
        // Gear 2: 301.5875 / (4.02 * 3.0) = 301.5875 / 12.06 = 25.0071 m/s
        // The expected values are rounded to 3 decimals (+-0.0005), so 0.001 covers them
        assert!(
            (clutch_lock_speeds[0] - 18.514).abs() < 1e-3,
            "Gear 1: {}",
            clutch_lock_speeds[0]
        );
        assert!(
            (clutch_lock_speeds[1] - 25.007).abs() < 1e-3,
            "Gear 2: {}",
            clutch_lock_speeds[1]
        );
    }

    #[test]
    fn test_upshift_speeds_match_docs() {
        let powertrain = prepared_powertrain();

        // docs/12 section 12.6, one per gear except the top gear
        let expected = [29.30, 38.37, 46.38, 54.39, 61.94, 70.50, 79.95];
        assert_eq!(powertrain.upshift_speeds.len(), expected.len());

        // Expected values are rounded to 2 decimals (+-0.005) and the scan stops up to one
        // 0.01 m/s step past the exact crossing, so they agree within 0.015 < 0.02 m/s
        for (gear_index, (&speed, &expected_speed)) in
            powertrain.upshift_speeds.iter().zip(&expected).enumerate()
        {
            assert!(
                (speed - expected_speed).abs() < 0.02,
                "Shift {} -> {}: {} m/s, expected {} m/s",
                gear_index + 1,
                gear_index + 2,
                speed,
                expected_speed
            );
        }
    }

    #[test]
    fn test_upshift_speeds_never_crossing() {
        // Flat torque: power 2 * PI * n * T rises with engine speed all the way to the limiter,
        // so at any speed the lower gear (higher engine speed) gives more force. The curves never
        // cross, so every upshift is at the limiter speed of its gear (docs/12, equation 12.13).
        let mut powertrain = Powertrain {
            torque_curve: vec![(66.67, 375.0), (216.67, 375.0)],
            ..Powertrain::default()
        };
        powertrain.clutch_lock_speeds = calculate_clutch_lock_speeds(&powertrain, WHEEL_RADIUS);
        powertrain.upshift_speeds = calculate_upshift_speeds(&powertrain, WHEEL_RADIUS);

        assert_eq!(powertrain.upshift_speeds.len(), 7);
        for (gear_index, &speed) in powertrain.upshift_speeds.iter().enumerate() {
            let gear = gear_index as u8 + 1;
            // The scan returns the limiter speed itself, so they are exactly equal
            assert_eq!(
                speed,
                calculate_limiter_speed(&powertrain, WHEEL_RADIUS, gear),
                "Gear {}",
                gear
            );
        }
    }

    #[test]
    fn test_best_force_slip_below_lock_speed() {
        let powertrain = prepared_powertrain();

        // Gear 1 below its lock speed (18.514 m/s): clutch slips, engine at peak torque
        // 375 * 5.43 * 3.0 / 0.36 = 375 * 16.29 / 0.36 = 16968.75 N, at rest and at 5 m/s.
        // 1e-9 relative covers rounding of 5.43 * 3.0 and the division.
        for speed in [0.0, 5.0] {
            let force = calculate_best_force(&powertrain, WHEEL_RADIUS, 1, speed);
            assert!(
                (force - 16968.75).abs() < 1e-9 * 16968.75,
                "Force at {} m/s: {}",
                speed,
                force
            );
        }
    }

    #[test]
    fn test_best_force_moving() {
        let powertrain = prepared_powertrain();

        // Gear 4 at 10 m/s, below its lock speed 301.5875 / 8.34 = 36.16 m/s, so it slips:
        // 375 * 2.78 * 3.0 / 0.36 = 375 * 8.34 / 0.36 = 8687.5 N (1e-9 relative: rounding only)
        let force = calculate_best_force(&powertrain, WHEEL_RADIUS, 4, 10.0);

        assert!((force - 8687.5).abs() < 1e-9 * 8687.5, "Force: {}", force);
    }

    #[test]
    fn test_best_force_locked() {
        let powertrain = prepared_powertrain();

        // Gear 3 with the engine at 150 Hz: v = 150 * 2 * PI * 0.36 / 9.87 = 34.376 m/s,
        // above the gear 3 lock speed 301.5875 / 9.87 = 30.556 m/s, so the clutch is locked.
        // Force = T(150 Hz) * 9.87 / 0.36 = 370.599472 * 9.87 / 0.36 = 10160.6022 N
        let speed = 150.0 * 2.0 * PI * WHEEL_RADIUS / (3.29 * 3.0);
        let force = calculate_best_force(&powertrain, WHEEL_RADIUS, 3, speed);

        // The expected value is rounded to 4 decimals (+-5e-5 N), so 1e-4 N covers it
        assert!((force - 10160.6022).abs() < 1e-4, "Force: {}", force);
    }

    #[test]
    fn test_best_force_above_limiter() {
        let powertrain = prepared_powertrain();

        // Gear 1 limiter speed = 2 * PI * 0.36 * 216.67 / 16.29 = 30.086 m/s
        assert_eq!(
            calculate_best_force(&powertrain, WHEEL_RADIUS, 1, 31.0),
            0.0
        );
        // Top gear limiter speed = 2 * PI * 0.36 * 216.67 / 4.95 = 99.009 m/s
        assert_eq!(
            calculate_best_force(&powertrain, WHEEL_RADIUS, 8, 100.0),
            0.0
        );
    }

    #[test]
    fn test_force_equals_power_over_speed() {
        let powertrain = prepared_powertrain();

        // Locked gear / speed pairs, all between the lock speed and the limiter speed:
        // gear 1 at 25 m/s -> 180.0 Hz, gear 3 at 40 m/s -> 174.5 Hz,
        // gear 5 at 60 m/s -> 191.8 Hz, gear 8 at 90 m/s -> 196.9 Hz
        for (gear, speed) in [(1, 25.0), (3, 40.0), (5, 60.0), (8, 90.0)] {
            assert!(speed >= powertrain.clutch_lock_speeds[gear as usize - 1]);

            let force = calculate_best_force(&powertrain, WHEEL_RADIUS, gear, speed);
            let revs_hz = calculate_transmission_input_revs(&powertrain, WHEEL_RADIUS, gear, speed);
            let power = 2.0 * PI * revs_hz * calculate_engine_torque(&powertrain, revs_hz);

            // docs/12, equation 12.5: F * v = P(n_k). Both sides are the same product in a
            // different order, so they agree to rounding, 1e-9 relative.
            assert!(
                (force * speed - power).abs() < 1e-9 * power,
                "Gear {} at {} m/s: F * v = {} W, P = {} W",
                gear,
                speed,
                force * speed,
                power
            );
        }
    }

    #[test]
    fn test_select_gear_upshift() {
        let powertrain = prepared_powertrain();
        let upshift_1_to_2 = powertrain.upshift_speeds[0];

        // Just below the 1 -> 2 upshift speed: stay in 1st
        assert_eq!(select_gear(&powertrain, 1, upshift_1_to_2 - 0.01, false), 1);
        // At the upshift speed: shift to 2nd, also while braking
        assert_eq!(select_gear(&powertrain, 1, upshift_1_to_2, false), 2);
        assert_eq!(select_gear(&powertrain, 1, upshift_1_to_2, true), 2);
        // One gear per call, even far above several upshift speeds
        assert_eq!(select_gear(&powertrain, 1, 90.0, false), 2);
        // Top gear can't go higher
        assert_eq!(select_gear(&powertrain, 8, 98.0, false), 8);
    }

    #[test]
    fn test_select_gear_downshift() {
        let powertrain = prepared_powertrain();
        // 2 -> 3 upshift is at 38.37 m/s, so 3rd drops to 2nd below 38.37 - 0.5 = 37.87 m/s
        let downshift_speed = powertrain.upshift_speeds[1] - powertrain.downshift_hysteresis;

        // Inside the hysteresis band: stay in 3rd
        assert_eq!(
            select_gear(&powertrain, 3, downshift_speed + 0.01, false),
            3
        );
        // Below the band: down to 2nd
        assert_eq!(
            select_gear(&powertrain, 3, downshift_speed - 0.01, false),
            2
        );
        // Never while braking
        assert_eq!(select_gear(&powertrain, 3, downshift_speed - 0.01, true), 3);
        // One gear per call, even at rest
        assert_eq!(select_gear(&powertrain, 8, 0.0, false), 7);
        // 1st can't go lower
        assert_eq!(select_gear(&powertrain, 1, 0.0, false), 1);
    }

    #[test]
    fn test_count_down_shift_time() {
        // 5 ms at 1 ms steps: still running after 4 steps, exactly 0 after 5
        let mut shift_time_left = 0.005;
        for _ in 0..4 {
            shift_time_left = count_down_shift_time(shift_time_left, 0.001);
            assert!(shift_time_left > 0.0);
        }
        shift_time_left = count_down_shift_time(shift_time_left, 0.001);
        assert_eq!(shift_time_left, 0.0);

        // 5.4 ms at 1 ms steps: 0.4 ms left after 5 steps, 0 after the 6th
        let mut shift_time_left = 0.0054;
        for _ in 0..5 {
            shift_time_left = count_down_shift_time(shift_time_left, 0.001);
        }
        assert!(shift_time_left > 0.0);
        assert_eq!(count_down_shift_time(shift_time_left, 0.001), 0.0);

        // Never below 0
        assert_eq!(count_down_shift_time(0.0, 0.001), 0.0);
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
