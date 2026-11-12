//! The driver: how the lap is planned before the run and which speed the car follows
//!
//! Underlying science equations are found in this documentation:
//! [Look-Ahead Braking Documentation](../../docs/17_Look_Ahead_Braking.MD)

use crate::physics::aero::calculate_aero_loads;
use crate::physics::braking::calculate_brake_force;
use crate::physics::car::Car;
use crate::physics::chassis::AxleDistances;
use crate::physics::cornering::{
    calculate_axle_lateral_forces, calculate_constant_speed_axle_loads,
    calculate_lateral_acceleration,
};
use crate::physics::environment::Environment;
use crate::physics::suspension::CornerLoads;
use crate::physics::track::{Track, find_track_segment};
use crate::physics::tyres::{Tyre, calculate_axle_grip, calculate_current_tyre_friction};
use serde::Deserialize;

/// Most rounds of the braking deceleration calculation (docs/17 section 17.4). It normally
/// settles in fewer than 10.
const DECELERATION_MAX_ROUNDS: u32 = 100;

/// The braking deceleration has settled when a round changes it by less than this, in m/s^2
const DECELERATION_TOLERANCE: f64 = 1e-9;

/// Times the backward pass goes around the track. The track is a loop, so the first time
/// around the points before the start/finish line do not know the final speed of the points
/// after it yet (docs/17 section 17.5).
const BACKWARD_PASS_LAPS: u32 = 2;

/// How the driver plans the lap (docs/17). In a run file: the optional `[driver]` table.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Driver {
    /// Fraction of the tyre grip the driver leaves unused when planning corner speeds and
    /// braking (sigma), dimensionless, 0.0 up to but not including 1.0. 0 = plan exactly on
    /// the limit (docs/17 section 17.2).
    pub grip_margin: f64,
}

impl Default for Driver {
    fn default() -> Self {
        Self { grip_margin: 0.0 }
    }
}

/// Makes the tyre the driver plans with: the real tyre with both friction coefficients
/// lowered by the grip margin (docs/17, equation 17.1). Everything else is unchanged.
pub(crate) fn calculate_planning_tyre(tyre: &Tyre, driver: &Driver) -> Tyre {
    let grip_used = 1.0 - driver.grip_margin;

    Tyre {
        mu_x_ref: tyre.mu_x_ref * grip_used,
        mu_y_ref: tyre.mu_y_ref * grip_used,
        reference_load: tyre.reference_load,
        load_sensitivity: tyre.load_sensitivity,
        mu_rolling: tyre.mu_rolling,
        mu_breakaway: tyre.mu_breakaway,
        wheel_radius: tyre.wheel_radius,
    }
}

/// Calculates the deceleration in m/s^2 (positive when slowing down) the car can reach at
/// `speed` in m/s on a line of `curvature` in 1/m, with the tyre `tyre` (docs/17 section 17.4):
/// 1. axle loads at constant speed plus the load braking moves to the front (17.4),
/// 2. the longitudinal grip left after the lateral force the corner needs (friction ellipse),
/// 3. the brake force for the fixed brake bias,
/// 4. deceleration from brake force, drag and rolling resistance (17.5).
///
/// Step 1 needs the deceleration of step 4, so the steps are repeated until it has settled.
pub(crate) fn calculate_braking_deceleration(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    speed: f64,
    curvature: f64,
) -> f64 {
    let constant_speed_loads =
        calculate_constant_speed_axle_loads(car, environment, axle_distances, speed);
    let total_load = constant_speed_loads.total();
    let drag = calculate_aero_loads(&car.aero, environment.air_density, speed).drag;
    let rolling_resistance =
        calculate_current_tyre_friction(total_load, tyre.mu_rolling, tyre.mu_breakaway, speed);
    let axle_lateral_forces = calculate_axle_lateral_forces(
        car.mass.total,
        calculate_lateral_acceleration(speed, curvature),
        axle_distances,
    );
    let wheelbase = axle_distances.front + axle_distances.rear;

    let mut deceleration = 0.0;
    for _ in 0..DECELERATION_MAX_ROUNDS {
        // (17.4): braking moves load to the front, never more than the rear axle carries
        let load_transfer = (car.mass.total * deceleration * car.geometry.cog_height / wheelbase)
            .min(constant_speed_loads.rear);
        let front_load = constant_speed_loads.front + load_transfer;
        let rear_load = constant_speed_loads.rear - load_transfer;
        let tyre_loads = CornerLoads {
            front_left: 0.5 * front_load,
            front_right: 0.5 * front_load,
            rear_left: 0.5 * rear_load,
            rear_right: 0.5 * rear_load,
        };

        let axle_grip = calculate_axle_grip(tyre, &tyre_loads, &axle_lateral_forces);
        let brake_force = calculate_brake_force(&axle_grip, car.brakes.front_bias);

        // (17.5)
        let new_deceleration = (brake_force + drag + rolling_resistance) / car.mass.total;
        let change = (new_deceleration - deceleration).abs();
        deceleration = new_deceleration;
        if change < DECELERATION_TOLERANCE {
            break;
        }
    }

    deceleration
}

/// Calculates the target speed in m/s at every point of the track line: the corner speed limit,
/// lowered before each corner so the car can still brake down to it (backward pass, docs/17
/// section 17.5, equation 17.6). Infinite where the car never has to lift.
///
/// * `tyre` - the tyre the driver plans with (`calculate_planning_tyre`)
/// * `corner_speed_limits` - corner speed limit at every track point, with the same tyre
pub(crate) fn calculate_target_speeds(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    track: &Track,
    corner_speed_limits: &[f64],
) -> Vec<f64> {
    let mut target_speeds = corner_speed_limits.to_vec();

    for _ in 0..BACKWARD_PASS_LAPS {
        lower_target_speeds_for_braking(
            car,
            tyre,
            environment,
            axle_distances,
            track,
            corner_speed_limits,
            &mut target_speeds,
        );
    }

    target_speeds
}

/// Goes once around the track against the driving direction and lowers every target speed to
/// the speed from which the car can still brake down to the target of the next point (docs/17,
/// equation 17.6)
fn lower_target_speeds_for_braking(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    track: &Track,
    corner_speed_limits: &[f64],
    target_speeds: &mut [f64],
) {
    let count = corner_speed_limits.len();
    for point in (0..count).rev() {
        let next = (point + 1) % count;
        let next_speed = target_speeds[next];
        if next_speed.is_infinite() {
            // Nothing to brake for after this point: its own limit stands
            continue;
        }

        // Deceleration at the lower speed and the larger curvature of the segment: a little
        // less than the car really has on it, so the error is on the safe side
        let curvature = track.curvatures[point]
            .abs()
            .max(track.curvatures[next].abs());
        let deceleration = calculate_braking_deceleration(
            car,
            tyre,
            environment,
            axle_distances,
            next_speed,
            curvature,
        );
        let segment_length = calculate_segment_length(track, point);
        let braking_speed = (next_speed * next_speed + 2.0 * deceleration * segment_length).sqrt();

        target_speeds[point] = corner_speed_limits[point].min(braking_speed);
    }
}

/// Length in meters, m, of the track segment from `point` to the next point (the last one goes
/// back to point 0)
fn calculate_segment_length(track: &Track, point: usize) -> f64 {
    let next = point + 1;
    if next == track.distances.len() {
        track.lap_length - track.distances[point]
    } else {
        track.distances[next] - track.distances[point]
    }
}

/// Calculates the corner speed limit in m/s between two track points, `fraction` (0.0..=1.0)
/// of the way from the first to the second (docs/17, equation 17.7).
///
/// The curvature changes linearly between the points and the limit goes with 1 / v^2 = kappa /
/// (mu * g) (docs/16 equation 16.5), so 1 / v^2 is interpolated, not v. A point without a
/// limit (infinite) has 1 / v^2 = 0, like a straight has zero curvature.
pub(crate) fn interpolate_corner_speed_limit(limit: f64, next_limit: f64, fraction: f64) -> f64 {
    let inverse_square = |speed: f64| 1.0 / (speed * speed);
    let interpolated =
        inverse_square(limit) + fraction * (inverse_square(next_limit) - inverse_square(limit));

    if interpolated > 0.0 {
        (1.0 / interpolated).sqrt()
    } else {
        f64::INFINITY
    }
}

/// Calculates the target speed in m/s at `lap_distance` meters from the start/finish line: the
/// lower one of
/// * the corner speed limit there (`interpolate_corner_speed_limit`, docs/17 equation 17.7),
/// * the braking curve towards the next point, on which the square of the speed changes
///   linearly with distance, as it does at constant deceleration (docs/17 equation 17.8). There
///   is none if the next point has no target.
///
/// `lap_distance` may be beyond the lap length (looking ahead across the start/finish line).
/// `segment` is where the search for the track segment starts (`find_track_segment`).
pub(crate) fn calculate_target_speed_at(
    track: &Track,
    corner_speed_limits: &[f64],
    target_speeds: &[f64],
    lap_distance: f64,
    segment: usize,
) -> f64 {
    let (segment, fraction) = find_track_segment(track, lap_distance % track.lap_length, segment);
    let next = (segment + 1) % target_speeds.len();

    let corner_speed_limit = interpolate_corner_speed_limit(
        corner_speed_limits[segment],
        corner_speed_limits[next],
        fraction,
    );

    let speed = target_speeds[segment];
    let next_speed = target_speeds[next];
    let braking_speed = if next_speed.is_infinite() {
        f64::INFINITY
    } else if speed.is_infinite() {
        // Does not come out of the backward pass (a point before a target always has one).
        // If it is ever given, hold the next point's speed: the safe side.
        next_speed
    } else {
        (speed * speed + fraction * (next_speed * next_speed - speed * speed)).sqrt()
    };

    corner_speed_limit.min(braking_speed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::chassis::calculate_axle_distances;
    use crate::physics::cornering::calculate_corner_speed_limits;
    use crate::physics::track::oval_test_track;

    /// Default car with the planning tyre for `grip_margin`
    fn planning_tyre(grip_margin: f64) -> Tyre {
        calculate_planning_tyre(&Tyre::default(), &Driver { grip_margin })
    }

    /// Braking deceleration of `car` with `tyre` in the default environment
    fn deceleration(car: &Car, tyre: &Tyre, speed: f64, curvature: f64) -> f64 {
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        calculate_braking_deceleration(
            car,
            tyre,
            &Environment::default(),
            &axle_distances,
            speed,
            curvature,
        )
    }

    /// A car whose braking is easy to calculate by hand: no aero, CoG on the ground (no load
    /// transfer) and a brake bias equal to its 50/50 weight split
    fn simple_braking_car() -> Car {
        let mut car = Car::default();
        car.aero.downforce_coefficient = 0.0;
        car.aero.drag_coefficient = 0.0;
        car.geometry.cog_height = 0.0;
        car.brakes.front_bias = 0.5;
        car
    }

    /// Corner speed limits and target speeds of `car` with `tyre` on the test oval
    fn oval_plan(car: &Car, tyre: &Tyre) -> (Track, Vec<f64>, Vec<f64>) {
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        let track = oval_test_track();
        let limits =
            calculate_corner_speed_limits(car, tyre, &environment, &axle_distances, &track);
        let targets =
            calculate_target_speeds(car, tyre, &environment, &axle_distances, &track, &limits);
        (track, limits, targets)
    }

    #[test]
    fn test_calculate_planning_tyre() {
        // docs/17 equation 17.1: both friction coefficients times (1 - 0.05), the rest unchanged
        let tyre = Tyre::default();
        let planning = planning_tyre(0.05);
        assert!((planning.mu_x_ref - 0.855).abs() < 1e-12);
        assert!((planning.mu_y_ref - 0.855).abs() < 1e-12);
        assert_eq!(planning.reference_load, tyre.reference_load);
        assert_eq!(planning.load_sensitivity, tyre.load_sensitivity);
        assert_eq!(planning.mu_rolling, tyre.mu_rolling);
        assert_eq!(planning.mu_breakaway, tyre.mu_breakaway);
        assert_eq!(planning.wheel_radius, tyre.wheel_radius);

        // No margin: the car's own tyre
        assert_eq!(planning_tyre(0.0), tyre);
        assert_eq!(Driver::default().grip_margin, 0.0);
    }

    #[test]
    fn test_braking_deceleration_on_straight() {
        // docs/17 section 17.4 table: baseline car, 5 % margin
        let car = Car::default();
        let tyre = planning_tyre(0.05);
        for (speed, expected) in [(20.0, 9.41), (40.0, 11.55), (60.0, 15.07)] {
            let deceleration = deceleration(&car, &tyre, speed, 0.0);
            assert!(
                (deceleration - expected).abs() < 0.01,
                "{} m/s: {}",
                speed,
                deceleration
            );
        }

        // Less grip, less braking
        let no_margin = deceleration(&car, &planning_tyre(0.0), 40.0, 0.0);
        assert!(no_margin > deceleration(&car, &tyre, 40.0, 0.0));
    }

    #[test]
    fn test_braking_deceleration_of_simple_car() {
        // No aero, no load transfer, bias = weight split: both axles reach their grip together,
        // a = mu_x * g + mu_rolling * g = (0.9 + 0.03) * 9.81 = 9.1233 m/s^2 at any speed
        let car = simple_braking_car();
        let tyre = Tyre::default();
        for speed in [5.0, 20.0, 60.0] {
            assert!((deceleration(&car, &tyre, speed, 0.0) - 9.1233).abs() < 1e-9);
        }

        // With the CoG at 0.3 m the load moves forward and the rear, with half of the brake
        // force but less load, limits sooner
        let mut tall_car = simple_braking_car();
        tall_car.geometry.cog_height = 0.3;
        assert!(deceleration(&tall_car, &tyre, 20.0, 0.0) < 9.1233);
    }

    #[test]
    fn test_braking_deceleration_in_corner() {
        // docs/17 section 17.4: at the corner speed limit the limiting axle is fully used
        // sideways, so only drag and rolling resistance slow the car
        let car = Car::default();
        let tyre = Tyre::default();
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        let curvature = 1.0 / 15.0;
        let limit = crate::physics::cornering::calculate_corner_speed_limit(
            &car,
            &tyre,
            &environment,
            &axle_distances,
            curvature,
        );

        let at_limit = deceleration(&car, &tyre, limit, curvature);
        let drag = calculate_aero_loads(&car.aero, environment.air_density, limit).drag;
        let loads = calculate_constant_speed_axle_loads(&car, &environment, &axle_distances, limit);
        let losses_only = (drag + tyre.mu_rolling * loads.total()) / car.mass.total;
        assert!(
            (at_limit - losses_only).abs() < 0.01,
            "{} {}",
            at_limit,
            losses_only
        );

        // Halfway to the limit there is grip left (trail braking), but less than on a straight
        let in_corner = deceleration(&car, &tyre, 0.5 * limit, curvature);
        let on_straight = deceleration(&car, &tyre, 0.5 * limit, 0.0);
        assert!(in_corner > at_limit + 1.0 && in_corner < on_straight);
    }

    #[test]
    fn test_target_speeds_follow_braking_distance() {
        // The simple car brakes at a constant 9.1233 m/s^2, so before the first corner of the
        // oval the plan is docs/17 equation 17.2 exactly: v_i^2 - v_(i+1)^2 = 2 * a * ds
        let car = simple_braking_car();
        let tyre = Tyre::default();
        let (track, limits, targets) = oval_plan(&car, &tyre);

        // Points 60 to 73 are on the straight (no limit of their own) and inside the braking
        for point in 60..73 {
            assert_eq!(limits[point], f64::INFINITY);
            let segment_length = track.distances[point + 1] - track.distances[point];
            let difference = targets[point].powi(2) - targets[point + 1].powi(2);
            assert!(
                (difference - 2.0 * 9.1233 * segment_length).abs() < 1e-6,
                "Point {}: {}",
                point,
                difference
            );
        }

        // In the middle of the corner the target is the corner speed limit, sqrt(mu g R)
        let corner = 75 + 23;
        assert_eq!(targets[corner], limits[corner]);
        assert!((targets[corner] - (0.9 * 9.81 * 15.0_f64).sqrt()).abs() < 0.01);
    }

    #[test]
    fn test_target_speeds_of_baseline_car() {
        let car = Car::default();
        let tyre = planning_tyre(0.05);
        let (track, limits, targets) = oval_plan(&car, &tyre);
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        let count = targets.len();

        for point in 0..count {
            let next = (point + 1) % count;
            // Never above the corner speed limit
            assert!(targets[point] <= limits[point]);
            // A point before a target has one too
            assert!(targets[point].is_finite() || targets[next].is_infinite());
        }

        // The corner asks for 11.31 m/s, and the braking for it reaches back onto the straight:
        // 20 m before the corner the target is well above the corner speed but finite
        assert!((targets[75 + 23] - 11.31).abs() < 0.005);
        assert!(
            targets[55] > 20.0 && targets[55].is_finite(),
            "{}",
            targets[55]
        );
        // docs/17 section 17.4: about 15 m from 20 m/s
        let braking_point = (0..75).rev().find(|&point| targets[point] >= 20.0).unwrap();
        assert!((58..=61).contains(&braking_point), "{}", braking_point);
        // Right after the corner the target is far above anything the car reaches there
        assert!(targets[75 + 47 + 5] > 35.0, "{}", targets[75 + 47 + 5]);

        // Going around once more changes nothing: twice around is enough for the loop
        let mut once_more = targets.clone();
        lower_target_speeds_for_braking(
            &car,
            &tyre,
            &environment,
            &axle_distances,
            &track,
            &limits,
            &mut once_more,
        );
        assert_eq!(once_more, targets);

        // The braking for the first corner does not depend on where the start/finish line is:
        // both corners of the oval are the same, so are the plans half a lap apart
        let half_lap = 75 + 47;
        for point in 40..75 {
            assert!((targets[point] - targets[point + half_lap]).abs() < 1e-9);
        }
    }

    #[test]
    fn test_interpolate_corner_speed_limit() {
        // docs/17 equation 17.7: 1 / v^2 changes linearly
        assert_eq!(interpolate_corner_speed_limit(10.0, 20.0, 0.0), 10.0);
        assert!((interpolate_corner_speed_limit(10.0, 20.0, 1.0) - 20.0).abs() < 1e-12);
        // Halfway: 1 / v^2 = (1 / 100 + 1 / 400) / 2 = 0.00625 -> v = 12.649 m/s
        assert!((interpolate_corner_speed_limit(10.0, 20.0, 0.5) - 12.649110641).abs() < 1e-8);

        // Towards a point without a limit the limit rises and is infinite at that point
        let infinity = f64::INFINITY;
        assert_eq!(interpolate_corner_speed_limit(10.0, infinity, 0.0), 10.0);
        // 1 / v^2 = 0.01 * 0.5 -> v = 14.142 m/s
        assert!((interpolate_corner_speed_limit(10.0, infinity, 0.5) - 14.142135624).abs() < 1e-8);
        assert_eq!(
            interpolate_corner_speed_limit(10.0, infinity, 1.0),
            infinity
        );
        assert_eq!(
            interpolate_corner_speed_limit(infinity, infinity, 0.5),
            infinity
        );
        // And away from one it comes down from infinity
        assert_eq!(
            interpolate_corner_speed_limit(infinity, 10.0, 0.0),
            infinity
        );
        assert!((interpolate_corner_speed_limit(infinity, 10.0, 0.5) - 14.142135624).abs() < 1e-8);
    }

    #[test]
    fn test_calculate_target_speed_at() {
        // Square 10 m x 10 m: points at 0, 10, 20 and 30 m, lap length 40 m
        let square = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let track = Track::from_points(square, 0.0).unwrap();
        let infinity = f64::INFINITY;
        let limits = [infinity, infinity, infinity, 10.0];
        let targets = [infinity, 20.0, 10.0, 10.0];
        let target_at = |lap_distance: f64| {
            calculate_target_speed_at(&track, &limits, &targets, lap_distance, 0)
        };

        // At a point: that point's target
        assert_eq!(target_at(10.0), 20.0);
        assert_eq!(target_at(20.0), 10.0);
        // Braking from 20 to 10 m/s: v^2 is linear, halfway sqrt((400 + 100) / 2) = 15.811 m/s
        assert!((target_at(15.0) - 15.811388301).abs() < 1e-8);
        // Before a point with a target but without one itself: the next point's speed
        assert_eq!(target_at(5.0), 20.0);
        // Braking curve flat at 10 m/s while the corner limit comes down to it: the lower one
        assert!((target_at(25.0) - 10.0).abs() < 1e-12);
        // Leaving the corner: the limit rises along the segment, 1 / v^2 = 0.01 * 0.5
        assert!((target_at(35.0) - 14.142135624).abs() < 1e-8);
        // Looking ahead across the start/finish line: 45 m is 5 m into the next lap
        assert_eq!(target_at(45.0), target_at(5.0));
    }
}
