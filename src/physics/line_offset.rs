//! Running wide: the car's sideways offset from the line, its heading error, how the driver
//! steers back and what the car does over the grip limit
//!
//! Underlying science equations are found in this documentation:
//! [Running Wide and Line Offset Documentation](../../docs/20_Running_Wide_and_Line_Offset.MD)

use crate::physics::cornering::AxleLateralForces;
use crate::physics::suspension::CornerLoads;
use crate::physics::tyres::{Tyre, calculate_max_tyre_force};

/// Return length L_back in meters, m: roughly how far ahead the driver looks to get back to the
/// line (docs/20 section 20.4). The offset is about halved after 1.7 L_back and nearly gone
/// after 4 L_back.
pub(crate) const RETURN_LENGTH: f64 = 5.0;

/// Smallest value of 1 - kappa * n in equation 20.3. It only matters for a car further inside a
/// corner than its centre, which the margin keeps far away; it stops a division by 0.
const MIN_LANE_FACTOR: f64 = 0.1;

/// Largest heading error in rad, 60 degrees. The offset and heading error describe the car
/// from the closest point of the line, which only works while the car still moves along the
/// line. A car that would turn further away has long left the margin, so its lap is invalid
/// anyway (docs/20 section 20.3).
const MAX_HEADING_ERROR: f64 = std::f64::consts::FRAC_PI_3;

/// Calculates the path curvature in 1/m the driver asks for: the curvature of a path parallel
/// to the line at the offset `offset` in m, kappa / (1 - kappa * n), plus a smooth turn back to
/// the line from the offset and the heading error `heading_error` in rad (docs/20, equation
/// 20.6). Positive = turning left.
pub(crate) fn calculate_wanted_curvature(curvature: f64, offset: f64, heading_error: f64) -> f64 {
    let lane_factor = (1.0 - curvature * offset).max(MIN_LANE_FACTOR);
    curvature / lane_factor
        - 2.0 * heading_error / RETURN_LENGTH
        - offset / (RETURN_LENGTH * RETURN_LENGTH)
}

/// Calculates the factor the driver lowers the target speed by while turning back to the line
/// needs a tighter turn than the line itself: kappa / kappa_want. The turn back then needs
/// kappa / kappa_want of the lateral acceleration the line needs at the target speed, so on the
/// limit there is grip to spare for getting back (docs/20 section 20.4). 1 on a straight and
/// when the turn back is not tighter than the line.
pub(crate) fn calculate_return_speed_factor(curvature: f64, wanted_curvature: f64) -> f64 {
    if curvature == 0.0 || wanted_curvature * curvature <= 0.0 {
        return 1.0;
    }
    (curvature / wanted_curvature).min(1.0)
}

/// Calculates how much of the lateral force `axle_lateral_forces` in Newtons, N, the tyres can
/// make: the smaller of the two axles' max lateral force / the force asked of it (docs/20,
/// equation 20.8). 1 or more if both fit, infinite if no lateral force is asked for.
pub(crate) fn calculate_lateral_grip_ratio(
    tyre: &Tyre,
    tyre_loads: &CornerLoads,
    axle_lateral_forces: &AxleLateralForces,
) -> f64 {
    let ratio = |load_left: f64, load_right: f64, axle_lateral_force: f64| {
        let max_lateral = calculate_max_tyre_force(tyre, tyre.mu_y_ref, load_left)
            + calculate_max_tyre_force(tyre, tyre.mu_y_ref, load_right);
        if axle_lateral_force == 0.0 {
            f64::INFINITY
        } else {
            max_lateral / axle_lateral_force.abs()
        }
    };

    ratio(
        tyre_loads.front_left,
        tyre_loads.front_right,
        axle_lateral_forces.front,
    )
    .min(ratio(
        tyre_loads.rear_left,
        tyre_loads.rear_right,
        axle_lateral_forces.rear,
    ))
}

/// Calculates the share of the lateral grip left for turning while the driver brakes with the
/// share `braking_share` of the longitudinal grip: sqrt(1 - beta^2) (docs/20, equation 20.10)
pub(crate) fn calculate_turning_share(braking_share: f64) -> f64 {
    (1.0 - braking_share * braking_share).max(0.0).sqrt()
}

/// Calculates how fast the car moves along the line in m/s (ds/dt): the part of its speed
/// `speed` in m/s along the line, on a circle 1 - kappa * n times the line's (docs/20,
/// equation 20.3)
pub(crate) fn calculate_line_progress_speed(
    speed: f64,
    curvature: f64,
    offset: f64,
    heading_error: f64,
) -> f64 {
    let lane_factor = (1.0 - curvature * offset).max(MIN_LANE_FACTOR);
    speed * heading_error.cos() / lane_factor
}

/// Offset from the line and heading error of the car after one time step
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LinePosition {
    /// Offset n in meters, m, positive = left of the line
    pub offset: f64,
    /// Heading error psi_e in rad, positive = pointing left of the line
    pub heading_error: f64,
}

/// Moves the offset and the heading error on by one time step `time_step` in s (docs/20,
/// equations 20.4 and 20.5), semi-implicit like the rest of the step: the heading error first,
/// then the offset with the new heading error.
///
/// * `speed` - speed after the step in m/s
/// * `path_curvature` - curvature of the path the tyres give the car, kappa_car, in 1/m
/// * `curvature` - curvature of the line at the car in 1/m
/// * `progress_speed` - speed along the line in m/s (equation 20.3)
pub(crate) fn update_line_position(
    position: LinePosition,
    speed: f64,
    path_curvature: f64,
    curvature: f64,
    progress_speed: f64,
    time_step: f64,
) -> LinePosition {
    let heading_error = (position.heading_error
        + (speed * path_curvature - curvature * progress_speed) * time_step)
        .clamp(-MAX_HEADING_ERROR, MAX_HEADING_ERROR);
    LinePosition {
        offset: position.offset + speed * heading_error.sin() * time_step,
        heading_error,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ON_LINE: LinePosition = LinePosition {
        offset: 0.0,
        heading_error: 0.0,
    };

    /// Drives `distance` m at constant `speed` along a line of `curvature` with the tyres
    /// giving the path curvature `path_curvature(position)`, and gives the end position
    fn drive(
        start: LinePosition,
        speed: f64,
        curvature: f64,
        distance: f64,
        path_curvature: impl Fn(LinePosition) -> f64,
    ) -> LinePosition {
        let time_step = 1e-4;
        let mut position = start;
        let mut travelled = 0.0;
        while travelled < distance {
            let progress = calculate_line_progress_speed(
                speed,
                curvature,
                position.offset,
                position.heading_error,
            );
            position = update_line_position(
                position,
                speed,
                path_curvature(position),
                curvature,
                progress,
                time_step,
            );
            travelled += progress * time_step;
        }
        position
    }

    #[test]
    fn test_wanted_curvature() {
        // On the line the driver asks for the line's curvature
        assert_eq!(calculate_wanted_curvature(0.1429, 0.0, 0.0), 0.1429);
        // 0.5 m right of a straight: 0.5 / 25 = 0.02 1/m back to the left (docs/20 section 20.4)
        assert!((calculate_wanted_curvature(0.0, -0.5, 0.0) - 0.02).abs() < 1e-12);
        // Already pointing back left at 0.05 rad: 0.02 - 2 * 0.05 / 5 = 0.0 1/m
        assert!(calculate_wanted_curvature(0.0, -0.5, 0.05).abs() < 1e-12);
    }

    #[test]
    fn test_return_speed_factor() {
        // Turning back needs 2 % more curvature than the line: 1 / 1.02 = 0.980 of the target
        // speed, so a_y = 0.980^2 * 1.02 = 0.980 of the line's
        let factor = calculate_return_speed_factor(0.1, 0.102);
        assert!((factor - 1.0 / 1.02).abs() < 1e-12);
        // Wider than the line, a straight, or turning the other way: no change
        assert_eq!(calculate_return_speed_factor(0.1, 0.09), 1.0);
        assert_eq!(calculate_return_speed_factor(0.0, 0.02), 1.0);
        assert_eq!(calculate_return_speed_factor(0.1, -0.01), 1.0);
    }

    #[test]
    fn test_turning_share() {
        // 30 % braking leaves sqrt(1 - 0.09) = 0.954 of the lateral grip (docs/20 section 20.6)
        assert!((calculate_turning_share(0.3) - 0.953_939).abs() < 1e-6);
        assert_eq!(calculate_turning_share(0.0), 1.0);
        assert_eq!(calculate_turning_share(1.0), 0.0);
    }

    #[test]
    fn test_lateral_grip_ratio() {
        let tyre = Tyre {
            load_sensitivity: 0.0,
            ..Tyre::default()
        };
        let loads = CornerLoads {
            front_left: 1000.0,
            front_right: 1000.0,
            rear_left: 1000.0,
            rear_right: 1000.0,
        };
        let max_axle = 2.0 * calculate_max_tyre_force(&tyre, tyre.mu_y_ref, 1000.0);
        // The rear asks for twice its max, the front for half: the rear limits, 0.5
        let forces = AxleLateralForces {
            front: 0.5 * max_axle,
            rear: -2.0 * max_axle,
        };
        assert!((calculate_lateral_grip_ratio(&tyre, &loads, &forces) - 0.5).abs() < 1e-12);
        // Nothing asked: no limit
        let none = AxleLateralForces::default();
        assert_eq!(
            calculate_lateral_grip_ratio(&tyre, &loads, &none),
            f64::INFINITY
        );
    }

    #[test]
    fn test_lane_effect() {
        // R 7 m left turn, 0.5 m outside: 1 - (1/7) * (-0.5) = 1.0714, so 7 % slower along the
        // line (docs/20 section 20.3)
        let progress = calculate_line_progress_speed(8.0, 1.0 / 7.0, -0.5, 0.0);
        assert!((progress - 8.0 / (1.0 + 0.5 / 7.0)).abs() < 1e-12);
        // On the line: the speed itself
        assert_eq!(calculate_line_progress_speed(8.0, 1.0 / 7.0, 0.0, 0.0), 8.0);
    }

    #[test]
    fn test_on_line_stays_on_line() {
        // Exactly on the line with the tyres giving the line's curvature: no offset
        let end = drive(ON_LINE, 7.89, 1.0 / 7.0, 20.0, |_| 1.0 / 7.0);
        assert!(end.offset.abs() < 1e-9, "{:?}", end);
        assert!(end.heading_error.abs() < 1e-9, "{:?}", end);
    }

    #[test]
    fn test_offset_grows_as_half_delta_kappa_s_squared() {
        // docs/20 section 20.2: line R 7 m, the car only manages R 7.78 m. After 2 m the
        // offset is about 0.5 * 0.0144 * 4 = 0.029 m, outward (negative in a left turn).
        let curvature = 1.0 / 7.0;
        let path_curvature = 8.0 / (7.89 * 7.89);
        let delta = curvature - path_curvature;
        let end = drive(ON_LINE, 7.89, curvature, 2.0, |_| path_curvature);
        let expected = -0.5 * delta * 4.0;
        assert!(
            (end.offset - expected).abs() < 0.01 * expected.abs(),
            "{} vs {}",
            end.offset,
            expected
        );
    }

    #[test]
    fn test_return_to_the_line() {
        // 0.5 m right of a straight, the driver steers back with equation 20.6: no overshoot
        // past the line, less than 5 cm left after 20 m
        let start = LinePosition {
            offset: -0.5,
            heading_error: 0.0,
        };
        let time_step = 1e-4;
        let speed = 8.0;
        let mut position = start;
        let mut travelled = 0.0;
        let mut largest = f64::NEG_INFINITY;
        while travelled < 20.0 {
            let progress =
                calculate_line_progress_speed(speed, 0.0, position.offset, position.heading_error);
            let wanted = calculate_wanted_curvature(0.0, position.offset, position.heading_error);
            position = update_line_position(position, speed, wanted, 0.0, progress, time_step);
            travelled += progress * time_step;
            largest = largest.max(position.offset);
        }
        assert!(largest <= 1e-6, "overshoot to {}", largest);
        assert!(position.offset.abs() < 0.05, "{:?}", position);
    }
}
