//! Steady-state cornering: the lateral force a corner needs, how the axles share it and the
//! corner speed limit
//!
//! Underlying science equations are found in this documentation:
//! [Steady-State Cornering Documentation](../../docs/16_Steady_State_Cornering.MD)

use crate::physics::aero::calculate_aero_loads;
use crate::physics::car::Car;
use crate::physics::chassis::AxleDistances;
use crate::physics::environment::Environment;
use crate::physics::loads::{AxleLoads, calculate_static_axle_loads};
use crate::physics::roll::{calculate_steady_state_lateral_transfer, split_axle_loads};
use crate::physics::suspension::CornerLoads;
use crate::physics::track::Track;
use crate::physics::tyres::{Tyre, calculate_max_tyre_force};

/// Highest speed the corner speed limit search looks at in meters per second, m/s.
/// A corner the car can still hold at this speed has no limit.
const SEARCH_MAX_SPEED: f64 = 500.0;

/// Number of times the corner speed limit search halves its range. 60 halvings of 500 m/s
/// are below the resolution of an f64.
const SEARCH_HALVINGS: u32 = 60;

/// During the run a tyre is reported as over its lateral limit when it needs more than this
/// fraction above the limit (0.01 %). A car that holds the corner speed limit sits exactly on
/// the limit, so without a margin rounding errors and the small body motion would report it
/// as over the limit every few steps.
pub(crate) const OVER_GRIP_LIMIT_MARGIN: f64 = 1e-4;

/// Lateral force each axle has to make in Newtons, N (docs/16, equation 16.4), positive to the
/// left like a_y (docs/19 section 19.1). The two tyres of an axle share it so both use the same
/// fraction of their own lateral grip (docs/19 section 19.9).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct AxleLateralForces {
    /// Front axle lateral force (F_y,f) in Newtons, N
    pub front: f64,
    /// Rear axle lateral force (F_y,r) in Newtons, N
    pub rear: f64,
}

/// Calculates the lateral acceleration in m/s^2 a car needs to follow a line of curvature
/// `curvature` in 1/m at `speed` in m/s: a_y = v^2 * kappa (docs/16, equation 16.1). Positive
/// to the left like the curvature, so positive in a left turn (docs/19 section 19.1).
pub(crate) fn calculate_lateral_acceleration(speed: f64, curvature: f64) -> f64 {
    speed * speed * curvature
}

/// Calculates the lateral force of each axle in Newtons, N, for the lateral acceleration
/// `lateral_acceleration` in m/s^2: the total m * a_y (docs/16, equation 16.2) split so the
/// car does not turn faster or slower, F_y,f = F_y * d_r / W and F_y,r = F_y * d_f / W
/// (docs/16, equation 16.4).
pub(crate) fn calculate_axle_lateral_forces(
    mass_total: f64,
    lateral_acceleration: f64,
    axle_distances: &AxleDistances,
) -> AxleLateralForces {
    let lateral_force = mass_total * lateral_acceleration;
    let wheelbase = axle_distances.front + axle_distances.rear;

    AxleLateralForces {
        front: lateral_force * axle_distances.rear / wheelbase,
        rear: lateral_force * axle_distances.front / wheelbase,
    }
}

/// Checks if an axle needs more lateral force than its two tyres can make together at their
/// loads (docs/16 section 16.4, docs/14 equation 14.4). Both tyres use the same fraction of
/// their own lateral grip (docs/19, equation 19.24), so an axle is over the limit exactly when
/// that fraction is above 1.
///
/// * `margin` - fraction above the limit that still counts as on the limit, dimensionless, 0 = none
pub(crate) fn is_over_lateral_grip(
    tyre: &Tyre,
    tyre_loads: &CornerLoads,
    axle_lateral_forces: &AxleLateralForces,
    margin: f64,
) -> bool {
    let is_over = |load_left: f64, load_right: f64, axle_lateral_force: f64| {
        let max_lateral = calculate_max_tyre_force(tyre, tyre.mu_y_ref, load_left)
            + calculate_max_tyre_force(tyre, tyre.mu_y_ref, load_right);
        axle_lateral_force.abs() > max_lateral * (1.0 + margin)
    };

    is_over(
        tyre_loads.front_left,
        tyre_loads.front_right,
        axle_lateral_forces.front,
    ) || is_over(
        tyre_loads.rear_left,
        tyre_loads.rear_right,
        axle_lateral_forces.rear,
    )
}

/// Calculates the axle loads in Newtons, N, of a car that holds `speed` in m/s: the static
/// weight, the downforce shared by where it acts, and the drag at its height moving load from
/// the front to the rear (docs/16, equations 16.6 to 16.8). An axle load is never below 0.
pub(crate) fn calculate_constant_speed_axle_loads(
    car: &Car,
    environment: &Environment,
    axle_distances: &AxleDistances,
    speed: f64,
) -> AxleLoads {
    let static_axle_loads = calculate_static_axle_loads(&car.mass, environment.g_acceleration);
    let aero_loads = calculate_aero_loads(&car.aero, environment.air_density, speed);
    let wheelbase = axle_distances.front + axle_distances.rear;

    let downforce_rear_share = car.aero.downforce_centre_of_pressure_from_front_axle / wheelbase;
    let drag_load_transfer = aero_loads.drag * car.aero.drag_centre_of_pressure_height / wheelbase;

    AxleLoads {
        front: (static_axle_loads.front + aero_loads.downforce * (1.0 - downforce_rear_share)
            - drag_load_transfer)
            .max(0.0),
        rear: (static_axle_loads.rear
            + aero_loads.downforce * downforce_rear_share
            + drag_load_transfer)
            .max(0.0),
    }
}

/// Checks if the tyres can hold a corner of `curvature` in 1/m at a constant `speed` in m/s:
/// no axle needs more lateral force than its tyres can make (docs/16 section 16.5, steps 1 to
/// 3), with the tyre loads after the settled lateral load transfer (docs/19 section 19.9)
fn can_hold_corner(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    curvature: f64,
    speed: f64,
) -> bool {
    let axle_loads = calculate_constant_speed_axle_loads(car, environment, axle_distances, speed);
    let lateral_acceleration = calculate_lateral_acceleration(speed, curvature);
    let tyre_loads = split_axle_loads(
        &axle_loads,
        &calculate_steady_state_lateral_transfer(car, axle_distances, lateral_acceleration),
    );
    let axle_lateral_forces =
        calculate_axle_lateral_forces(car.mass.total, lateral_acceleration, axle_distances);

    !is_over_lateral_grip(tyre, &tyre_loads, &axle_lateral_forces, 0.0)
}

/// Calculates the corner speed limit in m/s: the highest constant speed at which the tyres can
/// hold a corner of `curvature` in 1/m (docs/16 section 16.5).
///
/// It is found by halving the search range, with the tyre model of docs/14, so load
/// sensitivity is included. `tyre` is the tyre the driver plans with (docs/17 section 17.2),
/// the car's own tyre for a driver without a grip margin. Returns infinity if the car can hold the corner at any speed
/// (a straight, or a corner so wide that downforce grows faster than the force it needs).
pub(crate) fn calculate_corner_speed_limit(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    curvature: f64,
) -> f64 {
    if can_hold_corner(
        car,
        tyre,
        environment,
        axle_distances,
        curvature,
        SEARCH_MAX_SPEED,
    ) {
        return f64::INFINITY;
    }

    // The car holds every corner at 0 m/s and not at the highest speed: the limit is between
    let mut holds = 0.0;
    let mut slides = SEARCH_MAX_SPEED;
    for _ in 0..SEARCH_HALVINGS {
        let speed = 0.5 * (holds + slides);
        if can_hold_corner(car, tyre, environment, axle_distances, curvature, speed) {
            holds = speed;
        } else {
            slides = speed;
        }
    }

    holds
}

/// Calculates the corner speed limit in m/s at every point of the track line (docs/16 section
/// 16.7). Empty for a track without XY points.
pub(crate) fn calculate_corner_speed_limits(
    car: &Car,
    tyre: &Tyre,
    environment: &Environment,
    axle_distances: &AxleDistances,
    track: &Track,
) -> Vec<f64> {
    track
        .curvatures
        .iter()
        .map(|&curvature| {
            calculate_corner_speed_limit(car, tyre, environment, axle_distances, curvature)
        })
        .collect()
}

/// Calculates the acceleration in m/s^2 the driver asks for: the one that brings the car
/// exactly to `target_speed` in this time step (docs/16 equation 16.16, docs/17 equation 17.9).
/// Positive below the target, negative above it, infinite where there is no target (full
/// throttle).
pub(crate) fn calculate_wanted_acceleration(speed: f64, target_speed: f64, time_step: f64) -> f64 {
    (target_speed - speed) / time_step
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::car::Mass;
    use crate::physics::chassis::calculate_axle_distances;

    /// Speed limits of the front and rear axle in m/s for a constant mu (lambda = 0), from
    /// q_f, q_r (docs/16, equations 16.12 and 16.13). Infinite if the axle has no limit.
    fn constant_mu_axle_limits(car: &Car, environment: &Environment, curvature: f64) -> (f64, f64) {
        let (q_front, q_rear) = aero_load_gains_per_kilogram(car, environment);
        let mu = car.tyre.mu_y_ref;
        let limit = |q: f64| {
            let bottom = curvature.abs() - mu * q;
            if bottom <= 0.0 {
                f64::INFINITY
            } else {
                (mu * environment.g_acceleration / bottom).sqrt()
            }
        };
        (limit(q_front), limit(q_rear))
    }

    /// q_f and q_r in 1/m: the aero load gain of each axle per kilogram it carries (docs/16,
    /// equations 16.7, 16.8 and 16.12)
    fn aero_load_gains_per_kilogram(car: &Car, environment: &Environment) -> (f64, f64) {
        let aero = &car.aero;
        let wheelbase = car.geometry.wheelbase;
        let pressure = 0.5 * environment.air_density * aero.frontal_area;
        let centre_of_pressure = aero.downforce_centre_of_pressure_from_front_axle;
        let drag_part = aero.drag_coefficient * aero.drag_centre_of_pressure_height / wheelbase;
        let gain_front = pressure
            * (aero.downforce_coefficient * (wheelbase - centre_of_pressure) / wheelbase
                - drag_part);
        let gain_rear =
            pressure * (aero.downforce_coefficient * centre_of_pressure / wheelbase + drag_part);

        (gain_front / car.mass.front, gain_rear / car.mass.rear)
    }

    /// Speed limit of one axle in m/s with load sensitivity, from docs/16 equation 16.15 solved
    /// as a quadratic in v^2. Only valid while the tyre is below F_z,peak.
    ///
    /// * `axle_mass` - mass the axle carries in kilograms, kg
    /// * `q` - aero load gain of the axle per kilogram in 1/m
    fn load_sensitive_axle_limit(
        tyre: &Tyre,
        g_acceleration: f64,
        axle_mass: f64,
        q: f64,
        curvature: f64,
    ) -> f64 {
        // mu_y = mu_ref * (a - b * u) with u = v^2, from 14.2 with F_z,tyre = axle_mass * (g + q * u) / 2
        let load_per_acceleration = 0.5 * axle_mass / tyre.reference_load;
        let a = 1.0 + tyre.load_sensitivity
            - tyre.load_sensitivity * load_per_acceleration * g_acceleration;
        let b = tyre.load_sensitivity * load_per_acceleration * q;
        let mu = tyre.mu_y_ref;

        // u * kappa = mu * (a - b * u) * (g + q * u)
        // -> mu * q * b * u^2 + (kappa - mu * (q * a - g * b)) * u - mu * g * a = 0
        let quadratic = mu * q * b;
        let linear = curvature.abs() - mu * (q * a - g_acceleration * b);
        let constant = -mu * g_acceleration * a;
        let u =
            (-linear + (linear * linear - 4.0 * quadratic * constant).sqrt()) / (2.0 * quadratic);

        u.sqrt()
    }

    fn limit(car: &Car, curvature: f64) -> f64 {
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        calculate_corner_speed_limit(car, &car.tyre, &environment, &axle_distances, curvature)
    }

    /// The car of the balance flip example in docs/16 section 16.6: 52 % of the mass on the
    /// front, downforce at 1.0 m behind the front axle, lambda = 0.2
    fn balance_flip_car() -> Car {
        let mut car = Car::default();
        car.mass.front = 0.52 * car.mass.total;
        car.mass.rear = 0.48 * car.mass.total;
        car.aero.downforce_centre_of_pressure_from_front_axle = 1.0;
        car.tyre.load_sensitivity = 0.2;
        car
    }

    #[test]
    fn test_lateral_acceleration_and_axle_forces() {
        // docs/16 section 16.2: 10 m/s in a 15 m corner, a_y = 100 / 15 = 6.667 m/s^2,
        // F_y = 815.494 kg * 6.667 m/s^2 = 5436.6 N
        let car = Car::default();
        let lateral_acceleration = calculate_lateral_acceleration(10.0, 1.0 / 15.0);
        assert!((lateral_acceleration - 6.666666667).abs() < 1e-9);
        // A right corner needs the same size, pointing the other way (docs/19 section 19.1)
        assert_eq!(
            calculate_lateral_acceleration(10.0, -1.0 / 15.0),
            -lateral_acceleration
        );

        // 50/50 car: 2718.3 N per axle (docs/16 section 16.3)
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);
        let forces =
            calculate_axle_lateral_forces(car.mass.total, lateral_acceleration, &axle_distances);
        assert!((forces.front - 2718.314645).abs() < 1e-6, "{:?}", forces);
        assert!((forces.rear - 2718.314645).abs() < 1e-6, "{:?}", forces);

        // 40 % of the mass on the front: the front makes 40 % of the force
        let mass = Mass {
            total: 1000.0,
            front: 400.0,
            rear: 600.0,
        };
        let axle_distances = calculate_axle_distances(&mass, &car.geometry);
        let forces = calculate_axle_lateral_forces(mass.total, 5.0, &axle_distances);
        assert!((forces.front - 2000.0).abs() < 1e-9, "{:?}", forces);
        assert!((forces.rear - 3000.0).abs() < 1e-9, "{:?}", forces);
    }

    #[test]
    fn test_constant_speed_axle_loads() {
        // docs/16 equations 16.6 to 16.8 for the default car at 10 m/s:
        // D_f = 1.225 * (1.0 * 1.6 / 3.4 - 0.35 * 0.4 / 3.4) = 0.526029 -> 4000 + 52.603 N
        // D_r = 1.225 * (1.0 * 1.8 / 3.4 + 0.35 * 0.4 / 3.4) = 0.698971 -> 4000 + 69.897 N
        let car = Car::default();
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);

        let loads = calculate_constant_speed_axle_loads(&car, &environment, &axle_distances, 10.0);
        assert!((loads.front - 4052.602941).abs() < 1e-5, "{:?}", loads);
        assert!((loads.rear - 4069.897059).abs() < 1e-5, "{:?}", loads);
        // Together: the weight plus the downforce 0.5 * 1.225 * 1.0 * 2.0 * 100 = 122.5 N
        assert!((loads.total() - 8122.5).abs() < 1e-5);

        // At rest: the static loads
        let loads = calculate_constant_speed_axle_loads(&car, &environment, &axle_distances, 0.0);
        assert!((loads.front - 4000.0).abs() < 1e-6 && (loads.rear - 4000.0).abs() < 1e-6);
    }

    #[test]
    fn test_corner_speed_limit_without_downforce() {
        // docs/16 equation 16.5: sqrt(mu * g * R) = sqrt(0.9 * 9.81 * 15) = 11.508 m/s
        let mut car = Car::default();
        car.aero.downforce_coefficient = 0.0;
        car.aero.drag_centre_of_pressure_height = 0.0;

        assert!((limit(&car, 1.0 / 15.0) - 11.508040667).abs() < 1e-8);
        // Four times the radius doubles the speed
        assert!((limit(&car, 1.0 / 60.0) - 2.0 * 11.508040667).abs() < 1e-8);
        // The mass cancels
        car.mass.total *= 2.0;
        car.mass.front *= 2.0;
        car.mass.rear *= 2.0;
        assert!((limit(&car, 1.0 / 15.0) - 11.508040667).abs() < 1e-8);
    }

    #[test]
    fn test_corner_speed_limit_of_baseline_car() {
        // docs/16 section 16.5: 11.61 m/s at 15 m, the front axle limits
        let car = Car::default();
        let environment = Environment::default();

        let (q_front, q_rear) = aero_load_gains_per_kilogram(&car, &environment);
        assert!((q_front - 0.00129009).abs() < 1e-8, "{}", q_front);
        assert!((q_rear - 0.00171423).abs() < 1e-8, "{}", q_rear);

        assert!((limit(&car, 1.0 / 15.0) - 11.609581988).abs() < 1e-8);

        // docs/16 section 16.6 table, and the search matches 16.13 at every radius
        for (radius, expected_front, expected_rear) in [
            (9.125, 9.0237, 9.0396),
            (15.0, 11.6096, 11.6436),
            (50.0, 21.6485, 21.8712),
            (200.0, 47.9569, 50.5352),
        ] {
            let (front, rear) = constant_mu_axle_limits(&car, &environment, 1.0 / radius);
            assert!(
                (front - expected_front).abs() < 1e-4,
                "{} m: {}",
                radius,
                front
            );
            assert!(
                (rear - expected_rear).abs() < 1e-4,
                "{} m: {}",
                radius,
                rear
            );
            // The front has the smaller q, so it limits in every corner
            assert!(front < rear);
            assert!((limit(&car, 1.0 / radius) - front).abs() < 1e-8);
        }

        // Left and right corners have the same limit
        assert_eq!(limit(&car, -1.0 / 15.0), limit(&car, 1.0 / 15.0));
    }

    #[test]
    fn test_smaller_q_axle_limits() {
        // Downforce moved forward to 1.0 m: q_f > q_r, so now the rear limits in every corner
        let mut car = Car::default();
        car.aero.downforce_centre_of_pressure_from_front_axle = 1.0;
        let environment = Environment::default();

        let (q_front, q_rear) = aero_load_gains_per_kilogram(&car, &environment);
        assert!(q_front > q_rear);
        for radius in [9.125, 15.0, 50.0, 200.0] {
            let (front, rear) = constant_mu_axle_limits(&car, &environment, 1.0 / radius);
            assert!(rear < front);
            assert!((limit(&car, 1.0 / radius) - rear).abs() < 1e-8);
        }
    }

    #[test]
    fn test_no_corner_speed_limit() {
        let car = Car::default();
        // Straight
        assert_eq!(limit(&car, 0.0), f64::INFINITY);
        // Wider than 1 / (mu * q_f) = 861 m: downforce grows faster than the force needed
        assert_eq!(limit(&car, 1.0 / 900.0), f64::INFINITY);
        // Just inside it there is a limit, a very high one
        let fast = limit(&car, 1.0 / 800.0);
        assert!(fast.is_finite() && fast > 200.0, "{}", fast);
    }

    #[test]
    fn test_corner_speed_limit_with_load_sensitivity() {
        // docs/16 section 16.6: with lambda > 0 the search matches 16.15 solved as a quadratic,
        // and the balance flips between 15 m and 30 m. 16.15 has no lateral load transfer, so
        // the CoG goes on the ground for the comparison: m * a_y * h / b = 0 (docs/19, 19.1).
        let mut car = balance_flip_car();
        car.geometry.cog_height = 0.0;
        let environment = Environment::default();
        let (q_front, q_rear) = aero_load_gains_per_kilogram(&car, &environment);
        assert!((q_front - 0.00192).abs() < 1e-5 && (q_rear - 0.00105).abs() < 1e-5);

        for (radius, expected_front, expected_rear, front_limits) in [
            (9.125, 9.00, 9.04, true),
            (15.0, 11.58, 11.62, true),
            (30.0, 16.54, 16.53, false),
            (100.0, 31.70, 31.02, false),
        ] {
            let g = environment.g_acceleration;
            let front =
                load_sensitive_axle_limit(&car.tyre, g, car.mass.front, q_front, 1.0 / radius);
            let rear = load_sensitive_axle_limit(&car.tyre, g, car.mass.rear, q_rear, 1.0 / radius);

            assert!(
                (front - expected_front).abs() < 0.006,
                "{} m: {}",
                radius,
                front
            );
            assert!(
                (rear - expected_rear).abs() < 0.006,
                "{} m: {}",
                radius,
                rear
            );
            assert_eq!(front < rear, front_limits, "{} m", radius);
            assert!(
                (limit(&car, 1.0 / radius) - front.min(rear)).abs() < 1e-8,
                "{} m: search {} != {}",
                radius,
                limit(&car, 1.0 / radius),
                front.min(rear)
            );
        }

        // Downforce puts the tyres above the reference load, so with lambda > 0 the limit is
        // lower than with a constant mu
        let mut sensitive = Car::default();
        sensitive.tyre.load_sensitivity = 0.2;
        assert!(limit(&sensitive, 1.0 / 50.0) < limit(&Car::default(), 1.0 / 50.0));
    }

    #[test]
    fn test_is_over_lateral_grip() {
        // Default tyre: mu_y = 0.9, 2000 N per tyre -> 1800 N per tyre, 3600 N per axle
        let tyre = Tyre::default();
        let tyre_loads = CornerLoads {
            front_left: 2000.0,
            front_right: 2000.0,
            rear_left: 2000.0,
            rear_right: 2000.0,
        };
        let forces = |front: f64, rear: f64| AxleLateralForces { front, rear };

        assert!(!is_over_lateral_grip(
            &tyre,
            &tyre_loads,
            &forces(3600.0, 3600.0),
            0.0
        ));
        assert!(is_over_lateral_grip(
            &tyre,
            &tyre_loads,
            &forces(3600.1, 0.0),
            0.0
        ));
        assert!(is_over_lateral_grip(
            &tyre,
            &tyre_loads,
            &forces(0.0, 3600.1),
            0.0
        ));
        // Within the margin it still counts as on the limit
        assert!(!is_over_lateral_grip(
            &tyre,
            &tyre_loads,
            &forces(3600.1, 0.0),
            1e-4
        ));
        assert!(is_over_lateral_grip(
            &tyre,
            &tyre_loads,
            &forces(3601.0, 0.0),
            1e-4
        ));
        // docs/19 section 19.9 example: 1388 N and 2612 N can hold 3262 N together (limits 1249 N
        // and 2351 N), although half of it is more than the inside tyre could make alone
        let transferred = CornerLoads {
            front_left: 1388.0,
            front_right: 2612.0,
            rear_left: 2000.0,
            rear_right: 2000.0,
        };
        assert!(!is_over_lateral_grip(
            &tyre,
            &transferred,
            &forces(3262.0, 0.0),
            0.0
        ));
        assert!(is_over_lateral_grip(
            &tyre,
            &transferred,
            &forces(-3601.0, 0.0),
            0.0
        ));
        // A straight is never over the limit, even with a lifted wheel
        let lifted = CornerLoads::default();
        assert!(!is_over_lateral_grip(
            &tyre,
            &lifted,
            &forces(0.0, 0.0),
            0.0
        ));
    }

    #[test]
    fn test_lateral_load_transfer_lowers_limit() {
        // docs/19 section 19.9: with lambda = 0 the transfer changes nothing, with lambda > 0
        // it costs grip, and a stiffer front bar costs the front-limited baseline car more
        let constant_mu = Car::default();
        let mut flat = Car::default();
        flat.geometry.cog_height = 0.0;
        assert!((limit(&constant_mu, 1.0 / 15.0) - limit(&flat, 1.0 / 15.0)).abs() < 1e-9);

        let mut sensitive = Car::default();
        sensitive.tyre.load_sensitivity = 0.2;
        let mut sensitive_flat = Car::default();
        sensitive_flat.tyre.load_sensitivity = 0.2;
        sensitive_flat.geometry.cog_height = 0.0;
        let with_transfer = limit(&sensitive, 1.0 / 15.0);
        assert!(with_transfer < limit(&sensitive_flat, 1.0 / 15.0) - 0.01);

        sensitive.suspension.front_anti_roll_bar_stiffness = 300_000.0;
        assert!(limit(&sensitive, 1.0 / 15.0) < with_transfer - 0.01);
        // Left and right corners are the same
        assert_eq!(
            limit(&sensitive, 1.0 / 15.0),
            limit(&sensitive, -1.0 / 15.0)
        );
    }

    #[test]
    fn test_calculate_wanted_acceleration() {
        // docs/16 equation 16.16: (11 - 10) m/s in 1 ms
        assert!((calculate_wanted_acceleration(10.0, 11.0, 0.001) - 1000.0).abs() < 1e-9);
        assert!((calculate_wanted_acceleration(12.0, 11.0, 0.001) + 1000.0).abs() < 1e-9);
        assert_eq!(calculate_wanted_acceleration(11.0, 11.0, 0.001), 0.0);
        // No limit: full throttle
        assert_eq!(
            calculate_wanted_acceleration(50.0, f64::INFINITY, 0.001),
            f64::INFINITY
        );
    }

    #[test]
    fn test_corner_speed_limits_of_track() {
        let car = Car::default();
        let environment = Environment::default();
        let axle_distances = calculate_axle_distances(&car.mass, &car.geometry);

        // Square 10 m x 10 m: every point has the same curvature, so the same limit
        let square = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let track = Track::from_points(square, 0.0).unwrap();
        let limits =
            calculate_corner_speed_limits(&car, &car.tyre, &environment, &axle_distances, &track);
        assert_eq!(limits.len(), 4);
        assert!(
            limits
                .iter()
                .all(|&limit| limit == limits[0] && limit.is_finite())
        );

        // Track without XY points: no limits
        let limits = calculate_corner_speed_limits(
            &car,
            &car.tyre,
            &environment,
            &axle_distances,
            &Track::default(),
        );
        assert!(limits.is_empty());
    }
}
