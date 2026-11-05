//! Functions related to tyre physics
//!
//! Underlying science equations are found in this documentation:
//! [Tyre Friction Documentation](../../docs/03_Tire_Friction_and_Traction.MD)
//! [Load Sensitivity and Friction Ellipse Documentation](../../docs/14_Tyre_Load_Sensitivity_and_Friction_Ellipse.MD)

use crate::physics::suspension::CornerLoads;
use serde::Deserialize;

/// Model defines the tyres used to simulate the car
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tyre {
    /// Longitudinal (braking and driving) peak friction coefficient at the reference load,
    /// dimensionless (docs/14, equation 14.1)
    pub mu_x_ref: f64,

    /// Lateral (cornering) peak friction coefficient at the reference load, dimensionless
    /// (docs/14, equation 14.2)
    pub mu_y_ref: f64,

    /// Tyre load where mu_x_ref and mu_y_ref apply in Newtons, N
    #[serde(rename = "reference_load_N")]
    pub reference_load: f64,

    /// Load sensitivity lambda, dimensionless: the fraction by which mu drops when the load rises
    /// by reference_load. 0 = constant mu (docs/14 section 14.2)
    pub load_sensitivity: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    pub mu_rolling: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    pub mu_breakaway: f64,

    /// wheel radius in meters, m
    #[serde(rename = "wheel_radius_m")]
    pub wheel_radius: f64,
}

impl Default for Tyre {
    fn default() -> Self {
        Self {
            mu_x_ref: 0.9,
            mu_y_ref: 0.9,
            // Static corner load of the default car (4000 N per axle). With load_sensitivity = 0
            // it has no effect, mu stays constant as in docs/03
            reference_load: 2000.0,
            load_sensitivity: 0.0,
            // Placeholders pending a sourced value. mu_breakaway sits just above mu_rolling:
            // the old 0.7 gave 5600 N of breakaway resistance against 3600 N of traction,
            // so the car could never launch from rest.
            mu_rolling: 0.03,
            mu_breakaway: 0.035,
            wheel_radius: 0.36,
        }
    }
}

/// Longitudinal grip limit of each axle in Newtons, N: the sum of the available longitudinal
/// force of its two tyres (docs/14 section 14.5)
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AxleGrip {
    /// Front axle grip limit in Newtons, N
    pub front: f64,
    /// Rear axle grip limit in Newtons, N
    pub rear: f64,
}

/// Calculates the longitudinal grip limit of each axle in Newtons, N, from the four corner tyre
/// loads. Each tyre is calculated on its own and then summed, so the loss from load transfer
/// between the left and right tyre is kept (docs/14 sections 14.3 and 14.5).
///
/// The car only drives straight, so the lateral force of every tyre is 0 for now.
pub(crate) fn calculate_axle_grip(tyre: &Tyre, tyre_loads: &CornerLoads) -> AxleGrip {
    let lateral_force = 0.0;
    let tyre_grip =
        |tyre_load: f64| calculate_available_longitudinal_force(tyre, tyre_load, lateral_force);

    AxleGrip {
        front: tyre_grip(tyre_loads.front_left) + tyre_grip(tyre_loads.front_right),
        rear: tyre_grip(tyre_loads.rear_left) + tyre_grip(tyre_loads.rear_right),
    }
}

/// Calculates the longitudinal force in Newtons, N, one tyre can still give while it makes the
/// lateral force `lateral_force` in Newtons, N (friction ellipse, docs/14 equation 14.8).
///
/// Returns 0 if the lateral force alone is beyond the tyre's lateral limit.
pub(crate) fn calculate_available_longitudinal_force(
    tyre: &Tyre,
    tyre_load: f64,
    lateral_force: f64,
) -> f64 {
    let max_longitudinal = calculate_max_tyre_force(tyre, tyre.mu_x_ref, tyre_load);
    if lateral_force == 0.0 {
        // Straight line: the whole friction budget is longitudinal
        return max_longitudinal;
    }

    let max_lateral = calculate_max_tyre_force(tyre, tyre.mu_y_ref, tyre_load);

    if max_lateral <= 0.0 || lateral_force.abs() >= max_lateral {
        return 0.0;
    }

    let lateral_usage = lateral_force / max_lateral;
    max_longitudinal * (1.0 - lateral_usage * lateral_usage).sqrt()
}

/// Calculates the maximum force in Newtons, N, one tyre can make in one direction at the tyre
/// load `tyre_load` in Newtons, N (docs/14 equations 14.1 to 14.5).
///
/// * `mu_ref` - friction coefficient of that direction at the reference load (`mu_x_ref` or `mu_y_ref`)
///
/// mu falls linearly with load. Above the load F_z,peak the force would start to fall, so it is
/// held at its value at F_z,peak (equation 14.5). A lifted wheel (load <= 0) gives 0.
pub(crate) fn calculate_max_tyre_force(tyre: &Tyre, mu_ref: f64, tyre_load: f64) -> f64 {
    if tyre_load <= 0.0 {
        return 0.0;
    }

    let load = if tyre.load_sensitivity > 0.0 {
        // (14.5): F_z,peak = (1 + lambda) / (2 * lambda) * F_z,ref
        let peak_load =
            (1.0 + tyre.load_sensitivity) / (2.0 * tyre.load_sensitivity) * tyre.reference_load;
        tyre_load.min(peak_load)
    } else {
        tyre_load
    };

    let mu =
        mu_ref * (1.0 - tyre.load_sensitivity * (load - tyre.reference_load) / tyre.reference_load);

    mu * load
}

/// Calculates the largest total longitudinal force in Newtons, N, the tyres can transmit when
/// it is split front_share : (1 - front_share) between the front and rear axle.
///
/// The first axle to reach its grip limit caps the total, which models ideal threshold
/// braking / ideal traction control (docs/09, docs/10).
///
/// * `front_share` - fraction of the total force taken by the front axle, dimensionless, 0.0..=1.0
pub(crate) fn calculate_split_force_limit(axle_grip: &AxleGrip, front_share: f64) -> f64 {
    if front_share >= 1.0 {
        // Front axle carries everything, rear grip is unused
        axle_grip.front
    } else if front_share <= 0.0 {
        // Rear axle carries everything, front grip is unused
        axle_grip.rear
    } else {
        (axle_grip.front / front_share).min(axle_grip.rear / (1.0 - front_share))
    }
}

/// Calculates tyre friction depending on the car speed
pub(crate) fn calculate_current_tyre_friction(
    total_normal_force: f64,
    mu_rolling: f64,
    mu_breakaway: f64,
    speed: f64,
) -> f64 {
    if speed != 0.0 {
        calculate_force_tyre_friction_rolling(total_normal_force, mu_rolling)
    } else {
        calculate_force_tyre_friction_breakaway(total_normal_force, mu_breakaway)
    }
}

/// Calculates rolling friction force from tyres knowing tyre model
pub(crate) fn calculate_force_tyre_friction_rolling(
    total_normal_force: f64,
    mu_rolling: f64,
) -> f64 {
    total_normal_force * mu_rolling
}

/// Calculates breakaway friction force from tyres knowing tyre model
fn calculate_force_tyre_friction_breakaway(total_normal_force: f64, mu_breakaway: f64) -> f64 {
    total_normal_force * mu_breakaway
}

/// Test helper: axle grip for given axle loads, each split equally between the left and right
/// tyre (straight line, no roll)
#[cfg(test)]
pub(crate) fn calculate_axle_grip_from_axle_loads(
    tyre: &Tyre,
    axle_loads: &crate::physics::loads::AxleLoads,
) -> AxleGrip {
    calculate_axle_grip(
        tyre,
        &CornerLoads {
            front_left: axle_loads.front / 2.0,
            front_right: axle_loads.front / 2.0,
            rear_left: axle_loads.rear / 2.0,
            rear_right: axle_loads.rear / 2.0,
        },
    )
}

#[cfg(test)]
mod tests {
    use crate::physics::loads::AxleLoads;
    use crate::physics::suspension::CornerLoads;
    use crate::physics::tyres::{
        AxleGrip, Tyre, calculate_available_longitudinal_force, calculate_axle_grip,
        calculate_axle_grip_from_axle_loads, calculate_current_tyre_friction,
        calculate_force_tyre_friction_breakaway, calculate_force_tyre_friction_rolling,
        calculate_max_tyre_force, calculate_split_force_limit,
    };

    /// Placeholder FS tyre from docs/14: mu 1.6 at 700 N, lambda 0.2
    fn fs_tyre() -> Tyre {
        Tyre {
            mu_x_ref: 1.6,
            mu_y_ref: 1.6,
            reference_load: 700.0,
            load_sensitivity: 0.2,
            ..Tyre::default()
        }
    }

    /// Grip of the default tyre (mu 0.9, no load sensitivity) for the test axle loads
    fn test_axle_grip() -> AxleGrip {
        calculate_axle_grip_from_axle_loads(&Tyre::default(), &test_axle_loads())
    }

    fn test_axle_loads() -> AxleLoads {
        AxleLoads {
            front: 5000.0, // Newtons (N)
            rear: 3000.0,  // Newtons (N)
        }
    }

    #[test]
    fn test_rolling_tyre_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_rolling: f64 = 0.03;

        let tyre_rolling_friction: f64 =
            calculate_force_tyre_friction_rolling(normal_force, mu_rolling);

        // Expected value: 100 * 0.03 = 3 N
        assert!((tyre_rolling_friction - 3.0).abs() < 1e-3);
    }

    #[test]
    fn test_breakaway_tyre_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_breakaway: f64 = 0.9;

        let tyre_breakaway_friction: f64 =
            calculate_force_tyre_friction_breakaway(normal_force, mu_breakaway);

        // Expected value: 100 * 0.9 = 90 N
        assert!((tyre_breakaway_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_current_tyre_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_rolling: f64 = 0.03;
        let mu_breakaway: f64 = 0.9;
        let mut speed = 10.0; // meters per second m/s

        let mut current_tyre_friction: f64 =
            calculate_current_tyre_friction(normal_force, mu_rolling, mu_breakaway, speed);

        // As the speed != 0 then expected value: 100 * 0.03 = 3 N
        assert!((current_tyre_friction - 3.0).abs() < 1e-3);

        speed = 0.0;

        current_tyre_friction =
            calculate_current_tyre_friction(normal_force, mu_rolling, mu_breakaway, speed);

        // As the speed = 0 then expected value: 100 * 0.9 = 90N
        assert!((current_tyre_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_axle_grip_without_load_sensitivity() {
        // lambda = 0 -> back to docs/03: grip = mu * Fz_axle
        let grip = test_axle_grip();

        // Front: 0.9 * 5000 N = 4500 N, rear: 0.9 * 3000 N = 2700 N
        assert!((grip.front - 4500.0).abs() < 1e-9, "{:?}", grip);
        assert!((grip.rear - 2700.0).abs() < 1e-9, "{:?}", grip);
    }

    #[test]
    fn test_max_tyre_force_constant_mu_at_any_load() {
        let tyre = Tyre::default();
        for tyre_load in [100.0, 2000.0, 7000.0, 50000.0] {
            let force = calculate_max_tyre_force(&tyre, tyre.mu_x_ref, tyre_load);
            assert!(
                (force - 0.9 * tyre_load).abs() < 1e-9,
                "{} N at {} N",
                force,
                tyre_load
            );
        }
    }

    #[test]
    fn test_max_tyre_force_at_reference_load() {
        // At F_z,ref mu = mu_ref for any lambda: 1.6 * 700 N = 1120 N
        for load_sensitivity in [0.0, 0.1, 0.2, 0.5] {
            let tyre = Tyre {
                load_sensitivity,
                ..fs_tyre()
            };
            let force = calculate_max_tyre_force(&tyre, tyre.mu_x_ref, 700.0);
            assert!(
                (force - 1120.0).abs() < 1e-9,
                "{} N at lambda {}",
                force,
                load_sensitivity
            );
        }
    }

    #[test]
    fn test_load_transfer_loses_axle_grip() {
        // docs/14 section 14.3: two tyres at 700 N, then 300 N moved from one to the other
        let tyre = fs_tyre();
        let even = calculate_axle_grip(
            &tyre,
            &CornerLoads {
                front_left: 700.0,
                front_right: 700.0,
                ..CornerLoads::default()
            },
        );
        let transferred = calculate_axle_grip(
            &tyre,
            &CornerLoads {
                front_left: 1000.0,
                front_right: 400.0,
                ..CornerLoads::default()
            },
        );

        // Outside: 1.6 * (1 - 0.2 * 300 / 700) * 1000 N = 1462.857 N
        // Inside: 1.6 * (1 + 0.2 * 300 / 700) * 400 N = 694.857 N
        assert!((even.front - 2240.0).abs() < 1e-9, "{:?}", even);
        assert!(
            (transferred.front - 2157.714286).abs() < 1e-6,
            "{:?}",
            transferred
        );

        // (14.6): loss = 2 * mu_ref * lambda / F_z,ref * dFz^2 = 2 * 1.6 * 0.2 / 700 * 300^2
        let expected_loss = 2.0 * 1.6 * 0.2 / 700.0 * 300.0 * 300.0;
        assert!(((even.front - transferred.front) - expected_loss).abs() < 1e-9);
    }

    #[test]
    fn test_max_tyre_force_capped_above_peak_load() {
        // (14.5): F_z,peak = 1.2 / 0.4 * 700 N = 2100 N
        let tyre = fs_tyre();
        let peak = calculate_max_tyre_force(&tyre, tyre.mu_x_ref, 2100.0);

        let mut previous = 0.0;
        for step in 0..=400 {
            let tyre_load = step as f64 * 10.0;
            let force = calculate_max_tyre_force(&tyre, tyre.mu_x_ref, tyre_load);
            assert!(
                force >= previous,
                "Force fell from {} N to {} N at {} N",
                previous,
                force,
                tyre_load
            );
            if tyre_load >= 2100.0 {
                assert_eq!(force, peak, "Not capped at {} N", tyre_load);
            }
            previous = force;
        }
    }

    #[test]
    fn test_lifted_wheel_has_no_grip() {
        let tyre = fs_tyre();
        assert_eq!(calculate_max_tyre_force(&tyre, tyre.mu_x_ref, 0.0), 0.0);
        assert_eq!(calculate_max_tyre_force(&tyre, tyre.mu_x_ref, -10.0), 0.0);
        assert_eq!(calculate_available_longitudinal_force(&tyre, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_friction_ellipse() {
        let tyre = fs_tyre();
        let tyre_load = 1000.0;
        let max_force = calculate_max_tyre_force(&tyre, tyre.mu_x_ref, tyre_load);

        // No lateral force: the full longitudinal limit
        let full = calculate_available_longitudinal_force(&tyre, tyre_load, 0.0);
        assert_eq!(full, max_force);

        // All lateral grip used: nothing left, also beyond the limit
        assert_eq!(
            calculate_available_longitudinal_force(&tyre, tyre_load, max_force),
            0.0
        );
        assert_eq!(
            calculate_available_longitudinal_force(&tyre, tyre_load, -2.0 * max_force),
            0.0
        );

        // docs/14 section 14.4: 1000 N lateral of 1462.857 N leaves 1067.685 N, either direction
        let left = calculate_available_longitudinal_force(&tyre, tyre_load, 1000.0);
        let right = calculate_available_longitudinal_force(&tyre, tyre_load, -1000.0);
        assert!((left - 1067.684888).abs() < 1e-6, "{}", left);
        assert_eq!(left, right);
    }

    #[test]
    fn test_friction_ellipse_with_higher_longitudinal_mu() {
        // mu_x = 1.1 * mu_y: the ellipse is taller than the circle by 10 %
        let tyre = Tyre {
            mu_x_ref: 1.76,
            ..fs_tyre()
        };
        let circle = calculate_available_longitudinal_force(&fs_tyre(), 1000.0, 1000.0);
        let ellipse = calculate_available_longitudinal_force(&tyre, 1000.0, 1000.0);
        assert!(
            (ellipse - 1.1 * circle).abs() < 1e-9,
            "{} vs {}",
            ellipse,
            circle
        );
    }

    #[test]
    fn test_calculate_split_force_limit() {
        let front_share: f64 = 0.6;

        let limit = calculate_split_force_limit(&test_axle_grip(), front_share);

        // Front axle limit = 0.9 * 5000 N = 4500 N -> total limit = 4500 N / 0.6 = 7500 N
        // Rear axle limit = 0.9 * 3000 N = 2700 N -> total limit = 2700 N / 0.4 = 6750 N
        // Rear axle saturates first, expected value: 6750 N
        assert!((limit - 6750.0).abs() < 1e-9, "Calculated limit: {}", limit);
    }

    #[test]
    fn test_split_force_limit_single_axle() {
        let axle_grip = test_axle_grip();

        // All force on the front axle: 0.9 * 5000 N = 4500 N
        let front_only = calculate_split_force_limit(&axle_grip, 1.0);
        assert!((front_only - 4500.0).abs() < 1e-9);

        // All force on the rear axle: 0.9 * 3000 N = 2700 N
        let rear_only = calculate_split_force_limit(&axle_grip, 0.0);
        assert!((rear_only - 2700.0).abs() < 1e-9);
    }

    #[test]
    fn test_split_force_limit_never_exceeds_total_grip() {
        let axle_grip = test_axle_grip();
        // Total grip = 0.9 * 8000 N = 7200 N
        let total_grip = axle_grip.front + axle_grip.rear;

        for step in 0..=100 {
            let front_share = step as f64 / 100.0;
            let limit = calculate_split_force_limit(&axle_grip, front_share);

            assert!(
                limit <= total_grip + 1e-9,
                "Limit {} exceeds total grip {} at share {}",
                limit,
                total_grip,
                front_share
            );
            assert!(limit.is_finite() && limit >= 0.0);
        }
    }

    #[test]
    fn test_split_force_limit_ideal_share() {
        let axle_grip = test_axle_grip();

        // Ideal share = 4500 N / 7200 N = 0.625
        let ideal_share = axle_grip.front / (axle_grip.front + axle_grip.rear);

        let limit = calculate_split_force_limit(&axle_grip, ideal_share);

        // Front: 4500 N / 0.625 = 7200 N, rear: 2700 N / 0.375 = 7200 N
        // Both axles saturate together, expected value: 0.9 * 8000 N = 7200 N
        assert!((limit - 7200.0).abs() < 1e-9, "Calculated limit: {}", limit);
    }
}
