//! Functions related to tyre physics

use crate::physics::loads::AxleLoads;

/// Model defines the tyres used to simulate the car
pub struct Tyre {
    /// mu static friction which limits traction of the wheel
    pub mu_static: f64,

    /// mu rolling friction is the friction opposite to the movement that tries to stop the car
    pub mu_rolling: f64,

    /// mu breakaway friction is the friction that needs to be overcome to start moving the car
    pub mu_breakaway: f64,

    /// wheel radius
    pub wheel_radius: f64,
}

impl Default for Tyre {
    fn default() -> Self {
        Self {
            mu_static: 0.9,
            mu_rolling: 0.03,
            mu_breakaway: 0.7,
            wheel_radius: 0.36,
        }
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
fn calculate_force_tyre_friction_breakaway(
    total_normal_force: f64,
    mu_breakaway: f64,
) -> f64 {
    total_normal_force * mu_breakaway
}

/// Calculates tyre static friction which correlates to accessible traction limit
pub(crate) fn calculate_force_static_friction(
    axle_normal_force: f64,
    mu_static: f64,
) -> f64 {
    axle_normal_force * mu_static
}

/// Calculates the largest total longitudinal force in Newtons, N, the tyres can transmit when
/// it is split front_share : (1 - front_share) between the front and rear axle.
///
/// Each axle is limited to mu * Fz_axle. The first axle to reach its limit caps the total,
/// which models ideal threshold braking / ideal traction control (docs/09, docs/10).
///
/// * `mu` - peak static friction coefficient, dimensionless
/// * `front_share` - fraction of the total force taken by the front axle, dimensionless, 0.0..=1.0
pub(crate) fn calculate_split_force_limit(
    axle_loads: &AxleLoads,
    mu: f64,
    front_share: f64,
) -> f64 {
    let front_limit = calculate_force_static_friction(axle_loads.front, mu);
    let rear_limit = calculate_force_static_friction(axle_loads.rear, mu);

    if front_share >= 1.0 {
        // Front axle carries everything, rear grip is unused
        front_limit
    } else if front_share <= 0.0 {
        // Rear axle carries everything, front grip is unused
        rear_limit
    } else {
        (front_limit / front_share).min(rear_limit / (1.0 - front_share))
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

#[cfg(test)]
mod tests {
    use crate::physics::loads::AxleLoads;
    use crate::physics::tyres::{
        calculate_current_tyre_friction, calculate_force_static_friction,
        calculate_force_tyre_friction_breakaway, calculate_force_tyre_friction_rolling,
        calculate_split_force_limit,
    };

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

        let mut current_tyre_friction: f64 = calculate_current_tyre_friction(
            normal_force,
            mu_rolling,
            mu_breakaway,
            speed,
        );

        // As the speed != 0 then expected value: 100 * 0.03 = 3 N
        assert!((current_tyre_friction - 3.0).abs() < 1e-3);

        speed = 0.0;

        current_tyre_friction = calculate_current_tyre_friction(
            normal_force,
            mu_rolling,
            mu_breakaway,
            speed,
        );

        // As the speed = 0 then expected value: 100 * 0.9 = 90N
        assert!((current_tyre_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_static_tyre_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_static: f64 = 0.9;

        let tyre_static_friction: f64 =
            calculate_force_static_friction(normal_force, mu_static);

        // Expected value: 100 * 0.9 = 90 N
        assert!((tyre_static_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_split_force_limit() {
        let mu: f64 = 0.9;
        let front_share: f64 = 0.6;

        let limit = calculate_split_force_limit(&test_axle_loads(), mu, front_share);

        // Front axle limit = 0.9 * 5000 N = 4500 N -> total limit = 4500 N / 0.6 = 7500 N
        // Rear axle limit = 0.9 * 3000 N = 2700 N -> total limit = 2700 N / 0.4 = 6750 N
        // Rear axle saturates first, expected value: 6750 N
        assert!((limit - 6750.0).abs() < 1e-9, "Calculated limit: {}", limit);
    }

    #[test]
    fn test_split_force_limit_single_axle() {
        let axle_loads = test_axle_loads();
        let mu: f64 = 0.9;

        // All force on the front axle: 0.9 * 5000 N = 4500 N
        let front_only = calculate_split_force_limit(&axle_loads, mu, 1.0);
        assert!((front_only - 4500.0).abs() < 1e-9);

        // All force on the rear axle: 0.9 * 3000 N = 2700 N
        let rear_only = calculate_split_force_limit(&axle_loads, mu, 0.0);
        assert!((rear_only - 2700.0).abs() < 1e-9);
    }

    #[test]
    fn test_split_force_limit_never_exceeds_total_grip() {
        let axle_loads = test_axle_loads();
        let mu: f64 = 0.9;
        // Total grip = 0.9 * 8000 N = 7200 N
        let total_grip = mu * axle_loads.total();

        for step in 0..=100 {
            let front_share = step as f64 / 100.0;
            let limit = calculate_split_force_limit(&axle_loads, mu, front_share);

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
        let axle_loads = test_axle_loads();
        let mu: f64 = 0.9;

        // Ideal share = 0.9 * 5000 N / (0.9 * 8000 N) = 0.625
        let ideal_share = mu * axle_loads.front / (mu * axle_loads.total());

        let limit = calculate_split_force_limit(&axle_loads, mu, ideal_share);

        // Front: 4500 N / 0.625 = 7200 N, rear: 2700 N / 0.375 = 7200 N
        // Both axles saturate together, expected value: 0.9 * 8000 N = 7200 N
        assert!((limit - 7200.0).abs() < 1e-9, "Calculated limit: {}", limit);
    }
}
