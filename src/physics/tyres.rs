//! Functions related to tyre physics

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
    use crate::physics::tyres::{
        calculate_current_tyre_friction, calculate_force_static_friction,
        calculate_force_tyre_friction_breakaway, calculate_force_tyre_friction_rolling,
    };

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
}
