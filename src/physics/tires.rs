//! Functions related to tire physics


/// Calculates rolling friction force from tires knowing tire model
pub(crate) fn calculate_force_tyre_friction_rolling(total_normal_force: f64, mu_rolling_friction: f64) -> f64 {
    total_normal_force * mu_rolling_friction
}

/// Calculates breakaway friction force from tires knowing tire model
fn calculate_force_tyre_friction_breakaway(total_normal_force: f64, mu_breakaway_friction: f64) -> f64 {
    total_normal_force * mu_breakaway_friction
}

/// Calculates tire static friction which correlates to accessible traction limit
pub(crate) fn calculate_force_static_friction(axle_normal_force: f64, mu_static_friction: f64) -> f64 {
    axle_normal_force * mu_static_friction
}

/// Calculates tire friction depending on the vehicle speed
pub(crate) fn calculate_current_tire_friction(
    total_normal_force: f64,
    mu_rolling_friction: f64,
    mu_breakaway_friction: f64,
    speed: f64,
) -> f64 {
    if speed != 0.0 {
        calculate_force_tyre_friction_rolling(total_normal_force, mu_rolling_friction)
    } else {
        calculate_force_tyre_friction_breakaway(total_normal_force, mu_breakaway_friction)
    }
}


#[cfg(test)]
mod tests {
    use crate::physics::tires::{calculate_current_tire_friction, calculate_force_static_friction, calculate_force_tyre_friction_breakaway, calculate_force_tyre_friction_rolling};

    #[test]
    fn test_rolling_tire_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_rolling_friction: f64 = 0.03;

        let tire_rolling_friction: f64 = calculate_force_tyre_friction_rolling(normal_force, mu_rolling_friction);

        // Expected value: 100 * 0.03 = 3 N
        assert!((tire_rolling_friction - 3.0).abs() < 1e-3);
    }

    #[test]
    fn test_breakaway_tire_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_breakaway_friction: f64 = 0.9;

        let tire_breakaway_friction: f64 = calculate_force_tyre_friction_breakaway(normal_force, mu_breakaway_friction);

        // Expected value: 100 * 0.9 = 90 N
        assert!((tire_breakaway_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_current_tire_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_rolling_friction: f64 = 0.03;
        let mu_breakaway_friction: f64 = 0.9;
        let mut speed = 10.0; // meters per second m/s

        let mut current_tire_friction: f64 = calculate_current_tire_friction(normal_force, mu_rolling_friction, mu_breakaway_friction, speed);

        // As the speed != 0 then expected value: 100 * 0.03 = 3 N
        assert!((current_tire_friction - 3.0).abs() < 1e-3);

        speed = 0.0;

        current_tire_friction = calculate_current_tire_friction(normal_force, mu_rolling_friction, mu_breakaway_friction, speed);

        // As the speed = 0 then expected value: 100 * 0.9 = 90N
        assert!((current_tire_friction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn test_static_tire_friction_calculations() {
        let normal_force: f64 = 100.0; // Newtons (N)
        let mu_static_friction: f64 = 0.9;

        let tire_static_friction: f64 = calculate_force_static_friction(normal_force, mu_static_friction);

        // Expected value: 100 * 0.9 = 90 N
        assert!((tire_static_friction - 90.0).abs() < 1e-3);
    }
}