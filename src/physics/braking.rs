//! Functions related to braking
//!
//! Underlying science equations are found in this documentation:
//! [Braking Documentation](../../docs/09_Braking.MD)

use crate::physics::loads::AxleLoads;
use crate::physics::tyres::calculate_split_force_limit;

/// Contains the brake system settings
pub struct Brakes {
    /// Fraction of the braking force applied at the front axle, dimensionless, 0.0..=1.0
    /// (rear gets 1 - front_bias)
    pub front_bias: f64,
}

impl Default for Brakes {
    fn default() -> Self {
        Self { front_bias: 0.575 }
    }
}

/// Calculates the total braking force in Newtons, N, for threshold braking with a fixed bias:
/// the brakes are applied until the first axle reaches its traction limit mu * Fz_axle.
pub(crate) fn calculate_brake_force(axle_loads: &AxleLoads, mu_static: f64, front_bias: f64) -> f64 {
    calculate_split_force_limit(axle_loads, mu_static, front_bias)
}

#[cfg(test)]
mod tests {
    use crate::physics::braking::calculate_brake_force;
    use crate::physics::car::Car;
    use crate::physics::environment::Environment;
    use crate::physics::loads::calculate_static_axle_loads;
    use crate::physics::tyres::Tyre;

    #[test]
    fn test_calculate_brake_force() {
        let car = Car::default();
        let tyre = Tyre::default();
        let environment = Environment::default();

        let axle_loads = calculate_static_axle_loads(&car.mass, environment.g_acceleration);

        let brake_force = calculate_brake_force(&axle_loads, tyre.mu_static, car.brakes.front_bias);

        // Static axle loads = 407.747196738 kg * 9.81 m/s^2 = 4000 N on each axle
        // Front axle limit = 0.9 * 4000 N = 3600 N -> total = 3600 N / 0.575 = 6260.8696 N
        // Rear axle limit = 0.9 * 4000 N = 3600 N -> total = 3600 N / 0.425 = 8470.5882 N
        // Front axle locks first, expected value: 6260.8696 N
        assert!(
            (brake_force - 6260.8696).abs() < 1e-3,
            "failed to calculate brake force, calculated force: {}, expected force: {}",
            brake_force,
            6260.8696
        );
    }
}
