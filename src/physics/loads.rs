//! Functions related to axle normal loads

use crate::physics::car::{ChassisGeometry, Mass};
use crate::physics::forces::calculate_normal_force;

/// Normal forces acting on each axle
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxleLoads {
    /// Normal force on the front axle in Newtons, N
    pub front: f64,
    /// Normal force on the rear axle in Newtons, N
    pub rear: f64,
}

impl AxleLoads {
    /// Total normal force on both axles in Newtons, N
    pub(crate) fn total(&self) -> f64 {
        self.front + self.rear
    }
}

/// Calculates static axle normal forces from the mass distribution, without load transfer
pub(crate) fn calculate_static_axle_loads(mass: &Mass, g_acceleration: f64) -> AxleLoads {
    AxleLoads {
        front: calculate_normal_force(mass.front, g_acceleration),
        rear: calculate_normal_force(mass.rear, g_acceleration),
    }
}

/// Calculates axle normal forces including steady-state longitudinal load transfer
/// (docs/10, equations 10.2 and 10.3).
///
/// * `g_acceleration` - gravitational acceleration in meters per second squared, m/s^2
/// * `acceleration` - longitudinal acceleration in meters per second squared, m/s^2,
///   positive forward (accelerating moves load to the rear, braking moves load to the front)
///
/// The front load is clamped to [0, total weight] so no axle is ever pulled off the ground
/// with a negative load, and the rear load is the remainder, so front + rear = m * g always.
pub(crate) fn calculate_axle_loads(
    mass: &Mass,
    geometry: &ChassisGeometry,
    g_acceleration: f64,
    acceleration: f64,
) -> AxleLoads {
    let static_loads = calculate_static_axle_loads(mass, g_acceleration);
    let total_weight = calculate_normal_force(mass.total, g_acceleration);
    let load_transfer = mass.total * acceleration * geometry.cog_height / geometry.wheelbase;

    let front = (static_loads.front - load_transfer).clamp(0.0, total_weight);

    AxleLoads {
        front,
        rear: total_weight - front,
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::car::{ChassisGeometry, Mass};
    use crate::physics::loads::{AxleLoads, calculate_axle_loads, calculate_static_axle_loads};

    fn test_mass() -> Mass {
        Mass {
            total: 1000.0, // kilograms (kg)
            rear: 600.0,   // kilograms (kg)
            front: 400.0,  // kilograms (kg)
        }
    }

    fn test_geometry() -> ChassisGeometry {
        ChassisGeometry {
            wheelbase: 3.0,  // meters (m)
            cog_height: 0.3, // meters (m)
        }
    }

    #[test]
    fn test_axle_loads_total() {
        let axle_loads = AxleLoads {
            front: 3000.0, // Newtons (N)
            rear: 5000.0,  // Newtons (N)
        };

        // Expected value: 3000 N + 5000 N = 8000 N
        assert!((axle_loads.total() - 8000.0).abs() < 1e-3);
    }

    #[test]
    fn test_calculate_static_axle_loads() {
        let mass = Mass {
            total: 1000.0, // kilograms (kg)
            rear: 600.0,   // kilograms (kg)
            front: 400.0,  // kilograms (kg)
        };
        let g_acceleration: f64 = 9.81; // meters per second squared (m/s^2)

        let axle_loads = calculate_static_axle_loads(&mass, g_acceleration);

        // Expected value: front = 400 kg * 9.81 m/s^2 = 3924 N
        assert!((axle_loads.front - 3924.0).abs() < 1e-3);
        // Expected value: rear = 600 kg * 9.81 m/s^2 = 5886 N
        assert!((axle_loads.rear - 5886.0).abs() < 1e-3);
    }

    #[test]
    fn test_axle_loads_sum_to_weight() {
        let mass = test_mass();
        let geometry = test_geometry();
        let g_acceleration: f64 = 9.81; // meters per second squared (m/s^2)

        // Total weight = 1000 kg * 9.81 m/s^2 = 9810 N, for any acceleration.
        // +-100 m/s^2 go past the clamp in both directions (one axle fully unloaded).
        for acceleration in [-100.0, -50.0, -9.81, -5.0, -0.1, 0.0, 0.1, 5.0, 9.81, 50.0, 100.0] {
            let axle_loads = calculate_axle_loads(&mass, &geometry, g_acceleration, acceleration);

            assert!(
                (axle_loads.total() - 9810.0).abs() < 1e-9,
                "Loads don't sum to weight at a = {}: {:?}",
                acceleration,
                axle_loads
            );
            assert!(axle_loads.front >= 0.0 && axle_loads.rear >= 0.0);
        }
    }

    #[test]
    fn test_axle_loads_clamped() {
        let mass = test_mass();
        let geometry = test_geometry();
        let g_acceleration: f64 = 9.81;

        // Braking: front = 3924 N + 1000 * 100 * 0.3 / 3 = 13924 N > 9810 N -> clamped to 9810 N
        let braking = calculate_axle_loads(&mass, &geometry, g_acceleration, -100.0);
        assert_eq!(braking.front, 9810.0);
        assert_eq!(braking.rear, 0.0);

        // Accelerating: front = 3924 N - 10000 N < 0 N -> clamped to 0 N
        let accelerating = calculate_axle_loads(&mass, &geometry, g_acceleration, 100.0);
        assert_eq!(accelerating.front, 0.0);
        assert_eq!(accelerating.rear, 9810.0);
    }

    #[test]
    fn test_axle_loads_reduce_to_static() {
        let mass = test_mass();
        let g_acceleration: f64 = 9.81;
        let static_loads = calculate_static_axle_loads(&mass, g_acceleration);

        // No acceleration -> no load transfer
        let no_acceleration = calculate_axle_loads(&mass, &test_geometry(), g_acceleration, 0.0);
        assert!((no_acceleration.front - static_loads.front).abs() < 1e-9);
        assert!((no_acceleration.rear - static_loads.rear).abs() < 1e-9);

        // CoG on the ground -> no pitching couple -> no load transfer at any acceleration
        let flat_geometry = ChassisGeometry {
            wheelbase: 3.0,
            cog_height: 0.0,
        };
        for acceleration in [-20.0, -5.0, 5.0, 20.0] {
            let flat = calculate_axle_loads(&mass, &flat_geometry, g_acceleration, acceleration);
            assert!((flat.front - static_loads.front).abs() < 1e-9);
            assert!((flat.rear - static_loads.rear).abs() < 1e-9);
        }
    }

    #[test]
    fn test_calculate_axle_loads_braking() {
        let acceleration: f64 = -5.0; // meters per second squared (m/s^2), braking

        let axle_loads = calculate_axle_loads(&test_mass(), &test_geometry(), 9.81, acceleration);

        // Load transfer = 1000 kg * (-5 m/s^2) * 0.3 m / 3 m = -500 N
        // Front = 400 kg * 9.81 m/s^2 - (-500 N) = 3924 N + 500 N = 4424 N
        // Rear = 9810 N - 4424 N = 5386 N
        assert!((axle_loads.front - 4424.0).abs() < 1e-9);
        assert!((axle_loads.rear - 5386.0).abs() < 1e-9);
    }

    #[test]
    fn test_calculate_axle_loads_accelerating() {
        let acceleration: f64 = 5.0; // meters per second squared (m/s^2), accelerating

        let axle_loads = calculate_axle_loads(&test_mass(), &test_geometry(), 9.81, acceleration);

        // Load transfer = 1000 kg * 5 m/s^2 * 0.3 m / 3 m = 500 N
        // Front = 3924 N - 500 N = 3424 N
        // Rear = 9810 N - 3424 N = 6386 N
        assert!((axle_loads.front - 3424.0).abs() < 1e-9);
        assert!((axle_loads.rear - 6386.0).abs() < 1e-9);
    }
}
