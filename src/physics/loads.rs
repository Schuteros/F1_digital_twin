//! Functions related to axle normal loads

use crate::physics::Mass;
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

#[cfg(test)]
mod tests {
    use crate::physics::Mass;
    use crate::physics::loads::{AxleLoads, calculate_static_axle_loads};

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
}
