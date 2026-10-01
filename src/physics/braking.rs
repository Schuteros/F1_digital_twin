use crate::physics::loads::AxleLoads;

pub(crate) fn calculate_brake_force(axle_loads: &AxleLoads, mu_static: f64) -> f64 {
    axle_loads.front * mu_static + axle_loads.rear * mu_static
}

#[cfg(test)]
mod tests {
    use crate::physics::braking::calculate_brake_force;
    use crate::physics::loads::calculate_static_axle_loads;
    use crate::physics::{Car, Environment, Tyre};

    #[test]
    fn test_calculate_brake_force() {
        let car = Car::default();
        let tyre = Tyre::default();
        let environment = Environment::default();

        let axle_loads = calculate_static_axle_loads(&car.mass, environment.g_acceleration);

        let brake_force = calculate_brake_force(&axle_loads, tyre.mu_static);

        // brake force = 407.747196738 * 9.81 * 0.9 + 407.747196738 * 9.81 * 0.9 = 7200 N
        assert!(
            (brake_force - 7200.0).abs() < 1e-3,
            "failed to calculate brake force, calculated force: {}, expected force: {}",
            brake_force,
            7200.0
        );
    }
}
