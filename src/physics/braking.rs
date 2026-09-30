use crate::physics::Mass;
use crate::physics::forces::calculate_normal_force;

pub(crate) fn calculate_brake_force(
    mass: &Mass,
    mu_static: f64,
    g_acceleration: f64,
) -> f64 {
    let front_axle_normal_force =
        calculate_normal_force(mass.front, g_acceleration);
    let rear_axle_normal_force =
        calculate_normal_force(mass.rear, g_acceleration);

    front_axle_normal_force * mu_static + rear_axle_normal_force * mu_static
}

#[cfg(test)]
mod tests {
    use crate::physics::braking::calculate_brake_force;
    use crate::physics::{Car, Environment, Tyre};

    #[test]
    fn test_calculate_brake_force() {
        let car = Car::default();
        let tyre = Tyre::default();
        let environment = Environment::default();

        let brake_force = calculate_brake_force(
            &car.mass,
            tyre.mu_static,
            environment.g_acceleration,
        );

        // brake force = 407.747196738 * 9.81 * 0.9 + 407.747196738 * 9.81 * 0.9 = 7200 N
        assert!(
            (brake_force - 7200.0).abs() < 1e-3,
            "failed to calculate brake force, calculated force: {}, expected force: {}",
            brake_force,
            7200.0
        );
    }
}
