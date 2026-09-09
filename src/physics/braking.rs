use crate::physics::forces::calculate_normal_force;
use crate::physics::MassDistribution;

fn calculate_brake_force(mass_distribution: MassDistribution, mu_static_friction: f64, g_acceleration: f64) -> f64 {
    let front_axle_normal_force = calculate_normal_force(mass_distribution.front_axle_mass, g_acceleration);
    let rear_axle_normal_force = calculate_normal_force(mass_distribution.rear_axle_mass, g_acceleration);

    front_axle_normal_force * mu_static_friction + rear_axle_normal_force * mu_static_friction
}


#[cfg(test)]
mod tests {
    use crate::physics::{CarModel, EnvironmentModel, TyreModel};
    use crate::physics::braking::calculate_brake_force;

    #[test]
    fn test_calculate_brake_force() {
        let car_model = CarModel::default();
        let tyre_model = TyreModel::default();
        let enviroment_model = EnvironmentModel::default();

        let brake_force = calculate_brake_force(car_model.mass_distribution, tyre_model.mu_static_friction, enviroment_model.g_acceleration);

        // brake force = 407.747196738 * 9.81 * 0.9 + 407.747196738 * 9.81 * 0.9 = 7200 N
        assert!((brake_force - 7200.0).abs() < 1e-3, "failed to calculate brake force, calculated force: {}, expected force: {}", brake_force, 7200.0);
    }
}