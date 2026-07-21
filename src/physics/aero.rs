//! Functions related to the aerodynamic forces


/// Calculates air drag
pub(crate) fn calculate_air_drag(air_density: f64, drag_coefficient: f64, frontal_area: f64, speed: f64) -> f64 {
    0.5 * air_density * drag_coefficient * frontal_area * speed * speed.abs() // Newtons (N)
}

#[cfg(test)]
mod tests {
    use crate::physics::aero::calculate_air_drag;

    #[test]
    fn test_air_drag_calculations() {
        let air_density: f64 = 1.225; // Air density in kg/m^3
        let drag_coefficient: f64 = 0.35; // Good car drag coefficient
        let frontal_area: f64 = 2.0; // sports car frontal area in m^2
        let speed: f64 = 33.0; // normal cruise speed at m/s

        let air_drag: f64 = calculate_air_drag(air_density, drag_coefficient, frontal_area, speed);
        println!("{}", air_drag);

        // Expected air drag: 0.5 * 1.225 * 0.35 * 2 * 33^2 * (+1) = 466.90875 Newtons (N)
        assert!((air_drag - 466.90875).abs() < 1e-3, "Incorrect air drag");
    }


}