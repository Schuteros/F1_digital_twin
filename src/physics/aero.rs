//! Functions related to the aerodynamic forces (docs/04, docs/13)

use serde::Deserialize;

/// Contains all the info about the aerodynamics of the car
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Aero {
    /// drag coefficient of the car
    pub drag_coefficient: f64,

    /// frontal area of the car in square meters, m^2
    #[serde(rename = "frontal_area_m2")]
    pub frontal_area: f64,

    /// Downforce coefficient C_down (positive = pushes the car down), uses `frontal_area`
    /// as the reference area like the drag coefficient (docs/13, equation 13.1)
    pub downforce_coefficient: f64,

    /// Horizontal distance from the front axle back to the point where the downforce acts
    /// (downforce centre of pressure, x_cp) in meters, m (docs/13 section 13.2)
    #[serde(rename = "downforce_centre_of_pressure_from_front_axle_m")]
    pub downforce_centre_of_pressure_from_front_axle: f64,

    /// Height above the ground of the point where the drag acts (drag centre of pressure,
    /// h_drag) in meters, m (docs/13 section 13.2)
    #[serde(rename = "drag_centre_of_pressure_height_m")]
    pub drag_centre_of_pressure_height: f64,
}

impl Default for Aero {
    fn default() -> Self {
        Self {
            drag_coefficient: 0.35,
            frontal_area: 2.0,
            // Placeholder values: replace with sourced values
            downforce_coefficient: 1.0,
            downforce_centre_of_pressure_from_front_axle: 1.8,
            drag_centre_of_pressure_height: 0.4,
        }
    }
}

/// Aerodynamic forces on the car at one speed in Newtons, N
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct AeroLoads {
    /// Drag force in Newtons, N, positive = pushing the car backwards (when moving forward)
    pub drag: f64,
    /// Downforce in Newtons, N, positive = pushing the car down
    pub downforce: f64,
}

/// Calculates air drag
pub(crate) fn calculate_air_drag(
    air_density: f64,
    drag_coefficient: f64,
    frontal_area: f64,
    speed: f64,
) -> f64 {
    0.5 * air_density * drag_coefficient * frontal_area * speed * speed.abs() // Newtons (N)
}

/// Calculates downforce in Newtons, N: F_down = 0.5 * rho * C_down * A * v^2 (docs/13, equation 13.1).
///
/// Uses v^2 without a sign: the air pushes the car down whichever way it moves.
pub(crate) fn calculate_downforce(
    air_density: f64,
    downforce_coefficient: f64,
    frontal_area: f64,
    speed: f64,
) -> f64 {
    0.5 * air_density * downforce_coefficient * frontal_area * speed * speed
}

/// Calculates drag and downforce at `speed` in meters per second, m/s
pub(crate) fn calculate_aero_loads(aero: &Aero, air_density: f64, speed: f64) -> AeroLoads {
    AeroLoads {
        drag: calculate_air_drag(air_density, aero.drag_coefficient, aero.frontal_area, speed),
        downforce: calculate_downforce(
            air_density,
            aero.downforce_coefficient,
            aero.frontal_area,
            speed,
        ),
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::aero::{
        Aero, calculate_aero_loads, calculate_air_drag, calculate_downforce,
    };

    #[test]
    fn test_downforce_calculations() {
        // 0.5 * 1.225 * 1.0 * 2.0 * 80^2 = 7840 N
        let downforce = calculate_downforce(1.225, 1.0, 2.0, 80.0);
        assert!((downforce - 7840.0).abs() < 1e-9, "Incorrect downforce");

        // Same downforce when moving backwards, zero when standing still
        assert_eq!(calculate_downforce(1.225, 1.0, 2.0, -80.0), downforce);
        assert_eq!(calculate_downforce(1.225, 1.0, 2.0, 0.0), 0.0);
    }

    #[test]
    fn test_aero_loads_use_both_coefficients() {
        let aero = Aero::default();
        let loads = calculate_aero_loads(&aero, 1.225, 50.0);

        // drag: 0.5 * 1.225 * 0.35 * 2.0 * 50^2 = 1071.875 N
        assert!((loads.drag - 1071.875).abs() < 1e-9);
        // downforce: 0.5 * 1.225 * 1.0 * 2.0 * 50^2 = 3062.5 N
        assert!((loads.downforce - 3062.5).abs() < 1e-9);
    }

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
