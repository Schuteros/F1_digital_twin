//! Environment conditions the car is simulated in

use serde::Deserialize;

/// Contains the air and gravity the car is simulated in
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Environment {
    /// Air density in kilograms per cubic meter, kg/m^3
    #[serde(rename = "air_density_kg_per_m3")]
    pub air_density: f64,
    /// Gravitational acceleration in meters per second squared, m/s^2
    #[serde(rename = "g_acceleration_m_per_s2")]
    pub g_acceleration: f64,
}

impl Default for Environment {
    fn default() -> Self {
        Self {
            air_density: 1.225,
            g_acceleration: 9.81,
        }
    }
}
