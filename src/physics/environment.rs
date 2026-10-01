//! Environment conditions the car is simulated in

pub struct Environment {
    pub air_density: f64,
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
