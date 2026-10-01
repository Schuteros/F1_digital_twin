#[derive(Debug, Clone, Copy)]
pub struct CarState {
    pub speed: f64,
    pub distance: f64,
    pub gear: u8,
    pub revs_hz: f64,
    pub active_braking_zone: usize,
    pub braking: bool,
    /// Longitudinal acceleration in meters per second squared, m/s^2, positive forward.
    /// The next step uses it for load transfer (lagged acceleration, docs/10 section 10.6).
    pub acceleration: f64,
}

impl Default for CarState {
    fn default() -> Self {
        Self {
            speed: 10.0,
            distance: 100.0,
            gear: 1,
            revs_hz: 0.5,
            active_braking_zone: 0,
            braking: false,
            acceleration: 0.0,
        }
    }
}
