use crate::physics::chassis::SuspensionState;

#[derive(Debug, Clone, Copy)]
pub struct CarState {
    pub speed: f64,
    pub distance: f64,
    pub gear: u8,
    pub revs_hz: f64,
    pub active_braking_zone: usize,
    pub braking: bool,
    /// Longitudinal acceleration in meters per second squared, m/s^2, positive forward.
    /// Output only: load transfer now comes from the suspension state (docs/11 section 11.8).
    pub acceleration: f64,
    /// Body heave and pitch motion, measured from static (docs/11 section 11.1)
    pub suspension: SuspensionState,
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
            suspension: SuspensionState::default(),
        }
    }
}
