use crate::physics::chassis::SuspensionState;

#[derive(Debug, Clone, Copy)]
pub struct CarState {
    pub speed: f64,
    pub distance: f64,
    pub gear: u8,
    /// Transmission input speed if the clutch is locked (n_k) in Hertz, Hz (docs/12, equation 12.2)
    pub revs_hz: f64,
    pub active_braking_zone: usize,
    pub braking: bool,
    /// Time left of the gear shift in progress in seconds, s, 0 when not shifting.
    /// The powertrain force is zero while it is above 0 (docs/12 section 12.8).
    pub shift_time_left: f64,
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
            shift_time_left: 0.0,
            acceleration: 0.0,
            suspension: SuspensionState::default(),
        }
    }
}
