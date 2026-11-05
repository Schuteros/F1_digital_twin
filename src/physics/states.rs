use crate::physics::suspension::SuspensionState;
use serde::Deserialize;

/// State of the car at one moment of the simulation.
///
/// In a run file (`[initial_state]`) only the starting conditions are written. The rest
/// follows from them and the config, and `SimulationConfig::prepare` calculates it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CarState {
    /// Speed in meters per second, m/s
    #[serde(rename = "speed_m_per_s")]
    pub speed: f64,
    /// Distance along the track in meters, m
    #[serde(rename = "distance_m")]
    pub distance: f64,
    /// Current gear, 1 is the first gear
    pub gear: u8,
    /// Engine speed if the clutch is locked (n_k) in Hertz, Hz (docs/12, equation 12.2).
    /// Calculated from speed and gear, not in the run file.
    #[serde(skip)]
    pub revs_hz: f64,
    /// Index of the braking zone that is next or currently active.
    /// Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub active_braking_zone: usize,
    /// True while the car is inside a braking zone.
    /// Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub braking: bool,
    /// Time left of the gear shift in progress in seconds, s, 0 when not shifting.
    /// The powertrain force is zero while it is above 0 (docs/12 section 12.8).
    #[serde(rename = "shift_time_left_s")]
    pub shift_time_left: f64,
    /// Longitudinal acceleration in meters per second squared, m/s^2, positive forward.
    /// Output only: load transfer now comes from the suspension state (docs/11 section 11.8).
    /// Calculated from the rest of the state, not in the run file.
    #[serde(skip)]
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
