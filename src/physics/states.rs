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
    /// Number of times the distance has passed a whole lap length (k, docs/15 section 15.6),
    /// 0 on a track without XY points. Calculated from the distance and the track, not in the
    /// run file.
    #[serde(skip)]
    pub lap: u32,
    /// Index of the track segment the car is on (docs/15 section 15.6).
    /// Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub track_segment: usize,
    /// Position on the track in meters, m, 0 on a track without XY points.
    /// Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub x: f64,
    /// Position on the track in meters, m, 0 on a track without XY points.
    /// Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub y: f64,
    /// Smoothed track curvature at the car in 1/m, positive for a left turn (docs/15), 0 on a
    /// track without XY points. Calculated from the distance and the track, not in the run file.
    #[serde(skip)]
    pub curvature: f64,
    /// Corner speed limit at the car in meters per second, m/s (docs/16 section 16.5): the
    /// lower one of the two points of the track segment. Infinite where there is no limit and
    /// on a track without XY points. Calculated from the distance, the track and the car, not
    /// in the run file.
    #[serde(skip, default = "no_corner_speed_limit")]
    pub corner_speed_limit: f64,
    /// Lateral acceleration the line needs at the current speed in meters per second squared,
    /// m/s^2, always >= 0 (docs/16, equation 16.1). Output only, calculated from the speed and
    /// the curvature, not in the run file.
    #[serde(skip)]
    pub lateral_acceleration: f64,
    /// True if a tyre needs more lateral force than it can make, so in reality the car would
    /// run wide (docs/16 section 16.7). Output only, not in the run file.
    #[serde(skip)]
    pub over_grip_limit: bool,
}

/// Corner speed limit of a car state read from a run file, before `prepare` calculates it:
/// no limit
fn no_corner_speed_limit() -> f64 {
    f64::INFINITY
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
            lap: 0,
            track_segment: 0,
            x: 0.0,
            y: 0.0,
            curvature: 0.0,
            corner_speed_limit: f64::INFINITY,
            lateral_acceleration: 0.0,
            over_grip_limit: false,
        }
    }
}
