use crate::physics::suspension::{CornerLoads, SuspensionState};
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
    /// Body heave, pitch and roll motion, measured from static (docs/11 section 11.1, docs/19
    /// section 19.1)
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
    /// Speed the driver wants to have after the next time step in meters per second, m/s: the
    /// planned speed at the place the car will be then (docs/17 section 17.6). Infinite where
    /// the car never has to lift and on a track without XY points. Calculated from the
    /// distance, the speed and the plan, not in the run file.
    #[serde(skip, default = "no_corner_speed_limit")]
    pub target_speed: f64,
    /// Lateral acceleration the tyres give the car in meters per second squared, m/s^2,
    /// positive to the left, so positive in a left turn (docs/16, equation 16.1, docs/19 section
    /// 19.1). On the line it is what the line needs, over the grip limit it is less and the car
    /// runs wide (docs/20 section 20.5). Output only, not in the run file.
    #[serde(skip)]
    pub lateral_acceleration: f64,
    /// True if the tyres can not make the lateral force the driver asks for, so the car runs
    /// wide (docs/16 section 16.7, docs/20 section 20.5). Output only, not in the run file.
    #[serde(skip)]
    pub over_grip_limit: bool,
    /// Tyre load of each corner in Newtons, N (docs/19, equation 19.15). Output only, not in
    /// the run file.
    #[serde(skip)]
    pub(crate) tyre_loads: CornerLoads,
    /// Offset from the line in meters, m, positive = left of the line (n, docs/20 section
    /// 20.3). The run starts on the line, not in the run file.
    #[serde(skip)]
    pub line_offset: f64,
    /// Angle between the car's direction of travel and the line in rad, positive = pointing
    /// left of the line (psi_e, docs/20 section 20.3). Not in the run file.
    #[serde(skip)]
    pub heading_error: f64,
    /// True while the car is further from the line than the driver's max_line_offset_m
    /// (docs/20 section 20.7). Output only, not in the run file.
    #[serde(skip)]
    pub outside_margin: bool,
    /// Force of the brakes on the car in Newtons, N, this step: the part of the slowing-down
    /// force that drag and rolling resistance don't give, 0 when the brakes are off (docs/23
    /// section 23.5). Output only, not in the run file.
    #[serde(skip)]
    pub brake_force: f64,
}

/// Corner speed limit and target speed of a car state read from a run file, before `prepare`
/// calculates them: none
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
            target_speed: f64::INFINITY,
            lateral_acceleration: 0.0,
            over_grip_limit: false,
            tyre_loads: CornerLoads::default(),
            line_offset: 0.0,
            heading_error: 0.0,
            outside_margin: false,
            brake_force: 0.0,
        }
    }
}
