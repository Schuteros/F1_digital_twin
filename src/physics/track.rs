//! The track: braking zones (docs/09) and the XY line the car drives along (docs/15)

use std::f64::consts::PI;

/// Struct that holds information about the track on which the car is tested.
///
/// A track file gives either braking zones or XY points (docs/15 section 15.2). With braking
/// zones only, `points`, `distances` and `curvatures` are empty and `lap_length` is 0.
#[derive(Debug, Clone, PartialEq)]
pub struct Track {
    /// Braking zone starts and ends in meters, m
    pub braking_zones: Vec<(f64, f64)>, // start-end in meters, m
    /// Points (x_i, y_i) of the line the car drives, in driving order, in meters, m.
    /// The line is a closed loop: the last point joins point 0, the start/finish line.
    pub points: Vec<(f64, f64)>,
    /// Distance along the line from point 0 to each point (s_i) in meters, m (docs/15, equation 15.2)
    pub distances: Vec<f64>,
    /// Smoothed curvature at each point in 1/m, positive for a left turn (docs/15, equations
    /// 15.4 and 15.6)
    pub curvatures: Vec<f64>,
    /// Lap length (L_lap) in meters, m (docs/15, equation 15.2). 0 without XY points.
    pub lap_length: f64,
}

impl Default for Track {
    fn default() -> Self {
        Self {
            braking_zones: vec![(900.0, 1000.0)],
            points: Vec::new(),
            distances: Vec::new(),
            curvatures: Vec::new(),
            lap_length: 0.0,
        }
    }
}

/// Where the car is on the XY line
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackPosition {
    /// Index i of the segment the car is on, from point i to point i+1
    pub segment: usize,
    /// How far along the segment the car is, 0.0 at point i to 1.0 at point i+1
    pub fraction: f64,
    /// Position in meters, m
    pub x: f64,
    /// Position in meters, m
    pub y: f64,
    /// Smoothed curvature at the car in 1/m, positive for a left turn
    pub curvature: f64,
}

impl Track {
    /// Builds a track from the XY points of its line (docs/15): calculates the distance to
    /// each point, the lap length and the curvature, smoothed over `curvature_smoothing_window`
    /// meters (0 = no smoothing). The track has no braking zones.
    ///
    /// Returns a message if there are fewer than 3 points, a point is not finite, two points in
    /// a row are at the same place, or the window is negative.
    pub fn from_points(
        points: Vec<(f64, f64)>,
        curvature_smoothing_window: f64,
    ) -> Result<Self, String> {
        if points.len() < 3 {
            return Err(format!(
                "track needs at least 3 points, got {}",
                points.len()
            ));
        }

        if let Some(index) = points
            .iter()
            .position(|&(x, y)| !x.is_finite() || !y.is_finite())
        {
            return Err(format!(
                "track point {} must be finite, got ({}, {})",
                index, points[index].0, points[index].1
            ));
        }

        if !curvature_smoothing_window.is_finite() || curvature_smoothing_window < 0.0 {
            return Err(format!(
                "curvature_smoothing_window must be >= 0 m, got {}",
                curvature_smoothing_window
            ));
        }

        let segment_lengths = calculate_segment_lengths(&points);
        if let Some(index) = segment_lengths.iter().position(|&length| length <= 0.0) {
            return Err(format!(
                "track points {} and {} are at the same place",
                index,
                (index + 1) % points.len()
            ));
        }

        let (distances, lap_length) = calculate_point_distances(&segment_lengths);
        let curvatures = calculate_curvatures(&points, &segment_lengths);
        let curvatures = smooth_curvatures(
            &curvatures,
            &distances,
            lap_length,
            curvature_smoothing_window,
        );

        Ok(Self {
            braking_zones: Vec::new(),
            points,
            distances,
            curvatures,
            lap_length,
        })
    }

    /// True if the track has an XY line, so the car has a position, a curvature and laps
    pub fn has_line(&self) -> bool {
        !self.points.is_empty()
    }
}

/// Calculates the length of every segment in meters, m (docs/15, equation 15.1).
/// Segment i goes from point i to point i+1, the last one back to point 0.
fn calculate_segment_lengths(points: &[(f64, f64)]) -> Vec<f64> {
    (0..points.len())
        .map(|index| {
            let (x, y) = points[index];
            let (next_x, next_y) = points[(index + 1) % points.len()];
            (next_x - x).hypot(next_y - y)
        })
        .collect()
}

/// Calculates the distance along the line to every point and the lap length in meters, m
/// (docs/15, equation 15.2)
fn calculate_point_distances(segment_lengths: &[f64]) -> (Vec<f64>, f64) {
    let mut distances = Vec::with_capacity(segment_lengths.len());
    let mut distance = 0.0;

    for segment_length in segment_lengths {
        distances.push(distance);
        distance += segment_length;
    }

    (distances, distance)
}

/// Calculates the curvature at every point in 1/m, positive for a left turn: the change of
/// heading between the segment before and the segment after the point, divided by half of
/// both segment lengths (docs/15, equations 15.3 and 15.4)
fn calculate_curvatures(points: &[(f64, f64)], segment_lengths: &[f64]) -> Vec<f64> {
    let count = points.len();
    let headings: Vec<f64> = (0..count)
        .map(|index| {
            let (x, y) = points[index];
            let (next_x, next_y) = points[(index + 1) % count];
            (next_y - y).atan2(next_x - x)
        })
        .collect();

    (0..count)
        .map(|index| {
            let previous = (index + count - 1) % count;
            let heading_change = wrap_angle(headings[index] - headings[previous]);
            heading_change / (0.5 * (segment_lengths[previous] + segment_lengths[index]))
        })
        .collect()
}

/// Brings an angle in radians into -pi..=pi, so a turn across the +-pi heading is not seen
/// as a full spin
fn wrap_angle(angle: f64) -> f64 {
    let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
    // rem_euclid puts +pi at -pi. A half turn is the same either way, keep the sign given.
    if wrapped == -PI && angle > 0.0 {
        PI
    } else {
        wrapped
    }
}

/// Smooths the curvature with a moving average: each point gets the average of all points
/// within `window / 2` meters along the line, wrapping around the start/finish line
/// (docs/15, equation 15.6). A window of 0 returns the curvatures unchanged.
fn smooth_curvatures(
    curvatures: &[f64],
    distances: &[f64],
    lap_length: f64,
    window: f64,
) -> Vec<f64> {
    if window <= 0.0 {
        return curvatures.to_vec();
    }

    let half_window = 0.5 * window;

    (0..curvatures.len())
        .map(|index| {
            let mut sum = 0.0;
            let mut count = 0;
            for (other, &curvature) in curvatures.iter().enumerate() {
                // Shortest distance between the two points around the closed loop
                let direct = (distances[other] - distances[index]).abs();
                let around = direct.min(lap_length - direct);
                if around <= half_window {
                    sum += curvature;
                    count += 1;
                }
            }
            // The point itself is always in the window, so count >= 1
            sum / count as f64
        })
        .collect()
}

/// Finds where the car is on the XY line at `lap_distance` meters from the start/finish line
/// on the current lap (s, docs/15 equation 15.8): the segment, and x, y and curvature
/// interpolated linearly between the two points of the segment (docs/15 section 15.6).
///
/// The search starts at `segment`, the segment found in the previous step, and moves forward.
/// If the car is before that segment (a new lap has started), it starts again from point 0.
pub(crate) fn find_track_position(
    track: &Track,
    lap_distance: f64,
    segment: usize,
) -> TrackPosition {
    let (segment, fraction) = find_track_segment(track, lap_distance, segment);
    let next = (segment + 1) % track.points.len();

    let (x, y) = track.points[segment];
    let (next_x, next_y) = track.points[next];

    TrackPosition {
        segment,
        fraction,
        x: x + fraction * (next_x - x),
        y: y + fraction * (next_y - y),
        curvature: track.curvatures[segment]
            + fraction * (track.curvatures[next] - track.curvatures[segment]),
    }
}

/// Finds the segment of the XY line that is `lap_distance` meters from the start/finish line,
/// and how far along that segment it is as a fraction 0.0..=1.0 (docs/15 section 15.6).
///
/// The search starts at `segment` and moves forward. If `lap_distance` is before that segment,
/// it starts again from point 0.
pub(crate) fn find_track_segment(
    track: &Track,
    lap_distance: f64,
    mut segment: usize,
) -> (usize, f64) {
    let count = track.points.len();
    if segment >= count || lap_distance < track.distances[segment] {
        segment = 0;
    }
    while segment + 1 < count && lap_distance >= track.distances[segment + 1] {
        segment += 1;
    }

    let segment_end = if segment + 1 == count {
        track.lap_length
    } else {
        track.distances[segment + 1]
    };
    let segment_length = segment_end - track.distances[segment];
    let fraction = ((lap_distance - track.distances[segment]) / segment_length).clamp(0.0, 1.0);

    (segment, fraction)
}

/// Time in seconds, s, when the car crossed `line_distance` inside a time step that took it
/// from `old_distance` at `time` to `new_distance` at `time + time_step`, assuming constant
/// speed inside the step (docs/15, equation 15.9)
pub(crate) fn calculate_crossing_time(
    time: f64,
    time_step: f64,
    line_distance: f64,
    old_distance: f64,
    new_distance: f64,
) -> f64 {
    time + time_step * (line_distance - old_distance) / (new_distance - old_distance)
}

/// Function checks in which active braking zone are we
pub(crate) fn find_active_braking_zone(
    distance: f64,
    braking_zones: &[(f64, f64)],
    mut active_braking_zone: usize,
) -> usize {
    // Loop so that several zones passed within a single (large) time step are all skipped
    while let Some(&(_, end)) = braking_zones.get(active_braking_zone) {
        if distance > end && active_braking_zone + 1 < braking_zones.len() {
            active_braking_zone += 1;
        } else {
            break;
        }
    }
    active_braking_zone
}

/// Function checks if there are braking zones or not
pub(crate) fn is_braking_zone(
    braking_zones: &[(f64, f64)],
    position: f64,
    active_zone: usize,
) -> bool {
    if let Some(&(start, end)) = braking_zones.get(active_zone) {
        position >= start && position <= end
    } else {
        false
    }
}

/// Test oval of docs/15 section 15.9, built in code so tests do not need the data file: two
/// 75 m straights and two half circles of 15 m radius, points about 1 m apart, anticlockwise
/// from (0, 0) along +x. Points 0 to 74 are the first straight, 75 to 121 the first corner.
#[cfg(test)]
pub(crate) fn oval_test_track() -> Track {
    let (radius, straight, arc_segments) = (15.0, 75.0, 47);
    let mut points = Vec::new();
    points.extend((0..75).map(|index| (index as f64, 0.0)));
    points.extend((0..arc_segments).map(|index| {
        let angle = -PI / 2.0 + PI * index as f64 / arc_segments as f64;
        (
            straight + radius * angle.cos(),
            radius + radius * angle.sin(),
        )
    }));
    points.extend((0..75).map(|index| (straight - index as f64, 2.0 * radius)));
    points.extend((0..arc_segments).map(|index| {
        let angle = PI / 2.0 + PI * index as f64 / arc_segments as f64;
        (radius * angle.cos(), radius + radius * angle.sin())
    }));
    Track::from_points(points, 0.0).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Points of a circle of `radius` meters around (0, 0), anticlockwise from (radius, 0)
    fn circle_points(radius: f64, count: usize) -> Vec<(f64, f64)> {
        (0..count)
            .map(|index| {
                let angle = 2.0 * PI * index as f64 / count as f64;
                (radius * angle.cos(), radius * angle.sin())
            })
            .collect()
    }

    /// Points 1 m apart on a 100 m x 20 m rectangle, anticlockwise from (0, 0) along +x,
    /// so the first 100 m are a straight along the x axis
    fn rectangle_points() -> Vec<(f64, f64)> {
        let mut points = Vec::new();
        points.extend((0..100).map(|index| (index as f64, 0.0)));
        points.extend((0..20).map(|index| (100.0, index as f64)));
        points.extend((0..100).map(|index| (100.0 - index as f64, 20.0)));
        points.extend((0..20).map(|index| (0.0, 20.0 - index as f64)));
        points
    }

    #[test]
    fn test_square_distances() {
        // docs/15 section 15.10: square 10 m x 10 m, L_lap = 40 m, s = 0, 10, 20, 30
        let points = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let track = Track::from_points(points, 0.0).unwrap();

        assert_eq!(track.lap_length, 40.0);
        assert_eq!(track.distances, vec![0.0, 10.0, 20.0, 30.0]);
        assert!(track.braking_zones.is_empty());
        assert!(track.has_line());
        assert!(!Track::default().has_line());

        // Every corner turns left by 90 degrees over 10 m: (pi / 2) / 10
        for curvature in &track.curvatures {
            assert!((curvature - PI / 20.0).abs() < 1e-12, "{}", curvature);
        }
    }

    #[test]
    fn test_circle_curvature_and_length() {
        // docs/15 section 15.10: circle of 15 m radius with 100 points
        let radius = 15.0;
        let points = circle_points(radius, 100);
        let track = Track::from_points(points.clone(), 0.0).unwrap();

        for curvature in &track.curvatures {
            assert!(
                (curvature * radius - 1.0).abs() < 1e-3,
                "Curvature {} is not 1/15 to 0.1 %",
                curvature
            );
        }
        assert!((track.lap_length / (2.0 * PI * radius) - 1.0).abs() < 1e-3);

        // Driven the other way (clockwise) every corner is a right turn
        let clockwise: Vec<(f64, f64)> = points.into_iter().rev().collect();
        let track = Track::from_points(clockwise, 0.0).unwrap();
        for curvature in &track.curvatures {
            assert!((curvature * radius + 1.0).abs() < 1e-3, "{}", curvature);
        }

        // Smoothing does not change a constant curvature
        let smoothed = Track::from_points(circle_points(radius, 100), 5.0).unwrap();
        for curvature in &smoothed.curvatures {
            assert!((curvature * radius - 1.0).abs() < 1e-3, "{}", curvature);
        }
    }

    #[test]
    fn test_heading_wrap_gives_no_curvature_spike() {
        // The circle starts at heading +90 degrees and passes +-180 degrees a quarter lap
        // later. Every point has the same curvature, so the wrap is not visible.
        let track = Track::from_points(circle_points(15.0, 100), 0.0).unwrap();
        let first = track.curvatures[0];
        for curvature in &track.curvatures {
            assert!(
                (curvature - first).abs() < 1e-9,
                "{} != {}",
                curvature,
                first
            );
        }

        assert!((wrap_angle(3.0 - (-3.0)) - (6.0 - 2.0 * PI)).abs() < 1e-12);
        assert!((wrap_angle(-3.0 - 3.0) - (2.0 * PI - 6.0)).abs() < 1e-12);
        assert_eq!(wrap_angle(0.5), 0.5);
        assert_eq!(wrap_angle(PI), PI);
    }

    #[test]
    fn test_point_error_makes_fake_corner_and_smoothing_removes_it() {
        // docs/15 section 15.5: one point moved sideways by 1 cm on a straight, 1 m spacing
        let mut points = rectangle_points();
        points[50].1 += 0.01;

        // No smoothing: kappa = 2 * 0.01 / 1^2 = 0.02 1/m (equation 15.5), and the two
        // neighbours turn back by half of it each
        let track = Track::from_points(points.clone(), 0.0).unwrap();
        assert!(
            (track.curvatures[50] + 0.02).abs() < 1e-4,
            "{}",
            track.curvatures[50]
        );
        assert!(
            (track.curvatures[49] - 0.01).abs() < 1e-4,
            "{}",
            track.curvatures[49]
        );
        assert!(
            (track.curvatures[51] - 0.01).abs() < 1e-4,
            "{}",
            track.curvatures[51]
        );

        // 5 m window: the three values cancel inside the window
        let track = Track::from_points(points, 5.0).unwrap();
        assert!(
            track.curvatures[50].abs() < 1e-5,
            "{}",
            track.curvatures[50]
        );
        // Worst point: the window edge cuts the three values apart
        let worst = track.curvatures[40..60]
            .iter()
            .fold(0.0_f64, |worst, curvature| worst.max(curvature.abs()));
        assert!(worst < 0.0021, "{}", worst);
    }

    #[test]
    fn test_smoothing_wraps_around_the_start_line() {
        // A circle has the same curvature everywhere, so the windows at point 0 and at the
        // last point must average over points on both sides of the start/finish line
        let curvatures = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0];
        let distances: Vec<f64> = (0..10).map(|index| index as f64).collect();
        let smoothed = smooth_curvatures(&curvatures, &distances, 10.0, 2.0);

        // Point 0 sees points 9, 0 and 1. Point 9 sees points 8, 9 and 0.
        assert!((smoothed[0] - 5.0 / 3.0).abs() < 1e-12);
        assert!((smoothed[9] - 5.0 / 3.0).abs() < 1e-12);
        assert!((smoothed[5] - 0.0).abs() < 1e-12);
        // No window: unchanged
        assert_eq!(
            smooth_curvatures(&curvatures, &distances, 10.0, 0.0),
            curvatures
        );
    }

    #[test]
    fn test_find_track_position() {
        let points = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let mut track = Track::from_points(points, 0.0).unwrap();
        // Different curvature at each point, so the interpolation is visible
        track.curvatures = vec![0.0, 0.1, 0.2, 0.4];

        // At a point
        let position = find_track_position(&track, 10.0, 0);
        assert_eq!(position.segment, 1);
        assert_eq!((position.x, position.y), (10.0, 0.0));
        assert!((position.curvature - 0.1).abs() < 1e-12);

        // Halfway along segment 1
        let position = find_track_position(&track, 15.0, 1);
        assert_eq!(position.segment, 1);
        assert_eq!((position.x, position.y), (10.0, 5.0));
        assert!((position.curvature - 0.15).abs() < 1e-12);

        // The closing segment, from the last point back to point 0
        let position = find_track_position(&track, 37.5, 1);
        assert_eq!(position.segment, 3);
        assert_eq!((position.x, position.y), (0.0, 2.5));
        assert!((position.curvature - 0.1).abs() < 1e-12);

        // New lap: the car is before the segment of the previous step, so the search restarts
        let position = find_track_position(&track, 0.5, 3);
        assert_eq!(position.segment, 0);
        assert_eq!((position.x, position.y), (0.5, 0.0));
    }

    #[test]
    fn test_calculate_crossing_time() {
        // docs/15 equation 15.9: the line is 0.25 of the way through the step
        let time = calculate_crossing_time(2.0, 0.001, 100.0, 99.99, 100.03);
        assert!((time - 2.00025).abs() < 1e-12, "{}", time);
        // Landing exactly on the line: the crossing is at the end of the step
        let time = calculate_crossing_time(2.0, 0.001, 100.0, 99.99, 100.0);
        assert!((time - 2.001).abs() < 1e-12, "{}", time);
    }

    #[test]
    fn test_invalid_points_rejected() {
        let square = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];

        let message = Track::from_points(square[..2].to_vec(), 0.0).unwrap_err();
        assert!(message.contains("at least 3 points, got 2"), "{}", message);

        let mut repeated = square.clone();
        repeated.insert(2, (10.0, 0.0));
        let message = Track::from_points(repeated, 0.0).unwrap_err();
        assert!(
            message.contains("points 1 and 2 are at the same place"),
            "{}",
            message
        );

        // Point 0 repeated at the end: the closing segment has no length
        let mut closed = square.clone();
        closed.push((0.0, 0.0));
        let message = Track::from_points(closed, 0.0).unwrap_err();
        assert!(
            message.contains("points 4 and 0 are at the same place"),
            "{}",
            message
        );

        let mut not_finite = square.clone();
        not_finite[1].0 = f64::NAN;
        let message = Track::from_points(not_finite, 0.0).unwrap_err();
        assert!(message.contains("point 1 must be finite"), "{}", message);

        for window in [-1.0, f64::NAN, f64::INFINITY] {
            let message = Track::from_points(square.clone(), window).unwrap_err();
            assert!(
                message.contains("curvature_smoothing_window"),
                "{}",
                message
            );
        }
    }

    #[test]
    fn test_is_braking_zone() {
        let track = Track {
            braking_zones: vec![(200.0, 300.0), (300.0, 400.0), (450.0, 500.0)], // [(m, m)],
            ..Track::default()
        };

        let mut position_on_track: f64 = 0.0;
        let mut active_zone: usize = 0;

        for _ in 0..20 {
            if track.braking_zones[active_zone].1 < position_on_track
                && track.braking_zones.len() > active_zone + 1
            {
                active_zone += 1;
            }

            if (200.0..=300.0).contains(&position_on_track)
                || (300.0..=400.0).contains(&position_on_track)
                || (450.0..=500.0).contains(&position_on_track)
            {
                assert!(
                    is_braking_zone(&track.braking_zones, position_on_track, active_zone),
                    "Did not register braking zone"
                );
            } else {
                assert!(
                    !is_braking_zone(&track.braking_zones, position_on_track, active_zone),
                    "Registered non existing braking zone"
                );
            }

            position_on_track += 50.0;
        }
    }

    #[test]
    fn test_find_active_braking_zone() {
        let braking_zones = [(51.0, 89.0), (102.0, 202.0), (403.0, 505.0), (703.0, 804.0)];

        let mut distance = 0.0;

        let mut active_braking_zone: usize = 0;

        for _ in 0..20 {
            active_braking_zone =
                find_active_braking_zone(distance, &braking_zones, active_braking_zone);

            if distance <= braking_zones[0].1 {
                assert_eq!(active_braking_zone, 0, "Did not find active braking zone");
            }

            for i in 1..braking_zones.len() {
                if braking_zones[i - 1].1 <= distance && braking_zones[i].1 >= distance {
                    assert_eq!(
                        active_braking_zone, i,
                        "Did not resolve active braking zone"
                    );
                    break;
                }
            }

            distance += 50.0;
        }
    }

    #[test]
    fn test_find_active_braking_zone_skips_multiple_zones() {
        let braking_zones = [(10.0, 20.0), (30.0, 40.0), (50.0, 60.0), (70.0, 80.0)];

        // Car jumped from 0 m to 55 m in one step, so it is now inside the 3rd zone
        let active_braking_zone = find_active_braking_zone(55.0, &braking_zones, 0);

        assert_eq!(active_braking_zone, 2);
        assert!(is_braking_zone(&braking_zones, 55.0, active_braking_zone));
    }
}
