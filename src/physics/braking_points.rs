//! Braking points corrected with trial runs of the full simulation before the run
//!
//! Underlying science equations are found in this documentation:
//! [Braking Points From Trial Runs Documentation](../../docs/18_Braking_Points_From_Trial_Runs.MD)

use std::cell::Cell;

use crate::physics::driver::{calculate_planning_tyre, calculate_target_speed_at};
use crate::physics::simulation::{SimulationConfig, SimulationState, simulation_step};

/// Resolution of the braking point search in meters, m (epsilon_shift, docs/18 section 18.6)
const SHIFT_RESOLUTION: f64 = 0.01;

/// First step of the search for a shift that fails or passes in meters, m. It doubles
/// every try (docs/18 section 18.6, step 2).
const FIRST_SEARCH_STEP: f64 = 1.0;

/// Most times around the lap before the braking points must have settled (docs/18 section 18.7)
const MAX_CORRECTION_LAPS: u32 = 10;

/// How much further from the line than at the start of the section a trial car may get in
/// meters, m (epsilon_line, docs/20 section 20.8). A car held exactly on the grip limit runs
/// wide by a centimetre or so, which is still on the line for the plan.
const LINE_TOLERANCE: f64 = 0.01;

/// Target speeds within this fraction below the corner speed limit count as on the limit when
/// the corners are found
const ON_LIMIT_TOLERANCE: f64 = 1e-9;

/// A corner of the lap plan (docs/18 section 18.3)
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Corner {
    /// Slowest point: the first track point of the corner on the corner speed limit, where the
    /// braking ends
    pub apex_point: usize,
    /// Exit: the last track point of the corner on the corner speed limit
    pub exit_point: usize,
}

/// The braking point of one corner, found by trial runs (docs/18)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakingPoint {
    /// Distance of the slowest point of the corner from the start/finish line in meters, m
    /// (s_apex,c)
    pub apex_distance: f64,
    /// How far the braking point was moved from the docs/17 plan in meters, m (s_shift,c),
    /// positive = earlier
    pub shift: f64,
}

/// Finds the corners of a lap plan, in driving order (docs/18 section 18.3). A corner is a run
/// of track points on which the target speed is the corner speed limit, entered from a point
/// where the car is still braking (target lower than the limit and higher than the corner's
/// first point). A track without braking has none.
pub(crate) fn find_corners(target_speeds: &[f64], corner_speed_limits: &[f64]) -> Vec<Corner> {
    let count = target_speeds.len();
    let is_on_limit = |point: usize| {
        target_speeds[point].is_finite()
            && target_speeds[point] >= corner_speed_limits[point] * (1.0 - ON_LIMIT_TOLERANCE)
    };

    let mut corners = Vec::new();
    for apex_point in 0..count {
        let previous = (apex_point + count - 1) % count;
        if !is_on_limit(apex_point)
            || is_on_limit(previous)
            || target_speeds[previous] <= target_speeds[apex_point]
        {
            continue;
        }

        let mut exit_point = apex_point;
        while is_on_limit((exit_point + 1) % count) && (exit_point + 1) % count != apex_point {
            exit_point = (exit_point + 1) % count;
        }
        corners.push(Corner {
            apex_point,
            exit_point,
        });
    }

    corners
}

/// Calculates the target speeds of the points of one section with the braking point moved
/// `shift` meters earlier (docs/18, equation 18.1): the points after `start_point` up to and
/// including `apex_point`, across the start/finish line if needed. `plan` are the target speeds
/// of docs/17, `target_speeds` gets the result.
pub(crate) fn shift_braking_point(
    simulation_config: &SimulationConfig,
    plan: &[f64],
    start_point: usize,
    apex_point: usize,
    shift: f64,
    target_speeds: &mut [f64],
) {
    let track = &simulation_config.track;
    let limits = &simulation_config.corner_speed_limits;
    let count = plan.len();

    let mut point = start_point;
    loop {
        point = (point + 1) % count;
        let lap_distance = (track.distances[point] + shift).rem_euclid(track.lap_length);
        let shifted = calculate_target_speed_at(track, limits, plan, lap_distance, 0);
        target_speeds[point] = limits[point].min(shifted);

        if point == apex_point {
            break;
        }
    }
}

/// Distance along the track from `from_point` forwards to `to_point` in meters, m. A whole lap
/// if they are the same point.
fn calculate_section_length(
    simulation_config: &SimulationConfig,
    from_point: usize,
    to_point: usize,
) -> f64 {
    let track = &simulation_config.track;
    let length =
        (track.distances[to_point] - track.distances[from_point]).rem_euclid(track.lap_length);
    if length > 0.0 {
        length
    } else {
        track.lap_length
    }
}

/// Runs the simulation from `simulation_state` until the car has driven `distance` meters
/// further, or until it stops.
fn drive(
    simulation_state: &mut SimulationState,
    simulation_config: &SimulationConfig,
    distance: f64,
) {
    let end_distance = simulation_state.car.distance + distance;

    while simulation_state.car.distance < end_distance && simulation_state.car.speed > 0.0 {
        simulation_step(simulation_state, simulation_config);
    }
}

/// Result of one trial run (docs/20 section 20.8)
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrialResult {
    /// True if the car stayed within `LINE_TOLERANCE` of its starting distance from the line
    /// and did not stop
    passed: bool,
    /// Largest distance from the line in the trial in meters, m
    max_line_offset: f64,
}

/// Like `drive`, but checks how far the car gets from the line (docs/20 section 20.8). Stops
/// early once the trial has failed: further off than the tolerance, or stopped.
fn drive_and_check(
    simulation_state: &mut SimulationState,
    simulation_config: &SimulationConfig,
    distance: f64,
) -> TrialResult {
    let end_distance = simulation_state.car.distance + distance;
    let allowed_offset = simulation_state.car.line_offset.abs() + LINE_TOLERANCE;
    let mut max_line_offset = simulation_state.car.line_offset.abs();

    while simulation_state.car.distance < end_distance {
        simulation_step(simulation_state, simulation_config);
        let car = &simulation_state.car;
        max_line_offset = max_line_offset.max(car.line_offset.abs());

        if max_line_offset > allowed_offset || car.speed <= 0.0 {
            return TrialResult {
                passed: false,
                max_line_offset,
            };
        }
    }

    TrialResult {
        passed: true,
        max_line_offset,
    }
}

/// One section of the lap: from the exit of the previous corner to the exit of this one
#[derive(Debug, Clone, Copy)]
struct Section {
    /// Exit point of the previous corner, where the section starts
    start_point: usize,
    /// Slowest point of this corner, where the braking ends
    apex_point: usize,
    /// Exit point of this corner, where the section ends
    end_point: usize,
}

/// One trial run of a section with the braking point moved `shift` meters (docs/18 section
/// 18.5): from a copy of the state saved at the start of the section to the exit of the
/// corner, so the corner entry right after the braking is checked too.
/// `simulation_config.target_speeds` is changed for the section.
fn run_trial(
    simulation_config: &mut SimulationConfig,
    plan: &[f64],
    saved_state: &SimulationState,
    section: Section,
    shift: f64,
) -> TrialResult {
    let mut target_speeds = std::mem::take(&mut simulation_config.target_speeds);
    shift_braking_point(
        simulation_config,
        plan,
        section.start_point,
        section.apex_point,
        shift,
        &mut target_speeds,
    );
    simulation_config.target_speeds = target_speeds;

    let mut simulation_state = saved_state.clone();
    let distance =
        calculate_section_length(simulation_config, section.start_point, section.end_point);
    drive_and_check(&mut simulation_state, simulation_config, distance)
}

/// The braking point kept for one corner
#[derive(Debug, Clone, PartialEq)]
struct ShiftSearch {
    /// Shift from the docs/17 plan in meters, m, positive = earlier
    shift: f64,
    /// Warning if no shift passed (docs/20 section 20.8)
    warning: Option<String>,
}

/// Finds the latest braking point of one corner that passes a trial run, as a shift from the
/// docs/17 plan in meters, m (docs/18 section 18.6). Leaves the section's target speeds set to
/// that shift. If no shift up to the section length passes, keeps the one that got least far
/// from the line and gives a warning (docs/20 section 20.8).
fn find_braking_point_shift(
    simulation_config: &mut SimulationConfig,
    plan: &[f64],
    saved_state: &SimulationState,
    section: Section,
) -> ShiftSearch {
    let section_length =
        calculate_section_length(simulation_config, section.start_point, section.end_point);
    // Every failed trial, to fall back on the one that stayed closest to the line
    let closest: Cell<Option<(f64, f64)>> = Cell::new(None);
    let trial = |config: &mut SimulationConfig, shift: f64| {
        let result = run_trial(config, plan, saved_state, section, shift);
        if !result.passed
            && closest
                .get()
                .is_none_or(|(_, offset)| result.max_line_offset < offset)
        {
            closest.set(Some((shift, result.max_line_offset)));
        }
        result.passed
    };

    // Steps 1 and 2: one shift that passes and one that fails
    let mut passing;
    let mut failing;
    if trial(simulation_config, 0.0) {
        passing = 0.0;
        let mut step = FIRST_SEARCH_STEP;
        loop {
            if step > section_length {
                // Even braking a whole section later passes: the car does not reach the
                // braking curve from this state, so the plan stays as it is
                trial(simulation_config, 0.0);
                return ShiftSearch {
                    shift: 0.0,
                    warning: None,
                };
            }
            if trial(simulation_config, -step) {
                passing = -step;
                step *= 2.0;
            } else {
                failing = -step;
                break;
            }
        }
    } else {
        failing = 0.0;
        let mut step = FIRST_SEARCH_STEP;
        loop {
            if step > section_length {
                // No braking point keeps the car on the line: keep the one that got least far
                // from it, and say so (docs/20 section 20.8)
                let (shift, max_line_offset) = closest.get().unwrap_or((0.0, 0.0));
                trial(simulation_config, shift);
                let apex_distance = simulation_config.track.distances[section.apex_point];
                let mut warning = format!(
                    "the car can not keep to the line through the corner at {:.2} m, even braking \
                     {:.0} m earlier: kept the braking point {:+.2} m from the plan, which runs \
                     wide by up to {:.3} m (docs/20 section 20.8)",
                    apex_distance, section_length, shift, max_line_offset
                );
                if max_line_offset > simulation_config.driver.max_line_offset {
                    warning.push_str(&format!(
                        ". That is over max_line_offset_m = {} m, so laps through it will be \
                         invalid",
                        simulation_config.driver.max_line_offset
                    ));
                }
                return ShiftSearch {
                    shift,
                    warning: Some(warning),
                };
            }
            if trial(simulation_config, step) {
                passing = step;
                break;
            }
            failing = step;
            step *= 2.0;
        }
    }

    // Step 3: halve until the two are closer than the resolution
    while (passing - failing).abs() > SHIFT_RESOLUTION {
        let middle = 0.5 * (passing + failing);
        if trial(simulation_config, middle) {
            passing = middle;
        } else {
            failing = middle;
        }
    }

    // Step 4: keep the passing shift, with the section's targets set to it
    trial(simulation_config, passing);
    ShiftSearch {
        shift: passing,
        warning: None,
    }
}

/// Corrects the braking points of the lap plan with trial runs of the full simulation (docs/18
/// section 18.7) and gives them, one per corner in driving order from the start/finish line.
/// `simulation_config` must be prepared up to the docs/17 target speeds and the initial car
/// state. On return its target speeds include the corrected braking points, and everything else
/// is unchanged. Also gives the warnings of docs/20 section 20.8: corners where no braking
/// point keeps the car on the line, and braking points that did not settle.
pub(crate) fn correct_braking_points(
    simulation_config: &mut SimulationConfig,
) -> (Vec<BrakingPoint>, Vec<String>) {
    let plan = simulation_config.target_speeds.clone();
    let corners = find_corners(&plan, &simulation_config.corner_speed_limits);
    if corners.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // The trial car has the planning tyre (docs/18 section 18.5)
    let planning_tyre =
        calculate_planning_tyre(&simulation_config.car.tyre, &simulation_config.driver);
    let real_tyre = std::mem::replace(&mut simulation_config.car.tyre, planning_tyre);
    let result = correct_braking_points_with_trials(simulation_config, &plan, &corners);
    simulation_config.car.tyre = real_tyre;

    let (shifts, warnings) = result;
    let braking_points = corners
        .iter()
        .zip(shifts)
        .map(|(corner, shift)| BrakingPoint {
            apex_distance: simulation_config.track.distances[corner.apex_point],
            shift,
        })
        .collect();
    (braking_points, warnings)
}

/// The laps of docs/18 section 18.7 with the trial tyre already in `simulation_config`
fn correct_braking_points_with_trials(
    simulation_config: &mut SimulationConfig,
    plan: &[f64],
    corners: &[Corner],
) -> (Vec<f64>, Vec<String>) {
    let track = &simulation_config.track;
    let corner_count = corners.len();

    // Step 1: drive the plan from the start state to the exit of the first corner ahead of the
    // car. Not checked: this is the uncorrected plan, only the state at the exit counts.
    let start_lap_distance = simulation_config
        .initial_car_state
        .distance
        .rem_euclid(track.lap_length);
    let first = corners
        .iter()
        .position(|corner| track.distances[corner.exit_point] > start_lap_distance)
        .unwrap_or(0);
    let to_first_exit = (track.distances[corners[first].exit_point] - start_lap_distance)
        .rem_euclid(track.lap_length);

    let mut simulation_state = SimulationState::new(simulation_config);
    drive(&mut simulation_state, simulation_config, to_first_exit);

    // Steps 2 to 4: correct the sections in driving order, lap after lap
    let mut shifts = vec![0.0; corner_count];
    // The warnings of the last time around the lap, one per corner at most
    let mut corner_warnings: Vec<Option<String>> = vec![None; corner_count];
    let collect_warnings =
        |corner_warnings: Vec<Option<String>>| corner_warnings.into_iter().flatten().collect();
    for lap in 0..MAX_CORRECTION_LAPS {
        let mut largest_change: f64 = 0.0;

        for offset in 1..=corner_count {
            let corner = (first + offset) % corner_count;
            let section = Section {
                start_point: corners[(corner + corner_count - 1) % corner_count].exit_point,
                apex_point: corners[corner].apex_point,
                end_point: corners[corner].exit_point,
            };

            let search =
                find_braking_point_shift(simulation_config, plan, &simulation_state, section);
            let shift = search.shift;
            corner_warnings[corner] = search.warning;
            largest_change = largest_change.max((shift - shifts[corner]).abs());
            shifts[corner] = shift;

            // Drive the section once more with the shift that was kept, and save the state
            let distance =
                calculate_section_length(simulation_config, section.start_point, section.end_point);
            drive(&mut simulation_state, simulation_config, distance);
        }

        // The first lap started from the start state, not from a lap before, so it is never
        // the last one
        if lap > 0 && largest_change <= SHIFT_RESOLUTION {
            return (shifts, collect_warnings(corner_warnings));
        }
    }

    // Not settled: keep the last lap's braking points and say so (docs/20 section 20.8)
    let mut warnings: Vec<String> = collect_warnings(corner_warnings);
    warnings.push(format!(
        "the braking points did not settle in {} laps: kept those of the last lap (docs/18 \
         section 18.7)",
        MAX_CORRECTION_LAPS
    ));
    (shifts, warnings)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::driver::Driver;
    use crate::physics::simulation::EndCondition;
    use crate::physics::states::CarState;
    use crate::physics::track::{Track, oval_test_track};

    /// The test oval from the start/finish line, prepared, with or without trial runs
    fn oval_config(grip_margin: f64, braking_point_trials: bool) -> SimulationConfig {
        let mut simulation_config = SimulationConfig {
            track: oval_test_track(),
            driver: Driver {
                grip_margin,
                braking_point_trials,
                ..Default::default()
            },
            initial_car_state: CarState {
                distance: 0.0,
                ..CarState::default()
            },
            end_condition: EndCondition::Laps {
                laps: 3,
                max_time: 120.0,
            },
            ..SimulationConfig::default()
        };
        simulation_config.prepare().unwrap();
        simulation_config
    }

    /// Runs 3 laps and gives the number of steps over the grip limit, the lap times and the
    /// largest distance from the line in meters, m
    fn count_steps_over_grip_limit(simulation_config: &SimulationConfig) -> (usize, Vec<f64>, f64) {
        let mut simulation_state = SimulationState::new(simulation_config);
        let mut steps = 0;
        let mut max_line_offset: f64 = 0.0;
        while simulation_state.lap_times.len() < 3 {
            simulation_step(&mut simulation_state, simulation_config);
            if simulation_state.car.over_grip_limit {
                steps += 1;
            }
            max_line_offset = max_line_offset.max(simulation_state.car.line_offset.abs());
        }
        (steps, simulation_state.lap_times, max_line_offset)
    }

    #[test]
    fn test_find_corners() {
        // Braking from 20 to 10 m/s into a corner whose limit rises again to 15 m/s
        let infinity = f64::INFINITY;
        let limits = [infinity, infinity, 10.0, 10.0, 15.0, infinity];
        let targets = [infinity, 20.0, 10.0, 10.0, 15.0, infinity];
        assert_eq!(
            find_corners(&targets, &limits),
            vec![Corner {
                apex_point: 2,
                exit_point: 4
            }]
        );

        // No braking at all (a circle on the limit everywhere, or no limit): no corners
        assert!(find_corners(&[10.0; 6], &[10.0; 6]).is_empty());
        assert!(find_corners(&[infinity; 6], &[infinity; 6]).is_empty());

        // The oval: two corners half a lap apart, the braking ends at the entry of each half
        // circle (75 m straight, then the circle)
        let simulation_config = oval_config(0.05, false);
        let corners = find_corners(
            &simulation_config.target_speeds,
            &simulation_config.corner_speed_limits,
        );
        let distances = &simulation_config.track.distances;
        assert_eq!(corners.len(), 2);
        assert!((distances[corners[0].apex_point] - 76.0).abs() < 1.0);
        let half_lap = 0.5 * simulation_config.track.lap_length;
        let apart = distances[corners[1].apex_point] - distances[corners[0].apex_point];
        assert!((apart - half_lap).abs() < 0.01, "{}", apart);
    }

    #[test]
    fn test_shift_braking_point() {
        let simulation_config = oval_config(0.05, false);
        let plan = &simulation_config.target_speeds;
        let corners = find_corners(plan, &simulation_config.corner_speed_limits);
        let (start_point, apex_point) = (corners[1].exit_point, corners[0].apex_point);
        let track: &Track = &simulation_config.track;

        // No shift: the plan
        let mut targets = plan.clone();
        shift_braking_point(
            &simulation_config,
            plan,
            start_point,
            apex_point,
            0.0,
            &mut targets,
        );
        for (target, planned) in targets.iter().zip(plan) {
            assert!(
                (target - planned).abs() <= 1e-9 * planned,
                "{} {}",
                target,
                planned
            );
        }

        // 3 m earlier: the braking curve 3 m further back, the corner unchanged
        let mut targets = plan.clone();
        shift_braking_point(
            &simulation_config,
            plan,
            start_point,
            apex_point,
            3.0,
            &mut targets,
        );
        let braking_point = (0..apex_point)
            .rev()
            .find(|&point| plan[point] >= 20.0)
            .unwrap();
        let moved = (0..apex_point)
            .rev()
            .find(|&point| targets[point] >= 20.0)
            .unwrap();
        assert_eq!(braking_point - moved, 3);
        let at = |point: usize| track.distances[point];
        let expected = calculate_target_speed_at(
            track,
            &simulation_config.corner_speed_limits,
            plan,
            at(moved) + 3.0,
            0,
        );
        assert!((targets[moved] - expected).abs() < 1e-9);
        assert!((targets[apex_point] - plan[apex_point]).abs() < 1e-9);
        assert_eq!(targets[apex_point + 10], plan[apex_point + 10]);
    }

    #[test]
    fn test_trials_keep_the_car_on_the_line() {
        // docs/17 without a margin goes over the limit at every corner entry (docs/17
        // section 17.8). With the braking points from trial runs the car stays within 1 cm of
        // the line (docs/20 section 20.8)
        let (steps_without, _, _) = count_steps_over_grip_limit(&oval_config(0.0, false));
        assert!(steps_without > 0, "{}", steps_without);

        let simulation_config = oval_config(0.0, true);
        assert!(simulation_config.braking_point_warnings.is_empty());
        let braking_points = &simulation_config.braking_points;
        assert_eq!(braking_points.len(), 2);
        // Both corners of the oval are the same, so are their braking points
        let shift = braking_points[0].shift;
        assert!((braking_points[1].shift - shift).abs() <= 2.0 * SHIFT_RESOLUTION);
        // Close to the quasi-static plan
        assert!(shift.abs() < 5.0, "{}", shift);

        let (_, lap_times, max_line_offset) = count_steps_over_grip_limit(&simulation_config);
        assert!(max_line_offset < LINE_TOLERANCE, "{}", max_line_offset);
        assert!(
            (lap_times[2] - lap_times[1]).abs() < 1e-3,
            "{:?}",
            lap_times
        );
    }

    #[test]
    fn test_braking_later_than_found_runs_wide() {
        // The search finds the boundary: braking 3 cm later than found is too late, the car
        // gets further than 1 cm from the line
        let mut simulation_config = oval_config(0.0, true);
        let plan = oval_config(0.0, false).target_speeds;
        let corners = find_corners(&plan, &simulation_config.corner_speed_limits);
        for (index, corner) in corners.iter().enumerate() {
            let previous = corners[(index + 1) % corners.len()];
            let shift = simulation_config.braking_points[index].shift - 3.0 * SHIFT_RESOLUTION;
            let mut targets = std::mem::take(&mut simulation_config.target_speeds);
            shift_braking_point(
                &simulation_config,
                &plan,
                previous.exit_point,
                corner.apex_point,
                shift,
                &mut targets,
            );
            simulation_config.target_speeds = targets;
        }

        let (_, _, max_line_offset) = count_steps_over_grip_limit(&simulation_config);
        assert!(max_line_offset > LINE_TOLERANCE, "{}", max_line_offset);
    }

    #[test]
    fn test_trials_with_margin() {
        // With a margin the trial car has the planning tyre, so the braking points move the
        // same way, and the real car with its full grip never gets near the limit
        let without = oval_config(0.0, true);
        let with_margin = oval_config(0.05, true);
        let shift = with_margin.braking_points[0].shift;
        assert!(
            (shift - without.braking_points[0].shift).abs() < 0.5,
            "{}",
            shift
        );

        let (steps, _, max_line_offset) = count_steps_over_grip_limit(&with_margin);
        assert_eq!(steps, 0);
        assert!(max_line_offset < 1e-6, "{}", max_line_offset);
    }

    /// Closed test track with an R 7 m chicane on each long straight (left 40 degrees, right 80,
    /// left 40, all R 7 m) and four 90 degree left turns of R 4.5 m and R 9 m, points 0.5 m
    /// apart, 251.5 m long. Built by driving it: straights and arcs one after the other.
    fn chicane_test_track() -> Track {
        use std::f64::consts::FRAC_PI_2;
        let chicane = 40f64.to_radians();
        // (length of a straight, 0) or (radius, turn angle, positive = left)
        let half: [(f64, f64); 8] = [
            (30.0, 0.0),
            (7.0, -chicane),
            (7.0, 2.0 * chicane),
            (7.0, -chicane),
            (30.0, 0.0),
            (4.5, FRAC_PI_2),
            (25.0, 0.0),
            (9.0, FRAC_PI_2),
        ];
        let step: f64 = 0.5;
        let (mut x, mut y, mut heading) = (0.0f64, 0.0f64, 0.0f64);
        let mut points = vec![(0.0, 0.0)];
        for &(size, angle) in half.iter().chain(half.iter()) {
            if angle == 0.0 {
                for _ in 0..(size / step).round() as usize {
                    x += step * heading.cos();
                    y += step * heading.sin();
                    points.push((x, y));
                }
            } else {
                let count = ((size * angle.abs() / step).round() as usize).max(2);
                let turn = angle / count as f64;
                let chord = 2.0 * size * (turn.abs() / 2.0).sin();
                for _ in 0..count {
                    x += chord * (heading + turn / 2.0).cos();
                    y += chord * (heading + turn / 2.0).sin();
                    heading += turn;
                    points.push((x, y));
                }
            }
        }
        // The last point is back at the first
        points.pop();
        Track::from_points(points, 0.0).unwrap()
    }

    #[test]
    fn test_chicane_runs_without_error() {
        // The R 7 m chicane stopped docs/18 with an error, even with a 10 % margin: the car
        // held exactly on the limit flickers over it by a fraction of a percent. Now it runs a
        // few centimetres from the line at most, all laps valid (docs/20 section 20.8). With the
        // margin the braking points keep moving by a few tenths of a metre from lap to lap, so
        // the trials end with the "did not settle" warning, but every corner is kept on the line.
        for grip_margin in [0.0, 0.1] {
            let mut simulation_config = SimulationConfig {
                track: chicane_test_track(),
                driver: Driver {
                    grip_margin,
                    braking_point_trials: true,
                    ..Default::default()
                },
                initial_car_state: CarState {
                    distance: 0.0,
                    ..CarState::default()
                },
                end_condition: EndCondition::Laps {
                    laps: 3,
                    max_time: 200.0,
                },
                ..SimulationConfig::default()
            };
            simulation_config.prepare().unwrap();
            let warnings = &simulation_config.braking_point_warnings;
            if grip_margin == 0.0 {
                assert!(warnings.is_empty(), "{:?}", warnings);
            }
            assert!(
                warnings
                    .iter()
                    .all(|warning| warning.contains("did not settle")),
                "{:?}",
                warnings
            );

            let (_, lap_times, max_line_offset) = count_steps_over_grip_limit(&simulation_config);
            assert_eq!(lap_times.len(), 3);
            assert!(max_line_offset < 0.1, "{}", max_line_offset);
        }
    }

    #[test]
    fn test_impossible_corner_gives_a_warning() {
        // The plan is made with the real tyre, then the trials get a tyre with half the
        // lateral grip: no braking point keeps the car on the line. The trials still finish,
        // with a warning per corner instead of an error, and with a max offset of 1 micrometre
        // the warning says the laps will be invalid.
        let mut simulation_config = oval_config(0.0, false);
        simulation_config.driver.braking_point_trials = true;
        simulation_config.driver.max_line_offset = 1e-6;
        simulation_config.prepare().unwrap();
        let plan = simulation_config.target_speeds.clone();
        simulation_config.car.tyre.mu_y_ref *= 0.5;
        let (braking_points, warnings) = {
            simulation_config.target_speeds = plan;
            correct_braking_points(&mut simulation_config)
        };
        assert_eq!(braking_points.len(), 2);
        assert!(!warnings.is_empty());
        assert!(
            warnings[0].contains("can not keep to the line"),
            "{}",
            warnings[0]
        );
        assert!(warnings[0].contains("invalid"), "{}", warnings[0]);
    }
}
