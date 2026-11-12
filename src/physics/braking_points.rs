//! Braking points corrected with trial runs of the full simulation before the run
//!
//! Underlying science equations are found in this documentation:
//! [Braking Points From Trial Runs Documentation](../../docs/18_Braking_Points_From_Trial_Runs.MD)

use crate::physics::driver::{calculate_planning_tyre, calculate_target_speed_at};
use crate::physics::simulation::{SimulationConfig, SimulationState, simulation_step};

/// Resolution of the braking point search in meters, m (epsilon_shift, docs/18 section 18.6)
const SHIFT_RESOLUTION: f64 = 0.01;

/// First step of the search for a shift that fails or passes in meters, m. It doubles
/// every try (docs/18 section 18.6, step 2).
const FIRST_SEARCH_STEP: f64 = 1.0;

/// Most times around the lap before the braking points must have settled (docs/18 section 18.7)
const MAX_CORRECTION_LAPS: u32 = 10;

/// Speeds within this fraction above the corner speed limit count as on the limit. Only keeps
/// rounding errors of a car that holds the limit from failing a trial.
const SPEED_LIMIT_TOLERANCE: f64 = 1e-9;

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

/// Like `drive`, but gives false as soon as the car is over the grip limit or faster than the
/// corner speed limit, or if it stops before getting there (docs/18 section 18.5).
fn drive_and_check(
    simulation_state: &mut SimulationState,
    simulation_config: &SimulationConfig,
    distance: f64,
) -> bool {
    let end_distance = simulation_state.car.distance + distance;

    while simulation_state.car.distance < end_distance {
        simulation_step(simulation_state, simulation_config);
        let car = &simulation_state.car;

        if car.over_grip_limit
            || car.speed > car.corner_speed_limit * (1.0 + SPEED_LIMIT_TOLERANCE)
            || car.speed <= 0.0
        {
            return false;
        }
    }

    true
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
) -> bool {
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

/// Finds the latest braking point of one corner that passes a trial run, as a shift from the
/// docs/17 plan in meters, m (docs/18 section 18.6). Leaves the section's target speeds set to
/// that shift.
fn find_braking_point_shift(
    simulation_config: &mut SimulationConfig,
    plan: &[f64],
    saved_state: &SimulationState,
    section: Section,
) -> Result<f64, String> {
    let section_length =
        calculate_section_length(simulation_config, section.start_point, section.end_point);
    let trial = |config: &mut SimulationConfig, shift: f64| {
        run_trial(config, plan, saved_state, section, shift)
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
                return Ok(0.0);
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
                return Err(format!(
                    "the car cannot get through the corner at {:.2} m without going over the \
                     grip limit, even braking {:.0} m earlier (docs/18 section 18.6)",
                    simulation_config.track.distances[section.apex_point], section_length
                ));
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
    Ok(passing)
}

/// Corrects the braking points of the lap plan with trial runs of the full simulation (docs/18
/// section 18.7) and gives them, one per corner in driving order from the start/finish line.
/// `simulation_config` must be prepared up to the docs/17 target speeds and the initial car
/// state. On return its target speeds include the corrected braking points, and everything else
/// is unchanged.
pub(crate) fn correct_braking_points(
    simulation_config: &mut SimulationConfig,
) -> Result<Vec<BrakingPoint>, String> {
    let plan = simulation_config.target_speeds.clone();
    let corners = find_corners(&plan, &simulation_config.corner_speed_limits);
    if corners.is_empty() {
        return Ok(Vec::new());
    }

    // The trial car has the planning tyre (docs/18 section 18.5)
    let planning_tyre =
        calculate_planning_tyre(&simulation_config.car.tyre, &simulation_config.driver);
    let real_tyre = std::mem::replace(&mut simulation_config.car.tyre, planning_tyre);
    let result = correct_braking_points_with_trials(simulation_config, &plan, &corners);
    simulation_config.car.tyre = real_tyre;

    let shifts = result?;
    Ok(corners
        .iter()
        .zip(shifts)
        .map(|(corner, shift)| BrakingPoint {
            apex_distance: simulation_config.track.distances[corner.apex_point],
            shift,
        })
        .collect())
}

/// The laps of docs/18 section 18.7 with the trial tyre already in `simulation_config`
fn correct_braking_points_with_trials(
    simulation_config: &mut SimulationConfig,
    plan: &[f64],
    corners: &[Corner],
) -> Result<Vec<f64>, String> {
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
    for lap in 0..MAX_CORRECTION_LAPS {
        let mut largest_change: f64 = 0.0;

        for offset in 1..=corner_count {
            let corner = (first + offset) % corner_count;
            let section = Section {
                start_point: corners[(corner + corner_count - 1) % corner_count].exit_point,
                apex_point: corners[corner].apex_point,
                end_point: corners[corner].exit_point,
            };

            let shift =
                find_braking_point_shift(simulation_config, plan, &simulation_state, section)?;
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
            return Ok(shifts);
        }
    }

    Err(format!(
        "the braking points did not settle in {} laps (docs/18 section 18.7)",
        MAX_CORRECTION_LAPS
    ))
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

    /// Drives the whole run and counts the steps over the grip limit
    fn count_steps_over_grip_limit(simulation_config: &SimulationConfig) -> (usize, Vec<f64>) {
        let mut simulation_state = SimulationState::new(simulation_config);
        let mut steps = 0;
        while simulation_state.lap_times.len() < 3 {
            simulation_step(&mut simulation_state, simulation_config);
            if simulation_state.car.over_grip_limit {
                steps += 1;
            }
        }
        (steps, simulation_state.lap_times)
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
    fn test_trials_keep_the_car_under_the_grip_limit() {
        // docs/17 without a margin goes over the limit at every corner entry (docs/17
        // section 17.8). With the braking points from trial runs it never does (docs/18
        // section 18.10)
        let (steps_without, _) = count_steps_over_grip_limit(&oval_config(0.0, false));
        assert!(steps_without > 100, "{}", steps_without);

        let simulation_config = oval_config(0.0, true);
        let braking_points = &simulation_config.braking_points;
        assert_eq!(braking_points.len(), 2);
        // Both corners of the oval are the same, so are their braking points
        let shift = braking_points[0].shift;
        assert!((braking_points[1].shift - shift).abs() <= 2.0 * SHIFT_RESOLUTION);
        // The car needs to brake a little earlier than the quasi-static plan, but not much
        assert!(shift > 0.0 && shift < 5.0, "{}", shift);

        let (steps_with, lap_times) = count_steps_over_grip_limit(&simulation_config);
        assert_eq!(steps_with, 0);
        assert!(
            (lap_times[2] - lap_times[1]).abs() < 1e-3,
            "{:?}",
            lap_times
        );
    }

    #[test]
    fn test_braking_later_than_found_goes_over_the_limit() {
        // The search finds the boundary: braking 3 cm later than found is too late
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

        let (steps, _) = count_steps_over_grip_limit(&simulation_config);
        assert!(steps > 0);
    }

    #[test]
    fn test_trials_with_margin() {
        // With a margin the trial car has the planning tyre, so the braking points move the
        // same way, and the real car with its full grip never gets near the limit
        let without = oval_config(0.0, true);
        let with_margin = oval_config(0.05, true);
        let shift = with_margin.braking_points[0].shift;
        assert!(shift > 0.0, "{}", shift);
        assert!((shift - without.braking_points[0].shift).abs() < 0.5);

        let (steps, lap_times) = count_steps_over_grip_limit(&with_margin);
        assert_eq!(steps, 0);
        // Slower than docs/17 with the same margin, which brakes later
        let (_, plan_lap_times) = count_steps_over_grip_limit(&oval_config(0.05, false));
        assert!(lap_times[2] > plan_lap_times[2]);
    }
}
