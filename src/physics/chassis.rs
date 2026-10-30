//! Suspension model: body heave and pitch on four spring-damper corners (docs/11)
//!
//! All motion is measured from the static equilibrium (docs/11 section 11.2), so the state
//! starts at zero and the static loads never enter the body equations.

use crate::physics::car::{Car, ChassisGeometry, Mass};
use crate::physics::integrators::euler;
use crate::physics::loads::{AxleLoads, calculate_static_axle_loads};
use serde::Deserialize;

/// Spring, damper and end stops of one suspension corner, all measured at the wheel
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CornerSuspension {
    /// Wheel rate (spring rate at the wheel, motion ratio included) in Newtons per meter, N/m
    #[serde(rename = "spring_rate_N_per_m")]
    pub spring_rate: f64,
    /// Damping coefficient at the wheel in Newton seconds per meter, N·s/m
    #[serde(rename = "damping_N_s_per_m")]
    pub damping: f64,
    /// Compression travel from static before the bump stop touches in meters, m
    #[serde(rename = "bump_gap_m")]
    pub bump_gap: f64,
    /// Bump stop stiffness at the wheel in Newtons per meter, N/m
    #[serde(rename = "bump_rate_N_per_m")]
    pub bump_rate: f64,
    /// Extension travel from static before the droop limiter engages in meters, m
    #[serde(rename = "droop_gap_m")]
    pub droop_gap: f64,
    /// Droop stop stiffness at the wheel in Newtons per meter, N/m
    #[serde(rename = "droop_rate_N_per_m")]
    pub droop_rate: f64,
}

impl Default for CornerSuspension {
    fn default() -> Self {
        Self {
            // Placeholder values: replace with sourced values
            spring_rate: 130_000.0,
            damping: 5_000.0,
            bump_gap: 0.015,
            bump_rate: 1_000_000.0,
            droop_gap: 0.030,
            droop_rate: 1_000_000.0,
        }
    }
}

/// Suspension of the whole car: four corners and the pitch inertia of the body.
///
/// CoG to axle distances (d_f, d_r) are not stored here: they follow from the front/rear
/// split in `Mass`, like in docs/10 section 10.6, so there is one source of truth.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Suspension {
    /// Front left corner
    pub front_left: CornerSuspension,
    /// Front right corner
    pub front_right: CornerSuspension,
    /// Rear left corner
    pub rear_left: CornerSuspension,
    /// Rear right corner
    pub rear_right: CornerSuspension,
    /// Pitch moment of inertia of the body about its CoG in kilogram meters squared, kg·m^2
    #[serde(rename = "pitch_inertia_kg_m2")]
    pub pitch_inertia: f64,
    /// Front ride height with the car standing still on level ground in meters, m
    #[serde(rename = "static_ride_height_front_m")]
    pub static_ride_height_front: f64,
    /// Rear ride height with the car standing still on level ground in meters, m
    #[serde(rename = "static_ride_height_rear_m")]
    pub static_ride_height_rear: f64,
}

impl Default for Suspension {
    fn default() -> Self {
        Self {
            front_left: CornerSuspension::default(),
            front_right: CornerSuspension::default(),
            rear_left: CornerSuspension::default(),
            rear_right: CornerSuspension::default(),
            // Placeholder values: replace with sourced values
            pitch_inertia: 1_000.0,
            static_ride_height_front: 0.030,
            static_ride_height_rear: 0.060,
        }
    }
}

/// Body motion measured from the static equilibrium (docs/11 section 11.1)
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionState {
    /// Downward displacement of the CoG from static in meters, m, positive down
    #[serde(rename = "heave_m")]
    pub heave: f64,
    /// Heave velocity in meters per second, m/s, positive down
    #[serde(rename = "heave_velocity_m_per_s")]
    pub heave_velocity: f64,
    /// Rotation of the body from its static attitude in radians, rad, positive nose down
    #[serde(rename = "pitch_rad")]
    pub pitch: f64,
    /// Pitch rate in radians per second, rad/s, positive nose down
    #[serde(rename = "pitch_rate_rad_per_s")]
    pub pitch_rate: f64,
}

/// Horizontal distances from the CoG to the axles
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxleDistances {
    /// Distance from the CoG to the front axle (d_f) in meters, m
    pub front: f64,
    /// Distance from the CoG to the rear axle (d_r) in meters, m
    pub rear: f64,
}

/// Suspension compression and compression speed of each axle (both corners of an axle are equal without roll)
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxleTravel {
    /// Front compression from static in meters, m, positive = compressed
    pub front_compression: f64,
    /// Rear compression from static in meters, m, positive = compressed
    pub rear_compression: f64,
    /// Front compression speed in meters per second, m/s
    pub front_compression_speed: f64,
    /// Rear compression speed in meters per second, m/s
    pub rear_compression_speed: f64,
}

/// Suspension force of each corner on top of its static load in Newtons, N, positive = pushing the body up
#[derive(Debug, Clone, Copy)]
pub(crate) struct CornerForces {
    /// Front left corner force in Newtons, N
    pub front_left: f64,
    /// Front right corner force in Newtons, N
    pub front_right: f64,
    /// Rear left corner force in Newtons, N
    pub rear_left: f64,
    /// Rear right corner force in Newtons, N
    pub rear_right: f64,
}

/// Tyre normal load of each corner in Newtons, N
#[derive(Debug, Clone, Copy)]
pub(crate) struct CornerLoads {
    /// Front left tyre load in Newtons, N
    pub front_left: f64,
    /// Front right tyre load in Newtons, N
    pub front_right: f64,
    /// Rear left tyre load in Newtons, N
    pub rear_left: f64,
    /// Rear right tyre load in Newtons, N
    pub rear_right: f64,
}

/// Suspension force of each axle on top of its static load in Newtons, N, positive = pushing the body up
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxleForces {
    /// Front axle force (F_f) in Newtons, N
    pub front: f64,
    /// Rear axle force (F_r) in Newtons, N
    pub rear: f64,
}

/// Body accelerations (docs/11 section 11.5)
#[derive(Debug, Clone, Copy)]
pub(crate) struct BodyAccelerations {
    /// Heave acceleration in meters per second squared, m/s^2, positive down
    pub heave_acceleration: f64,
    /// Pitch acceleration in radians per second squared, rad/s^2, positive nose down
    pub pitch_acceleration: f64,
}

/// Front and rear ride heights in meters, m
#[allow(dead_code)] // Not used by the simulation until aerodynamics needs ride height
#[derive(Debug, Clone, Copy)]
pub(crate) struct RideHeights {
    /// Front ride height in meters, m
    pub front: f64,
    /// Rear ride height in meters, m
    pub rear: f64,
}

/// Undamped natural frequencies of the uncoupled heave and pitch modes
#[derive(Debug, Clone, Copy)]
pub(crate) struct NaturalFrequencies {
    /// Heave natural frequency in radians per second, rad/s
    pub heave: f64,
    /// Pitch natural frequency in radians per second, rad/s
    pub pitch: f64,
}

/// Damping ratios of the uncoupled heave and pitch modes, dimensionless
#[allow(dead_code)] // Only used by tests for now
#[derive(Debug, Clone, Copy)]
pub(crate) struct DampingRatios {
    /// Heave damping ratio, dimensionless
    pub heave: f64,
    /// Pitch damping ratio, dimensionless
    pub pitch: f64,
}

/// Calculates the CoG to axle distances from the mass split:
/// d_r / W = mass.front / mass.total and d_f / W = mass.rear / mass.total (docs/10 section 10.6)
pub(crate) fn calculate_axle_distances(mass: &Mass, geometry: &ChassisGeometry) -> AxleDistances {
    AxleDistances {
        front: geometry.wheelbase * mass.rear / mass.total,
        rear: geometry.wheelbase * mass.front / mass.total,
    }
}

/// Calculates front compression in meters, m: s_f = z + d_f * theta (docs/11, equation 11.4)
pub(crate) fn calculate_front_compression(heave: f64, pitch: f64, distance_front: f64) -> f64 {
    heave + distance_front * pitch
}

/// Calculates rear compression in meters, m: s_r = z - d_r * theta (docs/11, equation 11.4)
pub(crate) fn calculate_rear_compression(heave: f64, pitch: f64, distance_rear: f64) -> f64 {
    heave - distance_rear * pitch
}

/// Calculates front compression speed in meters per second, m/s:
/// v_s,f = v_z + d_f * omega (docs/11, equation 11.5)
pub(crate) fn calculate_front_compression_speed(
    heave_velocity: f64,
    pitch_rate: f64,
    distance_front: f64,
) -> f64 {
    heave_velocity + distance_front * pitch_rate
}

/// Calculates rear compression speed in meters per second, m/s:
/// v_s,r = v_z - d_r * omega (docs/11, equation 11.5)
pub(crate) fn calculate_rear_compression_speed(
    heave_velocity: f64,
    pitch_rate: f64,
    distance_rear: f64,
) -> f64 {
    heave_velocity - distance_rear * pitch_rate
}

/// Calculates compressions and compression speeds of both axles from the body state
/// (docs/11, equations 11.4 and 11.5)
pub(crate) fn calculate_axle_travel(
    state: &SuspensionState,
    distances: &AxleDistances,
) -> AxleTravel {
    AxleTravel {
        front_compression: calculate_front_compression(state.heave, state.pitch, distances.front),
        rear_compression: calculate_rear_compression(state.heave, state.pitch, distances.rear),
        front_compression_speed: calculate_front_compression_speed(
            state.heave_velocity,
            state.pitch_rate,
            distances.front,
        ),
        rear_compression_speed: calculate_rear_compression_speed(
            state.heave_velocity,
            state.pitch_rate,
            distances.rear,
        ),
    }
}

/// Calculates the end stop force in Newtons, N (docs/11, equation 11.8).
///
/// * `compression` - suspension compression from static in meters, m, positive = compressed
///
/// Positive past the bump gap (pushes the body up), negative past the droop gap
/// (pulls the body down), zero in between.
pub(crate) fn calculate_end_stop_force(corner: &CornerSuspension, compression: f64) -> f64 {
    if compression > corner.bump_gap {
        corner.bump_rate * (compression - corner.bump_gap)
    } else if compression < -corner.droop_gap {
        corner.droop_rate * (compression + corner.droop_gap)
    } else {
        0.0
    }
}

/// Calculates the corner force on top of the static load in Newtons, N, with the wheel lift
/// clamp (docs/11, equations 11.7 and 11.10).
///
/// * `compression` - suspension compression from static in meters, m, positive = compressed
/// * `compression_speed` - compression speed in meters per second, m/s
/// * `static_corner_load` - static tyre load of this corner in Newtons, N
///
/// A tyre can't pull on the road, so the force never goes below `-static_corner_load`
/// (tyre load 0 N). NaN is passed through, not hidden by the clamp.
pub(crate) fn calculate_corner_force(
    corner: &CornerSuspension,
    compression: f64,
    compression_speed: f64,
    static_corner_load: f64,
) -> f64 {
    let force = corner.spring_rate * compression
        + corner.damping * compression_speed
        + calculate_end_stop_force(corner, compression);

    if force < -static_corner_load {
        -static_corner_load
    } else {
        force
    }
}

/// Calculates static tyre load of each corner in Newtons, N: half of the axle's static load
/// (docs/11 section 11.4)
pub(crate) fn calculate_static_corner_loads(mass: &Mass, g_acceleration: f64) -> CornerLoads {
    let static_axle_loads = calculate_static_axle_loads(mass, g_acceleration);
    let front = static_axle_loads.front / 2.0;
    let rear = static_axle_loads.rear / 2.0;

    CornerLoads {
        front_left: front,
        front_right: front,
        rear_left: rear,
        rear_right: rear,
    }
}

/// Calculates the clamped force of every corner from the axle travel
/// (docs/11, equations 11.7 to 11.10)
pub(crate) fn calculate_corner_forces(
    suspension: &Suspension,
    travel: &AxleTravel,
    static_loads: &CornerLoads,
) -> CornerForces {
    CornerForces {
        front_left: calculate_corner_force(
            &suspension.front_left,
            travel.front_compression,
            travel.front_compression_speed,
            static_loads.front_left,
        ),
        front_right: calculate_corner_force(
            &suspension.front_right,
            travel.front_compression,
            travel.front_compression_speed,
            static_loads.front_right,
        ),
        rear_left: calculate_corner_force(
            &suspension.rear_left,
            travel.rear_compression,
            travel.rear_compression_speed,
            static_loads.rear_left,
        ),
        rear_right: calculate_corner_force(
            &suspension.rear_right,
            travel.rear_compression,
            travel.rear_compression_speed,
            static_loads.rear_right,
        ),
    }
}

/// Calculates tyre load of each corner in Newtons, N: static load + corner force
/// (docs/11, equation 11.9)
pub(crate) fn calculate_corner_tyre_loads(
    static_loads: &CornerLoads,
    corner_forces: &CornerForces,
) -> CornerLoads {
    CornerLoads {
        front_left: static_loads.front_left + corner_forces.front_left,
        front_right: static_loads.front_right + corner_forces.front_right,
        rear_left: static_loads.rear_left + corner_forces.rear_left,
        rear_right: static_loads.rear_right + corner_forces.rear_right,
    }
}

/// Calculates axle forces in Newtons, N: F_f = F_FL + F_FR, F_r = F_RL + F_RR
/// (docs/11, equation 11.11)
pub(crate) fn calculate_axle_forces(corner_forces: &CornerForces) -> AxleForces {
    AxleForces {
        front: corner_forces.front_left + corner_forces.front_right,
        rear: corner_forces.rear_left + corner_forces.rear_right,
    }
}

/// Calculates heave and pitch accelerations of the body (docs/11, equations 11.12 and 11.13).
///
/// * `mass_total` - total (sprung) mass in kilograms, kg
/// * `pitch_inertia` - pitch moment of inertia about the CoG in kilogram meters squared, kg·m^2
/// * `cog_height` - CoG height above the ground in meters, m
/// * `longitudinal_acceleration` - a_x in meters per second squared, m/s^2, positive forward
///
/// `axle_forces` must be the clamped values, the same ones that go into the tyre loads.
pub(crate) fn calculate_body_accelerations(
    mass_total: f64,
    pitch_inertia: f64,
    cog_height: f64,
    distances: &AxleDistances,
    longitudinal_acceleration: f64,
    axle_forces: &AxleForces,
) -> BodyAccelerations {
    // m * a_z = -(F_f + F_r) (11.12)
    let heave_acceleration = -(axle_forces.front + axle_forces.rear) / mass_total;
    // J * alpha = -m * a_x * h - d_f * F_f + d_r * F_r (11.13)
    let pitch_moment = -mass_total * longitudinal_acceleration * cog_height
        - distances.front * axle_forces.front
        + distances.rear * axle_forces.rear;

    BodyAccelerations {
        heave_acceleration,
        pitch_acceleration: pitch_moment / pitch_inertia,
    }
}

/// Calculates front and rear ride heights in meters, m: RH = RH_0 - s (docs/11, equation 11.6)
#[allow(dead_code)] // Not used by the simulation until aerodynamics needs ride height
pub(crate) fn calculate_ride_heights(suspension: &Suspension, travel: &AxleTravel) -> RideHeights {
    RideHeights {
        front: suspension.static_ride_height_front - travel.front_compression,
        rear: suspension.static_ride_height_rear - travel.rear_compression,
    }
}

/// Calculates the clamped corner forces from the suspension state (docs/11 section 11.8, steps 1 and 2)
pub(crate) fn calculate_corner_forces_from_state(
    car: &Car,
    state: &SuspensionState,
    g_acceleration: f64,
) -> CornerForces {
    let distances = calculate_axle_distances(&car.mass, &car.geometry);
    let travel = calculate_axle_travel(state, &distances);
    let static_loads = calculate_static_corner_loads(&car.mass, g_acceleration);

    calculate_corner_forces(&car.suspension, &travel, &static_loads)
}

/// Calculates the four corner tyre loads in Newtons, N, from the suspension state
/// (docs/11 section 11.8, steps 1 to 3)
pub(crate) fn calculate_tyre_loads(
    car: &Car,
    state: &SuspensionState,
    g_acceleration: f64,
) -> CornerLoads {
    let corner_forces = calculate_corner_forces_from_state(car, state, g_acceleration);
    let static_loads = calculate_static_corner_loads(&car.mass, g_acceleration);

    calculate_corner_tyre_loads(&static_loads, &corner_forces)
}

/// Sums the corner tyre loads into axle loads: front = FL + FR, rear = RL + RR
pub(crate) fn sum_axle_loads(tyre_loads: &CornerLoads) -> AxleLoads {
    AxleLoads {
        front: tyre_loads.front_left + tyre_loads.front_right,
        rear: tyre_loads.rear_left + tyre_loads.rear_right,
    }
}

/// Calculates heave and pitch accelerations from the suspension state and this step's a_x
/// (docs/11 section 11.8, steps 1, 2 and 5)
///
/// * `longitudinal_acceleration` - a_x in meters per second squared, m/s^2, positive forward
pub(crate) fn calculate_body_accelerations_from_state(
    car: &Car,
    state: &SuspensionState,
    g_acceleration: f64,
    longitudinal_acceleration: f64,
) -> BodyAccelerations {
    let distances = calculate_axle_distances(&car.mass, &car.geometry);
    let corner_forces = calculate_corner_forces_from_state(car, state, g_acceleration);

    calculate_body_accelerations(
        car.mass.total,
        car.suspension.pitch_inertia,
        car.geometry.cog_height,
        &distances,
        longitudinal_acceleration,
        &calculate_axle_forces(&corner_forces),
    )
}

/// Updates heave velocity and pitch rate by acceleration * time step
/// (docs/11 section 11.8, step 6)
pub(crate) fn update_suspension_velocities(
    state: &SuspensionState,
    accelerations: &BodyAccelerations,
    time_step: f64,
) -> SuspensionState {
    SuspensionState {
        heave_velocity: euler(
            state.heave_velocity,
            accelerations.heave_acceleration,
            time_step,
        ),
        pitch_rate: euler(
            state.pitch_rate,
            accelerations.pitch_acceleration,
            time_step,
        ),
        ..*state
    }
}

/// Updates heave and pitch by the **new** velocities * time step (semi-implicit Euler,
/// docs/11 section 11.8, step 7). Call after `update_suspension_velocities`.
pub(crate) fn update_suspension_positions(
    state: &SuspensionState,
    time_step: f64,
) -> SuspensionState {
    SuspensionState {
        heave: euler(state.heave, state.heave_velocity, time_step),
        pitch: euler(state.pitch, state.pitch_rate, time_step),
        ..*state
    }
}

/// Calculates the uncoupled heave and pitch natural frequencies in radians per second, rad/s
/// (docs/11, equation 11.17).
///
/// * `axle_rate_front`, `axle_rate_rear` - axle rates (two corners each) in Newtons per meter, N/m
pub(crate) fn calculate_natural_frequencies(
    mass_total: f64,
    pitch_inertia: f64,
    distances: &AxleDistances,
    axle_rate_front: f64,
    axle_rate_rear: f64,
) -> NaturalFrequencies {
    let pitch_stiffness =
        axle_rate_front * distances.front.powi(2) + axle_rate_rear * distances.rear.powi(2);

    NaturalFrequencies {
        heave: ((axle_rate_front + axle_rate_rear) / mass_total).sqrt(),
        pitch: (pitch_stiffness / pitch_inertia).sqrt(),
    }
}

/// Calculates the uncoupled heave and pitch damping ratios, dimensionless (docs/11, equation 11.18).
///
/// * `axle_rate_front`, `axle_rate_rear` - axle rates (two corners each) in Newtons per meter, N/m
/// * `axle_damping_front`, `axle_damping_rear` - axle damping (two corners each) in Newton seconds per meter, N·s/m
#[allow(dead_code)] // Only used by tests for now
pub(crate) fn calculate_damping_ratios(
    mass_total: f64,
    pitch_inertia: f64,
    distances: &AxleDistances,
    axle_rate_front: f64,
    axle_rate_rear: f64,
    axle_damping_front: f64,
    axle_damping_rear: f64,
) -> DampingRatios {
    let pitch_stiffness =
        axle_rate_front * distances.front.powi(2) + axle_rate_rear * distances.rear.powi(2);
    let pitch_damping =
        axle_damping_front * distances.front.powi(2) + axle_damping_rear * distances.rear.powi(2);

    DampingRatios {
        heave: (axle_damping_front + axle_damping_rear)
            / (2.0 * ((axle_rate_front + axle_rate_rear) * mass_total).sqrt()),
        pitch: pitch_damping / (2.0 * (pitch_stiffness * pitch_inertia).sqrt()),
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::car::{Car, ChassisGeometry, Mass};
    use crate::physics::chassis::{
        AxleDistances, AxleForces, AxleTravel, CornerSuspension, Suspension, SuspensionState,
        calculate_axle_distances, calculate_axle_forces, calculate_axle_travel,
        calculate_body_accelerations, calculate_body_accelerations_from_state,
        calculate_corner_force, calculate_corner_forces_from_state, calculate_damping_ratios,
        calculate_end_stop_force, calculate_front_compression, calculate_front_compression_speed,
        calculate_natural_frequencies, calculate_rear_compression,
        calculate_rear_compression_speed, calculate_ride_heights, calculate_static_corner_loads,
        calculate_tyre_loads, sum_axle_loads, update_suspension_positions,
        update_suspension_velocities,
    };
    use crate::physics::loads::{AxleLoads, calculate_axle_loads};
    use std::f64::consts::PI;

    const G_ACCELERATION: f64 = 9.81; // meters per second squared (m/s^2)
    const TIME_STEP: f64 = 0.001; // seconds (s)

    // Default car used below (placeholder values, see car.rs and chassis.rs):
    // m = 815.494393476 kg, front = rear = 407.747196738 kg -> m * g = 8000 N, 4000 N per axle, 2000 N per corner
    // W = 3.4 m -> d_f = d_r = 1.7 m, h = 0.3 m, J = 1000 kg·m^2
    // Corner: k = 130 000 N/m, c = 5000 N·s/m -> axle rate 260 000 N/m, axle damping 10 000 N·s/m

    /// Steps only the suspension under a constant a_x, in the order of docs/11 section 11.8
    /// (steps 1, 2, 5, 6, 7), so the tests don't depend on the powertrain or the tyres.
    fn step_suspension(
        car: &Car,
        state: &SuspensionState,
        longitudinal_acceleration: f64,
        time_step: f64,
    ) -> SuspensionState {
        let accelerations = calculate_body_accelerations_from_state(
            car,
            state,
            G_ACCELERATION,
            longitudinal_acceleration,
        );
        let state = update_suspension_velocities(state, &accelerations, time_step);

        update_suspension_positions(&state, time_step)
    }

    /// Applies the same change to all four corners
    fn change_all_corners(suspension: &mut Suspension, change: impl Fn(&mut CornerSuspension)) {
        change(&mut suspension.front_left);
        change(&mut suspension.front_right);
        change(&mut suspension.rear_left);
        change(&mut suspension.rear_right);
    }

    /// Axle loads (front = FL + FR, rear = RL + RR) from the suspension state
    fn axle_loads(car: &Car, state: &SuspensionState) -> AxleLoads {
        sum_axle_loads(&calculate_tyre_loads(car, state, G_ACCELERATION))
    }

    /// Settled pitch for a constant a_x with linear springs (docs/11, equations 11.14 and 11.15)
    fn steady_state_pitch(car: &Car, longitudinal_acceleration: f64) -> f64 {
        let suspension = &car.suspension;
        let axle_rate_front =
            suspension.front_left.spring_rate + suspension.front_right.spring_rate;
        let axle_rate_rear = suspension.rear_left.spring_rate + suspension.rear_right.spring_rate;
        let front_force = -car.mass.total * longitudinal_acceleration * car.geometry.cog_height
            / car.geometry.wheelbase;

        (front_force / axle_rate_front - (-front_force) / axle_rate_rear) / car.geometry.wheelbase
    }

    /// Kinetic + spring energy of the body in Joules, J (end stops not touched)
    fn energy(car: &Car, state: &SuspensionState) -> f64 {
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let travel = calculate_axle_travel(state, &distances);
        let suspension = &car.suspension;
        let spring_energy = 0.5
            * (suspension.front_left.spring_rate + suspension.front_right.spring_rate)
            * travel.front_compression.powi(2)
            + 0.5
                * (suspension.rear_left.spring_rate + suspension.rear_right.spring_rate)
                * travel.rear_compression.powi(2);

        0.5 * car.mass.total * state.heave_velocity.powi(2)
            + 0.5 * suspension.pitch_inertia * state.pitch_rate.powi(2)
            + spring_energy
    }

    #[test]
    fn test_calculate_axle_distances() {
        // Default car: d_f = d_r = 3.4 m * 407.747196738 kg / 815.494393476 kg = 1.7 m
        let car = Car::default();
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        assert!((distances.front - 1.7).abs() < 1e-12);
        assert!((distances.rear - 1.7).abs() < 1e-12);

        // Front-light car: d_r = 3 m * 400 kg / 1000 kg = 1.2 m, d_f = 3 m * 600 kg / 1000 kg = 1.8 m
        // Check: static front load m * g * d_r / W = 9810 N * 0.4 = 3924 N = 400 kg * 9.81 m/s^2
        let mass = Mass {
            total: 1000.0,
            rear: 600.0,
            front: 400.0,
        };
        let geometry = ChassisGeometry {
            wheelbase: 3.0,
            cog_height: 0.3,
        };
        let distances = calculate_axle_distances(&mass, &geometry);
        assert!((distances.front - 1.8).abs() < 1e-12);
        assert!((distances.rear - 1.2).abs() < 1e-12);
    }

    #[test]
    fn test_compression_and_compression_speed() {
        // d_f = 1.8 m, d_r = 1.2 m, z = 2 mm, theta = 1 mrad
        // s_f = 0.002 + 1.8 * 0.001 = 0.0038 m, s_r = 0.002 - 1.2 * 0.001 = 0.0008 m
        assert!((calculate_front_compression(0.002, 0.001, 1.8) - 0.0038).abs() < 1e-15);
        assert!((calculate_rear_compression(0.002, 0.001, 1.2) - 0.0008).abs() < 1e-15);

        // v_z = 0.1 m/s, omega = 0.05 rad/s
        // v_s,f = 0.1 + 1.8 * 0.05 = 0.19 m/s, v_s,r = 0.1 - 1.2 * 0.05 = 0.04 m/s
        assert!((calculate_front_compression_speed(0.1, 0.05, 1.8) - 0.19).abs() < 1e-15);
        assert!((calculate_rear_compression_speed(0.1, 0.05, 1.2) - 0.04).abs() < 1e-15);

        // Nose-down pitch compresses the front and extends the rear
        let state = SuspensionState {
            pitch: 0.01,
            ..SuspensionState::default()
        };
        let distances = AxleDistances {
            front: 1.8,
            rear: 1.2,
        };
        let travel = calculate_axle_travel(&state, &distances);
        assert!(travel.front_compression > 0.0 && travel.rear_compression < 0.0);
    }

    #[test]
    fn test_calculate_end_stop_force() {
        let corner = CornerSuspension::default(); // gaps 15 mm / 30 mm, rates 1 000 000 N/m

        // Inside the gaps (edges included): no end stop force
        assert_eq!(calculate_end_stop_force(&corner, 0.0), 0.0);
        assert_eq!(calculate_end_stop_force(&corner, 0.010), 0.0);
        assert_eq!(calculate_end_stop_force(&corner, 0.015), 0.0);
        assert_eq!(calculate_end_stop_force(&corner, -0.020), 0.0);
        assert_eq!(calculate_end_stop_force(&corner, -0.030), 0.0);

        // Bump: 1 000 000 N/m * (0.020 m - 0.015 m) = 5000 N, pushes the body up
        assert!((calculate_end_stop_force(&corner, 0.020) - 5000.0).abs() < 1e-6);
        // Droop: 1 000 000 N/m * (-0.040 m + 0.030 m) = -10 000 N, pulls the body down
        assert!((calculate_end_stop_force(&corner, -0.040) + 10_000.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_corner_force() {
        let corner = CornerSuspension::default();
        let static_corner_load = 2000.0; // Newtons (N)

        // Spring + damper: 130 000 N/m * 0.01 m + 5000 N·s/m * 0.1 m/s = 1300 N + 500 N = 1800 N
        let force = calculate_corner_force(&corner, 0.01, 0.1, static_corner_load);
        assert!((force - 1800.0).abs() < 1e-9);

        // Spring + bump stop: 130 000 * 0.02 + 1 000 000 * (0.02 - 0.015) = 2600 N + 5000 N = 7600 N
        let force = calculate_corner_force(&corner, 0.02, 0.0, static_corner_load);
        assert!((force - 7600.0).abs() < 1e-9);

        // Wheel lift: 130 000 * (-0.02) + 5000 * (-0.1) = -2600 N - 500 N = -3100 N < -2000 N
        // -> clamped to -2000 N, tyre load 0 N
        let force = calculate_corner_force(&corner, -0.02, -0.1, static_corner_load);
        assert_eq!(force, -2000.0);

        // NaN is not hidden by the clamp
        assert!(calculate_corner_force(&corner, f64::NAN, 0.0, static_corner_load).is_nan());
    }

    #[test]
    fn test_calculate_static_corner_loads() {
        let mass = Mass {
            total: 1000.0,
            rear: 600.0,
            front: 400.0,
        };

        // Front: 400 kg * 9.81 m/s^2 / 2 = 1962 N, rear: 600 kg * 9.81 m/s^2 / 2 = 2943 N
        let loads = calculate_static_corner_loads(&mass, G_ACCELERATION);
        assert!((loads.front_left - 1962.0).abs() < 1e-9);
        assert!((loads.front_right - 1962.0).abs() < 1e-9);
        assert!((loads.rear_left - 2943.0).abs() < 1e-9);
        assert!((loads.rear_right - 2943.0).abs() < 1e-9);
    }

    #[test]
    fn test_tyre_loads_from_state() {
        let car = Car::default();
        // Pure pitch 2 mrad: s_f = 1.7 m * 0.002 = 0.0034 m, s_r = -0.0034 m
        // Corner force = 130 000 N/m * 0.0034 m = 442 N
        // Front corners 2000 + 442 = 2442 N, rear corners 2000 - 442 = 1558 N
        let state = SuspensionState {
            pitch: 0.002,
            ..SuspensionState::default()
        };

        let tyre_loads = calculate_tyre_loads(&car, &state, G_ACCELERATION);
        // 1e-6 N: the static axle load is 3999.99999999978 N in floating point, not exactly 4000 N
        assert!((tyre_loads.front_left - 2442.0).abs() < 1e-6);
        assert!((tyre_loads.front_right - 2442.0).abs() < 1e-6);
        assert!((tyre_loads.rear_left - 1558.0).abs() < 1e-6);
        assert!((tyre_loads.rear_right - 1558.0).abs() < 1e-6);

        // Axle sums: front 4884 N, rear 3116 N, total 8000 N
        let loads = sum_axle_loads(&tyre_loads);
        assert!((loads.front - 4884.0).abs() < 1e-6);
        assert!((loads.rear - 3116.0).abs() < 1e-6);
        assert!((loads.total() - 8000.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_body_accelerations() {
        // m = 1000 kg, J = 500 kg·m^2, h = 0.3 m, d_f = 1.8 m, d_r = 1.2 m, a_x = -5 m/s^2
        // F_f = 600 N, F_r = -400 N
        // a_z = -(600 - 400) / 1000 = -0.2 m/s^2
        // J * alpha = -1000 * (-5) * 0.3 - 1.8 * 600 + 1.2 * (-400) = 1500 - 1080 - 480 = -60 N·m
        // alpha = -60 / 500 = -0.12 rad/s^2
        let distances = AxleDistances {
            front: 1.8,
            rear: 1.2,
        };
        let axle_forces = AxleForces {
            front: 600.0,
            rear: -400.0,
        };

        let accelerations =
            calculate_body_accelerations(1000.0, 500.0, 0.3, &distances, -5.0, &axle_forces);
        assert!((accelerations.heave_acceleration + 0.2).abs() < 1e-12);
        assert!((accelerations.pitch_acceleration + 0.12).abs() < 1e-12);
    }

    #[test]
    fn test_first_instant_of_braking() {
        // docs/11 section 11.7: at the first instant the body hasn't moved, the springs and
        // dampers carry nothing and the whole moment goes into pitch acceleration.
        // m * a_x = 815.494393476 kg * (-4 * 9.81 m/s^2) = -32 000 N
        // alpha = -(-32 000 N) * 0.3 m / 1000 kg·m^2 = 9.6 rad/s^2, a_z = 0
        let car = Car::default();
        let state = SuspensionState::default();

        let accelerations = calculate_body_accelerations_from_state(
            &car,
            &state,
            G_ACCELERATION,
            -4.0 * G_ACCELERATION,
        );
        assert_eq!(accelerations.heave_acceleration, 0.0);
        assert!((accelerations.pitch_acceleration - 9.6).abs() < 1e-9);

        // Tyre loads are still static: 4000 N per axle
        let loads = axle_loads(&car, &state);
        assert!((loads.front - 4000.0).abs() < 1e-6);
        assert!((loads.rear - 4000.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_ride_heights() {
        // RH_f = 0.030 m - 0.010 m = 0.020 m, RH_r = 0.060 m - (-0.005 m) = 0.065 m
        let travel = AxleTravel {
            front_compression: 0.010,
            rear_compression: -0.005,
            front_compression_speed: 0.0,
            rear_compression_speed: 0.0,
        };

        let ride_heights = calculate_ride_heights(&Suspension::default(), &travel);
        assert!((ride_heights.front - 0.020).abs() < 1e-15);
        assert!((ride_heights.rear - 0.065).abs() < 1e-15);
    }

    #[test]
    fn test_default_natural_frequencies_and_damping_ratios() {
        let car = Car::default();
        let distances = calculate_axle_distances(&car.mass, &car.geometry);

        // Heave: sqrt(520 000 N/m / 815.494393476 kg) = 25.2517 rad/s = 4.0189 Hz
        // Pitch: sqrt((260 000 * 1.7^2 * 2) N·m / 1000 kg·m^2) = sqrt(1502.8) = 38.7660 rad/s = 6.1698 Hz
        let frequencies = calculate_natural_frequencies(
            car.mass.total,
            car.suspension.pitch_inertia,
            &distances,
            260_000.0,
            260_000.0,
        );
        assert!((frequencies.heave - 25.2517).abs() < 1e-4);
        assert!((frequencies.pitch - 38.7660).abs() < 1e-4);
        assert!((frequencies.heave / (2.0 * PI) - 4.0189).abs() < 1e-4);
        assert!((frequencies.pitch / (2.0 * PI) - 6.1698).abs() < 1e-4);

        // Heave: 20 000 / (2 * sqrt(520 000 * 815.494393476)) = 0.48561
        // Pitch: (10 000 * 1.7^2 * 2) / (2 * sqrt(1 502 800 * 1000)) = 57 800 / 77 531.93 = 0.74550
        let damping_ratios = calculate_damping_ratios(
            car.mass.total,
            car.suspension.pitch_inertia,
            &distances,
            260_000.0,
            260_000.0,
            10_000.0,
            10_000.0,
        );
        assert!((damping_ratios.heave - 0.48561).abs() < 1e-5);
        assert!((damping_ratios.pitch - 0.74550).abs() < 1e-5);
    }

    #[test]
    fn test_static_stays_static() {
        let car = Car::default();
        let mut state = SuspensionState::default();

        // a_x = 0 at static: every force and moment is exactly 0, so nothing ever moves (10 s)
        for _ in 0..10_000 {
            state = step_suspension(&car, &state, 0.0, TIME_STEP);
        }

        assert_eq!(state, SuspensionState::default());
        let tyre_loads = calculate_tyre_loads(&car, &state, G_ACCELERATION);
        let static_loads = calculate_static_corner_loads(&car.mass, G_ACCELERATION);
        assert_eq!(tyre_loads.front_left, static_loads.front_left);
        assert_eq!(tyre_loads.front_right, static_loads.front_right);
        assert_eq!(tyre_loads.rear_left, static_loads.rear_left);
        assert_eq!(tyre_loads.rear_right, static_loads.rear_right);
    }

    #[test]
    fn test_steady_state_matches_docs_10() {
        let car = Car::default();

        // -4 g: F_f = 32 000 N * 0.3 m / 3.4 m = +2823.53 N, s_f = 10.86 mm, theta = 6.388 mrad = 0.366 deg
        // +1 g: F_f = -705.88 N, s_f = -2.71 mm, theta = -1.597 mrad
        for longitudinal_acceleration in [-4.0 * G_ACCELERATION, G_ACCELERATION] {
            let mut state = SuspensionState::default();
            // 3 s: the pitch mode decays as exp(-zeta * omega * t) = exp(-0.7455 * 38.77 * 3) = exp(-87),
            // and the symmetric car never excites heave, so only rounding error is left
            for _ in 0..3_000 {
                state = step_suspension(&car, &state, longitudinal_acceleration, TIME_STEP);
            }

            let loads = axle_loads(&car, &state);
            let expected = calculate_axle_loads(
                &car.mass,
                &car.geometry,
                G_ACCELERATION,
                longitudinal_acceleration,
            );
            assert!(
                (loads.front - expected.front).abs() < 0.1,
                "a_x = {}: front {} vs {}",
                longitudinal_acceleration,
                loads.front,
                expected.front
            );
            assert!((loads.rear - expected.rear).abs() < 0.1);
            // Front + rear = m * g = 8000 N (0.1 N, same tolerance as the axle loads)
            assert!((loads.total() - 8000.0).abs() < 0.1);

            // Settled pitch (11.15). 1e-9 rad is ~1e-7 of the 6.4 mrad value: only rounding is left
            let expected_pitch = steady_state_pitch(&car, longitudinal_acceleration);
            assert!(
                (state.pitch - expected_pitch).abs() < 1e-9,
                "pitch {} vs {}",
                state.pitch,
                expected_pitch
            );
        }

        // Spot check of the hand values at -4 g (docs/11 section 11.6)
        let expected_pitch = steady_state_pitch(&car, -4.0 * G_ACCELERATION);
        assert!((expected_pitch.to_degrees() - 0.366).abs() < 1e-3);
    }

    #[test]
    fn test_oscillations_die_out() {
        let car = Car::default();
        let longitudinal_acceleration = -4.0 * G_ACCELERATION;
        let steady_pitch = steady_state_pitch(&car, longitudinal_acceleration);

        let mut state = SuspensionState::default();
        let mut peaks: Vec<f64> = Vec::new();

        for _ in 0..3_000 {
            let previous = state;
            state = step_suspension(&car, &state, longitudinal_acceleration, TIME_STEP);

            // A peak of the deviation is where the pitch rate changes sign
            if previous.pitch_rate * state.pitch_rate < 0.0 {
                let deviation = (state.pitch - steady_pitch).abs();
                // Below 1e-15 rad the deviation is rounding noise (6.4 mrad * 2.2e-16 ~ 1e-18 rad,
                // with a wide margin), and successive noise "peaks" need not shrink
                if deviation > 1e-15 {
                    peaks.push(deviation);
                }
            }
        }

        // zeta = 0.75 -> each peak is exp(-pi * 0.75 / sqrt(1 - 0.75^2)) = 0.028 times the last,
        // so several peaks stay above the noise floor
        assert!(peaks.len() >= 3, "Too few peaks: {:?}", peaks);
        for pair in peaks.windows(2) {
            assert!(pair[1] < pair[0], "Peak didn't shrink: {:?}", peaks);
        }
        assert!((state.pitch - steady_pitch).abs() < 1e-6);
    }

    #[test]
    fn test_damping_ratio_matches_peak_ratio() {
        let mut car = Car::default();
        // Symmetric car (d_f = d_r, same corners) keeps heave out of it (11.16).
        // zeta = 4 * 670 * 1.7^2 / (2 * sqrt(1 502 800 * 1000)) = 7745.2 / 77 531.9 = 0.0999
        change_all_corners(&mut car.suspension, |corner| corner.damping = 670.0);
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let zeta = calculate_damping_ratios(
            car.mass.total,
            car.suspension.pitch_inertia,
            &distances,
            260_000.0,
            260_000.0,
            1_340.0,
            1_340.0,
        )
        .pitch;
        assert!((zeta - 0.0999).abs() < 1e-4);

        // Expected peak ratio (11.19): exp(-2 * pi * 0.0999 / sqrt(1 - 0.0999^2)) = 0.532
        let expected_ratio = (-2.0 * PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();

        // Free oscillation from 5 mrad: s = 1.7 m * 0.005 = 8.5 mm, inside the 15 mm bump gap
        let mut state = SuspensionState {
            pitch: 0.005,
            ..SuspensionState::default()
        };
        let mut positive_peaks: Vec<f64> = Vec::new();

        // 1 s = about 6 cycles
        for _ in 0..1_000 {
            let previous = state;
            state = step_suspension(&car, &state, 0.0, TIME_STEP);
            assert_eq!(state.heave, 0.0);

            if previous.pitch_rate > 0.0 && state.pitch_rate <= 0.0 {
                positive_peaks.push(state.pitch);
            }
        }

        assert!(
            positive_peaks.len() >= 4,
            "Too few peaks: {:?}",
            positive_peaks
        );
        // 2%: the peak is sampled every 1 ms, which misses the true peak by at most
        // (omega * dt)^2 / 8 = (0.039)^2 / 8 = 0.02%, and semi-implicit Euler shifts the
        // decay per cycle by O(omega * dt)
        for pair in positive_peaks.windows(2) {
            let ratio = pair[1] / pair[0];
            assert!(
                (ratio / expected_ratio - 1.0).abs() < 0.02,
                "Peak ratio {} vs expected {}",
                ratio,
                expected_ratio
            );
        }
    }

    #[test]
    fn test_energy_without_damping() {
        let mut car = Car::default();
        change_all_corners(&mut car.suspension, |corner| corner.damping = 0.0);

        // 5 mrad initial pitch: 8.5 mm travel, no end stops, no wheel lift (1105 N < 2000 N)
        let mut state = SuspensionState {
            pitch: 0.005,
            ..SuspensionState::default()
        };
        let initial_energy = energy(&car, &state);

        let mut energies: Vec<f64> = Vec::new();
        for _ in 0..10_000 {
            state = step_suspension(&car, &state, 0.0, TIME_STEP);
            let current = energy(&car, &state);
            // Theory: semi-implicit Euler wobbles by +-omega * dt / 2 = 38.77 * 0.001 / 2 = 1.9%;
            // 3% leaves margin without hiding a real energy leak
            assert!(
                (current / initial_energy - 1.0).abs() < 0.03,
                "Energy {} vs initial {}",
                current,
                initial_energy
            );
            energies.push(current);
        }

        // One pitch cycle: 2 * pi / 38.766 rad/s = 0.16208 s = 162 steps
        let cycle_steps = (2.0 * PI / 38.766 / TIME_STEP).round() as usize;
        let mean = |values: &[f64]| values.iter().sum::<f64>() / values.len() as f64;
        let first_cycle = mean(&energies[..cycle_steps]);
        let last_cycle = mean(&energies[energies.len() - cycle_steps..]);
        // No drift: the wobble averages out over a cycle (0.1%, well above the ~0.01% left by
        // averaging over a whole number of steps instead of exactly one period)
        assert!(
            (last_cycle / first_cycle - 1.0).abs() < 1e-3,
            "Energy drift: first cycle {} vs last cycle {}",
            first_cycle,
            last_cycle
        );
    }

    #[test]
    fn test_pitch_frequency() {
        let mut car = Car::default();
        change_all_corners(&mut car.suspension, |corner| corner.damping = 0.0);
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let pitch_frequency = calculate_natural_frequencies(
            car.mass.total,
            car.suspension.pitch_inertia,
            &distances,
            260_000.0,
            260_000.0,
        )
        .pitch;
        // T = 2 * pi / 38.766 rad/s = 0.16208 s
        let expected_period = 2.0 * PI / pitch_frequency;

        let mut state = SuspensionState {
            pitch: 0.005,
            ..SuspensionState::default()
        };
        let mut crossing_times: Vec<f64> = Vec::new();

        for step in 1..=10_000 {
            let previous = state;
            state = step_suspension(&car, &state, 0.0, TIME_STEP);

            // Upward zero crossing, time found by linear interpolation between the two steps
            if previous.pitch < 0.0 && state.pitch >= 0.0 {
                let fraction = -previous.pitch / (state.pitch - previous.pitch);
                crossing_times.push((step as f64 - 1.0 + fraction) * TIME_STEP);
            }
        }

        // 10 s / 0.162 s = 61 cycles
        assert!(crossing_times.len() > 50);
        let measured_period = (crossing_times[crossing_times.len() - 1] - crossing_times[0])
            / (crossing_times.len() - 1) as f64;
        // 0.5%: semi-implicit Euler shifts the frequency by (omega * dt)^2 / 24 = 0.006%, and the
        // interpolated crossing times are off by far less than one step over 61 cycles
        assert!(
            (measured_period / expected_period - 1.0).abs() < 0.005,
            "Period {} vs expected {}",
            measured_period,
            expected_period
        );
    }

    #[test]
    fn test_symmetry_decouples_heave() {
        // Default car meets 11.16: k_f * d_f = k_r * d_r and c_f * d_f = c_r * d_r
        let car = Car::default();
        let mut state = SuspensionState::default();

        for _ in 0..3_000 {
            state = step_suspension(&car, &state, -4.0 * G_ACCELERATION, TIME_STEP);
            assert!(state.heave.abs() < 1e-12, "Heave {}", state.heave);
        }
    }

    #[test]
    fn test_coupling_moves_heave() {
        let mut car = Car::default();
        // Front springs 1.5x the rear breaks 11.16
        car.suspension.front_left.spring_rate = 195_000.0;
        car.suspension.front_right.spring_rate = 195_000.0;

        let mut state = SuspensionState::default();
        for _ in 0..5_000 {
            state = step_suspension(&car, &state, -4.0 * G_ACCELERATION, TIME_STEP);
        }

        // Settled: s_f = 2823.53 N / 390 000 N/m = 7.240 mm, s_r = -2823.53 N / 260 000 N/m = -10.860 mm
        // z = (s_f + s_r) / 2 = -1.810 mm (body rises), from 11.4 with d_f = d_r
        // 1e-6 m: after 5 s both coupled modes have decayed far below that
        assert!(state.heave.abs() > 1e-3);
        assert!(
            (state.heave + 0.0018100).abs() < 1e-6,
            "Heave {}",
            state.heave
        );
    }

    #[test]
    fn test_bump_stop() {
        let mut car = Car::default();
        // Soft springs so the bump stop is reached at -4 g without lifting the rear wheels
        change_all_corners(&mut car.suspension, |corner| corner.spring_rate = 50_000.0);
        let bump_gap = car.suspension.front_left.bump_gap;
        let bump_rate = car.suspension.front_left.bump_rate;
        let spring_rate = car.suspension.front_left.spring_rate;

        // Settled front corner force = 2823.53 N / 2 = 1411.76 N
        let steady_corner_force = car.mass.total * 4.0 * G_ACCELERATION * car.geometry.cog_height
            / car.geometry.wheelbase
            / 2.0;
        // Bound: an undamped step load gives at most twice its steady force, so
        // excess force = 2 * 1411.76 N - 50 000 N/m * 0.015 m = 2073.5 N past the gap, and
        // s_f <= 0.015 m + 2073.5 N / 1 000 000 N/m = 17.07 mm (the spring in parallel makes it smaller still)
        let excess_force = 2.0 * steady_corner_force - spring_rate * bump_gap;
        let compression_limit = bump_gap + excess_force / bump_rate;

        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let mut state = SuspensionState::default();
        let mut max_front_compression: f64 = 0.0;

        for _ in 0..5_000 {
            state = step_suspension(&car, &state, -4.0 * G_ACCELERATION, TIME_STEP);
            let travel = calculate_axle_travel(&state, &distances);

            assert!(state.heave.is_finite() && state.pitch.is_finite());
            assert!(state.heave_velocity.is_finite() && state.pitch_rate.is_finite());
            // 1e-6 m margin covers rounding only, the bound itself is conservative
            assert!(
                travel.front_compression < compression_limit + 1e-6,
                "Front compression {} m past limit {} m",
                travel.front_compression,
                compression_limit
            );
            max_front_compression = max_front_compression.max(travel.front_compression);
        }

        // The bump stop was actually reached
        assert!(max_front_compression > bump_gap);

        // Settled on the bump stop: 50 000 * s + 1 000 000 * (s - 0.015) = 1411.76 N
        // -> s = (1411.76 + 15 000) / 1 050 000 = 15.630 mm. 1e-6 m: decayed after 5 s
        let travel = calculate_axle_travel(&state, &distances);
        assert!(
            (travel.front_compression - 0.015630).abs() < 1e-6,
            "Front compression {}",
            travel.front_compression
        );
        // The steady load is still the docs/10 value (11.14), stops or not
        let loads = axle_loads(&car, &state);
        let expected = calculate_axle_loads(
            &car.mass,
            &car.geometry,
            G_ACCELERATION,
            -4.0 * G_ACCELERATION,
        );
        assert!((loads.front - expected.front).abs() < 0.1);
    }

    #[test]
    fn test_wheel_lift_clamp() {
        let mut car = Car::default();
        // a_x = -5 g settles at 1764.7 N transfer per rear corner (< 2000 N static, so no tipping),
        // but with low damping (zeta ~ 0.1) the pitch overshoots and briefly lifts the rear.
        // Lift under a *constant* a_x would mean tipping over: |a_x| > g * d_r / h = 5.67 g.
        change_all_corners(&mut car.suspension, |corner| corner.damping = 670.0);
        let longitudinal_acceleration = -5.0 * G_ACCELERATION;
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let static_loads = calculate_static_corner_loads(&car.mass, G_ACCELERATION);

        let mut state = SuspensionState::default();
        let mut lift_steps = 0;

        for _ in 0..10_000 {
            let tyre_loads = calculate_tyre_loads(&car, &state, G_ACCELERATION);
            for load in [
                tyre_loads.front_left,
                tyre_loads.front_right,
                tyre_loads.rear_left,
                tyre_loads.rear_right,
            ] {
                assert!(load >= 0.0, "Negative tyre load {:?}", tyre_loads);
            }

            let corner_forces = calculate_corner_forces_from_state(&car, &state, G_ACCELERATION);
            if corner_forces.rear_left == -static_loads.rear_left {
                lift_steps += 1;

                // Manual body accelerations with the clamped rear force F_r = -2 * 2000 N
                let corner = &car.suspension.front_left;
                let travel = calculate_axle_travel(&state, &distances);
                let front_corner_force = corner.spring_rate * travel.front_compression
                    + corner.damping * travel.front_compression_speed
                    + calculate_end_stop_force(corner, travel.front_compression);
                let front_force = 2.0 * front_corner_force;
                let rear_force = -2.0 * static_loads.rear_left;
                let heave_acceleration = -(front_force + rear_force) / car.mass.total;
                let pitch_acceleration =
                    (-car.mass.total * longitudinal_acceleration * car.geometry.cog_height
                        - distances.front * front_force
                        + distances.rear * rear_force)
                        / car.suspension.pitch_inertia;

                // The rear really wanted to pull on the road
                let unclamped_rear = corner.spring_rate * travel.rear_compression
                    + corner.damping * travel.rear_compression_speed
                    + calculate_end_stop_force(corner, travel.rear_compression);
                assert!(unclamped_rear < -static_loads.rear_left);

                let accelerations = calculate_body_accelerations_from_state(
                    &car,
                    &state,
                    G_ACCELERATION,
                    longitudinal_acceleration,
                );
                // 1e-9 relative: same arithmetic, only the order of the sums differs
                assert!(
                    (accelerations.heave_acceleration - heave_acceleration).abs()
                        < 1e-9 * heave_acceleration.abs().max(1.0)
                );
                assert!(
                    (accelerations.pitch_acceleration - pitch_acceleration).abs()
                        < 1e-9 * pitch_acceleration.abs().max(1.0)
                );
                // Same clamped value in the axle forces
                assert_eq!(calculate_axle_forces(&corner_forces).rear, rear_force);
            }

            state = step_suspension(&car, &state, longitudinal_acceleration, TIME_STEP);
        }

        assert!(lift_steps > 0, "Rear wheels never lifted");

        // After the transient the car settles on all four wheels at the docs/10 loads.
        // 10 s: decay exp(-0.1 * 38.77 * 10) = 1.5e-17
        let loads = axle_loads(&car, &state);
        let expected = calculate_axle_loads(
            &car.mass,
            &car.geometry,
            G_ACCELERATION,
            longitudinal_acceleration,
        );
        assert!((loads.front - expected.front).abs() < 0.1);
        assert!((loads.rear - expected.rear).abs() < 0.1);
    }

    #[test]
    fn test_time_step_convergence() {
        let car = Car::default();

        let peak_front_load = |time_step: f64| {
            let mut state = SuspensionState::default();
            let mut peak: f64 = 0.0;
            // 0.5 s covers the overshoot (pitch period 0.16 s)
            let steps = (0.5 / time_step).round() as usize;
            for _ in 0..steps {
                state = step_suspension(&car, &state, -4.0 * G_ACCELERATION, time_step);
                peak = peak.max(axle_loads(&car, &state).front);
            }
            peak
        };

        let peak_coarse = peak_front_load(0.001);
        let peak_fine = peak_front_load(0.0005);
        // Steady front load is 4000 N + 2823.5 N = 6823.5 N, so the peak must at least reach it
        assert!(peak_fine > 6823.0);
        // 0.5%: semi-implicit Euler is first order, so the two peaks differ by about the 1 ms error.
        // That is at most O(omega * dt) = 4% of the transient part (~560 N of the ~7380 N peak),
        // i.e. ~0.3% of the peak. Measured: ~0.05%
        assert!(
            (peak_coarse / peak_fine - 1.0).abs() < 0.005,
            "Peak {} N at 1 ms vs {} N at 0.5 ms",
            peak_coarse,
            peak_fine
        );
    }
}
