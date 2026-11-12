//! Body roll and lateral load transfer: track widths, roll centres, anti-roll bars and the
//! optional frame torsional stiffness
//!
//! Underlying science equations are found in this documentation:
//! [Roll and Lateral Load Transfer Documentation](../../docs/19_Roll_and_Lateral_Load_Transfer.MD)

use crate::physics::car::Car;
use crate::physics::chassis::AxleDistances;
use crate::physics::cornering::{AxleLateralForces, calculate_axle_lateral_forces};
use crate::physics::loads::AxleLoads;
use crate::physics::suspension::{CornerForces, CornerLoads, Suspension, SuspensionState};

/// Values of the roll model that follow from the car data, so they are the same for the whole
/// run (docs/19 sections 19.3, 19.6 and 19.8). Part of `StaticCarValues`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct RollValues {
    /// Front track width (b_f) in meters, m
    pub front_track_width: f64,
    /// Rear track width (b_r) in meters, m
    pub rear_track_width: f64,
    /// Front roll centre height (h_rc,f) in meters, m
    pub front_roll_centre_height: f64,
    /// Rear roll centre height (h_rc,r) in meters, m
    pub rear_roll_centre_height: f64,
    /// Height of the roll axis under the CoG (h_ra) in meters, m (docs/19, equation 19.3)
    pub roll_axis_height: f64,
    /// Mass of the front half of the body (m_front) in kilograms, kg: the front axle's share
    pub front_mass: f64,
    /// Mass of the rear half of the body (m_rear) in kilograms, kg: the rear axle's share
    pub rear_mass: f64,
    /// Height of the CoG above the front roll centre (h - h_rc,f) in meters, m
    pub front_roll_moment_arm: f64,
    /// Height of the CoG above the rear roll centre (h - h_rc,r) in meters, m
    pub rear_roll_moment_arm: f64,
    /// Roll inertia of the whole body about the roll axis (J_phi,ra) in kilogram meters
    /// squared, kg·m^2 (docs/19, equation 19.13). Used with a rigid frame.
    pub rigid_roll_inertia: f64,
    /// Roll inertia of the front half about the front roll centre (J_phi,f) in kg·m^2
    /// (docs/19, equation 19.19). Used with a flexible frame.
    pub front_roll_inertia: f64,
    /// Roll inertia of the rear half about the rear roll centre (J_phi,r) in kg·m^2
    /// (docs/19, equation 19.19). Used with a flexible frame.
    pub rear_roll_inertia: f64,
    /// Frame torsional stiffness (K_ch) in Newton meters per radian, N·m/rad. None = rigid frame.
    pub frame_stiffness: Option<f64>,
}

/// Load moved to the right tyre of each axle, and taken from the left one, in Newtons, N.
/// Positive in a left turn (a_y > 0), where the right tyres are the outside ones.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct LateralLoadTransfer {
    /// Load moved to the front right tyre in Newtons, N
    pub front: f64,
    /// Load moved to the rear right tyre in Newtons, N
    pub rear: f64,
}

/// Roll accelerations of the front and rear half of the body in radians per second squared,
/// rad/s^2, positive right side down. Equal with a rigid frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct RollAccelerations {
    /// Front half (alpha_phi,f) in rad/s^2
    pub front: f64,
    /// Rear half (alpha_phi,r) in rad/s^2
    pub rear: f64,
}

/// Roll stiffness of each axle, springs and anti-roll bar together, in Newton meters per
/// radian, N·m/rad (docs/19, equations 19.4 and 19.5)
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RollStiffnesses {
    /// Front axle (K_phi,f) in N·m/rad
    pub front: f64,
    /// Rear axle (K_phi,r) in N·m/rad
    pub rear: f64,
}

/// Roll angles of the front and rear half of the body in radians, rad, positive right side down
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RollAngles {
    /// Front half (phi_f) in rad
    pub front: f64,
    /// Rear half (phi_r) in rad
    pub rear: f64,
}

/// Undamped natural frequencies of the roll motion in radians per second, rad/s. With a rigid
/// frame there is one roll motion and both are the same. With a flexible frame `slow` is the
/// whole body rolling and `fast` the frame twisting (docs/19, equation 19.23).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RollFrequencies {
    /// Slower roll motion in rad/s
    pub slow: f64,
    /// Faster roll motion in rad/s
    pub fast: f64,
}

/// Calculates the roll model values that stay the same for the whole run (`RollValues`)
pub(crate) fn calculate_roll_values(car: &Car, distances: &AxleDistances) -> RollValues {
    let suspension = &car.suspension;
    let mass_total = car.mass.total;
    let cog_height = car.geometry.cog_height;
    let wheelbase = distances.front + distances.rear;
    let front_roll_centre_height = suspension.front_roll_centre_height;
    let rear_roll_centre_height = suspension.rear_roll_centre_height;

    // (19.3)
    let roll_axis_height = (distances.rear * front_roll_centre_height
        + distances.front * rear_roll_centre_height)
        / wheelbase;
    let front_roll_moment_arm = cog_height - front_roll_centre_height;
    let rear_roll_moment_arm = cog_height - rear_roll_centre_height;
    let front_mass = car.mass.front;
    let rear_mass = car.mass.rear;

    RollValues {
        front_track_width: car.geometry.front_track_width,
        rear_track_width: car.geometry.rear_track_width,
        front_roll_centre_height,
        rear_roll_centre_height,
        roll_axis_height,
        front_mass,
        rear_mass,
        front_roll_moment_arm,
        rear_roll_moment_arm,
        // (19.13), parallel axis theorem
        rigid_roll_inertia: suspension.roll_inertia
            + mass_total * (cog_height - roll_axis_height).powi(2),
        // (19.19)
        front_roll_inertia: suspension.roll_inertia * front_mass / mass_total
            + front_mass * front_roll_moment_arm.powi(2),
        rear_roll_inertia: suspension.roll_inertia * rear_mass / mass_total
            + rear_mass * rear_roll_moment_arm.powi(2),
        frame_stiffness: car.frame.as_ref().map(|frame| frame.torsional_stiffness),
    }
}

/// Calculates the anti-roll bar force at each wheel of an axle in Newtons, N: positive pushes
/// the right corner up and the left one down (docs/19, equation 19.10).
///
/// * `stiffness` - roll stiffness the bar adds to its axle (K_arb) in N·m/rad
/// * `compression_left`, `compression_right` - corner compressions from static in meters, m
/// * `track_width` - track width of the axle (b) in meters, m
pub(crate) fn calculate_anti_roll_bar_force(
    stiffness: f64,
    compression_left: f64,
    compression_right: f64,
    track_width: f64,
) -> f64 {
    stiffness * (compression_right - compression_left) / (track_width * track_width)
}

/// Calculates the roll accelerations of the body halves (docs/19, equation 19.12 with a rigid
/// frame, equations 19.20 and 19.21 with a flexible one).
///
/// * `lateral_acceleration` - a_y in meters per second squared, m/s^2, positive to the left
/// * `corner_forces` - the clamped corner forces, the same ones that go into the tyre loads
pub(crate) fn calculate_roll_accelerations(
    roll: &RollValues,
    lateral_acceleration: f64,
    state: &SuspensionState,
    corner_forces: &CornerForces,
) -> RollAccelerations {
    // (19.18): each half's mass times a_y times its height above its roll centre
    let front_moment = roll.front_mass * lateral_acceleration * roll.front_roll_moment_arm;
    let rear_moment = roll.rear_mass * lateral_acceleration * roll.rear_roll_moment_arm;
    // The corners push back: a right corner pushing up lifts the right side
    let front_corner_moment =
        0.5 * roll.front_track_width * (corner_forces.front_right - corner_forces.front_left);
    let rear_corner_moment =
        0.5 * roll.rear_track_width * (corner_forces.rear_right - corner_forces.rear_left);

    match roll.frame_stiffness {
        None => {
            // (19.12): front_moment + rear_moment = m * a_y * (h - h_ra)
            let acceleration =
                (front_moment + rear_moment - front_corner_moment - rear_corner_moment)
                    / roll.rigid_roll_inertia;
            RollAccelerations {
                front: acceleration,
                rear: acceleration,
            }
        }
        Some(frame_stiffness) => {
            let frame_moment = frame_stiffness * (state.roll_front - state.roll_rear);
            RollAccelerations {
                // (19.20)
                front: (front_moment - front_corner_moment - frame_moment)
                    / roll.front_roll_inertia,
                // (19.21)
                rear: (rear_moment - rear_corner_moment + frame_moment) / roll.rear_roll_inertia,
            }
        }
    }
}

/// Calculates the geometric lateral load transfer of each axle in Newtons, N: its lateral force
/// times its roll centre height over its track width (docs/19, equation 19.14). It goes through
/// the links straight to the tyres, without roll.
pub(crate) fn calculate_geometric_transfer(
    roll: &RollValues,
    axle_lateral_forces: &AxleLateralForces,
) -> LateralLoadTransfer {
    LateralLoadTransfer {
        front: axle_lateral_forces.front * roll.front_roll_centre_height / roll.front_track_width,
        rear: axle_lateral_forces.rear * roll.rear_roll_centre_height / roll.rear_track_width,
    }
}

/// Calculates the roll stiffness of one axle in N·m/rad: both springs (docs/19, equation 19.4)
/// plus the anti-roll bar (equation 19.5).
///
/// * `rate_left`, `rate_right` - wheel rates of the two corners in Newtons per meter, N/m
/// * `track_width` - track width of the axle in meters, m
/// * `anti_roll_bar_stiffness` - roll stiffness of the axle's bar in N·m/rad
pub(crate) fn calculate_axle_roll_stiffness(
    rate_left: f64,
    rate_right: f64,
    track_width: f64,
    anti_roll_bar_stiffness: f64,
) -> f64 {
    (rate_left + rate_right) * track_width * track_width / 4.0 + anti_roll_bar_stiffness
}

/// Calculates the roll stiffness of both axles with the spring rates (end stops not touched)
/// in N·m/rad (docs/19, equations 19.4 and 19.5)
pub(crate) fn calculate_roll_stiffnesses(
    suspension: &Suspension,
    roll: &RollValues,
) -> RollStiffnesses {
    RollStiffnesses {
        front: calculate_axle_roll_stiffness(
            suspension.front_left.spring_rate,
            suspension.front_right.spring_rate,
            roll.front_track_width,
            suspension.front_anti_roll_bar_stiffness,
        ),
        rear: calculate_axle_roll_stiffness(
            suspension.rear_left.spring_rate,
            suspension.rear_right.spring_rate,
            roll.rear_track_width,
            suspension.rear_anti_roll_bar_stiffness,
        ),
    }
}

/// Calculates the settled roll of both body halves in a constant corner in radians, rad
/// (docs/19, equation 19.16 with a rigid frame, 19.22 with a flexible one). Linear springs,
/// no end stops touched.
///
/// * `lateral_acceleration` - a_y in meters per second squared, m/s^2, positive to the left
pub(crate) fn calculate_steady_state_roll(
    roll: &RollValues,
    stiffnesses: &RollStiffnesses,
    lateral_acceleration: f64,
) -> RollAngles {
    let front_moment = roll.front_mass * lateral_acceleration * roll.front_roll_moment_arm;
    let rear_moment = roll.rear_mass * lateral_acceleration * roll.rear_roll_moment_arm;

    match roll.frame_stiffness {
        None => {
            // (19.16)
            let angle = (front_moment + rear_moment) / (stiffnesses.front + stiffnesses.rear);
            RollAngles {
                front: angle,
                rear: angle,
            }
        }
        Some(frame) => {
            // (19.22)
            let determinant = stiffnesses.front * stiffnesses.rear
                + frame * (stiffnesses.front + stiffnesses.rear);
            RollAngles {
                front: (front_moment * (stiffnesses.rear + frame) + rear_moment * frame)
                    / determinant,
                rear: (rear_moment * (stiffnesses.front + frame) + front_moment * frame)
                    / determinant,
            }
        }
    }
}

/// Calculates the settled lateral load transfer of each axle in a constant corner in Newtons,
/// N: geometric (docs/19, equation 19.14) plus elastic, the axle's roll stiffness times its
/// settled roll over its track width (equations 19.17 and 19.22).
///
/// * `lateral_acceleration` - a_y in meters per second squared, m/s^2, positive to the left
pub(crate) fn calculate_steady_state_lateral_transfer(
    car: &Car,
    distances: &AxleDistances,
    lateral_acceleration: f64,
) -> LateralLoadTransfer {
    let roll = calculate_roll_values(car, distances);
    let stiffnesses = calculate_roll_stiffnesses(&car.suspension, &roll);
    let angles = calculate_steady_state_roll(&roll, &stiffnesses, lateral_acceleration);
    let geometric = calculate_geometric_transfer(
        &roll,
        &calculate_axle_lateral_forces(car.mass.total, lateral_acceleration, distances),
    );

    LateralLoadTransfer {
        front: geometric.front + stiffnesses.front * angles.front / roll.front_track_width,
        rear: geometric.rear + stiffnesses.rear * angles.rear / roll.rear_track_width,
    }
}

/// Splits each axle load between its two tyres with the lateral load transfer: half each, plus
/// the transfer on the right tyre and minus it on the left one. A tyre that would go below 0
/// has lifted: it carries 0 and the other tyre the whole axle load.
pub(crate) fn split_axle_loads(
    axle_loads: &AxleLoads,
    transfer: &LateralLoadTransfer,
) -> CornerLoads {
    let split = |axle_load: f64, transfer: f64| {
        let left = (0.5 * axle_load - transfer).clamp(0.0, axle_load);
        (left, axle_load - left)
    };
    let (front_left, front_right) = split(axle_loads.front, transfer.front);
    let (rear_left, rear_right) = split(axle_loads.rear, transfer.rear);

    CornerLoads {
        front_left,
        front_right,
        rear_left,
        rear_right,
    }
}

/// Calculates the undamped roll natural frequencies in radians per second, rad/s (docs/19,
/// equation 19.25 with a rigid frame, 19.23 with a flexible one).
pub(crate) fn calculate_roll_natural_frequencies(
    roll: &RollValues,
    stiffnesses: &RollStiffnesses,
) -> RollFrequencies {
    match roll.frame_stiffness {
        None => {
            let frequency =
                ((stiffnesses.front + stiffnesses.rear) / roll.rigid_roll_inertia).sqrt();
            RollFrequencies {
                slow: frequency,
                fast: frequency,
            }
        }
        Some(frame) => {
            let a = (stiffnesses.front + frame) / roll.front_roll_inertia;
            let d = (stiffnesses.rear + frame) / roll.rear_roll_inertia;
            let coupling = frame * frame / (roll.front_roll_inertia * roll.rear_roll_inertia);
            let mean = 0.5 * (a + d);
            let spread = (0.25 * (a - d).powi(2) + coupling).sqrt();
            RollFrequencies {
                slow: (mean - spread).max(0.0).sqrt(),
                fast: (mean + spread).sqrt(),
            }
        }
    }
}

/// Calculates the roll damping ratio of a rigid body, dimensionless (docs/19, equation 19.25):
/// the roll damping of the four corner dampers over the critical damping.
#[allow(dead_code)] // Only used by tests for now
pub(crate) fn calculate_roll_damping_ratio(
    suspension: &Suspension,
    roll: &RollValues,
    stiffnesses: &RollStiffnesses,
) -> f64 {
    let roll_damping = (suspension.front_left.damping + suspension.front_right.damping)
        * roll.front_track_width.powi(2)
        / 4.0
        + (suspension.rear_left.damping + suspension.rear_right.damping)
            * roll.rear_track_width.powi(2)
            / 4.0;

    roll_damping / (2.0 * ((stiffnesses.front + stiffnesses.rear) * roll.rigid_roll_inertia).sqrt())
}

#[cfg(test)]
mod tests {
    use crate::physics::aero::AeroLoads;
    use crate::physics::car::Car;
    use crate::physics::chassis::{Frame, calculate_axle_distances};
    use crate::physics::loads::AxleLoads;
    use crate::physics::roll::{
        LateralLoadTransfer, RollStiffnesses, calculate_anti_roll_bar_force,
        calculate_roll_damping_ratio, calculate_roll_natural_frequencies,
        calculate_roll_stiffnesses, calculate_roll_values, calculate_steady_state_lateral_transfer,
        calculate_steady_state_roll, split_axle_loads,
    };
    use crate::physics::suspension::{
        CornerForces, CornerLoads, StaticCarValues, SuspensionState, calculate_body_accelerations,
        calculate_corner_forces_from_state, calculate_corner_tyre_loads,
        calculate_static_car_values, update_suspension_positions, update_suspension_velocities,
    };
    use std::f64::consts::PI;

    const G_ACCELERATION: f64 = 9.81;
    const TIME_STEP: f64 = 0.001;

    /// The docs/19 section 19.7 car: baseline with roll centres of 0.05 m front and 0.10 m rear
    fn roll_centre_car() -> Car {
        let mut car = Car::default();
        car.suspension.front_roll_centre_height = 0.05;
        car.suspension.rear_roll_centre_height = 0.10;
        car
    }

    /// Settled lateral load transfer of `car` at a_y in m/s^2
    fn transfer(car: &Car, lateral_acceleration: f64) -> LateralLoadTransfer {
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        calculate_steady_state_lateral_transfer(car, &distances, lateral_acceleration)
    }

    /// One time step of the body alone: no a_x, no aero, the given a_y
    fn step(
        car: &Car,
        static_values: &StaticCarValues,
        state: &SuspensionState,
        lateral_acceleration: f64,
        longitudinal_acceleration: f64,
    ) -> SuspensionState {
        let corner_forces =
            calculate_corner_forces_from_state(&car.suspension, static_values, state);
        let accelerations = calculate_body_accelerations(
            car.mass.total,
            car.suspension.pitch_inertia,
            car.geometry.cog_height,
            &static_values.axle_distances,
            longitudinal_acceleration,
            lateral_acceleration,
            state,
            &corner_forces,
            &AeroLoads::default(),
            &static_values.aero_points,
            &static_values.roll,
        );
        let state = update_suspension_velocities(state, &accelerations, TIME_STEP);
        update_suspension_positions(&state, TIME_STEP)
    }

    #[test]
    fn test_worked_example_without_bars() {
        // docs/19 section 19.7: a_y = 8 m/s^2, h_ra = 0.075 m, 561 N front, 663 N rear, 0.25 deg
        let car = roll_centre_car();
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let roll = calculate_roll_values(&car, &distances);
        assert!((roll.roll_axis_height - 0.075).abs() < 1e-12);

        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, &roll);
        assert!((stiffnesses.front - 166_400.0).abs() < 1e-6);
        assert!((stiffnesses.rear - 166_400.0).abs() < 1e-6);

        let angles = calculate_steady_state_roll(&roll, &stiffnesses, 8.0);
        assert!((angles.front - 0.004411).abs() < 1e-6, "{:?}", angles);
        assert_eq!(angles.front, angles.rear);

        let loads = transfer(&car, 8.0);
        assert!((loads.front - 560.65).abs() < 0.01, "{:?}", loads);
        assert!((loads.rear - 662.59).abs() < 0.01, "{:?}", loads);
        // The total is m * a_y * h / b (19.1), whatever the roll centres
        let total = car.mass.total * 8.0 * car.geometry.cog_height / 1.6;
        assert!((loads.front + loads.rear - total).abs() < 1e-9);
    }

    #[test]
    fn test_worked_example_with_front_bar() {
        // docs/19 section 19.7: a front bar as stiff as the front springs, 714 N / 510 N, 0.17 deg
        let mut car = roll_centre_car();
        car.suspension.front_anti_roll_bar_stiffness = 166_400.0;
        let loads = transfer(&car, 8.0);
        assert!((loads.front - 713.56).abs() < 0.01, "{:?}", loads);
        assert!((loads.rear - 509.68).abs() < 0.01, "{:?}", loads);

        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let roll = calculate_roll_values(&car, &distances);
        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, &roll);
        let angles = calculate_steady_state_roll(&roll, &stiffnesses, 8.0);
        assert!((angles.front - 0.0029405).abs() < 1e-6, "{:?}", angles);

        // Same total as without the bar
        let total = car.mass.total * 8.0 * car.geometry.cog_height / 1.6;
        assert!((loads.front + loads.rear - total).abs() < 1e-9);
    }

    #[test]
    fn test_flexible_frame_example() {
        // docs/19 section 19.8: no roll centres, front bar, frame as stiff as both axles:
        // 778 N front / 445 N rear, front share 63.6 % instead of 66.7 %
        let mut car = Car::default();
        car.suspension.front_anti_roll_bar_stiffness = 166_400.0;
        let rigid = transfer(&car, 8.0);
        assert!((rigid.front - 815.49).abs() < 0.01, "{:?}", rigid);

        car.frame = Some(Frame {
            torsional_stiffness: 499_200.0,
        });
        let flexible = transfer(&car, 8.0);
        assert!((flexible.front - 778.4).abs() < 0.1, "{:?}", flexible);
        assert!((flexible.rear - 444.8).abs() < 0.1, "{:?}", flexible);
        let share = flexible.front / (flexible.front + flexible.rear);
        assert!((share - 0.636).abs() < 0.001, "{}", share);
        // The frame never changes the total
        assert!((flexible.front + flexible.rear - (rigid.front + rigid.rear)).abs() < 1e-9);
    }

    #[test]
    fn test_frame_limits() {
        let mut car = roll_centre_car();
        car.suspension.front_anti_roll_bar_stiffness = 200_000.0;
        let rigid = transfer(&car, 8.0);

        // A very stiff frame is the rigid frame
        car.frame = Some(Frame {
            torsional_stiffness: 1e12,
        });
        let stiff = transfer(&car, 8.0);
        assert!(
            (stiff.front - rigid.front).abs() < 1e-3,
            "{:?} {:?}",
            stiff,
            rigid
        );
        assert!((stiff.rear - rigid.rear).abs() < 1e-3);

        // A very soft frame: each half holds only its own moment, the bar no longer moves load
        // between the axles. Front elastic = m_front * a_y * (h - h_rc,f) / b_f
        car.frame = Some(Frame {
            torsional_stiffness: 1e-6,
        });
        let soft = transfer(&car, 8.0);
        let front_geometric = car.mass.total * 8.0 * 0.5 * 0.05 / 1.6;
        let front_elastic = car.mass.front * 8.0 * (0.3 - 0.05) / 1.6;
        assert!(
            (soft.front - front_geometric - front_elastic).abs() < 1e-3,
            "{:?}",
            soft
        );
        car.suspension.front_anti_roll_bar_stiffness = 0.0;
        let soft_no_bar = transfer(&car, 8.0);
        assert!((soft_no_bar.front - soft.front).abs() < 1e-3);
    }

    #[test]
    fn test_roll_centres_at_cog_height() {
        // Pin at the CoG (docs/19 section 19.3): no roll, everything geometric
        let mut car = Car::default();
        car.suspension.front_roll_centre_height = 0.3;
        car.suspension.rear_roll_centre_height = 0.3;
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let roll = calculate_roll_values(&car, &distances);
        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, &roll);
        assert_eq!(
            calculate_steady_state_roll(&roll, &stiffnesses, 8.0).front,
            0.0
        );

        let loads = transfer(&car, 8.0);
        let half = car.mass.total * 8.0 * 0.3 / 1.6 / 2.0;
        assert!((loads.front - half).abs() < 1e-9 && (loads.rear - half).abs() < 1e-9);
    }

    #[test]
    fn test_left_and_right_corners_mirror() {
        let mut car = roll_centre_car();
        car.suspension.rear_anti_roll_bar_stiffness = 50_000.0;
        car.frame = Some(Frame {
            torsional_stiffness: 300_000.0,
        });
        let left = transfer(&car, 8.0);
        let right = transfer(&car, -8.0);
        assert!((left.front + right.front).abs() < 1e-9);
        assert!((left.rear + right.rear).abs() < 1e-9);
    }

    #[test]
    fn test_anti_roll_bar_force() {
        // Pure roll phi: s_R - s_L = b * phi, force K * phi / b, moment F * b = K * phi (19.10)
        let force = calculate_anti_roll_bar_force(100_000.0, -0.008, 0.008, 1.6);
        assert!((force * 1.6 - 100_000.0 * 0.01).abs() < 1e-9);
        // Heave and pitch: both corners move together, no force
        assert_eq!(
            calculate_anti_roll_bar_force(100_000.0, 0.01, 0.01, 1.6),
            0.0
        );
    }

    #[test]
    fn test_bar_stiffness_from_dimensions() {
        // docs/19 section 19.4 example: 12 mm steel bar, 0.4 m, 80 mm arms, MR 0.5, b 1.2 m
        let bar = 79e9 * PI * 0.012_f64.powi(4) / (32.0 * 0.4);
        assert!((bar - 402.0).abs() < 0.5, "{}", bar);
        let axle = bar * (0.5 * 1.2 / 0.08_f64).powi(2);
        assert!((axle - 22_600.0).abs() < 50.0, "{}", axle);
    }

    #[test]
    fn test_split_axle_loads() {
        let loads = split_axle_loads(
            &AxleLoads {
                front: 4000.0,
                rear: 3000.0,
            },
            &LateralLoadTransfer {
                front: 500.0,
                rear: -2000.0,
            },
        );
        assert_eq!(loads.front_left, 1500.0);
        assert_eq!(loads.front_right, 2500.0);
        // The rear right tyre would go below 0: it lifts and the left one carries the axle
        assert_eq!(loads.rear_right, 0.0);
        assert_eq!(loads.rear_left, 3000.0);
    }

    #[test]
    fn test_tyre_load_never_below_zero() {
        // Geometric transfer larger than the tyre's load: the inside tyre stays at 0
        let static_loads = CornerLoads {
            front_left: 2000.0,
            front_right: 2000.0,
            rear_left: 2000.0,
            rear_right: 2000.0,
        };
        let corner_forces = CornerForces {
            front_left: -1500.0,
            front_right: 1500.0,
            rear_left: 0.0,
            rear_right: 0.0,
        };
        let loads = calculate_corner_tyre_loads(
            &static_loads,
            &corner_forces,
            &LateralLoadTransfer {
                front: 800.0,
                rear: 100.0,
            },
        );
        assert_eq!(loads.front_left, 0.0);
        assert_eq!(loads.front_right, 4300.0);
        assert_eq!(loads.rear_left, 1900.0);
        assert_eq!(loads.rear_right, 2100.0);
    }

    #[test]
    fn test_roll_frequency_and_damping_ratio() {
        // docs/19 section 19.10: J_phi,ra = 323 kg·m^2, 32.1 rad/s, zeta 0.62
        let car = Car::default();
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let roll = calculate_roll_values(&car, &distances);
        assert!((roll.rigid_roll_inertia - 323.39).abs() < 0.01);
        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, &roll);
        let frequencies = calculate_roll_natural_frequencies(&roll, &stiffnesses);
        assert!((frequencies.slow - 32.08).abs() < 0.01, "{:?}", frequencies);
        assert_eq!(frequencies.slow, frequencies.fast);
        let zeta = calculate_roll_damping_ratio(&car.suspension, &roll, &stiffnesses);
        assert!((zeta - 0.617).abs() < 0.001, "{}", zeta);
    }

    #[test]
    fn test_twist_frequency() {
        // docs/19 section 19.8: J_phi,f = J_phi,r = 162 kg·m^2, K_ch = 499 200 N·m/rad:
        // 32.1 rad/s together, 84.9 rad/s twisting
        let car = Car {
            frame: Some(Frame {
                torsional_stiffness: 499_200.0,
            }),
            ..Car::default()
        };
        let distances = calculate_axle_distances(&car.mass, &car.geometry);
        let roll = calculate_roll_values(&car, &distances);
        assert!((roll.front_roll_inertia - 161.70).abs() < 0.01);
        let frequencies = calculate_roll_natural_frequencies(
            &roll,
            &RollStiffnesses {
                front: 166_400.0,
                rear: 166_400.0,
            },
        );
        assert!((frequencies.slow - 32.08).abs() < 0.01, "{:?}", frequencies);
        assert!((frequencies.fast - 84.87).abs() < 0.01, "{:?}", frequencies);
    }

    /// Runs the body at a constant a_y until it has settled and checks roll and tyre loads
    /// against the steady state (docs/19 sections 19.7 and 19.8)
    fn check_settles_at_steady_state(car: &Car) {
        let static_values = calculate_static_car_values(car, G_ACCELERATION);
        let mut state = SuspensionState::default();
        for _ in 0..3_000 {
            state = step(car, &static_values, &state, 8.0, 0.0);
        }

        let roll = &static_values.roll;
        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, roll);
        let angles = calculate_steady_state_roll(roll, &stiffnesses, 8.0);
        assert!(
            (state.roll_front - angles.front).abs() < 1e-9,
            "{:?}",
            state
        );
        assert!((state.roll_rear - angles.rear).abs() < 1e-9, "{:?}", state);
        // Symmetric car: no heave or pitch from cornering
        assert!(state.heave.abs() < 1e-12 && state.pitch.abs() < 1e-12);

        let corner_forces =
            calculate_corner_forces_from_state(&car.suspension, &static_values, &state);
        let axle_lateral_forces = crate::physics::cornering::calculate_axle_lateral_forces(
            car.mass.total,
            8.0,
            &static_values.axle_distances,
        );
        let loads = calculate_corner_tyre_loads(
            &static_values.static_corner_loads,
            &corner_forces,
            &crate::physics::roll::calculate_geometric_transfer(roll, &axle_lateral_forces),
        );
        let expected = transfer(car, 8.0);
        let static_loads = &static_values.static_corner_loads;
        assert!((loads.front_right - static_loads.front_right - expected.front).abs() < 1e-3);
        assert!((static_loads.front_left - loads.front_left - expected.front).abs() < 1e-3);
        assert!((loads.rear_right - static_loads.rear_right - expected.rear).abs() < 1e-3);
        assert!((static_loads.rear_left - loads.rear_left - expected.rear).abs() < 1e-3);
    }

    #[test]
    fn test_dynamic_roll_settles_at_steady_state() {
        let mut car = roll_centre_car();
        check_settles_at_steady_state(&car);
        car.suspension.front_anti_roll_bar_stiffness = 166_400.0;
        check_settles_at_steady_state(&car);
        car.frame = Some(Frame {
            torsional_stiffness: 499_200.0,
        });
        check_settles_at_steady_state(&car);
    }

    #[test]
    fn test_bar_does_nothing_in_heave_and_pitch() {
        // Braking at -10 m/s^2 for 0.5 s with and without bars gives exactly the same motion
        let without = Car::default();
        let mut with = Car::default();
        with.suspension.front_anti_roll_bar_stiffness = 300_000.0;
        with.suspension.rear_anti_roll_bar_stiffness = 100_000.0;
        let static_without = calculate_static_car_values(&without, G_ACCELERATION);
        let static_with = calculate_static_car_values(&with, G_ACCELERATION);
        let mut state_without = SuspensionState::default();
        let mut state_with = SuspensionState::default();
        for _ in 0..500 {
            state_without = step(&without, &static_without, &state_without, 0.0, -10.0);
            state_with = step(&with, &static_with, &state_with, 0.0, -10.0);
        }
        assert!(state_with.pitch > 0.0);
        assert_eq!(state_with, state_without);
        assert_eq!(state_with.roll_front, 0.0);
    }

    #[test]
    fn test_free_roll_oscillation() {
        // Light damping (500 N·s/m per corner): zeta = 0.0617. Period 2 pi / omega_d and the
        // ratio of successive peaks exp(-2 pi zeta / sqrt(1 - zeta^2)) (docs/11, equation 11.19)
        let mut car = Car::default();
        for corner in [
            &mut car.suspension.front_left,
            &mut car.suspension.front_right,
            &mut car.suspension.rear_left,
            &mut car.suspension.rear_right,
        ] {
            corner.damping = 500.0;
        }
        let static_values = calculate_static_car_values(&car, G_ACCELERATION);
        let roll = &static_values.roll;
        let stiffnesses = calculate_roll_stiffnesses(&car.suspension, roll);
        let omega_n = calculate_roll_natural_frequencies(roll, &stiffnesses).slow;
        let zeta = calculate_roll_damping_ratio(&car.suspension, roll, &stiffnesses);
        let omega_d = omega_n * (1.0 - zeta * zeta).sqrt();

        let mut state = SuspensionState {
            roll_front: 0.01,
            roll_rear: 0.01,
            ..SuspensionState::default()
        };
        let mut peaks: Vec<(f64, f64)> = Vec::new();
        let mut previous = state;
        let mut before_previous = state;
        for index in 1..2_000 {
            state = step(&car, &static_values, &state, 0.0, 0.0);
            if index > 1
                && previous.roll_front > state.roll_front
                && previous.roll_front > before_previous.roll_front
            {
                peaks.push(((index - 1) as f64 * TIME_STEP, previous.roll_front));
            }
            before_previous = previous;
            previous = state;
        }

        assert!(peaks.len() >= 3, "{:?}", peaks);
        let period = peaks[1].0 - peaks[0].0;
        assert!(
            (period - 2.0 * PI / omega_d).abs() < 0.002,
            "{} vs {}",
            period,
            2.0 * PI / omega_d
        );
        let ratio = peaks[1].1 / peaks[0].1;
        let expected = (-2.0 * PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();
        assert!((ratio - expected).abs() < 0.01, "{} vs {}", ratio, expected);
    }

    #[test]
    fn test_undamped_roll_stays_bounded() {
        let mut car = Car::default();
        for corner in [
            &mut car.suspension.front_left,
            &mut car.suspension.front_right,
            &mut car.suspension.rear_left,
            &mut car.suspension.rear_right,
        ] {
            corner.damping = 0.0;
        }
        car.frame = Some(Frame {
            torsional_stiffness: 499_200.0,
        });
        let static_values = calculate_static_car_values(&car, G_ACCELERATION);
        // Start twisted and rolled: both roll motions ring
        let mut state = SuspensionState {
            roll_front: 0.01,
            roll_rear: 0.0,
            ..SuspensionState::default()
        };
        let mut max_late: f64 = 0.0;
        for index in 0..10_000 {
            state = step(&car, &static_values, &state, 0.0, 0.0);
            if index >= 9_000 {
                max_late = max_late.max(state.roll_front.abs());
            }
        }
        // Semi-implicit Euler keeps the energy bounded: no growth beyond a few %
        assert!(max_late < 0.0105, "{}", max_late);
        assert!(max_late > 0.005, "{}", max_late);
    }

    #[test]
    fn test_twist_oscillation_period() {
        // Pure twist (front +, rear -) of a symmetric car without damping: 84.9 rad/s
        let mut car = Car::default();
        for corner in [
            &mut car.suspension.front_left,
            &mut car.suspension.front_right,
            &mut car.suspension.rear_left,
            &mut car.suspension.rear_right,
        ] {
            corner.damping = 0.0;
        }
        car.frame = Some(Frame {
            torsional_stiffness: 499_200.0,
        });
        let static_values = calculate_static_car_values(&car, G_ACCELERATION);
        let mut state = SuspensionState {
            roll_front: 0.005,
            roll_rear: -0.005,
            ..SuspensionState::default()
        };
        let mut crossings: Vec<f64> = Vec::new();
        for index in 1..1_000 {
            let next = step(&car, &static_values, &state, 0.0, 0.0);
            if state.roll_front > 0.0 && next.roll_front <= 0.0 {
                // Interpolate the zero crossing inside the step
                let fraction = state.roll_front / (state.roll_front - next.roll_front);
                crossings.push(((index - 1) as f64 + fraction) * TIME_STEP);
            }
            state = next;
        }
        let period = crossings[1] - crossings[0];
        assert!((period - 2.0 * PI / 84.87).abs() < 0.0005, "{}", period);
        // The rear stays the mirror of the front: a pure twist mode
        assert!((state.roll_front + state.roll_rear).abs() < 1e-9);
    }
}
