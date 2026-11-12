//! Chassis geometry: dimensions of the car and where forces act relative to the CoG

use crate::physics::aero::Aero;
use crate::physics::car::Mass;
use serde::Deserialize;

/// Chassis dimensions needed for longitudinal load transfer and the suspension model.
///
/// CoG to axle distances (l_f, l_r) are not stored here: they follow from the
/// front/rear split in `Mass`, so the static weight distribution has one source of truth.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChassisGeometry {
    /// Distance between the front and rear axle in meters, m
    #[serde(rename = "wheelbase_m")]
    pub wheelbase: f64,
    /// Height of the centre of gravity above the ground in meters, m
    #[serde(rename = "cog_height_m")]
    pub cog_height: f64,
    /// Distance between the left and right front contact patch centres (b_f) in meters, m
    /// (docs/19)
    #[serde(rename = "front_track_width_m")]
    pub front_track_width: f64,
    /// Distance between the left and right rear contact patch centres (b_r) in meters, m
    /// (docs/19)
    #[serde(rename = "rear_track_width_m")]
    pub rear_track_width: f64,
}

/// Frame (chassis) between the front and the rear axle (docs/19 section 19.8). Optional in the
/// car file: without it the frame is rigid.
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    /// Torsional stiffness of the frame between the front and the rear axle (K_ch) in Newton
    /// meters per radian, N·m/rad
    #[serde(rename = "torsional_stiffness_N_m_per_rad")]
    pub torsional_stiffness: f64,
}

impl Default for ChassisGeometry {
    fn default() -> Self {
        Self {
            // Placeholder values: verify against current F1 regulations / sources
            wheelbase: 3.4,
            cog_height: 0.3,
            front_track_width: 1.6,
            rear_track_width: 1.6,
        }
    }
}

/// Horizontal distances from the CoG to the axles
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct AxleDistances {
    /// Distance from the CoG to the front axle (d_f) in meters, m
    pub front: f64,
    /// Distance from the CoG to the rear axle (d_r) in meters, m
    pub rear: f64,
}

/// Where the aerodynamic forces act, measured from the CoG (docs/13 section 13.2)
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct AeroPoints {
    /// Horizontal distance of the downforce centre of pressure ahead of the CoG (d_f - x_cp) in
    /// meters, m. Negative = behind the CoG.
    pub downforce_ahead_of_cog: f64,
    /// Height of the drag centre of pressure above the ground (h_drag) in meters, m
    pub drag_height: f64,
}

/// Calculates the CoG to axle distances from the mass split:
/// d_r / W = mass.front / mass.total and d_f / W = mass.rear / mass.total (docs/10 section 10.6)
pub(crate) fn calculate_axle_distances(mass: &Mass, geometry: &ChassisGeometry) -> AxleDistances {
    AxleDistances {
        front: geometry.wheelbase * mass.rear / mass.total,
        rear: geometry.wheelbase * mass.front / mass.total,
    }
}

/// Calculates where the aero forces act relative to the CoG: d_f - x_cp ahead of the CoG
/// (docs/13 section 13.3)
pub(crate) fn calculate_aero_points(aero: &Aero, distances: &AxleDistances) -> AeroPoints {
    AeroPoints {
        downforce_ahead_of_cog: distances.front - aero.downforce_centre_of_pressure_from_front_axle,
        drag_height: aero.drag_centre_of_pressure_height,
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::car::{Car, Mass};
    use crate::physics::chassis::{
        ChassisGeometry, calculate_aero_points, calculate_axle_distances,
    };

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
            ..ChassisGeometry::default()
        };
        let distances = calculate_axle_distances(&mass, &geometry);
        assert!((distances.front - 1.8).abs() < 1e-12);
        assert!((distances.rear - 1.2).abs() < 1e-12);
    }

    #[test]
    fn test_calculate_aero_points() {
        // Default car: d_f = 1.7 m, x_cp = 1.8 m -> d_f - x_cp = 1.7 - 1.8 = -0.1 m (0.1 m behind the CoG)
        let car = Car::default();
        let distances = calculate_axle_distances(&car.mass, &car.geometry);

        let points = calculate_aero_points(&car.aero, &distances);
        assert!((points.downforce_ahead_of_cog + 0.1).abs() < 1e-12);
        assert_eq!(points.drag_height, 0.4);
    }
}
