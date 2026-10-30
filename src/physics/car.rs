//! Car model: specs and mass distribution

use crate::physics::aero::Aero;
use crate::physics::braking::Brakes;
use crate::physics::chassis::Suspension;
use crate::physics::powertrain::Powertrain;
use crate::physics::tyres::Tyre;
use serde::Deserialize;

/// Contains all the variables that define the car needed to be simulated
#[derive(Debug, PartialEq)]
pub struct Car {
    /// Contains data about mass distribution
    pub mass: Mass,

    /// Defines the powertrain used to simulate the car
    pub powertrain: Powertrain,

    /// Defines the tyres used to simulate the car
    pub tyre: Tyre,

    /// Defines the aerodynamic model of the car
    pub aero: Aero,

    /// Defines the chassis geometry used for load transfer and the suspension model
    pub geometry: ChassisGeometry,

    /// Defines the brake system of the car
    pub brakes: Brakes,

    /// Defines the suspension (springs, dampers, end stops and pitch inertia) of the car
    pub suspension: Suspension,
}

/// Contains mass distribution data
#[derive(Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mass {
    /// Total mass of the car in kilograms, kg
    #[serde(rename = "total_kg")]
    pub total: f64,
    /// Mass on the rear axle in kilograms, kg
    #[serde(rename = "rear_kg")]
    pub rear: f64,
    /// Mass on the front axle in kilograms, kg
    #[serde(rename = "front_kg")]
    pub front: f64,
}

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
}

impl Default for ChassisGeometry {
    fn default() -> Self {
        Self {
            // Placeholder values: verify against current F1 regulations / sources
            wheelbase: 3.4,
            cog_height: 0.3,
        }
    }
}

impl Default for Car {
    fn default() -> Self {
        let total_mass = 815.494393476;
        let driven_axle_mass = 407.747196738;

        Self {
            mass: Mass {
                total: total_mass,
                rear: driven_axle_mass,
                front: total_mass - driven_axle_mass,
            },
            powertrain: Powertrain::default(),
            tyre: Tyre::default(),
            aero: Aero::default(),
            geometry: ChassisGeometry::default(),
            brakes: Brakes::default(),
            suspension: Suspension::default(),
        }
    }
}
