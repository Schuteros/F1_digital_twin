//! Car model: specs and mass distribution

use crate::physics::aero::Aero;
use crate::physics::braking::Brakes;
use crate::physics::chassis::ChassisGeometry;
use crate::physics::powertrain::Powertrain;
use crate::physics::suspension::Suspension;
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
