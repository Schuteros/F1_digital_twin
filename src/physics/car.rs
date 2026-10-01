//! Car model: specs and mass distribution

use crate::physics::aero::Aero;
use crate::physics::powertrain::Powertrain;
use crate::physics::tyres::Tyre;

/// Contains all the variables that define the car needed to be simulated
pub struct Car {
    /// Contains data about mass distribution
    pub mass: Mass,

    /// Defines the powertrain used to simulate the car
    pub powertrain: Powertrain,

    /// Defines the tyres used to simulate the car
    pub tyre: Tyre,

    /// Defines the aerodynamic model of the car
    pub aero: Aero,

    /// Defines the chassis geometry used for longitudinal load transfer
    pub geometry: ChassisGeometry,
}

/// Contains mass distribution data
pub struct Mass {
    /// Total mass of the car in kilograms, kg
    pub total: f64,
    /// Mass on the rear axle in kilograms, kg
    pub rear: f64,
    /// Mass on the front axle in kilograms, kg
    pub front: f64,
}

/// Chassis dimensions needed for longitudinal load transfer.
///
/// CoG to axle distances (l_f, l_r) are not stored here: they follow from the
/// front/rear split in `Mass`, so the static weight distribution has one source of truth.
pub struct ChassisGeometry {
    /// Distance between the front and rear axle in meters, m
    pub wheelbase: f64,
    /// Height of the centre of gravity above the ground in meters, m
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
        }
    }
}
