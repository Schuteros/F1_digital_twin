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
        }
    }
}
