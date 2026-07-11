//! This module contains pure numerical ordinary differential equation (ODE) solvers.
//!
//! Underlying science and mathematical stability proofs are located in:
//! [Kinematics Engine Documentation](../../docs/1_Acceleration_and_Velocity_Integration.MD)

/// Evaluates a single first-order numerical integration step using the Forward Euler method.
#[inline(always)]
pub fn euler(current_value: f64, derivative: f64, dt: f64) -> f64 {
    current_value + (derivative * dt)
}