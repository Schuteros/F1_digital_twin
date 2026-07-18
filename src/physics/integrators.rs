//! This module contains pure numerical ordinary differential equation (ODE) solvers.
//!
//! Underlying science and mathematical stability proofs are located in:
//! [Kinematics Engine Documentation](../../docs/1_Acceleration_and_Velocity_Integration.MD)

/// Evaluates a single first-order numerical integration step using the Forward Euler method.
#[inline(always)]
pub(crate) fn euler(current_value: f64, derivative: f64, dt: f64) -> f64 {
    current_value + (derivative * dt)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pure_euler_integration() {
        // Test a standalone calculus step: y = 10.0, dy/dt = 5.0, dt = 0.1
        let result = euler(10.0, 5.0, 0.1);
        assert!((result - 10.5).abs() < 1e-6, "Math engine failed basic integration!");
    }
}