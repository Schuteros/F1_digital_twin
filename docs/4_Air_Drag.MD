# 4. Aerodynamic Drag

As the vehicle moves through the air, it experiences aerodynamic resistance that opposes its forward motion. This force scales quadratically with velocity, making it the dominant parasitic loss at high speeds.

---

## 4.1. Quadratic Drag Model

For our MVP framework, we implement the standard, simplified aerodynamic drag equation:

$$F_{\text{drag}} = \frac{1}{2} \cdot \rho \cdot C_d \cdot A \cdot v^2 \cdot \text{sign}(v) \tag{4.1}$$

where:
* $F_{\text{drag}}$ is the aerodynamic drag force in Newtons (N),
* $\rho$ is the ambient air density in kilograms per cubic meter ($\text{kg/m}^3$), typically evaluated at $1.225 \text{ kg/m}^3$ for standard sea-level conditions,
* $C_d$ is the dimensionless drag coefficient representing the vehicle's aerodynamic efficiency,
* $A$ is the frontal area of the vehicle in square meters ($\text{m}^2$),
* $v$ is the longitudinal vehicle velocity in meters per second ($\text{m/s}$),
* $\text{sign}(v)$ ensures that the resulting drag force always acts in direct opposition to the direction of vehicle travel.

---

## 4.2. Performance Architecture

Because computing powers ($v^2$) and signs can introduce branching overhead, the high-performance Rust implementation eliminates branching by utilizing the CPU's native absolute value and multiplication instructions:

```rust
let drag_force = 0.5 * air_density * drag_coefficient * frontal_area * speed * speed.abs();