# 1. Acceleration and Velocity Integration

To accurately model the physical progression of the Formula 1 car through time, 
we must quantize its state changes. This is achieved by integrating 
the kinematic chain sequentially:
* **Linear Acceleration ($a$) in $m/s^2$:** Integrated over time to calculate **Linear Velocity ($v$)** in $m/s$.
* **Linear Velocity ($v$) in $m/s$:** Integrated over time to calculate **Displacement ($x$)** in $m$.

---

## 1.1 Mathematical Model: Euler Integration

We are currently utilizing the standard **Euler Method**. This is a first-order numerical procedure for solving ordinary differential equations (ODEs) with a given initial value ('Euler method', 2026).

### Govering Integration Equations
At each discrete time step ($\Delta t$), the car's state is updated as follows:

$$v_{t+1} = v_t + a_t \cdot \Delta t$$
$$x_{t+1} = x_t + v_t \cdot \Delta t$$

Where:
* $a_t$ = Net acceleration at current step ($\text{m/s}^2$)
* $v_t$ = Car velocity at current step ($\text{m/s}$)
* $x_t$ = Car position/displacement at current step ($\text{m}$)
* $\Delta t$ = The step size, or time delta (`t_step` in code) ($\text{s}$)

---

## 1.2 Engineering Limitations & Caveats

> [!WARNING]  
> Numerical Drift / Stability Risk: The Euler method is a first-order 
> approximation. Its local truncation error per step is $\mathcal{O}(\Delta t^2)$, 
> while its cumulative global truncation error is $\mathcal{O}(\Delta t)$.
>
> * **Risk:** If $\Delta t$ is too large, or if acceleration changes violently 
> (like dropping instantly due to traction breaking), 
> the Euler loop can accumulate energy and become unstable ("explode").
> * **Future Upgrade Plan:** If simulation drift becomes unacceptable during 
> braking or launching phases, we will migrate to a second-order 
> **Runge-Kutta (RK2 / Heun's Method)** or **Semi-Implicit Euler** 
> engine ('Euler method', 2026).
---

## 1.3 Parameter Sourcing & Rationale

* **Time Step ($\Delta t$):** Initialized at `0.001` seconds ($1\text{ ms}$). 
For now, it is in the hopes that it is small enough to not cause issues.
Later this value can be tuned. Higher $\Delta t$ reduces computational power used,
lower $\Delta t$ improves model accuracy.

---

## 1.4 References
* 'Euler Method' (2026). *Wikipedia.* Available at: https://en.wikipedia.org/wiki/Euler_method (Accessed: 9 July 2026)