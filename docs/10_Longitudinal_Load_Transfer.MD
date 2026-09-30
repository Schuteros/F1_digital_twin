# 10. Longitudinal Load Transfer (Steady-State Model)

When the vehicle accelerates or brakes, the normal forces on the front and rear axles change.
The tyre forces act at ground level, while the vehicle's inertia acts at the Centre of Gravity
(CoG), which sits above the ground. This height offset creates a pitching couple that shifts
normal load between the axles. The total normal force stays the same; only its distribution
changes.

---

## 10.1. Free Body Diagram and Sign Convention

Consider the vehicle as a single rigid body in side view, with the following sign convention:

* $a_x$ is the longitudinal acceleration of the vehicle, **positive forward**,
* $a_x > 0$ means accelerating (load moves to the rear, squat),
* $a_x < 0$ means braking (load moves to the front, dive).

Forces acting on the vehicle:
* Weight $m \cdot g$ acting down at the CoG,
* Front and rear axle normal forces $F_{z,\text{front}}$ and $F_{z,\text{rear}}$ acting up at the contact patches,
* Longitudinal tyre forces acting at ground level,
* Inertial force $-m \cdot a_x$ acting at the CoG height $h$ (d'Alembert's principle: in the vehicle's
  frame, the body behaves as if a force opposite to the acceleration acts at its CoG).

---

## 10.2. Moment Balance About the Rear Contact Patch

Taking moments about the rear contact patch removes the unknown rear normal force and the
longitudinal tyre forces (they act at ground level, so they have no lever arm about this point).
In steady state the vehicle does not rotate, so the moments must balance:

$$F_{z,\text{front}} \cdot L = m \cdot g \cdot l_r - m \cdot a_x \cdot h \tag{10.1.}$$

Dividing by the wheelbase:

$$F_{z,\text{front}} = m \cdot g \cdot \frac{l_r}{L} - \frac{m \cdot a_x \cdot h}{L} \tag{10.2.}$$

The first term is the static front axle load (equation 6.1). The second term is the load transfer.

where:
* $F_{z,\text{front}}$ is front axle normal force in Newtons (N),
* $m$ is total vehicle mass in kilograms (kg),
* $g$ is gravitational acceleration in meters per second squared (m/s²),
* $l_r$ is horizontal distance from the CoG to the rear axle in meters (m),
* $L$ is wheelbase in meters (m),
* $a_x$ is signed longitudinal acceleration in meters per second squared (m/s²),
* $h$ is CoG height above the ground in meters (m).

---

## 10.3. Vertical Force Balance

The vehicle does not accelerate vertically, so the axle normal forces must always sum to the weight:

$$F_{z,\text{front}} + F_{z,\text{rear}} = m \cdot g \tag{10.3.}$$

Any load gained by one axle is lost by the other. Combining with equation 10.2:

$$F_{z,\text{rear}} = m \cdot g \cdot \frac{l_f}{L} + \frac{m \cdot a_x \cdot h}{L} \tag{10.4.}$$

where:
* $F_{z,\text{rear}}$ is rear axle normal force in Newtons (N),
* $l_f$ is horizontal distance from the CoG to the front axle in meters (m).

---

## 10.4. Load Transfer Term

The magnitude of the load moved between the axles is:

$$\Delta F_z = \frac{m \cdot a_x \cdot h}{L} \tag{10.5.}$$

where:
* $\Delta F_z$ is longitudinal load transfer in Newtons (N).

Physical interpretation: the longitudinal tyre force (at ground level) and the inertial force
(at the CoG) are equal and opposite but separated by height $h$. They form a pitching couple
of magnitude $m \cdot a_x \cdot h$. The only way to balance it is an opposing couple formed by
the axle normal forces, $\Delta F_z \cdot L$.

Design consequences:
* A **lower CoG** ($h$ smaller) reduces load transfer,
* A **longer wheelbase** ($L$ larger) reduces load transfer,
* Spring rates do **not** appear: in steady state, suspension stiffness changes pitch angle, not load transfer.

---

## 10.5. Consequences for Braking and Brake Bias

Each axle can only produce a longitudinal force up to its own traction limit (equation 9.1),
which now uses the transferred load:

$$F_{\text{max, axle}} = \mu_{\text{static}} \cdot F_{z,\text{axle}}(a_x) \tag{10.6.}$$

Braking demand is split between the axles by a single front brake bias ratio $\beta$,
so that the split always sums to 100%:

$$F_{\text{demand, front}} = \beta \cdot F_{\text{demand}}, \qquad F_{\text{demand, rear}} = (1 - \beta) \cdot F_{\text{demand}} \tag{10.7.}$$

where:
* $\beta$ is front brake bias ratio, dimensionless, in the range $[0, 1]$,
* $F_{\text{demand}}$ is total requested braking force in Newtons (N).

Under braking, load transfer grows the front axle's traction limit and shrinks the rear's.
A fixed bias can therefore be ideal at only one deceleration. Above it, one axle saturates
while the other still has unused grip, and the vehicle cannot reach the theoretical limit
$\mu_{\text{static}} \cdot m \cdot g$. The ideal bias always lies further forward than the
static weight distribution.

---

## 10.6. Implementation in the Simulation Lifecycle

Load transfer depends on $a_x$, but $a_x$ depends on the axle forces, which depend on load
transfer. To break this circular dependency, the simulation uses the **acceleration from the
previous time step**, stored in `CarState`:

$$F_{z,\text{axle}}(t) = f\left(a_x(t - \Delta t)\right) \tag{10.8.}$$

At $\Delta t = 1 \text{ ms}$ this lag is negligible for straight-line acceleration and braking.
The geometric parameters $h$, $L$, $l_f$ and $l_r$ are held in the `ChassisGeometry` struct.

---

## 10.7. Architectural Notes and Real-World Limitations

[!WARNING]
This is a steady-state, rigid-body load transfer model. It assumes the load transfer reaches
its final value instantly, with no pitch dynamics. In reality, the sprung mass pitches on its
springs and dampers, and the tyre loads arrive through a second-order transient (possible
overshoot and oscillation) governed by pitch inertia, spring rates and damping. The model
also ignores anti-dive and anti-squat suspension geometry (which routes part of the load
directly through the suspension links), the separate CoG heights of sprung and unsprung mass,
the height at which aerodynamic drag acts, aerodynamic downforce, and tyre load sensitivity
(real tyres lose friction coefficient as normal load increases, so load transfer reduces total
available grip even with ideal brake bias). The one-step acceleration lag is a numerical
approximation, not a physical effect. These limitations will be addressed when the chassis
pitch model is introduced.