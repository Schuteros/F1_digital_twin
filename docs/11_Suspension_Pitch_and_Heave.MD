# 11. Suspension: Pitch and Heave Dynamics

Docs/10 moves load between the axles instantly. A real car does not work that way. The body
(sprung mass) sits on springs and dampers, so when the tyres start braking the body first
has to rotate, the springs have to compress, and only then does the load reach the tyres.
This document replaces the instant load transfer with a spring-damper suspension in which
the body can **heave** (move up and down) and **pitch** (rotate nose up or nose down).
Once the motion settles, the result is exactly the load transfer of docs/10. The difference
is in how the load gets there: with a delay, and possibly with overshoot and oscillation.

The car is stored as four corners (front left, front right, rear left, rear right) so the same
data works in 2D later. Roll is not modelled yet. With no lateral force and a left-right
symmetric car, roll stays exactly zero, so both corners on an axle always behave the same.

---

## 11.1. Symbols and Sign Convention

Every symbol has exactly one meaning in this document. Time derivatives get their own names
instead of dots.

Body motion (the state):
* $z$ is heave, the downward displacement of the CoG from its static position, in meters (m), **positive down**,
* $v_z$ is heave velocity in meters per second (m/s),
* $a_z$ is heave acceleration in meters per second squared (m/s²),
* $\theta$ is pitch, the rotation of the body from its static attitude, in radians (rad), **positive nose down**,
* $\omega$ is pitch rate in radians per second (rad/s),
* $\alpha$ is pitch acceleration in radians per second squared (rad/s²).

Geometry and mass:
* $m$ is total vehicle mass in kilograms (kg); for now all of it is treated as sprung mass,
* $J$ is pitch moment of inertia of the body about its CoG in kilogram meters squared (kg·m²),
* $d_f$ is horizontal distance from the CoG to the front axle in meters (m) (called $l_f$ in docs/10),
* $d_r$ is horizontal distance from the CoG to the rear axle in meters (m) (called $l_r$ in docs/10),
* $W$ is wheelbase, $W = d_f + d_r$, in meters (m) (called $L$ in docs/10),
* $h$ is CoG height above the ground in meters (m),
* $a_x$ is longitudinal acceleration in meters per second squared (m/s²), **positive forward**, negative when braking.

Suspension, per corner:
* $k$ is wheel rate (spring rate at the wheel, already including the motion ratio) in Newtons per meter (N/m),
* $c$ is damping coefficient at the wheel in Newton seconds per meter (N·s/m),
* $s$ is suspension compression from static in meters (m), **positive = compressed**,
* $v_s$ is compression speed in meters per second (m/s),
* $F$ is suspension force on top of the static load in Newtons (N), positive = pushing the body up.

Subscripts $f$ and $r$ mean front and rear axle. Corner subscripts are $FL$, $FR$, $RL$, $RR$.

---

## 11.2. Measuring Everything From Static

All motion is measured from the static equilibrium: the car standing still on level ground.
At static, $z = 0$, $\theta = 0$, every $s = 0$, and every spring already carries its share of
the weight.

The full vertical force balance on the body is:

$$m \cdot a_z = m \cdot g - \left( F_{\text{static}} + F_f + F_r \right) \tag{11.1.}$$

At static the springs carry exactly the weight, $F_{\text{static}} = m \cdot g$, so those two
terms cancel. For a linear spring this is exact, not an approximation:

$$k \cdot (s_{\text{static}} + s) = k \cdot s_{\text{static}} + k \cdot s \tag{11.2.}$$

The first part is the static load and the second part is the extra force $F$. Spring free
lengths and static deflections are therefore never needed, and the state starts at zero. The
only parts that are not linear are the end stops (section 11.4). Their gaps are measured from
static too, so they still only need $s$.

**Static rake.** Many cars sit with the rear higher than the front (rake). Because $\theta$ is
the rotation *from* the static attitude, rake never enters the force equations. It only
matters for ride height, which aerodynamics will use later. The static attitude follows from
the static ride heights:

$$\theta_0 \approx \frac{RH_{r0} - RH_{f0}}{W}, \qquad \theta_{\text{abs}} = \theta_0 + \theta \tag{11.3.}$$

where:
* $RH_{f0}$, $RH_{r0}$ are static front and rear ride heights in meters (m),
* $\theta_0$ is static rake angle in radians (rad), positive with the rear higher (nose down),
* $\theta_{\text{abs}}$ is absolute body pitch in radians (rad).

With a rake of 1°, $d_f$, $d_r$ and $h$ change by a factor $\cos\theta_0 \approx 0.9998$, which
is ignored.

---

## 11.3. Corner Compression

The body is a rigid plank. Each corner moves for two reasons: the whole body sinks by $z$, and
the body tilts by $\theta$. For a small angle, a point at horizontal distance $d$ from the CoG
moves vertically by $d \cdot \theta$. Nose-down pitch moves points ahead of the CoG down and
points behind it up:

$$s_f = z + d_f \cdot \theta, \qquad s_r = z - d_r \cdot \theta \tag{11.4.}$$

Differentiating gives the compression speeds:

$$v_{s,f} = v_z + d_f \cdot \omega, \qquad v_{s,r} = v_z - d_r \cdot \omega \tag{11.5.}$$

Without roll, both corners on an axle have the same compression:
$s_{FL} = s_{FR} = s_f$ and $s_{RL} = s_{RR} = s_r$.

The ride heights follow directly:

$$RH_f = RH_{f0} - s_f, \qquad RH_r = RH_{r0} - s_r \tag{11.6.}$$

**Small-angle approximation.** Using $\sin\theta \approx \theta$ gives a relative error of about
$\theta^2 / 6$: 0.005% at 1° and 0.05% at 3°. F1 pitch and roll stay well under 1° (ride heights
change by tens of millimeters over a wheelbase of about 3.6 m), so the error is negligible.
The approximation keeps the equations linear and easy to check by hand.

---

## 11.4. Corner Force

Each corner pushes on the body with a spring force, a damper force, and an end stop force once
the travel passes a gap:

$$F_{\text{corner}} = k \cdot s + c \cdot v_s + F_{\text{stop}}(s) \tag{11.7.}$$

The end stops are stiff springs that only act past their gap:

$$
F_{\text{stop}}(s) =
\begin{cases}
k_{\text{bump}} \cdot (s - g_{\text{bump}}) & \text{if } s > g_{\text{bump}} \quad \text{(bump stop, compression)} \\
k_{\text{droop}} \cdot (s + g_{\text{droop}}) & \text{if } s < -g_{\text{droop}} \quad \text{(droop stop, extension)} \\
0 & \text{otherwise}
\end{cases}
\tag{11.8.}
$$

where:
* $g_{\text{bump}}$ is compression travel from static before the bump stop touches, in meters (m), $\ge 0$,
* $g_{\text{droop}}$ is extension travel from static before the droop limiter engages, in meters (m), $\ge 0$,
* $k_{\text{bump}}$, $k_{\text{droop}}$ are end stop stiffnesses at the wheel in Newtons per meter (N/m).

The droop stop force is negative: it pulls the body down and stops the wheel hanging further.

**Wheel lift.** A tyre can push on the road but never pull on it. The tyre load of a corner is:

$$F_{z,\text{corner}} = F_{z,\text{static,corner}} + F_{\text{corner}} \tag{11.9.}$$

If equation 11.7 gives $F_{\text{corner}} < -F_{z,\text{static,corner}}$, the wheel has lifted and
the suspension cannot pull the body down any further. The corner force is clamped:

$$F_{\text{corner}} \leftarrow \max\left( F_{\text{corner}}, \, -F_{z,\text{static,corner}} \right) \tag{11.10.}$$

The **clamped** value is used both for the tyre load and for the body equations, so the forces
on the tyres and on the body always agree. The static corner load comes from the mass split:
$F_{z,\text{static},FL} = F_{z,\text{static},FR} = m_{\text{front}} \cdot g / 2$, and the same for the rear.

The axle forces are the sums of their corners:

$$F_f = F_{FL} + F_{FR}, \qquad F_r = F_{RL} + F_{RR} \tag{11.11.}$$

---

## 11.5. Body Equations of Motion

**Heave.** Newton's second law, using only the extra forces (section 11.2). The suspension pushes
the body up and $z$ is measured down, hence the minus sign:

$$m \cdot a_z = -\left( F_f + F_r \right) \tag{11.12.}$$

**Pitch.** This is Newton's second law for rotation: moment of inertia times angular acceleration
equals the sum of moments about the CoG. A moment is a force times its perpendicular distance from
the CoG. The moment of inertia $J$ is the rotating equivalent of mass: the sum of every bit of
mass times its distance squared from the CoG, so mass far from the CoG resists rotation much more.

The moments about the CoG, positive nose down:
* **Load transfer moment** $-m \cdot a_x \cdot h$. The longitudinal tyre forces act at ground level,
  a distance $h$ below the CoG, and their sum is $m \cdot a_x$. Under braking ($a_x < 0$) this term is
  positive and pitches the nose down. This is the same couple as in docs/10, section 10.4.
* **Front suspension** $-d_f \cdot F_f$. It pushes up ahead of the CoG, which lifts the nose.
* **Rear suspension** $+d_r \cdot F_r$. It pushes up behind the CoG, which pushes the nose down.

$$J \cdot \alpha = -m \cdot a_x \cdot h \; - \; d_f \cdot F_f \; + \; d_r \cdot F_r \tag{11.13.}$$

The static loads do not appear: at static they balance each other's moments about the CoG,
for the same reason as in section 11.2.

---

## 11.6. Steady State: Recovering Docs/10

Under constant $a_x$, once the motion has settled, $a_z = 0$ and $\alpha = 0$. Equation 11.12 gives
$F_r = -F_f$. Substituting into equation 11.13:

$$0 = -m \cdot a_x \cdot h - d_f \cdot F_f - d_r \cdot F_f \quad \Rightarrow \quad F_f = -\frac{m \cdot a_x \cdot h}{W} \tag{11.14.}$$

This is exactly the load transfer term of docs/10 (equation 10.5), and the total load stays $m \cdot g$.
Spring rates, damping and $J$ do not appear. They decide the settled pitch angle and the path to the
steady state, not the steady-state load. This is the first test of any implementation.

The settled compressions and pitch angle (linear springs, no stops touched) are:

$$s_f = \frac{F_f}{k_f}, \qquad s_r = \frac{F_r}{k_r}, \qquad \theta_{ss} = \frac{s_f - s_r}{W} \tag{11.15.}$$

where $k_f$ and $k_r$ are now the **axle** rates (two corners each).

---

## 11.7. Transient Behaviour

**The first instant.** When braking starts, the load transfer moment is applied in full right away.
The body has not moved and has no velocity yet, so the springs and dampers carry nothing, and the
tyre loads are still static. The whole moment goes into pitch acceleration, $\alpha = -m \cdot a_x \cdot h / J$.
The springs and dampers only start carrying load once the body is moving.

**Dampers act first.** Compression speed appears before compression does, so early in the transient
most of the extra tyre load comes through the dampers.

**Overshoot.** The body reaches the steady-state angle while still rotating and carries on past it.
At low damping, the tyre load can briefly exceed the steady-state value from equation 11.14. Even a
heavily damped body can overshoot slightly in *load*, because the damper force adds on top of the
spring force while the body is still moving.

**Heave-pitch coupling.** A pure pitch $\theta$ changes the total suspension force by
$(k_f \cdot d_f - k_r \cdot d_r) \cdot \theta$. Pitch and heave are independent only if

$$k_f \cdot d_f = k_r \cdot d_r \quad \text{and} \quad c_f \cdot d_f = c_r \cdot d_r \tag{11.16.}$$

Otherwise, braking also moves the body up or down. For a car that meets equation 11.16
(for example a symmetric one), $z$ stays exactly zero under braking.

**Natural frequencies and damping ratios** (uncoupled case, axle rates):

$$\omega_{n,\text{heave}} = \sqrt{\frac{k_f + k_r}{m}}, \qquad \omega_{n,\text{pitch}} = \sqrt{\frac{k_f \, d_f^2 + k_r \, d_r^2}{J}} \tag{11.17.}$$

$$\zeta_{\text{heave}} = \frac{c_f + c_r}{2\sqrt{(k_f + k_r)\, m}}, \qquad \zeta_{\text{pitch}} = \frac{c_f \, d_f^2 + c_r \, d_r^2}{2\sqrt{(k_f \, d_f^2 + k_r \, d_r^2)\, J}} \tag{11.18.}$$

The natural frequency in Hertz is $f_n = \omega_n / 2\pi$ and the free oscillation period is
$T = 2\pi / \omega_n$ (for small damping). For a damped free oscillation, each peak is smaller
than the one before by a constant ratio:

$$\frac{\text{peak}_{n+1}}{\text{peak}_n} = \exp\left( -\frac{2\pi\zeta}{\sqrt{1 - \zeta^2}} \right) \tag{11.19.}$$

Doubling a spring rate with the same damper lowers $\zeta$ by a factor of $\sqrt{2}$: the body rotates
less but rings more, with larger load overshoot.

---

## 11.8. Implementation in the Simulation Lifecycle

The suspension state is $(z, v_z, \theta, \omega)$, added to the longitudinal state (speed, distance).
Accelerations are not part of the state. They are calculated from the state every step and may be
stored as outputs.

Each time step runs in this order:

1. **Compressions** $s_f, s_r$ and speeds $v_{s,f}, v_{s,r}$ from the current state (equations 11.4 and 11.5).
2. **Corner forces** with end stops and the wheel lift clamp (equations 11.7 to 11.10), then axle forces (11.11).
3. **Tyre loads** = static + corner force (equation 11.9). These replace `calculate_axle_loads` as the
   source of the axle normal forces.
4. **Longitudinal acceleration** $a_x$ from the existing net force code, using those tyre loads.
5. **Body accelerations** $a_z$ and $\alpha$ (equations 11.12 and 11.13), using this step's $a_x$.
6. **Update velocities:** speed, $v_z$ and $\omega$, each by acceleration $\times \, \Delta t$.
7. **Update positions with the new velocities:** distance, $z$ and $\theta$, each by velocity $\times \, \Delta t$.

Steps 1 to 5 only use the state from the start of the step, so there is no circular dependency, and
the lagged acceleration of docs/10, section 10.6, is no longer needed. Step 5 can use this step's
$a_x$ because $a_x$ does not depend on anything calculated after it.

Steps 6 and 7 are semi-implicit (symplectic) Euler, the same scheme already used for speed and
distance. The order matters: updating positions with the old velocities (plain forward Euler) adds
energy every step, and an undamped oscillation grows without limit. At $\Delta t = 1$ ms and 4 Hz it
grows by about 1.9× in 2 seconds. Semi-implicit Euler keeps the energy of an undamped oscillation
bounded (it wobbles by about $\pm \omega \, \Delta t / 2$ per cycle but does not drift).

**Time step limit.** Semi-implicit Euler is only stable for $\omega_{\max} \cdot \Delta t < 2$, where
$\omega_{\max}$ is the highest natural frequency in the model, including the end stops when they are
touched. For accuracy, $\omega_{\max} \cdot \Delta t$ should be well below that, about 0.2 or less.

---

## 11.9. Architectural Notes and Real-World Limitations

[!WARNING]
This is a two-degree-of-freedom (heave and pitch) rigid-body model with linear springs and dampers.
It treats the whole car as sprung mass, so the instant load transfer path through the unsprung mass
($m_u \cdot a_x \cdot h_u$, about 10% of an F1 car's mass at wheel-centre height) is routed through
the springs instead. It ignores anti-dive and anti-squat geometry (0% anti: the whole load transfer
moment goes through the springs), tyre vertical stiffness (in series with the suspension), roll
(zero in 1D), the height at which aerodynamic drag acts (the full $m \cdot a_x$ is assumed to act at
ground level), aerodynamic downforce and its dependence on ride height, nonlinear and asymmetric
(bump/rebound) dampers, and suspension friction. Small angles are assumed throughout.
