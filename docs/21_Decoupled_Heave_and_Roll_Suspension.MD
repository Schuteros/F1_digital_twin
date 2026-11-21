# 21. Decoupled Heave and Roll Suspension

Up to docs/19 every corner has one spring and one damper (docs/11). That spring feels its own
wheel, so the same wheel rate $k$ sets the heave, the pitch **and** the roll stiffness of the axle.
An anti-roll bar can only add roll stiffness on top. So a car with soft roll and stiff heave can
not be built in the model: making the corner springs stiff enough for heave already gives more
roll stiffness than wanted, and no bar can take it away.

Many cars do not work that way. A **decoupled** axle has no corner springs. The two pushrods drive
two separate elements instead:
* a **heave element** (spring + damper) that only feels both wheels moving together,
* a **roll element** (spring + damper) that only feels the wheels moving opposite ways.

Heave and roll can then be set independently: stiff in heave for the aero ride height, soft in
roll for mechanical grip, or the other way around. The damping is independent too. A third case
is a car with normal corner springs **plus** a heave element (a "third spring").

This document adds an optional heave and roll element per axle. They add to the corner springs and
dampers, the same way a bar adds to the springs, so one model covers all three layouts:

| Layout | Corner springs and dampers | Heave element | Roll element |
|---|---|---|---|
| Conventional (docs/11, docs/19) | yes | no | no (bar optional) |
| Third spring | yes | yes | no |
| Fully decoupled | 0 | yes | yes |

Nothing changes for a car without the new keys.

---

## 21.1. Symbols

Same conventions as docs/11 and docs/19. New symbols, per axle (front $f$, rear $r$), all measured
**at the wheel**:
* $k_{heave,f}$, $k_{heave,r}$ are the heave element stiffness in Newtons per meter (N/m): the
  force on the whole axle (both wheels together) per meter of heave of that axle,
* $c_{heave,f}$, $c_{heave,r}$ are the heave element damping in Newton seconds per meter (N·s/m),
  in the same way,
* $K_{roll,f}$, $K_{roll,r}$ are the roll element stiffness in Newton meters per radian (N·m/rad),
* $C_{roll,f}$, $C_{roll,r}$ are the roll element damping in Newton meter seconds per radian
  (N·m·s/rad),
* $F_{heave,f}$, $F_{roll,f}$ are the heave and roll element force at each wheel of the axle in
  Newtons (N).

$K_{\varphi,f}$ stays the **total** roll stiffness of the axle (springs, bar and roll element), and
$C_\varphi$ the total roll damping of the car.

For converting shock values to the wheel (section 21.5):
* $k_s$ is the stiffness of a shock along its own axis in N/m, $c_s$ its damping in N·s/m,
* $MR_h$ is the heave motion ratio: shock travel per meter of heave, both wheels moving together,
  dimensionless,
* $MR_\varphi$ is the roll motion ratio: shock travel per radian of roll, in meters per radian
  (m/rad).

---

## 21.2. What Each Element Feels

Take the front axle with the corner compressions $s_{FL}$, $s_{FR}$ of equation 19.8.

**Heave element.** It sits on the centre line, between the two rockers or on a rocker that both
pushrods drive. It feels the **average** of the two wheels:

$$s_{heave,f} = \frac{s_{FL} + s_{FR}}{2} \tag{21.1.}$$

In pure roll $s_{FR} = -s_{FL}$, the average is 0 and the heave element does not move. In heave
and pitch both wheels move the same, and the heave element moves with them.

**Roll element.** It feels the **difference** of the two wheels, $s_{FR} - s_{FL}$, exactly like
an anti-roll bar (docs/19 section 19.4). In pure roll $s_{FR} - s_{FL} = b_f \cdot \varphi$. In
heave and pitch the difference is 0 and the roll element does not move.

---

## 21.3. Element Forces at the Wheels

**Heave element.** It pushes the axle up with $k_{heave,f} \cdot s_{heave,f}$ in total. The two
pushrods carry it equally (Newton's third law: a spring pushes both its ends with the same force),
so each wheel gets half:

$$F_{heave,f} = \frac{1}{2} \cdot \left( k_{heave,f} \cdot \frac{s_{FL} + s_{FR}}{2} + c_{heave,f} \cdot \frac{v_{s,FL} + v_{s,FR}}{2} \right) \tag{21.2.}$$

on **both** front wheels, positive = pushing the body up.

**Roll element.** The same form as the bar force 19.10, with damping:

$$F_{roll,f} = \frac{K_{roll,f} \cdot (s_{FR} - s_{FL}) + C_{roll,f} \cdot (v_{s,FR} - v_{s,FL})}{b_f^2} \tag{21.3.}$$

added to the right wheel and subtracted from the left one. In pure roll the two forces make the
moment $F_{roll,f} \cdot b_f = K_{roll,f} \cdot \varphi + C_{roll,f} \cdot \omega_\varphi$.

**Corner force.** Equation 19.11 becomes:

$$F_{corner} = k \cdot s + c \cdot v_s + F_{stop}(s) + F_{heave} \pm (F_{arb} + F_{roll}) \tag{21.4.}$$

with $+$ on the right corners and $-$ on the left ones. The wheel lift clamp (11.10) is applied to
this total, as before. The end stops stay at the corners.

**Checks.** Both front wheels 10 mm up, $k_{heave,f} = 260\,000$ N/m: each wheel gets
$\frac{1}{2} \cdot 260\,000 \cdot 0.010 = 1300$ N, the axle 2600 N $= k_{heave,f} \cdot 0.010$,
and the roll element 0 N. Pure roll of 0.01 rad on $b_f = 1.6$ m with $K_{roll,f} = 83\,200$ N·m/rad:
the wheels move ∓8 mm, the heave element 0 N, the roll element $83\,200 \cdot 0.016 / 1.6^2 = 520$ N
per wheel, and the moment $520 \cdot 1.6 = 832$ N·m $= 83\,200 \cdot 0.01$.

---

## 21.4. Axle Totals

Because the elements only add forces, every formula of docs/11 and docs/19 that uses an axle rate
or a roll stiffness keeps working with these totals:

$$k_f = k_{FL} + k_{FR} + k_{heave,f}, \qquad c_f = c_{FL} + c_{FR} + c_{heave,f} \tag{21.5.}$$

$$K_{\varphi,f} = (k_{FL} + k_{FR}) \cdot \frac{b_f^2}{4} + K_{arb,f} + K_{roll,f} \tag{21.6.}$$

$$C_\varphi = (c_{FL} + c_{FR}) \cdot \frac{b_f^2}{4} + C_{roll,f} + (c_{RL} + c_{RR}) \cdot \frac{b_r^2}{4} + C_{roll,r} \tag{21.7.}$$

and the same for the rear. So the heave and pitch frequencies and damping ratios (11.17, 11.18),
the settled roll and load transfer (19.16, 19.17, 19.22), the roll frequencies and damping ratio
(19.23, 19.25), the corner speed limits (docs/16 with 19.17) and the time step check all use 21.5
to 21.7. The time step check uses the stiffest rates, as before: corner springs plus end stops,
plus the heave element in heave and pitch and the roll element in roll.

The heave element changes the heave **and** the pitch stiffness ($k_f \cdot d_f^2 + k_r \cdot d_r^2$),
because in pitch both wheels of an axle move together. The roll element only changes roll.

---

## 21.5. From the Shock to the Wheel

The car file takes the values at the wheel. Teams usually know the shock rate and the motion
ratio. The conversion comes from energy: the spring stores the same energy whether we look at it
from the shock or from the wheel.

**Heave.** Both wheels move up by $x$, the shock shortens by $MR_h \cdot x$:

$$\frac{1}{2} \cdot k_s \cdot (MR_h \cdot x)^2 = \frac{1}{2} \cdot k_{heave} \cdot x^2 \quad \Rightarrow \quad k_{heave} = k_s \cdot MR_h^2, \qquad c_{heave} = c_s \cdot MR_h^2 \tag{21.8.}$$

$MR_h$ is the **total** shortening of the shock. If the shock floats between two rockers and each
end moves 1 mm per mm of heave, the shock shortens 2 mm, $MR_h = 2$ and $k_{heave} = 4 \cdot k_s$.
Each wheel feels $k_s \cdot 2x$ (the whole spring force, Newton's third law), and both wheels
together $2 \cdot k_s \cdot 2x = 4 \cdot k_s \cdot x$. The same answer.

**Roll.** The body rolls by $\varphi$, the roll shock moves by $MR_\varphi \cdot \varphi$:

$$K_{roll} = k_s \cdot MR_\varphi^2, \qquad C_{roll} = c_s \cdot MR_\varphi^2 \tag{21.9.}$$

A roll ratio given in degrees of roll per mm of shock travel, $r$ in °/mm, is

$$MR_\varphi = \frac{180}{\pi \cdot r} \text{ mm/rad} = \frac{0.0573}{r} \text{ m/rad}$$

**Example** (placeholder numbers). Heave shock $k_s = 50$ N/mm with $MR_h = 1.4$, roll shock
$k_s = 40$ N/mm with $r = 0.05$ °/mm:

$$k_{heave} = 50\,000 \cdot 1.4^2 = 98\,000 \text{ N/m}$$

$$MR_\varphi = \frac{0.0573}{0.05} = 1.146 \text{ m/rad}, \qquad K_{roll} = 40\,000 \cdot 1.146^2 = 52\,500 \text{ N·m/rad}$$

**Measuring on the car.** A force measured at the pushrod is not the force at the wheel. By energy,
$F_{wheel} = F_{pushrod} \cdot (\text{pushrod travel} / \text{wheel travel})$, roughly
$F_{pushrod} \cdot \sin(\text{pushrod angle to the horizontal})$ for a pushrod near the wheel.
Load both wheels of an axle equally and only the heave element moves. Load one side up and the
other down by the same amount and only the roll element moves. Loading one wheel alone moves both,
and the result is a mix of the two.

---

## 21.6. Worked Example

Baseline car (docs/19 section 19.7): $m = 815.5$ kg, $h = 0.3$ m, $b_f = b_r = 1.6$ m,
corner springs 130 000 N/m, roll centres 0.05 m / 0.10 m ($h_{ra} = 0.075$ m, so
$J_{\varphi,ra} = 250 + 815.5 \cdot 0.225^2 = 291$ kg·m²), $a_y = 8$ m/s², so
$m \cdot a_y \cdot (h - h_{ra}) = 1468$ N·m.

**Conventional.** $k_f = k_r = 260\,000$ N/m, heave frequency
$\sqrt{520\,000 / 815.5} = 25.3$ rad/s (4.0 Hz), $K_{\varphi,f} = K_{\varphi,r} = 166\,400$ N·m/rad,
roll 0.25°, transfer 561 N / 663 N.

**Half the roll stiffness with springs only.** Corner springs of 65 000 N/m give
$K_\varphi = 83\,200$ N·m/rad per axle, but the heave frequency drops to
$\sqrt{260\,000 / 815.5} = 17.9$ rad/s (2.8 Hz). The aero platform gets softer too.

**Half the roll stiffness, decoupled.** Corner springs and dampers 0, per axle
$k_{heave} = 260\,000$ N/m and $K_{roll} = 83\,200$ N·m/rad:

$$k_f + k_r = 520\,000 \text{ N/m} \quad \Rightarrow \quad 4.0 \text{ Hz, the same as conventional}$$

$$\varphi_{ss} = \frac{1468}{2 \cdot 83\,200} = 0.00882 \text{ rad} = 0.51°$$

The transfer is still 561 N / 663 N: with equal front and rear roll stiffness each axle takes half
of the elastic part, whatever the total. **The total roll stiffness sets how much the body rolls.
The front/rear ratio sets where the load goes.**

**Moving the balance.** Raise only the front roll element to $K_{roll,f} = 166\,400$ N·m/rad. The
front takes 2/3 of the elastic part, exactly as with the front bar of section 19.7:

$$\Delta F_{z,f} = 102 + \frac{2}{3} \cdot \frac{1468}{1.6} = 714 \text{ N}, \qquad \Delta F_{z,r} = 204 + \frac{1}{3} \cdot \frac{1468}{1.6} = 510 \text{ N}$$

$$\varphi_{ss} = \frac{1468}{249\,600} = 0.34°$$

and heave and pitch do not change at all.

**Damping.** With the corner dampers at 0, all damping comes from the elements. Heave:
$c_{heave} = 10\,000$ N·s/m per axle gives the same $\zeta_{heave} = 0.486$ as the baseline
corners (5000 N·s/m each). Roll, back to $K_{roll} = 83\,200$ N·m/rad per axle:

$$\omega_{n,roll} = \sqrt{\frac{166\,400}{291}} = 23.9 \text{ rad/s (3.8 Hz)}$$

$$C_{roll} = 4500 \text{ N·m·s/rad per axle:} \qquad \zeta_{roll} = \frac{9000}{2 \sqrt{166\,400 \cdot 291}} = 0.65$$

With corner dampers this ratio would be locked to the heave damping. Now it is set on its own.

---

## 21.7. Steady State and Transients

No new equations. The settled roll and transfer are 19.16 and 19.17 with $K_\varphi$ from 21.6. The
first instant of a corner is the same as docs/19 section 19.10: all of the elastic moment goes
into roll acceleration, because no element has moved yet.

A roll element with a damper only ($K_{roll} = 0$) and no corner springs or bar holds no roll moment
in a long corner. The body rolls until the end stops carry it, and only the geometric part of the
transfer goes through the links without roll. The model allows that, but the time step check and
the settled values then rely on the end stops. So at least one of the two axles must have some
roll stiffness (section 21.9).

---

## 21.8. Implementation in the Simulation Lifecycle

Step 3 of docs/19 section 19.11 (corner forces) adds the heave and roll element forces 21.2 and 21.3
of each axle before the wheel lift clamp. Every other step stays the same.

Before the run (`prepare`) and in the validation: the axle totals 21.5 to 21.7 replace the corner
sums in the frequencies, damping ratios, roll stiffnesses and the time step check.

Code (new names, nothing existing renamed):
* `AxleSuspension` struct with `heave_stiffness`, `heave_damping`, `roll_stiffness`,
  `roll_damping`, and two optional fields `front_axle`, `rear_axle` in `Suspension`,
* `calculate_heave_element_force` (21.2) and `calculate_roll_element_force` (21.3), used in
  `calculate_corner_forces` next to the bar force,
* `calculate_roll_stiffnesses` adds $K_{roll}$ (21.6), `calculate_roll_damping_ratio` adds
  $C_{roll}$ (21.7), and the heave and pitch frequencies and the time step check use 21.5.

**Change to existing validation, needs approval:** corner `spring_rate` must be $\ge 0$ instead of
$> 0$, so a fully decoupled axle can set it to 0. New checks: the axle heave rate $k_f$ (21.5) must
be $> 0$ for each axle, $K_{\varphi,f} + K_{\varphi,r} > 0$, and the four new values finite and $\ge 0$.

---

## 21.9. Config

Car file, new optional tables, one per axle. A missing table, or a missing key in it, means 0:

```toml
# Optional: heave and roll elements of the axle, measured at the wheel (docs/21).
# They add to the corner springs and dampers. A fully decoupled axle sets those to 0.
[suspension.front_axle]
heave_stiffness_N_per_m = 260_000.0
heave_damping_N_s_per_m = 10_000.0
roll_stiffness_N_m_per_rad = 83_200.0
roll_damping_N_m_s_per_rad = 4_500.0

[suspension.rear_axle]
heave_stiffness_N_per_m = 260_000.0
heave_damping_N_s_per_m = 10_000.0
roll_stiffness_N_m_per_rad = 83_200.0
roll_damping_N_m_s_per_rad = 4_500.0
```

"Stiffness" and not "rate", because `roll_rate_rad_per_s` already means the roll speed.
$K_{roll}$ and $K_{arb}$ do the same thing in the model. The roll element has its own key so a car
file says what the car really has: a roll spring, a bar, or both.

No new state, run file keys or log columns. The baseline car does not change. Version v0.15.0.

---

## 21.10. Tests

* No axle tables: every result is the same as v0.14.0.
* Pure heave moves only the heave element, pure roll only the roll element (21.2, 21.3 and the
  checks of section 21.3).
* A decoupled car with $k_{heave} = 2k$ and $K_{roll} = k \cdot b^2/2$ per axle and $c_{heave} = 2c$,
  $C_{roll} = c \cdot b^2/2$ gives the same motion and tyre loads as the conventional car with
  corner springs $k$ and dampers $c$ (braking and cornering together, then cornering alone).
* Section 21.6: roll 0.51° and transfer 561 N / 663 N with half the roll stiffness, heave frequency
  unchanged; 714 N / 510 N and 0.34° with the stiffer front roll element. The dynamic roll settles
  at both.
* A roll element does nothing in heave and pitch: same braking response with and without it. A
  heave element does nothing in roll: a pure roll start never moves heave or pitch.
* Roll frequency and damping ratio of section 21.6 (23.9 rad/s, 0.65). Undamped free heave and roll
  oscillations of the decoupled car have the periods of 21.5 and 21.6, and stay bounded.
* Config: a table with only some keys gives 0 for the others, an unknown key is rejected.
* Validation: corner `spring_rate = 0` is accepted while the axle keeps a heave rate and rejected
  when both corners of an axle are 0 without a heave element;
  zero total roll stiffness is rejected; negative or NaN element values are rejected; the time
  step check catches a very stiff heave element.

---

## 21.11. Limitations

[!WARNING]
The elements are linear springs and dampers. Air springs stiffen as they compress, and real
dampers are not linear in speed. The heave element has no bump stop or preload of its own, though
on aero cars the heave element's bump rubber often sets the ride height at high speed; the corner
end stops stand in for it. The motion ratios are constant over the travel, and the heave and roll
elements do not interact through the rocker geometry. Everything in docs/19 section 19.14 still
applies.
