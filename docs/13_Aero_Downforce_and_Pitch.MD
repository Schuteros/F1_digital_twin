# 13. Aerodynamic Loads: Downforce and Drag Pressure Points

Docs/04 treats drag as a force that only slows the car down, and docs/11 assumes the whole
longitudinal force acts at ground level. Two things are missing:

* **Downforce.** The air pushes the car down. That adds tyre load (more grip) and compresses the
  springs (lower ride height).
* **Where the aero forces act.** A force that does not act through the CoG also creates a pitch
  moment. Drag acts above the ground, so it lifts the nose. Downforce acts ahead of or behind the
  CoG, so it pitches the car forward or back.

This document adds both to the body equations of docs/11. The springs and dampers then carry the
aero loads to the four corners, so the tyre loads and the ride heights follow without any new
load calculation.

The points where the forces act (centres of pressure) are **fixed** for now. In reality they move
with speed, ride height and pitch. That can be added later without changing the equations below,
only the inputs.

---

## 13.1. Symbols and Sign Convention

Same conventions as docs/11: $z$ positive down, $\theta$ positive nose down, moments about the CoG
positive nose down. Symbols from docs/11 keep their meaning ($m$, $J$, $h$, $d_f$, $d_r$, $W$,
$a_x$, $a_z$, $\alpha$, $dF_f$, $dF_r$, $s_f$, $s_r$).

New symbols:
* $\rho$ is air density in kilograms per cubic meter (kg/m³),
* $v$ is car speed in meters per second (m/s),
* $A$ is frontal area in square meters (m²), the same reference area for drag and downforce,
* $C_d$ is the drag coefficient (docs/04), dimensionless,
* $C_{down}$ is the downforce coefficient, dimensionless, **positive = pushes the car down**,
* $F_{drag}$ is the drag force in Newtons (N), positive = pushing the car backwards,
* $F_{downforce}$ is the downforce in Newtons (N), positive = pushing the car down,
* $h_{drag}$ is the height of the drag centre of pressure above the ground in meters (m),
* $x_{cp}$ is the horizontal distance from the **front axle** back to the downforce centre of pressure in meters (m). It can be anywhere: $x_{cp} < d_f$ puts it ahead of the CoG, $x_{cp} > d_f$ behind it.

---

## 13.2. The Aero Forces and Where They Act

Drag is unchanged from docs/04, equation 4.1. Downforce uses the same quadratic law:

$$F_{downforce} = \frac{1}{2} \cdot \rho \cdot C_{down} \cdot A \cdot v^2 \tag{13.1.}$$

There is no $\text{sign}(v)$: the air pushes the car down whichever way it moves.

Only the product $C \cdot A$ matters physically. Using one $A$ for both forces means that a
change of area (for example active aero later) changes both forces together, and the coefficients
carry the difference between them.

The downforce point is given from the front axle, because that is a fixed point on the car. The
CoG position is not stored, it follows from the mass split (docs/10 section 10.6). The equations
below work for a centre of pressure ahead of the CoG, behind it, or even outside the wheelbase, the
sign takes care of it (section 13.3).

Aero engineers often give the **aero balance** instead, the fraction of the downforce on the front
axle. In steady state it is $\frac{W - x_{cp}}{W}$ (section 13.4). A choice between the two inputs can be
added later.

---

## 13.3. Body Equations of Motion

**Heave.** The downforce pushes the body down, which is the positive $z$ direction, so it adds
directly to equation 11.12:

$$m \cdot a_z = -\left( dF_f + dF_r \right) + F_{downforce} \tag{13.2.}$$

The body rotates about its CoG, and $J$ is the pitch inertia about the CoG, so the **dynamic**
pitch equation (11.13) has to sum the moments about the CoG. That is the only reason the CoG appears
here. For the settled loads it is not needed (section 13.4).

**Pitch: the drag.** The drag makes the same kind of moment as the CoG does in docs/10 and
docs/11, only at its own height. Compare the two:

* **Inertia (docs/10).** The tyres push the car forward with $m \cdot a_x$ at ground level. The
  body's mass resists at the CoG, at height $h$. A forward push at the ground and a backward
  resistance at height $h$ tip the car: the moment is $m \cdot a_x \cdot h$.
* **Drag.** To overcome the drag, the tyres push forward with another $F_{drag}$ at ground level.
  The air pushes back with $F_{drag}$ at height $h_{drag}$. Same picture, so the moment is
  $F_{drag} \cdot h_{drag}$.

In both cases: force × height above the ground of the point that pushes back. The CoG height
does not enter the drag term, because the drag does not act at the CoG. A forward push at the
ground with a backward push above it lifts the nose, hence the minus sign in the pitch equation:
$-F_{drag} \cdot h_{drag}$.

**Pitch: the downforce.** The downforce pushes down at $x_{cp}$ behind the front axle. The CoG is
$d_f$ behind the front axle, so the downforce acts $d_f - x_{cp}$ **ahead of** the CoG. Its moment,
positive nose down, is $F_{downforce} \cdot (d_f - x_{cp})$:

* Centre of pressure ahead of the CoG ($x_{cp} < d_f$): positive, pushes the nose down, loads the front.
* Centre of pressure behind the CoG ($x_{cp} > d_f$): negative, lifts the nose, loads the rear
  (the "stable rear at high speed" setup).
* Exactly at the CoG ($x_{cp} = d_f$): no moment, only heave.

Example: $d_f = 1.7$ m. With $x_{cp} = 1.8$ m the downforce is 0.1 m behind the CoG, and
$F_{downforce} \cdot (1.7 - 1.8) = -0.1 \cdot F_{downforce}$, nose up.

Equation 11.13 becomes:

$$J \cdot \alpha = -m \cdot a_x \cdot h \; - \; F_{drag} \cdot h_{drag} \; + \; F_{downforce} \cdot (d_f - x_{cp}) \; - \; d_f \cdot dF_f \; + \; d_r \cdot dF_r \tag{13.3.}$$

With $F_{drag} = F_{downforce} = 0$ both equations are exactly docs/11 again.

Note what docs/11 got wrong without the drag term: at constant top speed ($a_x = 0$) it gave no
pitch moment at all. In reality the drag still lifts the nose, because the tyres keep pushing at
the ground to hold the speed.

---

## 13.4. Steady State

At constant speed ($a_x = 0$), once the motion has settled, the car is in balance and the CoG is
not needed at all. Take moments about the front tyre contact patch, where the front tyre forces
have no lever arm:

* the rear axle pushes up with $dF_r$ at distance $W$ (nose down),
* the downforce pushes down at distance $x_{cp}$ (nose up),
* the drag pushes back at height $h_{drag}$ (nose up). The rear tyre forward forces are at ground
  level, in line with the contact patch, so they have no lever arm either.

$$dF_r \cdot W = F_{downforce} \cdot x_{cp} + F_{drag} \cdot h_{drag}$$

The front takes the rest of the downforce: $dF_f = F_{downforce} - dF_r$. Together:

$$dF_f = \frac{F_{downforce} \cdot (W - x_{cp}) - F_{drag} \cdot h_{drag}}{W}, \qquad dF_r = \frac{F_{downforce} \cdot x_{cp} + F_{drag} \cdot h_{drag}}{W} \tag{13.4.}$$

Reading it:
* The downforce splits between the axles like a load on a beam: the closer to an axle, the more
  that axle carries. The front share $\frac{W - x_{cp}}{W}$ is the aero balance.
* The drag moves $F_{drag} \cdot h_{drag} / W$ from the front axle to the rear axle.
* Like equation 11.14, spring rates, damping and $J$ do not appear. They decide how far the body
  moves (ride height and pitch), not the steady loads.

With acceleration as well, the docs/11 term simply adds: $dF_f$ gets another $-m \cdot a_x \cdot h / W$
and $dF_r$ another $+m \cdot a_x \cdot h / W$.

**Worked example** with the proposed placeholder values (section 13.6) at $v = 80$ m/s,
$\rho = 1.225$ kg/m³, $A = 2.0$ m²:

Downforce and drag:

$$F_{downforce} = 0.5 \cdot 1.225 \cdot 1.0 \cdot 2.0 \cdot 80^2 = 7840 \text{ N}$$

$$F_{drag} = 0.5 \cdot 1.225 \cdot 0.35 \cdot 2.0 \cdot 80^2 = 2744 \text{ N}$$

Axle forces from equation 13.4 ($W = 3.4$ m, $x_{cp} = 1.8$ m, $h_{drag} = 0.4$ m):

$$dF_f = \frac{7840 \cdot 1.6 - 2744 \cdot 0.4}{3.4} = 3367 \text{ N}$$

$$dF_r = \frac{7840 \cdot 1.8 + 2744 \cdot 0.4}{3.4} = 4473 \text{ N}$$

Check: $3367 + 4473 = 7840$ N, all of the downforce.

Compressions with 260 000 N/m per axle (springs only):

$$s_f = \frac{3367}{260\,000} = 13.0 \text{ mm}, \qquad s_r = \frac{4473}{260\,000} = 17.2 \text{ mm}$$

The rear is past its 15 mm bump gap, so it sits on the bump stops at about 15.3 mm. Ride heights:
front 30 − 13.0 = 17 mm, rear 60 − 15.3 = 45 mm.

So with these placeholders the car already reaches its rear bump stops at 80 m/s. That is not a bug:
real downforce cars run very stiff, or on their stops, for exactly this reason. It shows the
placeholder springs are too soft for this much downforce.

---

## 13.5. Implementation in the Simulation Lifecycle

Docs/11 section 11.8 stays the same, with one extra input:

1. to 4. unchanged: compressions, corner forces, tyre loads, then $a_x$.
5. **Aero forces** $F_{drag}$ and $F_{downforce}$ from the speed **at the start of the step** (the same
   speed the drag in the net force already uses), then the body accelerations with equations 13.2 and 13.3.
6. and 7. unchanged: velocities, then positions (semi-implicit Euler).

Everything in step 5 still uses only the state from the start of the step, so there is no circular
dependency.

The downforce reaches the tyres one step later than the forces in the body equations, through the
springs, exactly like the load transfer in docs/11. That extra tyre load then raises the grip limit
and also the rolling resistance, which both already use the tyre loads.

New keys in the car file, `[aero]` section:

| Key | Unit | Symbol |
|---|---|---|
| `downforce_coefficient` | | $C_{down}$ |
| `downforce_centre_of_pressure_from_front_axle_m` | m | $x_{cp}$ |
| `drag_centre_of_pressure_height_m` | m | $h_{drag}$ |

Validation: all three finite. $x_{cp}$ may lie outside the wheelbase (a big front wing can put the
centre of pressure ahead of the front axle), and $C_{down}$ may be negative (lift).

---

## 13.6. Placeholder Values

Proposed, to be replaced with sourced values:
* $C_{down} = 1.0$ (with $A = 2.0$ m², $C_{down} \cdot A = 2.0$ m²),
* $x_{cp} = 1.8$ m (aero balance $1.6 / 3.4 = 47\%$ front),
* $h_{drag} = 0.4$ m.

---

## 13.7. Tests

1. Downforce: $0.5 \cdot 1.225 \cdot 1.0 \cdot 2.0 \cdot 80^2 = 7840$ N, the same value at $-80$ m/s, 0 N at standstill.
2. Downforce lever for the default car: $d_f - x_{cp} = 1.7 - 1.8 = -0.1$ m (behind the CoG).
3. Body accelerations by hand, with the springs at static: $a_z = F_{downforce} / m$, $\alpha = (-F_{drag} \cdot h_{drag} + F_{downforce} \cdot (d_f - x_{cp})) / J$.
4. With zero aero forces the results equal docs/11 exactly (all existing suspension tests unchanged).
5. Steady state at constant aero forces (equation 13.4): axle loads, total load = $m \cdot g + F_{downforce}$,
   both axles compressed, nose up for the default car.
6. Moving the drag point higher moves load to the rear; moving $x_{cp}$ forward moves load to the front.
7. A full run still gives sensible results: the ride heights drop with speed, and the total tyre
   load rises with $v^2$.

---

## 13.8. Limitations

[!WARNING]
Centres of pressure and coefficients are constant. Real ones change with ride height, pitch, speed,
yaw and DRS or active aero. Ground effect (downforce rising sharply as ride height drops) is not
modelled, so porpoising cannot appear. The aero forces use the speed at the start of the step.
Wind is ignored (air speed = car speed). Drag is treated as purely horizontal and downforce as purely
vertical, so pitch does not tilt them. The heave and pitch limits of docs/11 still apply.
