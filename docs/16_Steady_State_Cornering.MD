# 16. Steady-State Cornering

Docs/15 gave every point of the track a curvature. This document uses it: a car that follows a
curved line needs a sideways force, only the tyres can give it, and the tyres have a limit
(docs/14). From that we get three things:

* the lateral force each tyre has to make at the current speed and curvature,
* how much grip is then left for driving and braking (the friction ellipse of docs/14, which was
  waiting for a lateral force),
* the **corner speed limit**: the highest speed at which the tyres can still hold the line. It is
  calculated for every track point before the run starts.

"Steady state" means the car is treated as if it were going round a constant circle at constant
speed: no turning in, no roll motion, no yaw motion. That is enough to find how fast a corner can
be taken.

This step does **not** make the car brake before a corner. It only stops accelerating when it
reaches the limit. Braking points come in docs/17.

---

## 16.1. Symbols

Same conventions as the earlier documents. New symbols:
* $a_y$ is the lateral acceleration of the car in m/s², towards the centre of the corner,
* $F_y$ is the total lateral force the corner needs in Newtons (N),
* $F_{y,f}$, $F_{y,r}$ are the lateral forces of the front and rear axle in Newtons (N),
* $F_{z,f}$, $F_{z,r}$ are the loads on the front and rear axle in Newtons (N),
* $D_f$, $D_r$ are how much load each axle gains per unit of speed squared from the air, in N/(m/s)²,
* $q_f$, $q_r$ are the aero load gain of each axle per kilogram it carries, in 1/m (section 16.5),
* $v_{limit}$ is the corner speed limit in m/s,
* $v_{limit,f}$, $v_{limit,r}$ are the speeds at which the front and the rear axle run out of grip, in m/s.

From earlier documents: $m$ mass, $v$ speed, $g$ gravity, $\bar{\kappa}$ smoothed curvature
(docs/15), $W$ wheelbase, $d_f$, $d_r$ distances from the CoG to the front and rear axle (docs/11),
$\mu_y$, $F_{y,max}$, $F_{x,avail}$ (docs/14), $\rho$, $A$, $C_d$, $C_{down}$, $x_{cp}$,
$h_{drag}$ (docs/04, docs/13).

---

## 16.2. The Force a Corner Needs

A body that moves at speed $v$ on a circle of radius $R$ is always changing direction. That change
of direction is an acceleration towards the centre of the circle:

$$a_y = \frac{v^2}{R} = v^2 \cdot |\bar{\kappa}| \tag{16.1.}$$

By Newton's second law something has to push the car towards the centre with

$$F_y = m \cdot v^2 \cdot |\bar{\kappa}| \tag{16.2.}$$

This is the centripetal force. Nothing else touches the car sideways, so the four tyres have to
make all of it. If they cannot, the car does not turn as much as the line does: it runs wide.

Example: baseline car, $m = 815.5$ kg, corner of $R = 15$ m, $v = 10$ m/s:

$$a_y = \frac{10^2}{15} = 6.67 \text{ m/s}^2 \; (0.68 \text{ g}), \qquad F_y = 815.5 \cdot 6.67 = 5437 \text{ N}$$

The force grows with the **square** of the speed: 10 % faster needs 21 % more grip.

---

## 16.3. How the Axles Share It

In steady state the car does not spin faster or slower around its vertical axis, so the two axle
forces must not turn it. Their moments around the CoG cancel:

$$F_{y,f} \cdot d_f = F_{y,r} \cdot d_r \tag{16.3.}$$

Together with $F_{y,f} + F_{y,r} = F_y$:

$$F_{y,f} = F_y \cdot \frac{d_r}{W}, \qquad F_{y,r} = F_y \cdot \frac{d_f}{W} \tag{16.4.}$$

This is the same split as the static weight (docs/10): the axle that carries more of the mass has
to make more of the cornering force. The baseline car is 50/50, so each axle makes 2718 N in the
example.

The left and right tyre of an axle each make half of the axle force. In a real corner load moves
from the inside to the outside tyre (lateral load transfer) and the outside tyre does more of the
work. That is left out here, see section 16.10.

---

## 16.4. Grip Check and What Is Left for Driving and Braking

Each tyre has the lateral limit $F_{y,max}$ at its current load (docs/14, equation 14.4). The load
comes from the suspension as before, so it already includes downforce and pitch.

While the tyre is below its limit, the friction ellipse gives the longitudinal force that is left
(docs/14, equation 14.8):

$$F_{x,avail} = F_{x,max} \cdot \sqrt{1 - \left( \frac{F_{y,tyre}}{F_{y,max}} \right)^2}$$

So far $F_{y,tyre}$ was always 0. Now it is half of the axle force from 16.4, and both traction and
braking get weaker in a corner without any new rule.

Example, front tyre of the baseline car at 10 m/s in the 15 m corner. Its load is 2026 N (2000 N
static plus downforce), $\mu_x = \mu_y = 0.9$:

$$F_{x,max} = F_{y,max} = 0.9 \cdot 2026 = 1824 \text{ N}, \qquad F_{y,tyre} = \frac{2718}{2} = 1359 \text{ N}$$

$$F_{x,avail} = 1824 \cdot \sqrt{1 - \left( \frac{1359}{1824} \right)^2} = 1216 \text{ N}$$

The corner uses 75 % of the lateral grip and 67 % of the braking grip is still there.

---

## 16.5. The Corner Speed Limit

The limit is the speed where an axle needs exactly as much lateral force as it has.

**How the simulation calculates it.** It uses the tyre model of docs/14 as it is in the code, with
load sensitivity, tyre by tyre. For a test speed $v$:
1. the axle loads at that speed, static weight plus downforce and drag (16.6 below), half on each tyre,
2. the lateral limit of each tyre at its load, $F_{y,max}$ from `calculate_max_tyre_force`
   (docs/14 equations 14.1 to 14.5, so $\mu_y$ drops as downforce raises the load),
3. the lateral force each axle needs (16.2 and 16.4).

The highest speed where both axles still have enough is $v_{limit}$. It is found by halving the
search range about 40 times. This is done **once for every track point before the run starts**
(244 points on the oval), not in the time steps, so it costs nothing during the run. A time step
only looks the stored value up.

The formulas below are the special case of constant $\mu$ ($\lambda = 0$). They are here to show
what the limit depends on, and the tests use them to check the search.

**Without downforce** the car has $\mu_y \cdot m \cdot g$ of grip in total, so
$m \cdot v^2 \cdot |\bar{\kappa}| = \mu_y \cdot m \cdot g$ and the mass cancels:

$$v_{limit} = \sqrt{\frac{\mu_y \cdot g}{|\bar{\kappa}|}} = \sqrt{\mu_y \cdot g \cdot R} \tag{16.5.}$$

For $R = 15$ m and $\mu_y = 0.9$:

$$v_{limit} = \sqrt{0.9 \cdot 9.81 \cdot 15} = 11.51 \text{ m/s}$$

A heavier car is not slower here, and four times the radius only doubles the speed.

**With downforce** the load on each axle grows with $v^2$, so the faster the car goes the more grip
it has. At constant speed the axle loads are (docs/13):

$$F_{z,f} = m \cdot g \cdot \frac{d_r}{W} + D_f \cdot v^2, \qquad F_{z,r} = m \cdot g \cdot \frac{d_f}{W} + D_r \cdot v^2 \tag{16.6.}$$

$$D_f = \tfrac{1}{2} \cdot \rho \cdot A \cdot \left( C_{down} \cdot \frac{W - x_{cp}}{W} - C_d \cdot \frac{h_{drag}}{W} \right) \tag{16.7.}$$

$$D_r = \tfrac{1}{2} \cdot \rho \cdot A \cdot \left( C_{down} \cdot \frac{x_{cp}}{W} + C_d \cdot \frac{h_{drag}}{W} \right) \tag{16.8.}$$

The first part is the downforce shared by where it acts, the second is the drag that pushes on the
car at height $h_{drag}$ and moves load from the front to the rear.

Each axle is checked on its own. For the front, the force it needs (16.4) equals the force it has:

$$m \cdot v^2 \cdot |\bar{\kappa}| \cdot \frac{d_r}{W} = \mu_y \cdot \left( m \cdot g \cdot \frac{d_r}{W} + D_f \cdot v^2 \right)$$

Solved for the speed, and the same for the rear:

$$v_{limit,f} = \sqrt{\frac{\mu_y \cdot m \cdot g \cdot \frac{d_r}{W}}{m \cdot |\bar{\kappa}| \cdot \frac{d_r}{W} - \mu_y \cdot D_f}} \tag{16.9.}$$

$$v_{limit,r} = \sqrt{\frac{\mu_y \cdot m \cdot g \cdot \frac{d_f}{W}}{m \cdot |\bar{\kappa}| \cdot \frac{d_f}{W} - \mu_y \cdot D_r}} \tag{16.10.}$$

$$v_{limit} = \min(v_{limit,f},\; v_{limit,r}) \tag{16.11.}$$

The axle with the lower speed gives up first. If it is the front, the car understeers at the limit.
If it is the rear, it oversteers.

**Which axle limits.** Divide the top and bottom of 16.9 by $m \cdot d_r / W$ (and 16.10 by
$m \cdot d_f / W$). Everything about the car collapses into one constant per axle:

$$q_f = \frac{D_f \cdot W}{m \cdot d_r}, \qquad q_r = \frac{D_r \cdot W}{m \cdot d_f} \tag{16.12.}$$

$$v_{limit,f} = \sqrt{\frac{\mu_y \cdot g}{|\bar{\kappa}| - \mu_y \cdot q_f}}, \qquad v_{limit,r} = \sqrt{\frac{\mu_y \cdot g}{|\bar{\kappa}| - \mu_y \cdot q_r}} \tag{16.13.}$$

$q$ is how much aero load an axle gains compared with the mass it has to turn. The two formulas
differ only in $q$, so **the axle with the smaller $q$ is the limiting one in every corner, at
every speed**. With constant $\mu$ the two limits never cross: a car is either always front limited
or always rear limited, and it is known before the run from $q_f$ and $q_r$ alone. If $q_f = q_r$
both axles give up together. Without downforce both are 0 and 16.13 is 16.5.

The axle has no limit when $|\bar{\kappa}| \le \mu_y \cdot q$, that is in corners wider than
$R = 1 / (\mu_y \cdot q)$: there the downforce grows faster than the force the corner needs.

Baseline car ($\rho = 1.225$, $A = 2.0$, $C_{down} = 1.0$, $C_d = 0.35$, $x_{cp} = 1.8$ m,
$h_{drag} = 0.4$ m, $W = 3.4$ m), $R = 15$ m:

$$D_f = 1.225 \cdot (0.471 - 0.041) = 0.526, \qquad D_r = 1.225 \cdot (0.529 + 0.041) = 0.699$$

$$v_{limit,f} = \sqrt{\frac{3600}{27.18 - 0.47}} = 11.61 \text{ m/s}, \qquad v_{limit,r} = \sqrt{\frac{3600}{27.18 - 0.63}} = 11.64 \text{ m/s}$$

$$q_f = \frac{0.526}{407.7} = 0.00129 \text{ 1/m}, \qquad q_r = \frac{0.699}{407.7} = 0.00171 \text{ 1/m}$$

So $v_{limit} = 11.61$ m/s, the front has the smaller $q$ and gives up first in every corner (a
little understeer), and the downforce is worth only 0.1 m/s in such a slow corner. A corner wider
than $1 / (0.9 \cdot 0.00129) = 861$ m radius has no limit for this car.

**With load sensitivity** ($\lambda > 0$) $\mu_y$ is no longer one number: it is lower on the
axle that carries more load per tyre, and it changes with speed as the downforce builds. Then the
two limits **can** cross, for example front limited in slow corners and rear limited in fast ones,
and there is no short formula. This is why the simulation uses the search above and not 16.13.
For $\lambda = 0$ the search gives the same number as 16.13, which is one of the tests.

---

## 16.6. The Balance Is Not Constant

Which axle gives up first, and by how much, is the balance of the car. It changes with speed.

**Constant $\mu$ ($\lambda = 0$).** At low speed there is almost no downforce, both $q$ terms in
16.13 are tiny next to $|\bar{\kappa}|$, and the two axles run out of grip together: the car is
neutral. The faster the corner, the more the downforce split matters. Baseline car:

| Corner radius | $v_{limit,f}$ | $v_{limit,r}$ | Gap |
|---|---|---|---|
| 15 m | 11.61 m/s | 11.64 m/s | 0.3 % |
| 50 m | 21.65 m/s | 21.87 m/s | 1.0 % |
| 200 m | 47.96 m/s | 50.54 m/s | 5.4 % |

So the baseline car goes from almost neutral to clear understeer as the corners get faster. The
limiting axle stays the same (the one with the smaller $q$), only the gap grows.

**With load sensitivity ($\lambda > 0$).** The load on one tyre of an axle is half the axle load.
With $q$ it is short to write:

$$F_{z,tyre,f} = \tfrac{1}{2} \cdot m \cdot \frac{d_r}{W} \cdot (g + q_f \cdot v^2), \qquad F_{z,tyre,r} = \tfrac{1}{2} \cdot m \cdot \frac{d_f}{W} \cdot (g + q_r \cdot v^2) \tag{16.14.}$$

Docs/14 equation 14.2 turns that load into the $\mu_y$ of the axle, and the limit of each axle is
where the lateral acceleration the corner needs equals what the axle can give per kilogram it
carries:

$$v^2 \cdot |\bar{\kappa}| = \mu_y(F_{z,tyre}) \cdot (g + q \cdot v^2) \tag{16.15.}$$

With $\lambda = 0$, $\mu_y$ is a constant and 16.15 solved for $v$ is 16.13. With $\lambda > 0$,
$\mu_y$ falls as $v$ rises, so $\lambda$ sits inside the same equation next to $q$. Two things
follow:

* **At low speed** ($q \cdot v^2$ small) the axle with the higher static load per tyre has the
  lower $\mu_y$ and gives up first. A nose-heavy car understeers in slow corners, whatever the
  aero does.
* **At high speed** the axle with the smaller $q$ falls behind, as before. Downforce also helps
  less than with constant $\mu$, because the extra load lowers $\mu_y$.

If these two point in opposite directions, the balance **flips** at some speed. Example: baseline
car changed to 52 % of the mass on the front, $x_{cp} = 1.0$ m (downforce further forward,
$q_f = 0.00192$, $q_r = 0.00105$ 1/m) and $\lambda = 0.2$:

| Corner radius | $v_{limit,f}$ | $v_{limit,r}$ | Gives up first |
|---|---|---|---|
| 9.125 m | 9.00 m/s | 9.04 m/s | front (understeer) |
| 15 m | 11.58 m/s | 11.62 m/s | front (understeer) |
| 30 m | 16.54 m/s | 16.53 m/s | rear (oversteer) |
| 100 m | 31.70 m/s | 31.02 m/s | rear (oversteer) |

The heavier front limits the slow corners, the lower rear $q$ limits the fast ones, and the flip
is at about 16 m/s.

16.15 is a quadratic equation in $v^2$ and could be solved directly, as long as no tyre is above
$F_{z,peak}$ (docs/14 equation 14.5). The simulation still uses the search of section 16.5,
because it works with the cap and will keep working when lateral load transfer is added. $q_f$,
$q_r$ and 16.15 are used in the tests to check it.

**What else changes the balance in a real car, and is not in the model yet:**
* The pressure points move with ride height and pitch, so $x_{cp}$ and with it $q_f$ and $q_r$
  change with speed. Here they are fixed (docs/13).
* Lateral load transfer, shared between the axles by their roll stiffness (springs, anti-roll
  bars). It is the main setup tool for the low-speed balance.

---

## 16.7. What the Car Does

Before the run, $v_{limit}$ is calculated for every track point and stored. During the run the
limit at the car is the lower one of the two points of the segment it is on. It is not
interpolated like the curvature, because a straight has no limit (infinity) and there is nothing
to interpolate with. With points 1 m apart the difference is tiny.

The driver rule in this step is the simplest one that works:

* Below the limit: full throttle, but no more than what brings the car exactly to $v_{limit}$ in
  this time step. The force is also capped by the grip that is left (section 16.4), as before.
* Above the limit: brake, but no harder than what brings the car exactly to $v_{limit}$ in this
  time step.

Written as one rule, the acceleration the driver asks for is

$$a_{x,wanted} = \frac{v_{limit} - v}{\Delta t} \tag{16.16.}$$

and the car gives as much of it as the engine, the brakes and the tyres allow. On a straight
$v_{limit}$ is infinite and this is plain full throttle.

Docs/17 keeps this rule and only changes the stored speed: it lowers it before each corner so the
car has room to brake.

**Over the grip limit.** A step is marked `over_grip_limit` in the log when a tyre needs more than
0.01 % above its lateral limit $F_{y,max}$ at its current load. In reality the car would then run
wide (docs/18). Until then the car stays on the line. It happens in two ways:

* **Too fast for the corner** ($v > v_{limit}$). Without braking points the car can arrive at a
  corner much too fast. The driver then uses the whole grip for braking in a straight line
  ($F_{y,tyre} = 0$ in the ellipse) until the speed is down to the limit. Once docs/17 is in, this
  should only happen when the data is wrong.
* **Accelerating close to the limit.** $v_{limit}$ is the limit at constant speed. While the car
  still accelerates, load moves from the front to the rear (docs/10), so the front tyres have a
  little less grip than at constant speed and can be over their limit just before the car reaches
  $v_{limit}$. On the skidpad from 5 m/s this lasts about 0.3 s. A tyre that is over its limit has
  no longitudinal grip left ($F_{x,avail} = 0$), the others still drive.

A car that holds $v_{limit}$ sits exactly on the limit and is not marked, which is what the 0.01 %
margin is for.

Track files with braking zones (no XY points) work as before.

---

## 16.8. Implementation in the Simulation Lifecycle

Before the run (`SimulationConfig::prepare`):
1. For every track point, find $v_{limit}$ by the search of section 16.5
   (`calculate_corner_speed_limit`) and store the list (`corner_speed_limits`).

Every step, on a track with XY points:
1. Curvature and $v_{limit}$ at the car, updated with the position at the end of the previous
   step (`update_track_position`, docs/15 section 15.6).
2. In `calculate_step_loads`: $a_y$ from 16.1 (`calculate_lateral_acceleration`) and the axle
   forces from 16.2 and 16.4 (`calculate_axle_lateral_forces`).
3. Corner loads from the suspension as before. `is_over_lateral_grip` sets `over_grip_limit`.
   `calculate_axle_grip` now gets the axle lateral forces instead of 0, or 0 if the car is above
   $v_{limit}$ (section 16.7).
4. Wanted acceleration from 16.16 (`calculate_wanted_acceleration`), then the net force between
   full braking and full throttle that gives it (`calculate_net_force_for_wanted_acceleration`).

The new functions are in `src/physics/cornering.rs`. The constant speed axle loads of 16.6 to 16.8
are `calculate_constant_speed_axle_loads`.

New `CarState` values: `corner_speed_limit`, `lateral_acceleration`, `over_grip_limit`. New log
columns: `corner_speed_limit_m_per_s` (`inf` where there is no limit), `lateral_acceleration_m_per_s2`,
`over_grip_limit`. The `braking` column still only shows braking zones, so on a track with XY
points look at the acceleration.

No new car, tyre or run file keys are needed.

---

## 16.9. Test Track and Tests

`data/tracks/skidpad_circle.csv`: one circle of 9.125 m radius, the centre line of a Formula
Student skidpad circle (15.25 m inner diameter, 3 m wide), 57 points. The car should speed up to
the limit and stay there:

$$v_{limit} = 9.02 \text{ m/s}, \qquad t_{lap} = \frac{2 \cdot \pi \cdot 9.125}{9.02} = 6.35 \text{ s}$$

`data/runs/skidpad_5_laps.toml` drives five laps of it with the baseline car from 5 m/s. Result:
lap 1 takes 6.60 s (speeding up), every lap after it 6.352 s at 9.021 m/s.

The oval of docs/15 now takes 13.24 s per lap instead of 3.3 s: the car is flat out on the
straights, arrives at each corner far too fast, brakes in the corner and then holds 11.61 m/s.

Tests:
* Section 16.2 example: $F_y = 5437$ N, 2718 N per axle for a 50/50 car. A 40/60 car puts 40 % on the front.
* No downforce: the search gives $\sqrt{\mu_y \cdot g \cdot R}$ (16.5), 11.51 m/s at 15 m.
* Baseline car at 15 m: 11.61 m/s (16.13), front axle limits. Moving $x_{cp}$ forward until $q_f > q_r$ makes the rear limit, in every corner.
* Straight ($\bar{\kappa} = 0$) and a very wide corner: no limit.
* Limit is the same for left and right corners.
* Section 16.6 tables: the baseline gaps at 15, 50 and 200 m, and the balance flip of the 52 % front, $x_{cp} = 1.0$ m, $\lambda = 0.2$ car between 15 m and 30 m.
* With $\lambda > 0$ the search matches 16.15 solved as a quadratic, for both axles.
* With $\lambda > 0$ the limit is lower than with $\lambda = 0$ at the same $\mu_{ref}$ when the load is above the reference load.
* Section 16.4 example: 1216 N left on the front tyre.
* Wanted acceleration (16.16), and the net force for it: exactly $m \cdot a_{x,wanted}$ between full braking and full throttle, capped at both ends.
* Skidpad circle: the speed never goes above the limit, settles at it and stays there, flying lap time 6.35 s, `over_grip_limit` not set once the speed has settled. The axle loads the suspension settles at are the ones from 16.6.
* Oval into the first corner: `over_grip_limit` is set and the car brakes in the corner, because nothing brakes before the corner yet.
* Baseline run (braking-zone track) unchanged.

---

## 16.10. Limitations

[!WARNING]
Load sensitivity is used, tyre by tyre, for everything the code already models: downforce and
pitch change the tyre loads and with them $\mu$. What is still missing is lateral load transfer,
which the code never had: the left and right tyre of an axle carry the same load and make the same
force, so the car does not roll, and the extra grip loss from loading the outside tyre (docs/14
section 14.3) is not there yet. No yaw or steering: the car is always exactly on the line and pointing along it, and
there is no slip angle. The speed limit assumes constant speed in a constant corner, so it ignores
the small drive force needed to hold the speed against drag. The car does not brake before corners
(docs/17) and does not run wide when it is too fast (docs/18).
