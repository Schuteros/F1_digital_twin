# 17. Look-Ahead Braking

After docs/16 the car knows how fast each corner can be taken, but it only finds out when it is
already in the corner. It arrives too fast, goes over the grip limit and brakes in the corner.
This document makes it brake **before** the corner, as late as possible and just enough to arrive
at the corner speed.

Everything is worked out before the run starts and stored as one number per track point: the
**target speed**. During the run the driver only follows it. Three things are new:

* a **grip margin**: the driver plans with a little less grip than the tyres have, so the car
  never uses 100 % of the tyre grip and keeps a reserve,
* the **backward pass** that turns corner speed limits into braking points,
* a better way to read the stored speed between two track points.

The car does not learn from earlier laps and does not correct anything. If it is slower than the
target it accelerates, if it is faster it brakes.

---

## 17.1. Symbols

Same conventions as the earlier documents. New symbols:
* $\sigma$ is the grip margin, dimensionless: the fraction of the tyre grip the driver leaves unused when planning (0.05 = 5 %),
* $a_{brake}$ is the deceleration the car can reach at a track point in m/s², positive when slowing down,
* $F_{brake}$ is the total brake force at the tyres in Newtons (N), positive,
* $F_{roll}$ is the rolling resistance in Newtons (N), positive,
* $v_{target,i}$ is the target speed at track point $i$ in m/s,
* $v_{target}(s)$ is the target speed at the distance $s$ along the lap in m/s.

From earlier documents: $v_{limit,i}$ corner speed limit at point $i$ (docs/16), $\Delta s_i$
length of segment $i$, $\bar{\kappa}_i$ curvature (docs/15), $h$ CoG height, $W$ wheelbase,
$F_{drag}$ (docs/04), brake bias (docs/09), $F_{x,avail}$ (docs/14), $\Delta t$ time step.

---

## 17.2. The Grip Margin

A car that drives exactly on the limit has nothing left when something is a little different from
the plan: a bump, a gust, a slightly late brake. In reality that means running wide or spinning,
which costs far more time than the margin does.

So the driver **plans** with tyres that are a little worse than the real ones. In every calculation
of this document and in the corner speed limit of docs/16, both friction coefficients are
multiplied by $(1 - \sigma)$:

$$\mu_{x,plan} = (1 - \sigma) \cdot \mu_{x,ref}, \qquad \mu_{y,plan} = (1 - \sigma) \cdot \mu_{y,ref} \tag{17.1.}$$

The simulation itself still uses the real tyres. The difference is the reserve.

What it costs: corner speed goes with the square root of the grip (docs/16 equation 16.5), so a
5 % margin costs about 2.5 % of corner speed. Baseline car, 15 m corner:

$$\sigma = 0: \; v_{limit} = 11.61 \text{ m/s}, \qquad \sigma = 0.05: \; v_{limit} = 11.31 \text{ m/s}$$

$\sigma = 0$ gives the car of docs/16, exactly on the limit.

---

## 17.3. The Idea: Braking Distance

A car that slows down at a constant deceleration $a$ over a distance $s$ from $v_0$ to $v$ obeys

$$v_0^2 - v^2 = 2 \cdot a \cdot s \tag{17.2.}$$

Read backwards from the corner: if the car must have the speed $v$ at a point, and it can
decelerate at $a$, then a distance $s$ before that point it may be at most

$$v_0 = \sqrt{v^2 + 2 \cdot a \cdot s} \tag{17.3.}$$

Example: the corner needs 11.31 m/s and the car can brake at 9.4 m/s². One metre before the corner
it may do $\sqrt{11.31^2 + 2 \cdot 9.4 \cdot 1} = 12.11$ m/s, and from 20 m/s it needs

$$s = \frac{20^2 - 11.31^2}{2 \cdot 9.4} = 14.5 \text{ m}$$

That is the braking point: 14.5 m before the corner. Later would be too late, earlier wastes time.

The only difficulty is that $a$ is not a constant. A fast car has downforce and drag helping it,
and a car that is already turning has less grip left for braking. So 17.3 is applied one track
segment at a time (about 1 m), with the deceleration the car has there.

---

## 17.4. How Hard the Car Can Brake at a Point

For a speed $v$ and a curvature $\bar{\kappa}$, with the planning tyres of 17.1:

1. **Axle loads.** The constant speed loads of docs/16 (equation 16.6: weight, downforce, drag at
   its height), plus the load that braking moves to the front (docs/10):

$$F_{z,f} = F_{z,f,16.6} + \frac{m \cdot a_{brake} \cdot h}{W}, \qquad F_{z,r} = F_{z,r,16.6} - \frac{m \cdot a_{brake} \cdot h}{W} \tag{17.4.}$$

2. **Grip left for braking.** Each tyre makes its share of the lateral force the corner needs
   (docs/16 equation 16.4). The friction ellipse gives what is left, $F_{x,avail}$ (docs/14 equation
   14.8), summed per axle.
3. **Brake force.** The brakes are split by the fixed bias, and the first axle to reach its grip
   sets the total, exactly as in the simulation (docs/09): $F_{brake}$.
4. **Deceleration.** Drag and rolling resistance help:

$$a_{brake} = \frac{F_{brake} + F_{drag} + F_{roll}}{m} \tag{17.5.}$$

Step 1 needs $a_{brake}$ and step 4 gives it, so the four steps are repeated, starting from
$a_{brake} = 0$, until the number stops changing (a handful of rounds).

These are the same functions the simulation calls every step (`calculate_axle_grip`,
`calculate_brake_force`, load sensitivity included). Nothing is modelled twice.

Baseline car on a straight with $\sigma = 0.05$:

| Speed | $a_{brake}$ |
|---|---|
| 20 m/s | 9.4 m/s² (0.96 g) |
| 40 m/s | 11.6 m/s² (1.18 g) |
| 60 m/s | 15.1 m/s² (1.54 g) |

The car brakes much harder at high speed, because of downforce and drag. With these numbers it
needs 15 m to brake from 20 m/s to the 15 m corner, 73 m from 40 m/s and 149 m from 60 m/s.

In a corner the same car has less: at the corner speed limit itself the tyres are fully used
sideways and $F_{brake} = 0$. In between, the ellipse gives the mix. This is trail braking: the car
can still be slowing down while the corner tightens.

---

## 17.5. The Backward Pass

Start with the corner speed limits (with the margin):

$$v_{target,i} = v_{limit,i}$$

Then go through the track points **backwards**, against the driving direction. Each point may not
be faster than the speed from which the car can still brake down to the next point:

$$v_{target,i} = \min \left( v_{limit,i},\; \sqrt{v_{target,i+1}^2 + 2 \cdot a_{brake} \cdot \Delta s_i} \right) \tag{17.6.}$$

$a_{brake}$ is taken from section 17.4 at the speed of the next point, $v_{target,i+1}$, and at the
larger curvature of the two points. Both choices give a little less deceleration than the car
really has on that segment, so the error is on the safe side.

The track is a loop, so the point before the start/finish line depends on the points after it. The
pass therefore goes around the track twice. After the second time nothing changes any more.

What comes out, read in driving direction:
* right after a corner the target is far above the speed the car has there, so the car is at full
  throttle. On the test oval it is 39 m/s at the corner exit, where the car does 11 m/s. A track
  without any corner has no target at all (infinite),
* towards the next corner the target comes down along the braking curve. The place where the car,
  at full throttle, meets this curve is its **braking point**. It is not stored, it simply happens,
* in the corner the target is the corner speed limit,
* after the corner the target jumps back up and the car accelerates as hard as the ellipse allows.

A faster car on the same track meets the curve earlier, a slower one later. Both brake as late as
they can.

**Two curves that meet.** This is the usual picture of a lap: from each corner exit an
acceleration curve goes forwards, from each corner a braking curve goes backwards, and where they
cross is the braking point. The braking curves are calculated here, before the run. The
acceleration curve is not calculated beforehand: the simulation itself drives it, with the full
car model. The crossing is found during the run, when the car's speed reaches the target.

**No search for the tightest point.** The pass does not have to start at the point of largest
curvature. Every point brings its own limit into 17.6 and the $\min$ keeps whichever is lower, so
the tightest point of each corner ends up as the bottom of the curve by itself.

---

## 17.6. Following the Target During the Run

The driver rule stays the one of docs/16 (equation 16.16): ask for the acceleration that puts the
car on the target at the end of this time step, and take as much of it as the car can give.

Two details change.

**Between two points.** The stored numbers belong to the track points, and the car is almost
always between two of them. Two things are read between point $i$ and point $i+1$, and the target
is the lower one.

The corner speed limit. The curvature changes linearly between two points (docs/15), and the limit
goes with $1 / v^2 \propto \bar{\kappa}$ (docs/16 equation 16.5), so $1 / v^2$ is interpolated:

$$\frac{1}{v_{limit}(s)^2} = \frac{1}{v_{limit,i}^2} + \frac{s - s_i}{\Delta s_i} \cdot \left( \frac{1}{v_{limit,i+1}^2} - \frac{1}{v_{limit,i}^2} \right) \tag{17.7.}$$

A point without a limit (a straight) counts as $1 / v^2 = 0$. This replaces the "lower one of the
two points" of docs/16, and it is what the `corner_speed_limit` log column now shows.

The braking curve. At constant deceleration $v^2$ changes linearly with distance (17.2):

$$v_{brake}(s)^2 = v_{target,i}^2 + \frac{s - s_i}{\Delta s_i} \cdot \left( v_{target,i+1}^2 - v_{target,i}^2 \right) \tag{17.8.}$$

$$v_{target}(s) = \min \left( v_{limit}(s),\; v_{brake}(s) \right)$$

Both are a few multiplications and one square root per step. Nothing is searched or solved during
the run.

**Where to look.** The car wants the right speed at the place it will be after the step, not where
it is now:

$$a_{x,wanted} = \frac{v_{target}(s + v \cdot \Delta t) - v}{\Delta t} \tag{17.9.}$$

With a 1 ms step that is a few centimetres ahead. The planning happened before the run. This is
only the last look at the plan.

**No correction, no learning.** If the car is above the target it brakes, if it is below, for
whatever reason, it accelerates, in both cases with what the friction ellipse leaves after the
lateral force the corner needs. Only a car that is over the grip limit **and** above the target
gives up the line and uses all its grip for braking (the rule of docs/16 section 16.7, now tied to
the target). The plan itself is never changed.

---

## 17.7. Implementation in the Simulation Lifecycle

Before the run (`SimulationConfig::prepare`), on a track with XY points:
1. Corner speed limits as in docs/16, now with the planning tyres of 17.1 (`corner_speed_limits`).
2. Backward pass 17.6, twice around (`calculate_target_speeds`), with `calculate_braking_deceleration`
   (section 17.4) for each segment. Stored as `target_speeds`.

Both use the planning tyre (`calculate_planning_tyre`, 17.1). Everything else in the simulation
uses the real tyre.

Every step:
1. Corner speed limit where the car is (17.7, `interpolate_corner_speed_limit`).
2. Target speed at the place the car will be after the step (17.7 and 17.8,
   `calculate_target_speed_at`).
3. Wanted acceleration (17.9). The rest of the step is unchanged.

Run file, new optional table:

```toml
# How the driver plans the lap (docs/17)
[driver]
# Fraction of the tyre grip left unused when planning corner speeds and braking points,
# 0.0 up to (not including) 1.0
grip_margin = 0.05
```

Without the table the margin is 0. In Rust: `SimulationConfig.driver: Driver { grip_margin }`.

New `CarState` value and log column: `target_speed` / `target_speed_m_per_s`. The existing
`corner_speed_limit` column stays and now includes the margin.

Nothing existing is renamed. Three existing functions get one more argument: the corner speed limit
functions of docs/16 now take the tyre to use, so they can be given the planning tyre. Tracks with braking zones (no XY points) work as before.

---

## 17.8. Tests and Results

Tests:
* $\sigma = 0$: the planning tyre is the real tyre. $\sigma = 0.05$: the limit at 15 m is 11.31 m/s.
* Deceleration on a straight at 20, 40 and 60 m/s: the table of section 17.4. In a corner at the
  speed limit: only drag and rolling resistance.
* A car without aero, with the CoG on the ground and a brake bias equal to its weight split brakes
  at $\mu_x \cdot g$ plus rolling resistance, and its braking distance is 17.2 exactly.
* Backward pass on the oval: the target is never above the corner speed limit, it is the limit in
  the middle of a corner, the braking curve reaches 20 m/s about 15 m before the corner, both
  corners get the same plan, and a third time around changes nothing.
* 17.7 and 17.8: at a point the value is that point's value, halfway the braking curve is the root
  of the mean of the squares, and the corner speed limit halfway out of a 10 m/s corner is 14.14 m/s.
* Oval with $\sigma = 0.05$: the car brakes before each corner, is never above the corner speed
  limit, `over_grip_limit` is never set, and laps 2 and 3 take the same time.
* A larger margin gives a slower lap. A margin outside 0.0..1.0 is rejected.
* Skidpad (no margin) and baseline run: results unchanged.

Results on the test oval (75 m straights, 15 m corners, baseline car, 1 ms steps):

| $\sigma$ | Lap time | Steps over the grip limit, 3 laps | Most above the target |
|---|---|---|---|
| 0 | 16.40 s | about 500 | 0.054 m/s |
| 0.01 | 16.48 s | 0 | 0.045 m/s |
| 0.02 | 16.54 s | 0 | 0.036 m/s |
| 0.05 | 16.75 s | 0 | 0.022 m/s |

For comparison, docs/16 without braking before the corner: 13.24 s, but that car entered every
corner far too fast and could not have held the line.

With $\sigma = 0.05$ the car reaches 24.3 m/s on the straight and starts braking 25 m before the
corner, at up to 10.2 m/s² (1.04 g).

With $\sigma = 0$ the car goes over the limit for a moment each time the braking starts, for the
reason in section 17.9. On this car and track a margin of 1 % is already enough to prevent it. The
rest of a larger margin is reserve.

---

## 17.9. Limitations

[!WARNING]
The plan assumes the braking load transfer is there at once (17.4). The simulated car has to pitch
first, which takes about 0.1 s, so at the first moment of braking it has a little less grip than
planned. The grip margin has to cover this. The plan is made once and never changed: no learning
from lap to lap and no reaction to what the car actually does. The margin is one number for the
whole lap. The line is fixed, so the car cannot use the track width to make a corner wider. Gear
shifts, and the moment without drive during them, are not in the plan.
