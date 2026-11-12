# 18. Braking Points From Trial Runs

docs/17 plans the braking before the run with a quasi-static model: the braking load transfer is
there at once, and the deceleration of a segment is taken at the next point's speed and the larger
curvature. The simulated car is not quasi-static. It has to pitch, and without a margin that puts
it over the grip limit for a moment at every corner (docs/17 section 17.8: about 500 steps over 3
laps of the oval). Measured, those steps are all at the corner entry, in the first metre after the
braking ends: the nose is still down from the braking, so the rear tyres carry less than in steady
state just when the corner needs all their lateral grip.

The braking point of docs/17 is already where the two curves meet: the acceleration curve, driven
forwards by the full simulation from a known state, and the braking curve, calculated backwards from
the corner. The meeting point is right. What is not right is the braking curve after it, because it
comes from the simple model.

This document starts from that meeting point and checks the braking with the simulation itself,
before the run. From the state saved at the corner before, the full car model drives the
acceleration, meets the braking curve and brakes into the corner. If it goes over the limit, the
braking point moves earlier. If it arrives with grip to spare, it moves later. The latest braking
point that works is kept, and how far it moved is printed for every corner.

Everything is still worked out before the run. During the run the driver follows the stored
target speeds exactly as in docs/17. Nothing is added to the time step.

---

## 18.1. Symbols

Same conventions as the earlier documents. New symbols:
* $c$ is the number of a corner, counted in driving order from the start/finish line,
* $s_{apex,c}$ is the distance from the start/finish line to the slowest point of corner $c$ in m,
  where its braking ends,
* $s_{shift,c}$ is how far the braking point of corner $c$ is moved in m, positive = earlier,
* $\varepsilon_{shift}$ is the resolution of the braking point search in m.

From earlier documents: $v_{target}(s)$ target speed (docs/17 equations 17.7, 17.8),
$v_{limit}(s)$ corner speed limit, $\sigma$ grip margin (docs/17 equation 17.1), $L_{lap}$ lap
length (docs/15).

---

## 18.2. Why Not Integrate Backwards

The obvious idea is to run the full simulation backwards from the corner, as docs/17 does with the
simple model. It does not work. Running time backwards turns the dampers into negative dampers:
every small motion of the body grows instead of dying out, and the result explodes within a few
steps. Friction and tyre grip have the same problem.

So the full model only runs forwards. docs/17 gives the first guess for each braking point, and
forward trial runs from a saved state correct it.

---

## 18.3. Corners and Sections

A **corner** is a run of track points on which the docs/17 target speed is the corner speed
limit, entered from a point where the car is still braking. Its **slowest point** $s_{apex,c}$ is
the first point of that run, where the braking ends. Its **exit** is the last point of the run,
where the target leaves the limit and the car accelerates. On the test oval the slowest point is
the entry of each half circle (76.0 m) and the exit is the end of it. A track on which the car never
brakes (a skidpad circle) has no corners and nothing to correct.

A **section** runs from the exit of one corner to the exit of the next: acceleration, the straight,
the braking and the whole next corner. The section ends at the exit and not at the slowest point,
because the trouble comes right after the braking, at the corner entry, and the trial run has to
see it. Each section has exactly one braking point. The last section of the lap goes across the
start/finish line.

---

## 18.4. Moving a Braking Point

From the start of section $c$ to its slowest point the target speed is the docs/17 target taken $s_{shift,c}$ further along the
track, but never above the corner speed limit:

$$v_{target,c}(s) = \min \left( v_{limit}(s),\; v_{target}(s + s_{shift,c}) \right) \tag{18.1.}$$

The braking curve keeps its shape and moves as a whole: $s_{shift,c} = 2$ m means the car starts
braking 2 m earlier and reaches the corner speed 2 m before the corner. $s_{shift,c} = -2$ m is
2 m later. $s_{shift,c} = 0$ is the docs/17 plan. The $\min$ keeps the corner itself unchanged.

Moving the curve is chosen over changing the deceleration, because the error is at the start of
the braking (the pitch), not in the deceleration later in the zone.

---

## 18.5. The Trial Run

A trial run of section $c$ with a shift $s_{shift,c}$:

1. Start from the car state saved at the exit of the previous corner. `CarState` holds
   everything the step needs (speed, gear, suspension heave, pitch and their rates, shift time), so
   a copy of it is a complete restart point.
2. Run the normal simulation step by step, with the targets of 18.1, until the car reaches the
   exit of corner $c$.
3. The trial **passes** if in no step the car was over the grip limit (`over_grip_limit`, docs/16)
   and it was never faster than the corner speed limit where it was.

The trial runs use the **planning tyre** of docs/17 (both friction coefficients times
$1 - \sigma$), not the real tyre. The braking points are then exact for a car that is $\sigma$
worse than the real one, so the real car keeps the reserve everywhere: in the corner and in the
braking zone. With $\sigma = 0$ the trial car is the real car and the braking point is exactly on
the limit.

---

## 18.6. Finding the Latest Braking Point

A later braking point is faster but needs more grip, so if a shift passes, every larger (earlier)
shift passes too. The latest braking point that passes is found by halving:

1. Try $s_{shift,c} = 0$, the docs/17 plan.
2. If it passes, try $-1$, $-2$, $-4$, ... m until one fails. If it fails, try $1, 2, 4, ...$ m
   until one passes. Now one shift fails and one passes.
3. Try the middle. Keep the passing half. Repeat until the two are less than
   $\varepsilon_{shift} = 0.01$ m apart.
4. Keep the passing shift.

About 10 to 15 trial runs per corner. If no shift up to the whole section length passes, the run
stops with an error that names the corner: the car cannot get through it from that state at all.

---

## 18.7. Around the Lap Until Nothing Changes

The corners are corrected in driving order, because each section starts where the previous one
ends:

1. Drive the docs/17 plan from the start state of the run to the exit of the first corner after
   the start/finish line, and save the state. This drive is not checked, only its end state counts.
2. Correct the next section (18.6), drive it once more with the shift that was kept, and save the
   state at its end.
3. Repeat 2 around the lap until every corner has been corrected once.
4. The first section was started from the state of the lap before, which has now changed. So go
   around again with the saved state, starting from the corrected corners.

Stop when no shift changes by more than $\varepsilon_{shift}$ in a full lap. On the oval this should
take two laps. After 10 laps without settling, the run stops with an error.

The shifts are then built into the stored target speeds with 18.1, and the run starts as usual.

The first lap of the real run starts slower than a flying lap, so it arrives at the first corner
slower than the trial car did and meets the braking curve later. That is safe: a slower car at
the same place has less to brake.

---

## 18.8. Cost

Before the run: one trial run is one section, a few seconds of simulated time. The oval has 2
corners, so 2 laps × 2 corners × about 15 trials, about 60 short runs. A real track with 15 corners
is about 450 short runs, still under a second.

During the run: nothing changes. The target speeds are one stored number per track point, read as
in docs/17.

---

## 18.9. Implementation in the Simulation Lifecycle

New run file key in the `[driver]` table of docs/17:

```toml
[driver]
grip_margin = 0.05
# true = before the run, move each braking point earlier or later with trial runs of the full
# simulation until the car gets through the corner without going over the limit (docs/18).
# false or missing = the docs/17 plan as it is.
braking_point_trials = true
```

`grip_margin` is now optional as well (0 when missing), so `braking_point_trials` can be used
alone.

New file `src/physics/braking_points.rs`:
* `find_corners`: slowest point and exit of every corner (18.3),
* `shift_braking_point`: the targets of one section with a shift (18.1),
* `run_trial`: one trial run from a saved state, passes or not (18.5, `drive_and_check`),
* `find_braking_point_shift`: the search of 18.6,
* `correct_braking_points`: the laps of 18.7, called by `SimulationConfig::prepare` after the
  docs/17 plan. It changes `target_speeds`.

`SimulationConfig` gets `braking_points`: for every corner $s_{apex,c}$ and $s_{shift,c}$. The
program prints them after the lap times:

```
Braking points moved from the docs/17 plan (positive = earlier):
Corner at    76.00 m: +0.98 m
Corner at   198.12 m: +0.98 m
```

`SimulationState` can now be copied (for the saved state) and `simulation_step` is callable from
the new file. Nothing existing is renamed.

---

## 18.10. Tests and Results

Tests:
* `find_corners`: a small made-up plan gives one corner with the right slowest point and exit; a
  circle on the limit and a track without limits give none; the oval gives two corners half a lap
  apart, the first at 76 m.
* A shift of 0 gives the docs/17 targets. A shift of 3 m moves the braking curve exactly 3 m
  earlier and leaves the corner unchanged.
* Oval with $\sigma = 0$: without trial runs the car is over the limit in more than 100 steps,
  with them in none. Both corners get the same shift, and laps 2 and 3 take the same time.
* Braking 3 cm later than found puts the car over the limit again: the search finds the boundary.
* With $\sigma = 0.05$ the shift is the same as with $\sigma = 0$ (the trial car has the
  planning tyre, so it is the same problem), and the lap is slower than docs/17.
* `braking_point_trials` reads from the run file, also without `grip_margin`.
* Baseline 60 s log identical, skidpad unchanged (no corners to correct).

Results on the test oval (baseline car, 1 ms steps):

| $\sigma$ | docs/17 lap | docs/18 lap | Shift | Steps over the limit, 3 laps |
|---|---|---|---|---|
| 0 | 16.40 s | 16.49 s | +0.98 m | 504, now 0 |
| 0.01 | 16.48 s | 16.57 s | +0.98 m | 0 |
| 0.05 | 16.75 s | 16.84 s | +0.98 m | 0 |

The braking points move about 1 m earlier, for the reason in the introduction: the body needs a
moment to come back from the braking pitch before the corner can use all the grip. That costs
about 0.05 s per corner. With $\sigma = 0$ the car now really drives on the limit and never over
it, and a margin is a free choice again instead of a repair. All trial runs together take about
0.2 s before the run.

---

## 18.11. Limitations

[!WARNING]
The braking point is found for a flying lap. The search assumes that an earlier braking point never
makes things worse, which holds for a single braking zone but would not for two corners so close
together that the car cannot accelerate between them (one section then has two braking zones). The
corner itself is not corrected: the corner speed limit is still the steady-state one of docs/16.
The plan is still made once before the run, with no learning during the run.
