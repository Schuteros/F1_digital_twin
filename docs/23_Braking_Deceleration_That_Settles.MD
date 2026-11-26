# 23. A Braking Deceleration That Settles on Corner Entry

Docs/17 plans how hard the car can brake at every point of the line, and docs/18 checks the plan
with trial runs. On a measured track with a tight, tightening hairpin this went wrong. The car arrived
at the hairpin several km/h above its target speed, could not slow down any more because the
tyres were already used sideways, and ran wide by about 2 m. With a strict driver
(`over_limit_braking_share = 0.0`, `max_line_offset_m = 0.3`) every lap was invalid.

The physics of 17.4 was right. It already takes the friction ellipse into account: the grip used
sideways is not available for braking. What went wrong was the **solving**. The equation of 17.4 was
solved by repeating its steps, and on corner entry that repetition never settled. It jumped
between two or more values, and after the last round it returned whichever one it was on. The
planned braking deceleration was therefore close to random point by point: sometimes too high,
sometimes too low. The car could not follow the too-high parts, fell behind, and the tight part of
the hairpin did the rest.

What changes in the code, in short:

* `calculate_braking_deceleration` (driver.rs) finds the deceleration by halving the range it
  lies in (bisection), not by repeating the steps (23.3). It always settles.
* The log gets a `brake_force_N` column: the force the brakes really make in each step (23.5).

Nothing else changes. On a straight the old repetition did settle, so straight-line braking, the
oval and the skidpads give the same results as before.

---

## 23.1. Symbols

Same as docs/17, plus:
* $f(a)$ is one pass through steps 1 to 4 of 17.4: the deceleration in m/s² that the car can reach
  if the load transfer is the one of an assumed deceleration $a$,
* $a^*$ is the answer: the deceleration that gives itself back, $f(a^*) = a^*$,
* $F_{brake,used}$ is the force the brakes make in a simulation step, in N.

---

## 23.2. The Equation of 17.4 Is a Fixed Point

Braking moves load to the front axle (equation 17.4). How much load moves depends on the
deceleration, and the grip of each tyre, and so the brake force, depends on the load. So the
deceleration appears on both sides:

$$a^* = f(a^*) \tag{23.1.}$$

A value with this property is called a **fixed point** of $f$. Graphically it is where the curve
$y = f(a)$ crosses the line $y = a$. Docs/17 found it by starting at $a = 0$ and repeating:

$$a_{n+1} = f(a_n) \tag{23.2.}$$

---

## 23.3. Why Repeating Can Jump Forever, and the Fix

Repeating 23.2 only settles if $f$ changes slower than $a$ near the answer, $|f'(a^*)| < 1$. Each
round then shrinks the error by the factor $|f'|$. If $|f'(a^*)| > 1$, each round makes the error
bigger, and the values jump from one side of $a^*$ to the other.

**On a straight** $f$ is flat. The front gains as much load as the rear loses, and with a sensible
brake bias both axles stay usable. The repetition settles in a handful of rounds.

**On corner entry** $f$ can be very steep:

1. The lateral load transfer (docs/19) has already taken most of the load off the inside tyres.
2. The inside rear tyre has little load, and most of its grip is used sideways. By the friction
   ellipse (docs/14), what is left for braking is

$$F_{x,avail} = \mu F_z \sqrt{1 - \left(\frac{F_y}{\mu F_z}\right)^2}$$

   Near $F_y = \mu F_z$ this changes very fast with $F_z$. A little less load on that tyre takes
   almost all of its braking grip away.
3. With the fixed brake bias, the first axle to run out of grip sets the whole brake force (docs/09).
   When the rear is that axle, more deceleration → more load to the front → less rear load → much
   less brake force. So $f$ falls steeply, $f'(a^*) \ll -1$.

Measured on the hairpin: the rounds jumped between about 0.06 g and 1.4 g and never settled
within the 100 rounds allowed. The result was whichever value round 100 happened to give.

**The fix: bisection.** Write $g(a) = f(a) - a$. The answer is where $g$ changes sign:

* $g(0) = f(0) \ge 0$: even with no load transfer, drag, rolling resistance and the brakes slow
  the car down,
* for large $a$, $g(a) < 0$: the car can not brake harder than its grip allows, whatever $a$ was
  assumed.

So the answer lies in $[0, a_{high}]$, where $a_{high}$ is found by doubling from
$\max(f(0), 1)$ until $g(a_{high}) \le 0$. Then the range is halved again and again: the half where
$g$ still changes sign is kept. 60 halvings shrink the range to far below the 10⁻⁹ m/s² tolerance.
Bisection does not care how steep $f$ is, so it always settles. On a straight it gives the same
answer as before (the test values of 17.4 are unchanged).

The cost is about 60 evaluations of $f$ per point instead of about 10. The plan is made once,
before the run, so the run itself is not slower.

---

## 23.4. Results

With the fix, the planned deceleration changes smoothly along the hairpin entry. The car then
follows its target speed through the whole hairpin, with $v$ equal to the target to within
0.1 km/h, and stays on the line.

* The measured track (team data, private) with the strict driver of the introduction: every lap
  valid, before every lap invalid and up to 2–3 m off the line.
* The same track with the earlier, forgiving driver: about 0.9 s per lap faster than before, because
  the car no longer runs wide and scrubs speed on corner entries.
* Oval, skidpads and the straight-line baseline: unchanged, to the printed digits.

---

## 23.5. Brake Force in the Log

Until now the log's `braking` column was the only clue to the brakes. On a track with XY points it
is always `false`, because it is the braking-zone flag of the straight-line mode (docs/09), not
"brakes on". The new column `brake_force_N` is the force the brakes make in the step:

$$F_{brake,used} = \max\left(0,\; -m \cdot a_x - F_{loss}\right) \tag{23.3.}$$

with $F_{loss}$ the drag and rolling resistance of the step (`calculate_force_losses`). The car
slows down with $-m a_x$, and whatever drag and rolling resistance do not give, the brakes give.
It is 0 on the throttle and while coasting.

---

## 23.6. Tests

* `test_braking_deceleration_settles_in_corner` (driver.rs): a car with a tall CoG and a
  front-heavy brake bias, in a 10 m corner, from half the corner speed limit to the limit in 0.25 %
  steps. The deceleration must never jump by 0.5 m/s² or more from one step to the next. The old
  repetition fails it (a jump of about 0.6 m/s²), bisection passes.
* `test_brake_force_output` (simulation.rs): on the oval, $m a_x + F_{brake,used} + F_{loss} = 0$
  whenever the brakes are on, $F_{brake,used} = 0$ whenever the car accelerates, and the car does
  brake.
* All earlier tests, including the docs/17 values, pass unchanged.

---

## 23.7. Left for Later

* **Backward pass at the point's own speed.** 17.5 calculates the deceleration of a segment at the
  lower speed of its end. In a corner the start of the segment is faster and needs more lateral
  grip, so a little less braking is left. With 1 m points and a settling 17.4 the difference was
  below 0.01 g on the hairpin, so it is left as it is.
* **Trial run window (docs/18).** A trial checks the line offset up to the corner's exit point,
  where the target speed leaves the corner speed limit. On a long corner whose limit changes all the
  time this can be a short window. It was not the cause here, but a check of "car more than
  ε behind its target speed" next to the line offset would make the trials more robust.
* **Driver styles.** Real drivers use much less than the plan in braking (measured −0.8 to −0.9 g
  against about −2 g planned) and less on throttle out of slow corners. Separate grip margins for
  braking, cornering and throttle would let the sim reproduce a given driver.
