# 20. Running Wide and Line Offset

Up to docs/19 the car's position is one number, the distance $s$ along the line. It can not leave
the line. When a corner needs more lateral grip than the tyres have, the step is only marked
`over_grip_limit` (docs/16 section 16.7) and the car keeps following the curve anyway.

The braking point trials of docs/18 then fail on any step over the limit, and two things stop the
run with an error before it starts:

* "the car cannot get through the corner ... even braking N m earlier" (docs/18 section 18.6),
* "the braking points did not settle in 10 laps" (docs/18 section 18.7).

The first one is easy to hit. On a test track with an R 7 m chicane it happens even with
`grip_margin = 0.1`. The car holds exactly the corner speed limit, 7.89 m/s, and small roll and
pitch load swings put it over the limit by a fraction of a percent, on and off, in the middle of
the corner. Braking earlier can not fix something that happens at constant speed.

This document gives the car a sideways position. When grip is short it runs wide, and when grip is
back the driver steers it back to the line. A run-file margin says how far from the line it may
go. A lap where the car goes further is marked invalid, and the run goes on. Nothing stops the run
any more because a corner is too hard.

---

## 20.1. Symbols

Same conventions as the earlier documents. New symbols:
* $n$ is the offset of the car from the line in m, positive = left of the line,
* $\psi_e$ is the heading error in rad: the angle between the car's direction of travel and the
  line, positive = pointing left of the line,
* $\kappa_{car}$ is the curvature of the path the car actually drives in 1/m, positive = turning
  left,
* $\kappa_{want}$ is the path curvature the driver asks for in 1/m,
* $\Delta\kappa$ is the curvature the car is short of, $\kappa_{want} - \kappa_{car}$, in 1/m,
* $L_{back}$ is the return length in m: roughly how far the driver looks ahead to get back to the
  line,
* $n_{max}$ is the allowed offset in m (run key `max_line_offset_m`),
* $\beta$ is the share of the grip the driver uses for braking when over the limit (run key
  `over_limit_braking_share`), 0 to 1.

From earlier documents: $s$ distance along the line, $v$ speed, $\kappa$ line curvature (docs/15),
$a_y$ lateral acceleration (docs/16), $F_{y,max}$ lateral tyre limit (docs/14).

---

## 20.2. How Far a Curve Moves Away From a Straight Line

Curvature is $\kappa = 1/R$. An arc that starts along a straight line moves away from it sideways
by

$$y \approx \frac{s^2}{2R} = \frac{1}{2}\kappa s^2 \tag{20.1.}$$

after $s$ metres (small angles). Example: after 5 m an R 7 m arc is $25/(2 \cdot 7) = 1.79$ m from
the straight.

Now take two arcs that start at the same point in the same direction: the line, and the path the
car drives. The gap between them is the offset:

$$n \approx \frac{1}{2}(\kappa - \kappa_{car}) s^2 = \frac{1}{2}\Delta\kappa\, s^2 \tag{20.2.}$$

**Example.** Line R 7 m, $\kappa = 0.1429$ 1/m. At 7.89 m/s the tyres give only 8.0 m/s² instead
of the 8.9 m/s² the line needs (grip 0.9x of the plan). The tightest circle the car can drive is
$R = v^2/a_y = 62.3/8.0 = 7.78$ m, so $\kappa_{car} = 0.1285$ 1/m and $\Delta\kappa = 0.0144$ 1/m.

| $s$ (m) | line moved (m) | car moved (m) | $n$ (m) |
|---|---|---|---|
| 5 | 1.79 | 1.61 | 0.18 |
| 10 | 7.14 | 6.43 | 0.72 |

Equation 20.2 is only for a constant shortfall from a start on the line. The simulation uses the
exact equations of 20.3, so the speed, the shortfall and the direction can all change.

---

## 20.3. Position Relative to the Line

The car's position is the point on the line closest to it ($s$) plus the sideways offset $n$. Its
direction is the line's direction plus $\psi_e$. Per time step:

$$\dot{s} = \frac{v\cos\psi_e}{1 - \kappa n} \tag{20.3.}$$

$$\dot{n} = v\sin\psi_e \tag{20.4.}$$

$$\dot{\psi}_e = v\,\kappa_{car} - \kappa\,\dot{s} \tag{20.5.}$$

What they say:

* **20.3:** only the part of the speed along the line moves the car along it. The $1 - \kappa n$
  is the lane effect: in a left turn ($\kappa > 0$) a car on the outside ($n < 0$) is on a bigger
  circle and moves along the line more slowly. Example: R 7 m, $n = -0.5$ m:
  $1 - 0.1429 \cdot (-0.5) = 1.071$, so 7 % slower progress. Running wide costs lap time on its
  own, with no extra rule.
* **20.4:** the car moves sideways when it points away from the line.
* **20.5:** the car turns at $v\,\kappa_{car}$ and the line turns at $\kappa\,\dot{s}$. The
  difference is how fast the heading error grows.

On the line ($n = 0$, $\psi_e = 0$, $\kappa_{car} = \kappa$) these give $\dot{s} = v$,
$\dot{n} = 0$, $\dot{\psi}_e = 0$: exactly the model up to docs/19.

$n$ and $\psi_e$ are new states in `CarState`, integrated with the same scheme as distance and
speed (semi-implicit: $\psi_e$ first, then $n$ with the new $\psi_e$). The run starts on the line
($n = 0$, $\psi_e = 0$).

The offset is measured from the closest point of the line, which only works while the car still
moves along it. $\psi_e$ is therefore kept within $\pm 60°$. A car that would turn further away is
metres off the line, so its lap is invalid anyway (20.7), but the numbers stay finite and the run
goes on.

---

## 20.4. What the Driver Asks For

The driver wants to follow the line and, if off it, get back smoothly. A path parallel to the line
at offset $n$ has the curvature $\kappa/(1 - \kappa n)$ (R 7 m line, 0.5 m outside: R 7.5 m).
Using $dn/ds \approx \psi_e$ and $d^2n/ds^2 \approx \kappa_{car} - \kappa$ (small angles), a smooth
return with no overshoot over a distance of a few $L_{back}$ is

$$\kappa_{want} = \frac{\kappa}{1 - \kappa n} - \frac{2\,\psi_e}{L_{back}} - \frac{n}{L_{back}^2} \tag{20.6.}$$

the parallel path, plus a turn back towards the line, minus some of the turn if the car already
points back towards it. $L_{back} = 5$ m, a constant in the code for now.

**Example.** On a straight, $n = -0.5$ m (right of the line), $\psi_e = 0$:
$\kappa_{want} = 0.5/25 = 0.02$ 1/m, a 50 m radius back to the left. At 8 m/s that needs
$a_y = 64 \cdot 0.02 = 1.28$ m/s², which costs little grip. The offset is about halved after
1.7 $L_{back}$ = 8 m and nearly gone after 4 $L_{back}$ = 20 m.

**Easing off to get back.** A car on the corner speed limit has no grip to spare, so a turn back
that is tighter than the line would put it over the limit and it would never get back. While the
turn back is tighter than the line (same direction, $\lvert\kappa_{want}\rvert > \lvert\kappa\rvert$), the
driver lowers the target speed of docs/17 by

$$f_{back} = \frac{\kappa}{\kappa_{want}} \tag{20.6a.}$$

The turn back then needs $f_{back}^2 \kappa_{want} v^2 = f_{back}\, a_{y,line}$: a little less
than the line at the target speed, so there is grip to spare. Example: 2 % tighter, $f_{back} =
0.98$, 2 % slower and 2 % less lateral acceleration. As the car gets back, $f_{back}$ goes back to 1.
On a straight the turn back always fits in the spare grip, so nothing changes there.

The lateral force this needs is docs/16 equation 16.1 with $\kappa_{want}$ in place of $\kappa$:

$$F_y = m\,v^2\,\kappa_{want} \tag{20.7.}$$

shared by the axles as in docs/16 section 16.3.

---

## 20.5. What the Car Gets

Each axle can give at most the sum of its tyres' $F_{y,max}$ at their loads (docs/14, docs/19).
The car has no yaw model yet (docs/16), so the axle that runs out first limits the whole car:

$$F_{y,car} = \min\left(\lvert F_y\rvert,\ \frac{F_{y,max,front}}{d_r/W},\ \frac{F_{y,max,rear}}{d_f/W}\right) \tag{20.8.}$$

with the sign of $F_y$, and

$$\kappa_{car} = \frac{F_{y,car}}{m\,v^2} \tag{20.9.}$$

If $\lvert F_y\rvert$ fits, $\kappa_{car} = \kappa_{want}$ and the car goes where the driver wants.
If not, the step is `over_grip_limit` (same flag and meaning as docs/16) and the car runs wide by
20.5.

---

## 20.6. The Driver Over the Limit

Up to docs/19, a car that was over the limit and too fast used the whole grip for braking in a
straight line (docs/16 section 16.7). With a real position that sends it straight off the track.
Turning only is not best either, because the car then never slows down. A little braking costs
little cornering, because of the friction ellipse (docs/14): braking with a share $\beta$ of the
grip leaves

$$\frac{F_{y,max}(\beta)}{F_{y,max}} = \sqrt{1 - \beta^2} \tag{20.10.}$$

of the lateral grip. $\beta = 0.3$ leaves $\sqrt{1 - 0.09} = 0.954$, so 30 % of the braking for
less than 5 % of the cornering.

A quick test of this (R 7 m corner, 8.3 m/s = 5 % too fast, 8.0 m/s² grip, no drag):

| What the driver does | $n$ after 5 m | $n$ after 10 m |
|---|---|---|
| Brake only | straight off the track | |
| Turn only | 0.33 m | 1.23 m |
| $\beta = 0.2$ | 0.23 m | 0.45 m |
| $\beta = 0.4$ | 0.21 m | 0.37 m |

**The rule.** While the step is `over_grip_limit`:

* no throttle,
* brake with the share $\beta$ of the longitudinal grip,
* turn with the lateral grip that is left, $\sqrt{1 - \beta^2}\, F_{y,max}$ per axle in 20.8.

As soon as the wanted lateral force fits again, the driver goes back to the docs/16 and docs/17
rule (throttle or brake towards the target speed), and the spare grip turns the car back to the
line by 20.6. Steering back uses lateral grip, so less is left for the throttle: getting back costs
time too.

$\beta$ is the run key `over_limit_braking_share`, default 0.3.

---

## 20.7. The Margin and Invalid Laps

$n_{max}$ is the run key `max_line_offset_m`, default 0.5 m. It stands in for the track width until
roadmap step 3 adds a real one.

* A lap where $\lvert n\rvert > n_{max}$ in any step is **invalid**. The run goes on, the car is
  not stopped or moved (there are no walls in the model), and the lap time is printed with
  "invalid" and the largest offset of that lap.
* The log gets three columns: `line_offset_m`, `heading_error_rad`, `outside_margin`. The column
  `lateral_acceleration_m_per_s2` is now what the tyres give, not what the line needs.

---

## 20.8. Changes to the Braking Point Trials (docs/18)

The trial now checks where the car is instead of the grip flag. A trial **passes** if the car never
gets more than $\varepsilon_{line} = 0.01$ m further from the line than it was at the start of the
section, and does not stop. The plan still aims at the line: the margin is a reserve for the real
run (real grip, wind, wrong data), not track space for the plan to use. A step a hair over the limit
no longer fails a trial by itself, it only moves the car by micrometres to millimetres.

The two errors become warnings, and the run starts anyway:

* **No shift passes** up to the whole section: keep the shift with the smallest largest offset,
  and warn with the corner and that offset. If that offset is above $n_{max}$, say so: the laps
  will be invalid.
* **Not settled** in 10 laps: keep the shifts of the last lap and warn.

Both are printed after the lap times, starting with "Warning:".

---

## 20.9. Run File

```toml
[driver]
grip_margin = 0.05
braking_point_trials = true
# Furthest the car may be from the line. A lap that goes further is invalid, the run goes on.
max_line_offset_m = 0.5
# Over the grip limit the driver brakes with this share of the grip and turns with the rest
# (docs/20 section 20.6). 0.0 = turn only, 1.0 = brake only.
over_limit_braking_share = 0.3
```

Both keys are optional with these defaults. `max_line_offset_m` must be > 0,
`over_limit_braking_share` must be 0.0 to 1.0. No existing names change.

---

## 20.10. Tests and Results

* **Kinematics** (`line_offset.rs`): on the line the car stays on it; a constant shortfall matches
  equation 20.2 within 1 % for the first 2 m; the lane effect at $n = -0.5$ m in an R 7 m turn is
  $\dot{s} = v/1.071$; from 0.5 m off a straight the car gets back with no overshoot, below 5 cm
  after 20 m.
* **Skidpad** (R 9.125 m): speeding up close to the limit the front runs about 1 mm wide, the
  driver eases off by 20.6a and the car ends on the line at the limit. Flying laps 6.3521 s, as
  before.
* **Oval run file** (`grip_margin = 0.05`, trials): braking points 0.09 m later than the docs/17
  plan (docs/18 had 0.98 m earlier), the car stays within 1 cm of the line, flying laps 16.743 s
  (docs/18: 16.844 s). Faster, because a step a hair over the limit no longer fails a trial.
* **The same with `grip_margin = 0.0`:** braking points again 0.09 m later than the plan (docs/18:
  0.98 m earlier), flying laps 16.426 s (docs/18: 16.493 s).
* **Oval, grip 0.9x of the plan** (the planning tyre is the full tyre, the real one has 10 % less):
  the car runs about 1.1 m wide in each R 15 m corner, brakes with the share $\beta$, gets back on
  the straights, and every lap is invalid with the default 0.5 m margin.
* **Far too fast** (no braking plan, about twice the corner speed): the car leaves the line by
  metres, the numbers stay finite, the lap finishes and is invalid.
* **Chicane** (R 7 m, test track built in the tests): no error, all laps valid, a few cm off the
  line at most. With `grip_margin = 0.1` the braking points keep moving by a few tenths of a metre
  from lap to lap, so the trials end with the "did not settle" warning.
* **Impossible corner:** trials with half the lateral grip: a warning per corner instead of an
  error, saying the laps will be invalid.
* **Config:** both keys optional with the defaults, out-of-range values rejected.

208 tests pass.

---

## 20.11. Limitations

* No yaw, sideslip or steering angle yet. The weaker axle limits the whole car (20.8), so the model
  shows understeer-like running wide but never a spin. That comes with transient handling.
* $L_{back}$ is a constant, not a key.
* The trial check is relative to where the section starts, so on a track like the chicane the
  braking points can keep moving a little from lap to lap ("did not settle" warning).
* No walls or run-off: past the margin the car is free, the lap is only marked.
* The real tyre still has the same grip as the planned one apart from `grip_margin`. Grip that
  differs during the run (the 0.9x / 1.1x idea) needs a grip factor in the environment or the tyre,
  a later step.
