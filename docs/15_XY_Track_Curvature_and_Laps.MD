# 15. XY Track: Distance, Curvature and Laps

Until now the track file only listed braking zones (start and end in metres along a straight line).
This document replaces it with a real track: a list of $x, y$ points that the car follows. From the
points we calculate, **once, before the run starts**:

* the distance along the line $s$ to every point, and the lap length $L_{lap}$,
* the curvature $\kappa = 1/R$ at every point, smoothed so that measurement noise does not create
  fake corners.

During the run the car still moves along one coordinate, the distance $s$ (docs/01). The points only
tell us where the car is, how sharp the road is there, and when a lap is finished.

This step does **not** make the car slow down for corners. The car drives exactly as before (full
throttle, braking zones), so lap times on a twisty track are not physical yet. The corner speed
limit comes in docs/16 and the braking before corners in docs/17.

---

## 15.1. Symbols

Same conventions as the earlier documents. New symbols:
* $x_i, y_i$ are the coordinates of track point $i$ in metres (m), $i = 0 \ldots N-1$,
* $N$ is the number of points,
* $\Delta s_i$ is the length of segment $i$, from point $i$ to point $i+1$, in metres (m),
* $s_i$ is the distance along the line from the start/finish line (point 0) to point $i$ in metres (m),
* $L_{lap}$ is the lap length in metres (m),
* $\psi_i$ is the heading of segment $i$ in radians (rad), measured from the $x$ axis, positive anticlockwise,
* $\kappa_i$ is the curvature at point $i$ in 1/m, positive for a left turn, $R = 1/|\kappa|$ is the radius in metres (m),
* $\bar{\kappa}_i$ is the smoothed curvature at point $i$ in 1/m,
* $w$ is the smoothing window length in metres (m),
* $\delta x_i, \delta y_i$ are the position errors of point $i$ in metres (m): how far it is from where it should be, in $x$ and in $y$,
* $d$ is the total distance the car has driven, measured from the start/finish line, in metres (m), the existing `CarState.distance`,
* $k$ is the number of completed laps.

---

## 15.2. The Track File

`data/tracks/<track>.csv`, one point per row, in driving order:

```
x_m,y_m
0.0,0.0
1.0,0.0
...
```

* The points describe the line the car drives. For now this is the centre line. A racing line can be
  given the same way later.
* The track is a **closed loop**: after the last point the car drives straight back to point 0. Do
  not repeat point 0 at the end.
* Point 0 is the start/finish line.
* Two points in a row must not be at the same place ($\Delta s_i > 0$), and at least 3 points are needed.
* The track is flat. Height is not used.

The old format (`start_m,end_m`, braking zones) is still read, chosen by the header row, until
docs/17 replaces braking zones with braking before corners. Then it is removed.

---

## 15.3. Distance Along the Line

Each segment is a straight line between two points:

$$\Delta s_i = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2} \tag{15.1.}$$

with point $N$ meaning point 0 (the closing segment). The distance to each point and the lap length
are the running sums:

$$s_0 = 0, \qquad s_{i+1} = s_i + \Delta s_i, \qquad L_{lap} = \sum_{i=0}^{N-1} \Delta s_i \tag{15.2.}$$

The straight segments cut every curve a little short. For points 1 m apart on a circle of radius
15 m the error is $1 - \frac{\sin(\pi/N)}{\pi/N} \approx 0.02$ %, small enough to ignore.

---

## 15.4. Heading and Curvature

The heading of each segment:

$$\psi_i = \operatorname{atan2}(y_{i+1} - y_i,\; x_{i+1} - x_i) \tag{15.3.}$$

Curvature is how fast the heading turns per metre driven, $\kappa = d\psi/ds$. At point $i$ the
heading turns from $\psi_{i-1}$ to $\psi_i$ over half of each neighbouring segment:

$$\kappa_i = \frac{\operatorname{wrap}(\psi_i - \psi_{i-1})}{\tfrac{1}{2}(\Delta s_{i-1} + \Delta s_i)} \tag{15.4.}$$

$\operatorname{wrap}$ brings the angle into $(-\pi, \pi]$, so a turn across the $\pm\pi$ heading
does not look like a full spin. On the closed loop, point 0 uses segment $N-1$ as its previous one.

Check on a circle of radius $R$ with $N$ points: the heading turns $2\pi/N$ at every point, and the
segment length is $2\pi R/N$ (to 0.02 %), so $\kappa = 1/R$.

Sign: a left turn has $\kappa > 0$, a right turn $\kappa < 0$, a straight $\kappa = 0$. The sign
is not needed for the corner speed (docs/16 uses $|\kappa|$), but it tells which side is the outside
when lateral load transfer comes in.

---

## 15.5. Noise and Smoothing

Real track points (GPS, a drawing, a cone survey) are never exact. Take a straight along the $x$
axis with spacing $\Delta s$ and move point $i$ sideways by $\delta y_i$ (an error $\delta x_i$ along the
straight only changes the spacing, not the heading): the segment before it turns by $\delta y_i/\Delta s$ and
the segment after it turns back by the same amount, so at that point

$$\kappa \approx \frac{2 \cdot \delta y_i}{\Delta s^2} \tag{15.5.}$$

With $\Delta s = 1$ m and $\delta y_i = 1$ cm this is $\kappa = 0.02$ 1/m, a **fake corner of 50 m
radius**. That is a normal FS corner, and in docs/16 the car would slow down for it.

The fix is a moving average over a window $w$ centred on each point:

$$\bar{\kappa}_i = \text{average of } \kappa_j \text{ for all points with } |s_j - s_i| \le w/2 \tag{15.6.}$$

(on the closed loop the window wraps around the start/finish line). An average of curvature is the
heading change across the window divided by its length. One bad point turns the heading one way and
then straight back, so inside the window it cancels out, and only the error at the window edges is
left. For random sideways errors $\delta y_i$ of about the same size on every point this is roughly

$$\bar{\kappa}_{error} \approx \frac{2 \cdot \delta y_i}{\Delta s \cdot w} \tag{15.7.}$$

With $\Delta s = 1$ m, $\delta y_i = 1$ cm and $w = 5$ m: 0.004 1/m, a fake radius of 250 m, which a
car takes flat out.

**The cost.** The average also blurs real corners: a corner that starts suddenly now builds up over
$w$, and a corner shorter than $w$ looks wider than it is. The tightest FS corner (hairpin, about
9 m radius) is about 28 m long, so $w = 5$ m is fine. Points for the test oval below are calculated
exactly, so it uses $w = 0$ (no smoothing).

The window is set per run (section 15.8), because it depends on how good the track data is, not on
the car.

---

## 15.6. Where the Car Is

The car keeps one distance $d$, the total distance driven from the start/finish line, as today
(`initial_state.distance_m` gives where it starts). Its position on the current lap is

$$s = d - k \cdot L_{lap} \tag{15.8.}$$

To find $x, y$ and $\bar{\kappa}$ at $s$, find the segment with $s_i \le s < s_{i+1}$ and interpolate
linearly between its two points. The search starts at the segment found in the previous step and
moves forward, the same way `find_active_braking_zone` works, so it does not scan the whole track
every step.

---

## 15.7. Laps and Lap Times

Lap $k$ is finished when $d$ passes $k \cdot L_{lap}$. The crossing happens somewhere inside a time step,
between the old distance $d_{old}$ at time $t$ and the new distance $d_{new}$ at $t + \Delta t$.
Assuming constant speed inside the step:

$$t_{cross} = t + \Delta t \cdot \frac{k \cdot L_{lap} - d_{old}}{d_{new} - d_{old}} \tag{15.9.}$$

The lap time is the time between two crossings:

$$t_{lap,k} = t_{cross,k} - t_{cross,k-1} \tag{15.10.}$$

The first lap starts at the start time. Without the interpolation, lap times would be rounded to the
time step (1 ms here). The interpolation makes the error much smaller than that.

* **Standing start:** `distance_m = 0`, `speed_m_per_s` small. Lap 1 includes the launch, laps 2+
  are flying laps (as in FS endurance).
* **Starting somewhere else:** `distance_m = 100` gives a lap 1 that is only $L_{lap} - 100$ m long. It is
  marked as a partial lap in the output.

The lap length and the lap times are printed at the end of the run and kept in the simulation state. The CSV log
(`--log`) gets four new columns: `lap`, `x_m`, `y_m`, `curvature_1_per_m`.

---

## 15.8. When the Run Ends

Today the run file has `end_time_s`. With a lap counter you usually want "stop after 3 laps"
instead. A run ends on exactly one of two conditions:

```toml
# stop at a fixed time
end_condition = { time_s = 30.0 }

# or stop after a number of completed laps, with a time limit in case the car never gets there
end_condition = { laps = 3, max_time_s = 600.0 }
```

In Rust this is one enum:

```rust
pub enum EndCondition {
    /// Stop when the simulation time reaches end_time (s)
    Time { end_time: f64 },
    /// Stop when `laps` laps are completed, or at max_time (s), whichever comes first
    Laps { laps: u32, max_time: f64 },
}
```

The `max_time_s` limit is there because a car that stops (empty braking zone setup, wrong data)
would never finish a lap and the run would go on forever. Reaching it is reported as a warning.
`laps` needs an XY track. With a braking-zone track it is a validation error.

`end_condition` replaces the old run key `end_time_s`, and `SimulationConfig.end_condition` replaces
`SimulationConfig.end_time`. `start_time_s` stays as it is.

New optional run file key: `curvature_smoothing_window_m` ($w$, >= 0, 0 or missing = no smoothing).
New `Track` fields, all calculated when the file is loaded (`Track::from_points`): `points`
($x_i, y_i$), `distances` ($s_i$), `curvatures` ($\bar{\kappa}_i$), `lap_length` ($L_{lap}$).

In the simulation step, after the distance update: `record_completed_laps` (15.9 and 15.10), then
`update_track_position` (15.8 and section 15.6). New `CarState` values: `lap` ($k$),
`track_segment`, `x`, `y`, `curvature`. The lap times are in `SimulationState.lap_times`.

---

## 15.9. Test Track

`data/tracks/oval_244m.csv`: two 75 m straights joined by two half circles of radius 15 m, points
about 1 m apart, calculated exactly (no noise):

$$L_{lap} = 2 \cdot 75 + 2 \cdot \pi \cdot 15 = 244.248 \text{ m}$$

The straight segments cut the two half circles a little short (section 15.3), so the simulation
gets $L_{lap} = 244.230$ m, 0.007 % less.

`data/runs/oval_3_laps.toml` drives three laps of it with the baseline car.

Straights have $\kappa = 0$ and the half circles $\kappa = +1/15 = 0.0667$ 1/m (anticlockwise, left
turns). Because the corners are known, the oval also checks docs/16 later: the corner speed limit
can be calculated by hand.

---

## 15.10. Tests

* Square 10 m x 10 m (4 points): $L_{lap} = 40$ m, $s = 0, 10, 20, 30$.
* Circle of radius 15 m with 100 points: every $\kappa_i = 1/15$ to 0.1 %, $L_{lap} = 2\pi \cdot 15$ to 0.1 %.
  The same circle driven clockwise gives $\kappa = -1/15$.
* Heading wrap: a track that turns across $\psi = \pm\pi$ gives no curvature spike.
* Noise: one point moved by 1 cm on a straight with 1 m spacing gives $\kappa \approx 0.02$ without
  smoothing (15.5), and almost 0 with $w = 5$ m.
* Oval: $L_{lap} = 244.248$ m to 0.02 m, curvature 0 on the straights and $1/15$ in the middle of the corners.
* Position: at $s = s_i$ the car is at point $i$; halfway along a segment it is halfway between its
  points; past $L_{lap}$ it is back on lap 2 near point 0.
* Lap time: a car at a constant 10 m/s on the oval has every lap time $= 24.4248$ s, to much better
  than the 1 ms time step.
* End condition: `laps = 2` stops right after the second crossing; `time_s` behaves as `end_time_s` did.
* Validation: fewer than 3 points, two equal points in a row, `laps` with a braking-zone track,
  negative window, are all errors.
* Baseline run (braking-zone track, `end_condition = { time_s = 100000.0 }`) gives the same results as before.

---

## 15.11. What Comes Next

* **docs/16 Steady-state cornering:** the lateral force needed is $F_y = m \cdot v^2 \cdot |\bar{\kappa}|$.
  With the tyre grip (docs/14) and downforce (docs/13) this gives the highest speed for every point.
  That speed profile is calculated before the run, like the curvature.
* **docs/17 Look-ahead braking:** braking points are also calculated before the run, from
  $v^2 - v_0^2 = 2 \cdot a \cdot s$, braking as late as possible (just in time) so the car arrives at
  the corner exactly at its limit speed. The car does not learn from earlier laps yet.
* **docs/18 Too fast or too slow in a corner.**

---

## 15.12. Limitations

[!WARNING]
The track is flat (no height, banking or camber) and has no width: the car drives exactly on the
line in the file. The line is the centre line unless a racing line is given. Curvature comes only
from the points and is blurred by the smoothing window. The car does not yet react to curvature at
all: it does not slow down for corners and has no lateral force, so lap times on a track with
corners are too fast until docs/16 and docs/17.
