# 12. Torque Curve, Clutch Slip and Optimal Shifting

Docs/02 and docs/07 model the engine with two constants: a maximum torque and a maximum
power. The powertrain force is the smaller of the torque limit and the power limit $P/v$.
A real engine works differently. Its torque changes with engine speed, and that
dependence is described by a **torque curve**. This document replaces the two constants
with a torque curve stored as data. From that curve it then works out:

* the wheel force in any gear at any speed, with the clutch locked or slipping,
* the speeds where clutch slip gives more force than a locked clutch,
* the optimal upshift speed for every gear, computed once before the simulation starts,
* the downshift rule,
* shift time.

Section 12.9 gives approximate F1 numbers for all of this.

The driver is treated as automatic. When the tyres limit the force, the force is simply
capped at the grip limit (docs/06, docs/10). The model does not track how a real driver
would get there with throttle and clutch.

Not included yet: rotating inertia (effective mass), drivetrain losses, part throttle,
engine braking, the MGU-K (hybrid motor) and wheelspin. They are listed at the end.

---

## 12.1. Symbols

Every symbol has exactly one meaning in this document.

Engine:
* $n_e$ is engine speed in Hertz (Hz), revolutions per second, as `revs_hz` in the code (rpm $= 60\,n_e$),
* $n_{min}$ is the lowest engine speed before stalling in Hertz (Hz), `min_revs`,
* $n_{max}$ is the rev limit in Hertz (Hz), `max_revs`,
* $T_e(n_e)$ is engine torque at full throttle, read from the torque curve, in Newton meters (N·m),
* $T_{pk}$ is the highest torque on the curve (peak torque) in Newton meters (N·m),
* $n_{pk}$ is the engine speed where peak torque occurs in Hertz (Hz),
* $P(n_e)$ is engine power in Watts (W).

Gearbox and wheel:
* $i_p$ is the primary ratio between the engine and the gearbox input, dimensionless
  (1 when the engine drives the gearbox directly),
* $i_k$ is the gear ratio of gear $k$, dimensionless,
* $i_f$ is the final drive ratio between the gearbox output and the wheels, dimensionless,
* $G_k = i_p\, i_k\, i_f$ is the total ratio from the engine to the wheel in gear $k$, dimensionless,
* $\rho_k = G_{k+1}/G_k$ is the step between neighbouring gears, dimensionless, always below 1,
* $r$ is wheel radius in meters (m),
* $n_k$ is the engine speed that gear $k$ would force with the clutch locked, in Hertz (Hz).

Car and forces:
* $v$ is car speed in meters per second (m/s),
* $a_x$ is longitudinal acceleration in meters per second squared (m/s²),
* $F_k(v)$ is the powertrain force at the tyre contact patch in gear $k$ at speed $v$, in Newtons (N),
* $v_{up,k}$ is the optimal upshift speed from gear $k$ to gear $k+1$ in meters per second (m/s),
* $v_{lock,k}$ is the speed above which the clutch stays locked in gear $k$, in meters per second (m/s),
* $\Delta v$ is the downshift hysteresis in meters per second (m/s),
* $t_{shift}$ is the shift time in seconds (s).

---

## 12.2. The Torque Curve as Data

The curve is a list of points $(n_e, T_e)$ measured at full throttle, sorted by engine
speed. To find the torque at an engine speed $n_e$ that lies between two points, call the
point just below it $(n_a, T_a)$ and the point just above it $(n_b, T_b)$. The torque
is then found by linear interpolation along the straight line between them:

$$T_e(n_e) = T_a + (T_b - T_a)\,\frac{n_e - n_a}{n_b - n_a}, \qquad n_a \le n_e \le n_b \tag{12.1.}$$

The fraction $(n_e - n_a)/(n_b - n_a)$ is how far along the gap $n_e$ is: 0 at the lower
point and 1 at the upper point.

Outside the curve:
* below the first point, use the first point's torque (the clutch slips there anyway, see 12.5),
* above $n_{max}$, the torque is zero, because the rev limiter cuts the engine.

The curve must have at least two points, sorted by engine speed, and must cover
$n_{min}$ to $n_{max}$. The simulation checks this before it starts, in the same place
`front_torque_split` is checked now.

Keeping the curve as data means a different engine is a different list of numbers, with
no code change. That matters for the moddable game idea.

---

## 12.3. Wheel Force With the Clutch Locked

With the clutch locked, the engine speed is tied to the car speed by the gearing (docs/07, 7.3):

$$n_k = \frac{v\,G_k}{2\pi r} \tag{12.2.}$$

Torque passes through the gearbox multiplied by $G_k$ (docs/07, 7.2) and becomes a force at
the contact patch:

$$F_k(v) = \frac{T_e(n_k)\,G_k}{r} \tag{12.3.}$$

That is the whole engine model. Power is not an input any more. The `power` and
`max_torque` fields are replaced by the curve.

---

## 12.4. What Power Still Tells Us

Power is torque times angular speed:

$$P(n_e) = 2\pi\, n_e\, T_e(n_e) \tag{12.4.}$$

Rearranging 12.2 gives $G_k/r = 2\pi n_k / v$. Putting that into 12.3:

$$F_k(v) = \frac{T_e(n_k)\,2\pi n_k}{v} = \frac{P(n_k)}{v} \tag{12.5.}$$

This is the key result. **At a given car speed, a locked gear gives a force equal to the
engine power in that gear divided by the speed.** So, at a given speed, the gear that pushes
hardest is the one that puts the engine at the highest power, not the highest torque.
Comparing torques across gears is misleading because $G_k$ differs: a lower gear can sit at
lower engine torque and still give more wheel force.

12.5 also gives a ceiling that no gearbox can beat:

$$F_k(v) \le \frac{P_{max}}{v} \tag{12.6.}$$

An ideal gearbox with infinitely many ratios (a CVT) would hold the engine at peak power and
follow this curve exactly. The gap between the real $F_k(v)$ curves and $P_{max}/v$ shows how
much a gear set loses. That is the tool for judging gear ratios on the concept car. The
simulation does not need 12.6, but it is worth plotting.

---

## 12.5. When Clutch Slip Beats a Locked Clutch

A slipping clutch passes torque from the faster side to the slower side. For the
engine to drive the car through a slipping clutch, the engine must spin **faster** than
the gearbox input, $n_e > n_k$. If the engine speed holds steady, the clutch passes on
exactly the engine torque, so:

$$F_{slip,k} = \frac{T_e(n_e)\,G_k}{r}, \qquad n_e > n_k \tag{12.7.}$$

The force does not depend on car speed. The engine speed is free, as long as it stays
above $n_k$. The best choice is peak torque, $n_e = n_{pk}$, which gives
$F_{slip,k} = T_{pk} G_k / r$. That choice is only possible while $n_{pk} > n_k$.

So the rule is:
* **Locked engine speed below peak-torque speed ($n_k < n_{pk}$):** slip the clutch and hold
  the engine at $n_{pk}$. You get peak torque, which is more than the locked clutch would give.
* **Locked engine speed at or above $n_{pk}$:** slipping can only raise the engine speed past
  $n_k$, towards lower torque, so a locked clutch is best.

The clutch locks in gear $k$ when $n_k$ reaches $n_{pk}$ (or $n_{min}$, if that is higher):

$$v_{lock,k} = \frac{2\pi r\,\max(n_{pk},\,n_{min})}{G_k} \tag{12.8.}$$

This is a second precomputed table, one speed per gear. It covers the launch, and any
other moment the car is slow for its gear. Below $v_{lock,k}$, the gearbox side spins
slower than the engine, and the difference is lost as heat in the clutch:

$$P_{clutch} = 2\pi\,(n_{pk} - n_k)\,T_{pk} \tag{12.9.}$$

The model does not track clutch heat yet, but 12.9 is the number to watch if it ever does.

Combining 12.3 and 12.7, the best force in gear $k$ at speed $v$ is:

$$F_k^{best}(v) = \begin{cases} T_{pk}\,G_k / r & v < v_{lock,k} \quad \text{(slip)} \\ T_e(n_k)\,G_k / r & v_{lock,k} \le v,\ n_k \le n_{max} \quad \text{(locked)} \\ 0 & n_k > n_{max} \quad \text{(limiter)} \end{cases} \tag{12.10.}$$

As before, the result is then capped at the tyre grip limit. With F1 numbers, first-gear
force is well above grip, so a launch is grip-limited and the slip details do not change
the acceleration. They do decide when the clutch locks.

---

## 12.6. Optimal Upshift

Assume a shift takes $t_{shift}$ (12.8), but ignore it when choosing the shift point. The
best upshift from gear $k$ is then the speed at which gear $k+1$ starts giving more force:

$$F_{k+1}^{best}(v_{up,k}) = F_k^{best}(v_{up,k}) \tag{12.11.}$$

When both gears are locked, 12.5 gives the same $1/v$ on both sides, so this means equal
power before and after the shift. After the shift, the engine speed drops by the gear step:

$$P(n_k) = P(\rho_k\, n_k) \tag{12.12.}$$

In words: **shift when the engine speed you land on gives as much power as you have now.**
Peak power lies between the two speeds, so the shift happens past peak power.

If the two curves never cross before the rev limit, gear $k$ is better all the way to the
limiter, and the shift happens there:

$$v_{up,k} = \frac{2\pi r\, n_{max}}{G_k} \tag{12.13.}$$

This happens when the gear step is large. Landing at $\rho_k n_{max}$ then drops the
engine too far below peak power.

### Computing the table before the simulation

$v_{up,k}$ depends only on the curve, the gear ratios and the wheel radius. So it is
computed once, before the simulation starts, and stored as a table `upshift_speeds` with
one entry per gear except the top gear. For each gear $k$:

1. Start at $v_{lock,k}$.
2. Step the speed up in small steps (e.g. 0.01 m/s) up to the gear $k$ limiter speed (12.13).
3. The first speed where $F_{k+1}^{best}(v) \ge F_k^{best}(v)$ is $v_{up,k}$.
4. If no such speed is found, $v_{up,k}$ is the limiter speed (12.13).

Scanning is slower than solving 12.12 exactly, but it runs once, is easy to read, and works
for any curve shape.

With the F1 numbers from 12.9, every shift lands near the start of the flat-power region
(about 10,500 rpm):

| Shift | Step $\rho_k$ | $v_{up,k}$ (km/h) | Engine before → after (rpm) |
|---|---|---|---|
| 1 → 2 | 0.741 | 105.5 | 12,658 → 9,376 |
| 2 → 3 | 0.818 | 138.1 | 12,274 → 10,043 |
| 3 → 4 | 0.846 | 166.9 | 12,137 → 10,270 |
| 4 → 5 | 0.867 | 195.6 | 12,035 → 10,430 |
| 5 → 6 | 0.882 | 223.1 | 11,897 → 10,497 |
| 6 → 7 | 0.879 | 253.6 | 11,932 → 10,492 |
| 7 → 8 | 0.879 | 288.5 | 11,939 → 10,492 |

All shifts come before the 13,000 rpm limiter. This matches what F1 onboards show (shifts
around 11,500 to 12,500 rpm). The first gear step is the biggest, so the 1 → 2 shift
comes latest.

Clutch lock speeds, from 12.8 with $n_{pk}$ = 8000 rpm: 66.7 km/h in 1st and 90.0 km/h in
2nd. The car reaches 2nd above 105.5 km/h, so in practice the clutch only slips in 1st.

---

## 12.7. Downshift

Downshifts are considered **all the time, also while braking**, so the car leaves a braking
zone in the right gear. Under braking the gear has no effect on the forces yet, because engine
braking is not modelled, but the car is ready to drive away. If the engine speed drops below
$n_{min}$, the clutch opens, as it does now.

Gear $k$ drops to $k-1$ when

$$v < v_{up,k-1} - \Delta v \tag{12.14.}$$

This uses the same table as the upshift, so the car is always in the gear with the most
force (12.11). It cannot over-rev, because $v_{up,k-1}$ is never above the limiter speed
of gear $k-1$. $\Delta v$ is a small hysteresis (e.g. 0.5 m/s), so the car does not
switch between two gears if the speed wobbles around $v_{up,k-1}$.

The table is used on purpose, instead of "downshift when the lower gear lands just under the
limiter". With early upshifts, that limiter rule would downshift straight after every upshift
and switch gears back and forth.

Under braking the car downshifts one gear at a time, each taking $t_{shift}$, as a sequential
gearbox does. Braking from top speed takes seconds, while 7 downshifts take 35 ms, so the
car keeps up with the table.

`shift_up_margin` is no longer needed. `shift_down_margin` becomes $\Delta v$, now in m/s
instead of Hz.

---

## 12.8. Shift Time

During a shift, no torque reaches the wheels for $t_{shift}$. F1 seamless-shift gearboxes
get this down to a few milliseconds. We use **$t_{shift}$ = 5 ms** as an average, stored
as data.

With the current time step of 1 ms, a shift lasts 5 steps. The car state gets a timer
`shift_time_left`:

* when a shift starts, the gear changes and `shift_time_left` $= t_{shift}$,
* while `shift_time_left` $> 0$, the powertrain force is zero and the timer counts down by the time step,
* no new shift starts until the timer reaches zero.

Each shift costs a small speed loss, $\Delta v_{shift} \approx a_x\, t_{shift}$. At 2 g
that is about 0.1 m/s, which the car carries for the rest of the straight. The simulation
will show the real lap time cost. The shift points do not change.

---

## 12.9. Approximate F1 Data (2026-style)

These are estimates for a reference car, not team data. They replace the current
defaults (800 kW, 800 N·m, 5 gears), which are not realistic.

**ICE torque curve.** The 2026 rules give about 400 kW of ICE power, set by a fuel energy
flow limit of 3000 MJ/h. In the 2014 to 2025 rules, fuel flow was capped above 10,500 rpm
and scaled with rpm below that. Assuming 2026 has the same shape, power rises up to about
10,500 rpm and then stays flat at the cap. Torque is roughly flat in the mid range and falls
as $1/n_e$ on the flat-power part. The low end is reduced for turbo lag.

| rpm | $n_e$ (Hz) | $T_e$ (N·m) | $P$ (kW) |
|---|---|---|---|
| 4,000 | 66.67 | 300 | 126 |
| 6,000 | 100.00 | 360 | 226 |
| 8,000 | 133.33 | 375 | 314 |
| 10,500 | 175.00 | 364 | 400 |
| 12,000 | 200.00 | 318 | 400 |
| 13,000 | 216.67 | 250 | 340 |

* $n_{min}$ = 4,000 rpm (66.67 Hz), $n_{max}$ = 13,000 rpm (216.67 Hz). The rules allow
  15,000 rpm, but the fuel-flow cap makes power fall past about 12,000 rpm, so teams do not
  use that range.
* $T_{pk}$ = 375 N·m at $n_{pk}$ = 8,000 rpm.

**Gearing.** 8 forward gears. One set of ratios is fixed for the whole season, so it is a
compromise across tracks. The totals below come from typical top-of-gear speeds at
12,000 rpm (about 100, 135, 165, 195, 225, 255, 290 and 330 km/h) with $r$ = 0.36 m.
Only $G_k$ matters to the physics, so the split into $i_p$ = 1.0, $i_k$ and $i_f$ = 3.0 is
arbitrary. A car with a primary reduction and a chain or sprocket final drive (for example an
FS car with $i_p$ = 1.925 and sprockets 42/12, so $i_f$ = 3.5) writes them separately.

| Gear | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| $G_k$ | 16.29 | 12.06 | 9.87 | 8.35 | 7.24 | 6.39 | 5.62 | 4.94 |
| $i_k$ ($i_f$ = 3.0) | 5.43 | 4.02 | 3.29 | 2.78 | 2.41 | 2.13 | 1.87 | 1.65 |

**Shift time.** $t_{shift}$ = 5 ms. **Downshift hysteresis.** $\Delta v$ = 0.5 m/s.

Sources: 2026 ICE figures from
[PMW Magazine on the 2026 PU rules](https://pmw-magazine.com/news/race-series-news/fia-signs-off-2026-f1-pu-rules-with-mgu-h-dropped-and-tight-constraints-on-ice-design.html).
The curve shape and gear speeds are estimates (fuel-flow rule shape, typical onboard speeds).
Forum gearing analyses from onboard videos, e.g.
[f1technical: 2023 gearing](https://www.f1technical.net/forum/viewtopic.php?p=1133899),
can refine them later.

---

## 12.10. Step Order in the Simulation

Before the simulation:
1. Check the torque curve (12.2).
2. Compute `clutch_lock_speeds` (12.8) and `upshift_speeds` (12.6).

Each step:
1. If `shift_time_left` > 0: count it down, powertrain force is zero, skip to step 5.
2. Pick the gear: upshift if $v \ge v_{up,k}$, downshift if 12.14 holds (also while braking).
   A gear change starts the shift timer (12.8).
3. Compute $n_k$ for that gear (12.2).
4. Force from 12.10 (slip below $v_{lock,k}$, locked above).
5. Cap the force at the tyre grip limit (unchanged).

---

## 12.11. Left for Later

* **Rotating inertia.** Effective mass $m + (I_{wheel} + I_{engine} G_k^2)/r^2$ depends on the
  gear. The shift test then compares accelerations $F_k / m_{eff,k}$ instead of forces, which
  moves upshifts slightly earlier.
* **Drivetrain efficiency.** A factor $\eta$ in 12.3, stored as data.
* **Part throttle.** A torque map over engine speed and throttle instead of one curve.
* **Engine braking.** Negative torque at closed throttle. This makes downshift timing under
  braking matter.
* **MGU-K.** The 2026 hybrid motor adds up to 350 kW, about as much as the ICE. Its force is
  $P_K(v)/v$ and does not depend on the gear, so it does not move the shift points. It needs
  a power taper with speed and a battery energy budget per lap.
* **Wheelspin.** Needs a wheel rotation state and a slip-ratio tyre model. The real launch
  strategy (launch rpm, clutch modulation) becomes meaningful then.
* **Clutch heat.** 12.9 tells you the heat input while slipping.
