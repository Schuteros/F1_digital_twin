# 22. Rotating Inertia and Drivetrain Efficiency

Up to docs/21 the whole engine force goes into moving the car's mass $m$. In a real car part of it
is spent on two other things:

* **spinning up the parts that rotate**: the wheels, tyres, brake discs and drive shafts, and
  the engine's crankshaft, flywheel, clutch and gearbox shafts,
* **friction in the drivetrain**: gear teeth, chain or sprockets, bearings and the differential.

The first one makes the car behave as if it were heavier when the engine accelerates it. That
extra mass depends on the gear, and it is biggest in 1st gear. Docs/12 section 12.11 listed it as
"left for later" together with drivetrain efficiency, and this document adds both.

What changes in the code, in short:

* The car file gets four optional keys: the inertia of each front and rear wheel, the engine
  inertia, and the drivetrain efficiency. Without them the car behaves exactly as before this document.
* Before the run, `prepare()` works out an effective mass for every gear, plus one for while
  the engine is not connected to the wheels. No step does more divisions than now.
* Full-throttle acceleration is the smaller of an engine-limited value (with the effective mass)
  and a grip-limited value (almost the plain mass). Section 22.5 shows why it is exactly this.
* Threshold braking and cornering do not change (22.7).
* The upshift table compares accelerations instead of forces, so shifts come slightly earlier
  (22.8).

Nothing existing is renamed or removed.

---

## 22.1. Symbols

Same conventions as docs/12. New symbols:
* $I_e$ is the engine inertia: everything that turns with the crankshaft (crank, flywheel,
  clutch, the parts of the gearbox that turn with it), in kg·m²,
* $I_{wf}$ and $I_{wr}$ are the inertias of one front and one rear wheel about its axle: tyre,
  rim, hub, brake disc and the drive shaft or other parts turning at wheel speed, in kg·m²,
* $I_{wheels} = 2 I_{wf} + 2 I_{wr}$ is the inertia of all four wheels, in kg·m²,
* $I_{undriven}$ is the inertia of the wheels on the axles the engine does not drive, in kg·m²
  ($2 I_{wf}$ for rear wheel drive, $2 I_{wr}$ for front wheel drive, 0 for all wheel drive),
* $\eta$ is the drivetrain efficiency, dimensionless, $0 < \eta \le 1$,
* $\alpha_w$ and $\alpha_e$ are the angular accelerations of a wheel and of the engine, in rad/s²,
* $m_{eff,k}$ is the effective mass in gear $k$ with the clutch locked, in kg,
* $m_{free}$ is the effective mass while the engine is not connected to the wheels (shifting or
  clutch slipping), in kg,
* $m_{grip}$ is the effective mass when the driven tyres are at their grip limit, in kg,
* $\delta_k = m_{eff,k} / m$ is the rotating mass factor in gear $k$, dimensionless (books and
  team sheets often give this number),
* $F_{loss}$ is drag plus rolling resistance (docs/04, docs/03), in N,
* $F_{grip}$ is the largest drive force the driven axles can pass to the road (docs/14, the same
  limit as now), in N.

From docs/12: $G_k$ (total ratio in gear $k$), $r$ (wheel radius), $F_k$ (powertrain force at
the contact patch, 12.10), $v$, $a_x$.

---

## 22.2. Why a Spinning Part Acts Like Extra Mass

**One wheel.** The wheel rolls without slipping, so its spin is tied to the car speed:
$\omega_w = v / r$. When the car speeds up at $a_x$, the wheel's spin speeds up at

$$\alpha_w = \frac{a_x}{r} \tag{22.1.}$$

Newton's second law for rotation (the same law as $F = m a$, docs/11 section 11.6) says the
torque that takes is

$$T_w = I_w\,\alpha_w = \frac{I_w\,a_x}{r} \tag{22.2.}$$

That torque has to come from a force at the contact patch, at radius $r$. So the force used up on
spinning the wheel is

$$F_w = \frac{T_w}{r} = \frac{I_w}{r^2}\,a_x \tag{22.3.}$$

That looks exactly like $F = m a$ with a mass of $I_w / r^2$. **A wheel feels like an extra mass
of $I_w / r^2$.**

**The engine.** The engine turns $G_k$ times faster than the wheels (docs/07, 7.3), so its
angular acceleration is $G_k$ times bigger:

$$\alpha_e = G_k\,\frac{a_x}{r} \tag{22.4.}$$

The torque that takes at the crankshaft is $I_e \alpha_e$. On its way to the wheels the gearbox
multiplies that torque by $G_k$ again (docs/07, 7.2), so at the contact patch it uses up

$$F_e = \frac{G_k\,I_e\,\alpha_e}{r} = \frac{I_e\,G_k^2}{r^2}\,a_x \tag{22.5.}$$

$G_k$ comes in twice: once because the engine spins faster, and once because the gearbox
multiplies its torque. **The engine feels like an extra mass of $I_e G_k^2 / r^2$.** In a low
gear $G_k$ is large, so a small, light engine part matters a lot.

**All of it together.** With the clutch locked in gear $k$:

$$m_{eff,k} = m + \frac{I_{wheels} + I_e\,G_k^2}{r^2} \tag{22.6.}$$

$$\delta_k = \frac{m_{eff,k}}{m} \tag{22.7.}$$

**Worked example** with the baseline placeholders (22.10): $m$ = 815.5 kg, $r$ = 0.36 m,
$I_{wf}$ = 1.0 kg·m², $I_{wr}$ = 1.2 kg·m², $I_e$ = 0.05 kg·m², 1st gear $G_1$ = 16.29.

$$\frac{I_{wheels}}{r^2} = \frac{2 \cdot 1.0 + 2 \cdot 1.2}{0.36^2} = \frac{4.4}{0.1296} = 34.0 \text{ kg}$$

$$\frac{I_e\,G_1^2}{r^2} = \frac{0.05 \cdot 16.29^2}{0.1296} = \frac{13.3}{0.1296} = 102.4 \text{ kg}$$

$$m_{eff,1} = 815.5 + 34.0 + 102.4 = 951.9 \text{ kg}, \qquad \delta_1 = 1.167$$

In 8th gear ($G_8$ = 4.95) the engine part is only 9.4 kg, so $\delta_8$ = 1.053. The engine
inertia is 88 times smaller than the four wheels together, but in 1st gear it adds three times
as much mass.

| Gear | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| $m_{eff,k}$ (kg) | 951.8 | 905.6 | 887.0 | 876.3 | 869.6 | 865.2 | 861.6 | 858.9 |
| $\delta_k$ | 1.167 | 1.110 | 1.088 | 1.075 | 1.066 | 1.061 | 1.057 | 1.053 |

---

## 22.3. Getting the Inertias From a Table of Factors

Team sheets and books often give only $\delta_k$ for each gear, wheels and engine together. 22.6
can be split back into the two parts, because the wheel part is the same in every gear and the
engine part grows with $G_k^2$. Rearranging 22.6 and 22.7:

$$(\delta_k - 1)\,m\,r^2 = I_{wheels} + I_e\,G_k^2 \tag{22.8.}$$

The left side is a number for each gear. Plotted against $G_k^2$ it is a straight line: the slope
is $I_e$ and where it crosses zero is $I_{wheels}$. With two gears, subtract one from the other:

$$I_e = \frac{(\delta_1 - \delta_n)\,m\,r^2}{G_1^2 - G_n^2} \tag{22.9.}$$

$$I_{wheels} = (\delta_n - 1)\,m\,r^2 - I_e\,G_n^2 \tag{22.10.}$$

With more gears, a least-squares straight line through all of them is better. Sheet factors are
usually rounded to two decimals. That makes $I_e$ fairly reliable (it comes from the big
differences between gears), but $I_{wheels}$ is only good to about ±0.005 · $m r^2$. The
front/rear split of $I_{wheels}$ can not come from the factors at all; without better data, split
it equally.

**Check with the baseline:** putting $\delta_1$ = 1.167 and $\delta_8$ = 1.053 from 22.2 into
22.9 and 22.10 gives back $I_e$ = 0.05 kg·m² and $I_{wheels}$ = 4.4 kg·m².

---

## 22.4. Drivetrain Efficiency

Between the crankshaft and the contact patch, friction turns some of the power into heat. The
efficiency $\eta$ is the fraction that arrives. It scales the force of docs/12 equation 12.10:

$$F_{drive,k} = \eta\,F_k \tag{22.11.}$$

$\eta$ is one constant for the whole run. Real losses change a bit with load, speed and gear, and
are often quoted at 0.85 to 0.95 for a chain-driven car with a differential.

It applies to the clutch slip part of 12.10 too. It does not change the clutch lock speeds (12.8),
which only depend on the engine speed.

---

## 22.5. Full-Throttle Acceleration

There are two cases. The code takes the smaller of the two, and this section shows why that is
exactly right.

**Engine-limited.** The engine force at the contact patch has to do three things: accelerate the
car, spin up everything that rotates, and overcome drag and rolling resistance:

$$F_{drive,k} = m_{eff,k}\,a_x + F_{loss}$$

$$a_{engine} = \frac{F_{drive,k} - F_{loss}}{m_{eff,k}} \tag{22.12.}$$

**Grip-limited.** The driven tyres already push as hard as the road allows, $F_{grip}$. The engine
has enough torque left to spin up the engine and the driven wheels on top of that, so they no
longer slow the car. Only the undriven wheels still take their share. Their spin has to come from
their own tyres, which push slightly backwards:

$$m\,a_x = F_{grip} - \frac{I_{undriven}}{r^2}\,a_x - F_{loss}$$

$$m_{grip} = m + \frac{I_{undriven}}{r^2} \tag{22.13.}$$

$$a_{grip} = \frac{F_{grip} - F_{loss}}{m_{grip}} \tag{22.14.}$$

**Which case applies.** The engine is the limit when the driven tyres would need less than
$F_{grip}$. In the engine-limited case the driven tyres pass on $m_{grip}\,a_{engine} + F_{loss}$
(from the same force balance as 22.13). So the engine is the limit when

$$m_{grip}\,a_{engine} + F_{loss} \le F_{grip} \quad\Longleftrightarrow\quad a_{engine} \le a_{grip}$$

Which gives one rule for both cases:

$$a_x = \min\left(a_{engine},\ a_{grip}\right) \tag{22.15.}$$

This is the same "smaller of the two" the code uses now. Today it compares forces, because both
cases divide by the same $m$. With two different masses it has to compare accelerations.

**While shifting or while the clutch slips** the engine is not tied to the car speed, so its
inertia drops out:

$$m_{free} = m + \frac{I_{wheels}}{r^2} \tag{22.16.}$$

During a shift $F_{drive}$ = 0 (docs/12 section 12.8), so the car slows at $F_{loss} / m_{free}$.
That's a little gentler than now, because the spinning wheels keep pushing the car. While the
clutch slips, the engine is held at peak torque (docs/12 section 12.5), and 22.12 uses $m_{free}$
instead of $m_{eff,k}$.

**Worked example.** Baseline car in 8th gear at 80 m/s, with $\eta$ = 0.95 (22.10). Engine force
from 12.10, losses from docs/04 and docs/03 with downforce:

$$F_8 = 5003 \text{ N}, \qquad F_{drive,8} = 0.95 \cdot 5003 = 4753 \text{ N}, \qquad F_{loss} = 3219 \text{ N}$$

$$a_{engine} = \frac{4753 - 3219}{858.9} = 1.79 \text{ m/s}^2$$

Without rotating parts and with $\eta$ = 1 the same moment gives

$$\frac{5003 - 3219}{815.5} = 2.19 \text{ m/s}^2$$

so 18 % less acceleration near top speed. About a quarter of that comes from the rotating
inertia and the rest from $\eta$, because $\eta$ takes 5 % of the whole engine force while most
of it is already eaten by drag.

**In 1st gear at a launch** the baseline car is grip-limited, so 22.14 applies and only the two
front wheels count: $m_{grip}$ = 815.5 + 2.0 / 0.1296 = 830.9 kg, 1.9 % more than $m$, not the
16.7 % of $\delta_1$. **The big factors in low gears only matter where the engine runs out of
torque.**

---

## 22.6. Part Throttle and the Driver

On an XY track the driver asks for an acceleration (docs/16 section 16.7), and the code turns it
into a force with $m \cdot a_{wanted}$, kept between full braking and full throttle. That stays as
it is. The full-throttle end becomes $m \cdot a_x$ from 22.15, the force on the car body, so the
whole range is in the same units again. With part throttle the driver gives exactly the
acceleration they want, and the engine supplies whatever the rotating parts need on top.

The load transfer and pitch (docs/10, docs/11) use $m \cdot a_x$ and stay as they are.

---

## 22.7. What Does Not Change

* **Threshold braking.** The tyres brake at their grip limit, so the deceleration stays
  $(F_{brake} + F_{loss}) / m$. The brakes need more torque to also slow the spinning parts, but
  the model does not limit brake torque, so this is not visible. The braking planner (docs/17)
  and the braking point trials (docs/18) therefore stay as they are.
* **Cornering.** Rotating inertia acts only along the car, not sideways. Corner speed limits
  (docs/16) and roll (docs/19) are unchanged.
* **Constant speed.** With $a_x$ = 0 nothing has to spin up, so neither $\delta_k$ nor the gear
  matters.

---

## 22.8. Upshift Table

Docs/12 section 12.6 shifts when the next gear gives at least as much **force**. With different
effective masses per gear, what matters is **acceleration**. The scan in `prepare()` becomes:
the first speed where

$$\frac{\eta F_{k+1}(v) - F_{loss}(v)}{m_{eff,k+1}} \ge \frac{\eta F_k(v) - F_{loss}(v)}{m_{eff,k}} \tag{22.17.}$$

$F_{loss}(v)$ has to be included now: it no longer cancels, because the two sides divide by
different masses. It is computed on a straight at constant speed: drag (docs/04) plus rolling
resistance on the weight plus downforce (docs/13). The scan runs once before the run, so this
costs nothing per step.

The next gear has the smaller effective mass, so it wins a bit earlier than before. With the
baseline placeholders (the code gives the same speeds, test `test_docs_22_upshift_table`):

| Shift | by force, 12.11 (km/h) | by acceleration, 22.17 (km/h) | Engine speed at the shift, before → now (rpm) |
|---|---|---|---|
| 1 → 2 | 105.5 | 103.8 | 12,662 → 12,459 |
| 2 → 3 | 138.1 | 137.1 | 12,274 → 12,187 |
| 3 → 4 | 167.0 | 166.3 | 12,142 → 12,092 |
| 4 → 5 | 195.8 | 195.3 | 12,033 → 12,004 |
| 5 → 6 | 223.0 | 222.2 | 11,880 → 11,839 |
| 6 → 7 | 253.8 | 253.2 | 11,949 → 11,922 |
| 7 → 8 | 287.8 | 287.5 | 11,897 → 11,884 |

The downshift rule (12.14) uses the same table and needs no change.

---

## 22.9. Step Order in the Simulation

Before the simulation, in `prepare()`:
1. Check the new keys (22.10).
2. Work out $m_{eff,k}$ for every gear (22.6), $m_{free}$ (22.16) and $m_{grip}$ (22.13), and
   store them.
3. Work out the upshift table with 22.17.

Each step, at full throttle:
1. Engine force from 12.10, times $\eta$ (22.11). 0 while shifting.
2. $a_{engine}$ from 22.12, with $m_{eff,k}$, or $m_{free}$ while shifting or slipping.
3. $a_{grip}$ from 22.14.
4. $a_x$ = the smaller one (22.15). The step's full-throttle force on the body is $m \cdot a_x$.

That's two divisions per step instead of one, by numbers stored before the run.

---

## 22.10. Data

All new keys are optional. Without them the car has no rotating inertia and $\eta$ = 1, exactly
as before this document.

| Key | Table | Unit | Meaning |
|---|---|---|---|
| `front_wheel_inertia_kg_m2` | `[mass]` | kg·m² | $I_{wf}$, one front wheel, >= 0, default 0 |
| `rear_wheel_inertia_kg_m2` | `[mass]` | kg·m² | $I_{wr}$, one rear wheel, >= 0, default 0 |
| `engine_inertia_kg_m2` | `[powertrain]` | kg·m² | $I_e$, at crankshaft speed, >= 0, default 0 |
| `drivetrain_efficiency` | `[powertrain]` | | $\eta$, 0 < $\eta$ <= 1, default 1 |

The two wheel keys are per wheel, like the corner keys of the suspension. Parts that turn with a
driven wheel, such as the drive shafts, differential and sprocket, go into the rear (or front)
wheel value, split equally between the two wheels.

**Baseline placeholders** (estimates, not sourced): an F1 wheel with tyre weighs about 20 kg with
most of it near the rim, giving roughly 1.0 kg·m² at the front and 1.2 kg·m² at the rear with
drive shafts. $I_e$ = 0.05 kg·m² for crank, flywheel, clutch and gearbox input. $\eta$ = 0.95.
They are written in `data/cars/baseline.toml` as commented-out lines, so the baseline car keeps
no rotating parts and the docs/12 numbers stay valid. Remove the `#` to use them. With them the
oval lap goes from 16.743 s to 16.774 s (the oval is mostly grip-limited) and the top speed on the
baseline straight from 92.6 m/s to 91.5 m/s (mostly from $\eta$).

---

## 22.11. Left for Later

* **Gearbox parts turning at output speed.** They spin at $\omega_w\,i_f$, not with the engine,
  so their share depends on $i_f$ only. They are lumped into $I_e$ or the wheel values here.
* **Brake torque and bias.** `front_bias` is a split of tyre force. On a real car it is a split of
  brake torque. Spinning parts take a different share of the torque front and rear (the engine
  too, if the clutch stays closed), so the tyre force split moves slightly forward. This only
  matters once brake torque is modelled.
* **Engine pitch reaction.** Spinning up a crankshaft that lies across the car pushes back on
  the chassis about the pitch axis. Small, left out.
* **Engine braking.** With the clutch closed and the throttle shut, the engine's friction and
  pumping losses slow the car. Still left for later, as in docs/12.
* **Load- and speed-dependent efficiency.** $\eta$ as a map instead of one number.
