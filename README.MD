# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current version 0.11.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone (0.11.0):** Look-ahead braking (docs/17): on a track with XY points the car now brakes before a corner, as late as it can, to arrive at the corner speed. Before the run a target speed is calculated for every track point: going backwards through the points, each one may be no faster than the speed from which the car can still brake down to the next one (v² = v_next² + 2·a·Δs), with the deceleration the car has there (downforce, drag, load transfer, friction ellipse, the docs/14 tyre model). During the run the driver only follows the stored target. A new optional run file table `[driver]` with `grip_margin` makes the driver plan with a little less grip than the tyres have, so the car keeps a reserve. The plan is fixed: no learning between laps. The baseline and skidpad results are unchanged
* **Previous Milestone (0.10.0):** Steady-state cornering (docs/16): on a track with XY points the car needs the lateral force m·v²·|κ|, shared between the axles like the static weight. Each tyre's share goes into the friction ellipse, so traction and braking are weaker in a corner. Before the run a corner speed limit is calculated for every track point with the docs/14 tyre model (load sensitivity included). The driver asks for the acceleration that brings the car to the limit and gets as much of it as the car can give. The car has no lateral load transfer yet. The baseline results are unchanged
* **Earlier (0.9.0):** XY track, curvature and laps (docs/15): a track file can now hold the `x_m,y_m` points of the line the car drives. Before the run, the distance to each point, the lap length and the smoothed curvature are calculated. During the run the car has a position, a curvature, a lap counter and lap times (crossing time interpolated inside the step), and a run can end at a time or after a number of laps (`end_condition`). The car does not slow down for corners yet, and the baseline results are unchanged
* **Earlier (0.8.0):** Tyre load sensitivity and the friction ellipse (docs/14): mu falls linearly with the tyre load, the grip is calculated per tyre from the four corner loads and summed per axle, and the longitudinal grip left over from a lateral force follows the friction ellipse (lateral force is 0 until cornering arrives). The baseline tyre has no load sensitivity, so the baseline results are unchanged
* **Earlier (0.7.0):** Aerodynamic downforce and pressure points (docs/13): downforce acts at a centre of pressure measured from the front axle and drag acts at its own height, both fixed for now. They feed the body heave and pitch equations, so the springs carry them to the four tyre loads and the ride heights
 
---

## 2. Running a Simulation

A simulation is started from a run file, which picks the car, tyres, track and environment files and sets the time
settings and the starting state of the car:

```
cargo run --release -- data/runs/baseline.toml
```

To write the car state to a CSV file while it runs, add `--log <file>`, and optionally `--log-every <steps>`
to write only every Nth step (the first and last steps are always written). The columns are time, distance, speed,
acceleration, gear, revs, braking flag, shift time left, braking zone index, heave, heave velocity, pitch, pitch
rate, lap, x, y, track curvature, corner speed limit (`inf` where there is none), target speed (the planned speed where the car will be after the step, `inf` without XY points), lateral acceleration and the over grip limit flag
(lap to curvature and the lateral acceleration are 0 on a track without XY points), in SI units named in the header:

```
cargo run --release -- data/runs/baseline.toml --log baseline.csv --log-every 10
```

On a track with XY points the program prints the lap length and every lap time at the end. To drive three laps of
the test oval (docs/15 section 15.9), braking before each corner (docs/17):

```
cargo run --release -- data/runs/oval_3_laps.toml
```

Or five laps of a Formula Student skidpad circle, where the car speeds up to the corner speed limit and holds it
(docs/16 section 16.9):

```
cargo run --release -- data/runs/skidpad_5_laps.toml
```

Without a run file the program prints a usage line and exits with an error. A bad data file (missing or unknown key,
wrong `format_version`, bad CSV row, or a value `validate()` rejects) is reported with the file path and the key or
line, and the program exits with an error.

```
data/
  cars/baseline.toml           mass, geometry, aero, brakes, powertrain (incl. torque curve), suspension
  tyres/baseline.toml          mu_x_ref, mu_y_ref, reference_load_N, load_sensitivity, mu_rolling, mu_breakaway, wheel_radius_m
  tracks/straight_1000m.csv    braking zones, one per row
  tracks/oval_244m.csv         XY points of a test oval, one per row (docs/15)
  tracks/skidpad_circle.csv    XY points of one skidpad circle, 9.125 m radius (docs/16)
  environments/standard.toml   air density, gravity
  runs/baseline.toml           picks the files above + time settings + initial state
  runs/oval_3_laps.toml        three laps of the oval, ends on laps, driver with a 5 % grip margin (docs/17)
  runs/skidpad_5_laps.toml     five laps of the skidpad circle
  README.md                    what each file is, every key with its unit
```

Paths inside a file are relative to that file's folder, so runs work from any working directory. The baseline files
hold exactly the values of the Rust `Default` impls, so the baseline run gives the same results as before the data
files. To try a change, copy a file, edit the copy and point a run file at it. See [data/README.md](data/README.md)
for every key and its unit.

---

## 3. Current Physics Architecture

We have a 1D physics model that looks at a vehicle accelerating.

We have set initial conditions and constants. Simulation is made on cycles and integration.

Before the first step, `start_simulation` calls `SimulationConfig::prepare()`, which first checks the config with
`SimulationConfig::validate()` (brake bias and AWD split in 0..=1, wheelbase > 0, CoG height >= 0, time step > 0,
front + rear mass = total mass, suspension spring rates > 0, damping / gaps / end stop rates >= 0, pitch inertia > 0,
all suspension values finite, aero downforce coefficient and centre of pressure positions finite, tyre `mu_x_ref`, `mu_y_ref` and `reference_load_N` > 0 and `load_sensitivity` >= 0 (docs/14), the time step stability limit omega_max * time_step < 2 from docs/11 section 11.8,
and for the powertrain (docs/12): a torque curve of at least 2 points with strictly increasing engine speeds and
finite torques >= 0 that covers `min_revs` to `max_revs`, `min_revs` < `max_revs`, non-empty strictly decreasing
gear ratios > 0, primary ratio > 0, final drive > 0, `shift_time` >= 0, `downshift_hysteresis` >= 0, wheel radius > 0, the starting
gear within the gearbox, all values finite, braking zones finite with end >= start, the end condition (end time or max time > start time, laps >= 1 and only on a track with XY points, docs/15 section 15.8), and the starting state: speed >= 0,
shift time left >= 0, distance and suspension state finite, distance >= 0 on a track with XY points, and the driver's `grip_margin` in 0.0..1.0, docs/17) and panics with the reason if it is invalid.
It then computes everything that stays the same for the whole run, so the steps don't recalculate it:
`static_values` (`calculate_static_car_values`: CoG to axle distances, static corner loads, where the aero forces act)
and, with `fill_powertrain_tables`, the peak torque, `clutch_lock_speeds` (docs/12 equation 12.8) and
`upshift_speeds` (docs/12 section 12.6, scan in 0.01 m/s steps), and `corner_speed_limits`, the corner speed limit at
every point of an XY track (`calculate_corner_speed_limits`, docs/16 section 16.5: the highest constant speed at which
no tyre needs more lateral force than it has, found by halving the search range), and `target_speeds`, the planned
speed at every point (`calculate_target_speeds`, docs/17 section 17.5: the backward pass over the corner speed limits,
twice around the lap, with `calculate_braking_deceleration` for each segment). Both are calculated with the planning
tyre (`calculate_planning_tyre`, docs/17 equation 17.1: both friction coefficients times 1 - `grip_margin`). The
simulation itself uses the real tyre.
Last, it calculates the part of the starting car state that follows from the starting conditions and the config
(`calculate_initial_car_state`): active braking zone, braking flag, lap, track segment, position, curvature (docs/15), corner speed limit (docs/16) and target speed (docs/17), lateral acceleration, locked engine speed `revs_hz` and acceleration.
The first step recalculates them the same way, so they don't change the results. `load_run` calls `prepare()` when the
data files are loaded, so a bad file is reported before the simulation starts and the loaded config is complete.

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update (docs/12 section 12.10):** If no shift is in progress (`shift_time_left` = 0), pick the gear with `select_gear`: upshift when speed >= `upshift_speeds[k]`, downshift when speed < `upshift_speeds[k-1] - downshift_hysteresis` (also while braking), one gear per step. A gear change sets `shift_time_left = shift_time`. Then recalculate the locked engine speed `revs_hz` for the gear (`calculate_engine_revs_from_wheel_speed`, equation 12.2, with the total ratio primary × gear × final drive).
3. **Loads at the Start of the Step (`calculate_step_loads`, docs/11 section 11.8 steps 1 to 3, docs/13 section 13.5):** From the suspension state in `CarState::suspension` (heave, pitch and their velocities), compute the corner compressions and compression speeds and the corner forces with end stops and the wheel lift clamp (`calculate_corner_forces_from_state`), then the four corner tyre loads (`calculate_corner_tyre_loads`). They are summed into `AxleLoads` (front = FL + FR, rear = RL + RR) with `sum_axle_loads`. The lateral acceleration the line needs is v²·|κ| (`calculate_lateral_acceleration`, 0 without XY points) and it is shared between the axles (`calculate_axle_lateral_forces`, docs/16 equation 16.4). If a tyre needs more lateral force than it has, `over_grip_limit` is set (`is_over_lateral_grip`). The longitudinal grip of each tyre is calculated from its own load and its lateral force with the friction ellipse and summed per axle into `AxleGrip` (`calculate_axle_grip`, docs/14 section 14.5, docs/16 section 16.4). A car that is over the grip limit and above its target speed uses the whole grip for braking. Drag and downforce come from the speed at the start of the step (`calculate_aero_loads`). All of these are calculated once and used by both a_x and the body accelerations.
4. **Net Force Evaluation (`calculate_net_force(car, car_state, axle_loads, axle_grip, aero_loads)`, step 4):** Resistive losses are rolling/breakaway friction (on `AxleLoads::total()`) + air drag.
    * **If Braking:** If car speed is 0 m/s, net force is 0 N. Otherwise net force equals `-(losses + brake force)`, where brake force is the threshold braking limit for the front brake bias (`calculate_brake_force` → `calculate_split_force_limit`).
    * **If Accelerating:** Calculate powertrain force: 0 while `shift_time_left` > 0, otherwise the full-throttle force in the gear (`calculate_powertrain_force`, docs/12 equation 12.10: clutch slip at peak torque below the clutch lock speed, torque curve through the gearing above it, 0 above the rev limit). Limit it by the tyre grip of the driven axles for the drivetrain's front share (`calculate_split_force_limit`), and subtract losses. At standstill the net force is never negative.
    * **On a track with XY points (docs/16 section 16.7, docs/17 section 17.6):** there are no braking zones. The driver asks for the acceleration that brings the car to the target speed in this step (`calculate_wanted_acceleration`), and the net force is m × that, kept between the braking and the accelerating net force above (`calculate_net_force_for_wanted_acceleration`).
5. **Acceleration (`get_car_acceleration`):** Divide net force by total mass (`calculate_acceleration`) to get a_x and store it in `CarState::acceleration` (output only, it is not used for load transfer). Then count `shift_time_left` down by the time step (`count_down_shift_time`), so a 5 ms shift at 1 ms steps is exactly 5 steps without drive, including the step that starts it.
6. **Body Accelerations (step 5):** Heave and pitch accelerations from the same corner forces, this step's a_x and the aero forces (`calculate_body_accelerations`, docs/13 equations 13.2 and 13.3).
7. **Velocity Updates (step 6):** Speed (`get_car_speed`), heave velocity and pitch rate (`update_suspension_velocities`), each by acceleration × time step. If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s and acceleration to 0.0 m/s². From then on a_x is 0 and the suspension keeps integrating, so the body settles back to static.
8. **Position Updates (step 7):** Distance (`get_car_distance`), heave and pitch (`update_suspension_positions`), each by the **new** velocity × time step (semi-implicit Euler).
9. **Laps and Track Position (docs/15, only on a track with XY points):** If the distance passed a whole number of lap lengths in this step, `record_completed_laps` interpolates the crossing time inside the step (`calculate_crossing_time`, equation 15.9), stores the lap time and counts the lap. Then `update_track_position` finds the segment the car is on, searching forward from the previous one, and interpolates x, y and the curvature (`find_track_position`). The corner speed limit at the car is interpolated between the two points of that segment (`interpolate_corner_speed_limit`, linear in 1/v², which is linear in the curvature). The target speed is read at the place the car will be after the next step, distance + speed × time step (`calculate_target_speed_at`): the lower one of the corner speed limit there and the braking curve between the two stored target speeds (linear in v²). Nothing is searched or solved during the run, the plan is only read.
10. **Time Step Advancement:** Increment total simulation runtime by `time_step`.

The run ends at the end time, or after the laps of `EndCondition::Laps` (at `max_time` at the latest), or when the car has braked to a stop.

The track line is calculated once when the track file is loaded (`Track::from_points`, docs/15): segment lengths, distance to each point and lap length (equations 15.1 and 15.2), curvature from the heading change at each point (15.3 and 15.4) and the moving average over `curvature_smoothing_window_m` (15.6).

---

## 4. Engineering Backlog & TODOs

1. Replace placeholder values with sourced ones (in `data/cars/baseline.toml` and `data/tyres/baseline.toml`, and the matching `Default` impls): wheelbase and CoG height (`ChassisGeometry`), downforce coefficient and centre of pressure positions (`Aero`), `mu_rolling` and `mu_breakaway` (`Tyre`), brake bias, suspension (`CornerSuspension` rates, gaps and damping, `pitch_inertia`, static ride heights), powertrain (torque curve, gear ratios, shift time, `Powertrain`).
2. Add partial brake demand (pedal input below the threshold braking limit).
3. Split unsprung mass from sprung mass (docs/11 section 11.9 treats all mass as sprung).
4. Add anti-dive / anti-squat suspension geometry.
5. Add roll for 2D (docs file, model and tests; the four-corner suspension data is already in place).
6. Add tyre vertical stiffness (in series with the suspension).
7. Add downforce that depends on ride height and pitch (docs/13 section 13.8). Replace the placeholder tyre grip values with measured tyre data (docs/14 section 14.6).
8. Add rotating inertia / effective mass (docs/12 section 12.11).
9. Add drivetrain efficiency.
10. Add a part throttle map (torque over engine speed and throttle).
11. Add engine braking.
12. Add the MGU-K (hybrid motor, power taper and battery energy budget).
13. Add wheelspin and slip ratio (wheel rotation state and a slip-ratio tyre model).
14. Add clutch heat (docs/12 equation 12.9).
15. Convert codebase to a crate library and publish it at crates.io.
16. Plan out TODOs for 2D simulation integration. 

---

Licensing: This project is licensed under AGPLv3 for open-source and non-commercial use. For commercial licensing inquiries, contact [uldis.kurpnieks@proton.me].
