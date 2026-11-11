# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current version 0.9.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone (0.9.0):** XY track, curvature and laps (docs/15): a track file can now hold the `x_m,y_m` points of the line the car drives. Before the run, the distance to each point, the lap length and the smoothed curvature are calculated. During the run the car has a position, a curvature, a lap counter and lap times (crossing time interpolated inside the step), and a run can end at a time or after a number of laps (`end_condition`). The car does not slow down for corners yet, and the baseline results are unchanged
* **Previous Milestone (0.8.0):** Tyre load sensitivity and the friction ellipse (docs/14): mu falls linearly with the tyre load, the grip is calculated per tyre from the four corner loads and summed per axle, and the longitudinal grip left over from a lateral force follows the friction ellipse (lateral force is 0 until cornering arrives). The baseline tyre has no load sensitivity, so the baseline results are unchanged
* **Previous Milestone (0.7.0):** Aerodynamic downforce and pressure points (docs/13): downforce acts at a centre of pressure measured from the front axle and drag acts at its own height, both fixed for now. They feed the body heave and pitch equations, so the springs carry them to the four tyre loads and the ride heights
 
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
rate, lap, x, y and track curvature (the last four are 0 on a track without XY points), in SI units named in the header:

```
cargo run --release -- data/runs/baseline.toml --log baseline.csv --log-every 10
```

On a track with XY points the program prints the lap length and every lap time at the end. To drive three laps of
the test oval (docs/15 section 15.9):

```
cargo run --release -- data/runs/oval_3_laps.toml
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
  environments/standard.toml   air density, gravity
  runs/baseline.toml           picks the files above + time settings + initial state
  runs/oval_3_laps.toml        three laps of the oval, ends on laps
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
shift time left >= 0, distance and suspension state finite, distance >= 0 on a track with XY points) and panics with the reason if it is invalid.
It then computes everything that stays the same for the whole run, so the steps don't recalculate it:
`static_values` (`calculate_static_car_values`: CoG to axle distances, static corner loads, where the aero forces act)
and, with `fill_powertrain_tables`, the peak torque, `clutch_lock_speeds` (docs/12 equation 12.8) and
`upshift_speeds` (docs/12 section 12.6, scan in 0.01 m/s steps).
Last, it calculates the part of the starting car state that follows from the starting conditions and the config
(`calculate_initial_car_state`): active braking zone, braking flag, lap, track segment, position and curvature (docs/15), locked engine speed `revs_hz` and acceleration.
The first step recalculates them the same way, so they don't change the results. `load_run` calls `prepare()` when the
data files are loaded, so a bad file is reported before the simulation starts and the loaded config is complete.

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update (docs/12 section 12.10):** If no shift is in progress (`shift_time_left` = 0), pick the gear with `select_gear`: upshift when speed >= `upshift_speeds[k]`, downshift when speed < `upshift_speeds[k-1] - downshift_hysteresis` (also while braking), one gear per step. A gear change sets `shift_time_left = shift_time`. Then recalculate the locked engine speed `revs_hz` for the gear (`calculate_engine_revs_from_wheel_speed`, equation 12.2, with the total ratio primary × gear × final drive).
3. **Loads at the Start of the Step (`calculate_step_loads`, docs/11 section 11.8 steps 1 to 3, docs/13 section 13.5):** From the suspension state in `CarState::suspension` (heave, pitch and their velocities), compute the corner compressions and compression speeds and the corner forces with end stops and the wheel lift clamp (`calculate_corner_forces_from_state`), then the four corner tyre loads (`calculate_corner_tyre_loads`). They are summed into `AxleLoads` (front = FL + FR, rear = RL + RR) with `sum_axle_loads`. The longitudinal grip of each tyre is calculated from its own load and summed per axle into `AxleGrip` (`calculate_axle_grip`, docs/14 section 14.5). Drag and downforce come from the speed at the start of the step (`calculate_aero_loads`). All of these are calculated once and used by both a_x and the body accelerations.
4. **Net Force Evaluation (`calculate_net_force(car, car_state, axle_loads, axle_grip, aero_loads)`, step 4):** Resistive losses are rolling/breakaway friction (on `AxleLoads::total()`) + air drag.
    * **If Braking:** If car speed is 0 m/s, net force is 0 N. Otherwise net force equals `-(losses + brake force)`, where brake force is the threshold braking limit for the front brake bias (`calculate_brake_force` → `calculate_split_force_limit`).
    * **If Accelerating:** Calculate powertrain force: 0 while `shift_time_left` > 0, otherwise the full-throttle force in the gear (`calculate_powertrain_force`, docs/12 equation 12.10: clutch slip at peak torque below the clutch lock speed, torque curve through the gearing above it, 0 above the rev limit). Limit it by the tyre grip of the driven axles for the drivetrain's front share (`calculate_split_force_limit`), and subtract losses. At standstill the net force is never negative.
5. **Acceleration (`get_car_acceleration`):** Divide net force by total mass (`calculate_acceleration`) to get a_x and store it in `CarState::acceleration` (output only, it is not used for load transfer). Then count `shift_time_left` down by the time step (`count_down_shift_time`), so a 5 ms shift at 1 ms steps is exactly 5 steps without drive, including the step that starts it.
6. **Body Accelerations (step 5):** Heave and pitch accelerations from the same corner forces, this step's a_x and the aero forces (`calculate_body_accelerations`, docs/13 equations 13.2 and 13.3).
7. **Velocity Updates (step 6):** Speed (`get_car_speed`), heave velocity and pitch rate (`update_suspension_velocities`), each by acceleration × time step. If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s and acceleration to 0.0 m/s². From then on a_x is 0 and the suspension keeps integrating, so the body settles back to static.
8. **Position Updates (step 7):** Distance (`get_car_distance`), heave and pitch (`update_suspension_positions`), each by the **new** velocity × time step (semi-implicit Euler).
9. **Laps and Track Position (docs/15, only on a track with XY points):** If the distance passed a whole number of lap lengths in this step, `record_completed_laps` interpolates the crossing time inside the step (`calculate_crossing_time`, equation 15.9), stores the lap time and counts the lap. Then `update_track_position` finds the segment the car is on, searching forward from the previous one, and interpolates x, y and the curvature (`find_track_position`).
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
