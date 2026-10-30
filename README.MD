# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current version 0.5.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **In Progress (unreleased):** Data files for the car, tyres, track and environment: every parameter now lives in editable TOML / CSV files under `data/` (with SI units in every key name), and a run is started from one run file (section 2)
* **Latest Milestone (0.5.0):** Torque curve, clutch slip and optimal shifting (docs/12): the engine is a full-throttle torque curve stored as data (estimated 2026-style F1 ICE without MGU-K, 8 gears), the clutch slips below a per-gear lock speed, upshift speeds come from a table computed before the simulation, downshifts only happen when not braking, and each shift cuts the drive for `shift_time`
* **Previous Milestone (0.4.0):** Suspension pitch and heave dynamics (docs/11): the body sits on four spring-damper corners with bump/droop stops and a wheel lift clamp, so load transfer now arrives through a transient instead of instantly
 
---

## 2. Running a Simulation

A simulation is started from a run file, which picks the car, tyres, track and environment files and sets the time
settings and the starting state of the car:

```
cargo run --release -- data/runs/baseline.toml
```

Without a run file the program prints a usage line and exits with an error. A bad data file (missing or unknown key,
wrong `format_version`, bad CSV row, or a value `validate()` rejects) is reported with the file path and the key or
line, and the program exits with an error.

```
data/
  cars/baseline.toml           mass, geometry, aero, brakes, powertrain (incl. torque curve), suspension
  tyres/baseline.toml          mu_static, mu_rolling, mu_breakaway, wheel_radius_m
  tracks/straight_1000m.csv    braking zones, one per row
  environments/standard.toml   air density, gravity
  runs/baseline.toml           picks the files above + time settings + initial state
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
all suspension values finite, the time step stability limit omega_max * time_step < 2 from docs/11 section 11.8,
and for the powertrain (docs/12): a torque curve of at least 2 points with strictly increasing engine speeds and
finite torques >= 0 that covers `min_revs` to `max_revs`, `min_revs` < `max_revs`, non-empty strictly decreasing
gear ratios > 0, final drive > 0, `shift_time` >= 0, `downshift_hysteresis` >= 0, wheel radius > 0, the starting
gear within the gearbox, all values finite, and braking zones finite with end >= start) and panics with the reason if
it is invalid. `load_run` runs the same check when the data files are loaded, so a bad file is reported before the
simulation starts.
It then computes the two powertrain tables once from the torque curve, gear ratios and wheel radius:
`clutch_lock_speeds` (docs/12 equation 12.8) and `upshift_speeds` (docs/12 section 12.6, scan in 0.01 m/s steps).

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update (docs/12 section 12.10):** If no shift is in progress (`shift_time_left` = 0), pick the gear with `select_gear`: upshift when speed >= `upshift_speeds[k]`, downshift only when not braking and speed < `upshift_speeds[k-1] - downshift_hysteresis`, one gear per step. A gear change sets `shift_time_left = shift_time`. Then recalculate the locked transmission input speed `revs_hz` for the gear (`calculate_transmission_input_revs`, equation 12.2).
3. **Tyre Loads (`get_car_acceleration`, docs/11 section 11.8 steps 1 to 3):** From the suspension state in `CarState::suspension` (heave, pitch and their velocities), compute the corner compressions and compression speeds, the corner forces with end stops and the wheel lift clamp, and the four corner tyre loads (`calculate_tyre_loads`). They are summed into `AxleLoads` (front = FL + FR, rear = RL + RR) with `sum_axle_loads`.
4. **Net Force Evaluation (`calculate_net_force(car, environment, car_state, axle_loads)`, step 4):** Resistive losses are rolling/breakaway friction (on `AxleLoads::total()`) + air drag.
    * **If Braking:** If car speed is 0 m/s, net force is 0 N. Otherwise net force equals `-(losses + brake force)`, where brake force is the threshold braking limit for the front brake bias (`calculate_brake_force` → `calculate_split_force_limit`).
    * **If Accelerating:** Calculate powertrain force: 0 while `shift_time_left` > 0, otherwise the best force in the gear (`calculate_best_force`, docs/12 equation 12.10: clutch slip at peak torque below the clutch lock speed, torque curve through the gearing above it, 0 above the rev limit). Limit it by the tyre grip of the driven axles for the drivetrain's front share (`calculate_split_force_limit`), and subtract losses. At standstill the net force is never negative.
5. **Acceleration:** Divide net force by total mass (`calculate_acceleration`) to get a_x and store it in `CarState::acceleration` (output only, it is not used for load transfer). Then count `shift_time_left` down by the time step (`count_down_shift_time`), so a 5 ms shift at 1 ms steps is exactly 5 steps without drive, including the step that starts it.
6. **Body Accelerations (step 5):** Heave and pitch accelerations from the same suspension state and this step's a_x (`calculate_body_accelerations_from_state`, docs/11 equations 11.12 and 11.13).
7. **Velocity Updates (step 6):** Speed (`get_car_speed`), heave velocity and pitch rate (`update_suspension_velocities`), each by acceleration × time step. If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s and acceleration to 0.0 m/s². From then on a_x is 0 and the suspension keeps integrating, so the body settles back to static.
8. **Position Updates (step 7):** Distance (`get_car_distance`), heave and pitch (`update_suspension_positions`), each by the **new** velocity × time step (semi-implicit Euler).
9. **Time Step Advancement:** Increment total simulation runtime by `time_step`.

---

## 4. Engineering Backlog & TODOs

1. Replace placeholder values with sourced ones (in `data/cars/baseline.toml` and `data/tyres/baseline.toml`, and the matching `Default` impls): wheelbase and CoG height (`ChassisGeometry`), `mu_rolling` and `mu_breakaway` (`Tyre`), brake bias, suspension (`CornerSuspension` rates, gaps and damping, `pitch_inertia`, static ride heights), powertrain (torque curve, gear ratios, shift time, `Powertrain`).
2. Add partial brake demand (pedal input below the threshold braking limit).
3. Split unsprung mass from sprung mass (docs/11 section 11.9 treats all mass as sprung).
4. Add anti-dive / anti-squat suspension geometry.
5. Add roll for 2D (docs file, model and tests; the four-corner suspension data is already in place).
6. Add tyre vertical stiffness (in series with the suspension).
7. Add aerodynamic downforce and tyre load sensitivity.
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
