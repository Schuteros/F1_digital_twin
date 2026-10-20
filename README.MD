# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current version 0.4.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone (0.4.0):** Suspension pitch and heave dynamics (docs/11): the body sits on four spring-damper corners with bump/droop stops and a wheel lift clamp, so load transfer now arrives through a transient instead of instantly
* **Previous Milestone (0.3.0):** Longitudinal load transfer (docs/10), brake bias with threshold braking (docs/09), selectable drivetrain (FWD / RWD / AWD split) and config validation
 
---

## 2. Current Physics Architecture

We have a 1D physics model that looks at a vehicle accelerating.

We have set initial conditions and constants. Simulation is made on cycles and integration.

Before the first step, `start_simulation` checks the config with `SimulationConfig::validate()`
(brake bias and AWD split in 0..=1, wheelbase > 0, CoG height >= 0, time step > 0, front + rear mass = total mass,
suspension spring rates > 0, damping / gaps / end stop rates >= 0, pitch inertia > 0, all suspension values finite,
and the time step stability limit omega_max * time_step < 2 from docs/11 section 11.8)
and panics with the reason if it is invalid.

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update:** Recalculate input engine revs (`calculate_transmission_input_revs`) and select optimal gear (`select_best_gear`).
3. **Tyre Loads (`get_car_acceleration`, docs/11 section 11.8 steps 1 to 3):** From the suspension state in `CarState::suspension` (heave, pitch and their velocities), compute the corner compressions and compression speeds, the corner forces with end stops and the wheel lift clamp, and the four corner tyre loads (`calculate_tyre_loads`). They are summed into `AxleLoads` (front = FL + FR, rear = RL + RR) with `sum_axle_loads`.
4. **Net Force Evaluation (`calculate_net_force(car, environment, car_state, axle_loads)`, step 4):** Resistive losses are rolling/breakaway friction (on `AxleLoads::total()`) + air drag.
    * **If Braking:** If car speed is 0 m/s, net force is 0 N. Otherwise net force equals `-(losses + brake force)`, where brake force is the threshold braking limit for the front brake bias (`calculate_brake_force` → `calculate_split_force_limit`).
    * **If Accelerating:** Calculate powertrain force (minimum of torque limit and power limit), limit it by the tyre grip of the driven axles for the drivetrain's front share (`calculate_split_force_limit`), and subtract losses. At standstill the net force is never negative.
5. **Acceleration:** Divide net force by total mass (`calculate_acceleration`) to get a_x and store it in `CarState::acceleration` (output only, it is not used for load transfer).
6. **Body Accelerations (step 5):** Heave and pitch accelerations from the same suspension state and this step's a_x (`calculate_body_accelerations_from_state`, docs/11 equations 11.12 and 11.13).
7. **Velocity Updates (step 6):** Speed (`get_car_speed`), heave velocity and pitch rate (`update_suspension_velocities`), each by acceleration × time step. If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s and acceleration to 0.0 m/s². From then on a_x is 0 and the suspension keeps integrating, so the body settles back to static.
8. **Position Updates (step 7):** Distance (`get_car_distance`), heave and pitch (`update_suspension_positions`), each by the **new** velocity × time step (semi-implicit Euler).
9. **Time Step Advancement:** Increment total simulation runtime by `time_step`.

---

## 3. Engineering Backlog & TODOs

1. Replace placeholder values with sourced ones: wheelbase and CoG height (`ChassisGeometry`), `mu_rolling` and `mu_breakaway` (`Tyre`), brake bias, suspension (`CornerSuspension` rates, gaps and damping, `pitch_inertia`, static ride heights).
2. Add partial brake demand (pedal input below the threshold braking limit).
3. Split unsprung mass from sprung mass (docs/11 section 11.9 treats all mass as sprung).
4. Add anti-dive / anti-squat suspension geometry.
5. Add roll for 2D (docs file, model and tests; the four-corner suspension data is already in place).
6. Add tyre vertical stiffness (in series with the suspension).
7. Add aerodynamic downforce and tyre load sensitivity.
8. Convert codebase to a crate library and publish it at crates.io.
9. Plan out TODOs for 2D simulation integration. 

---

Licensing: This project is licensed under AGPLv3 for open-source and non-commercial use. For commercial licensing inquiries, contact [uldis.kurpnieks@proton.me].
