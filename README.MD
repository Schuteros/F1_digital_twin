# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current version 0.3.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone (0.3.0):** Longitudinal load transfer (docs/10), brake bias with threshold braking (docs/09), selectable drivetrain (FWD / RWD / AWD split) and config validation
 
---

## 2. Current Physics Architecture

We have a 1D physics model that looks at a vehicle accelerating.

We have set initial conditions and constants. Simulation is made on cycles and integration.

Before the first step, `start_simulation` checks the config with `SimulationConfig::validate()`
(brake bias and AWD split in 0..=1, wheelbase > 0, CoG height >= 0, time step > 0, front + rear mass = total mass)
and panics with the reason if it is invalid.

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update:** Recalculate input engine revs (`calculate_transmission_input_revs`) and select optimal gear (`select_best_gear`).
3. **Axle Loads (`get_car_acceleration`):** Compute axle normal forces with longitudinal load transfer (`calculate_axle_loads`) from the **previous step's** acceleration stored in `CarState::acceleration` (lagged acceleration, docs/10 section 10.6). Front + rear always equals m·g.
4. **Net Force Evaluation (`calculate_net_force(car, environment, car_state, axle_loads)`):** Resistive losses are rolling/breakaway friction (on `AxleLoads::total()`) + air drag.
    * **If Braking:** If car speed is 0 m/s, net force is 0 N. Otherwise net force equals `-(losses + brake force)`, where brake force is the threshold braking limit for the front brake bias (`calculate_brake_force` → `calculate_split_force_limit`).
    * **If Accelerating:** Calculate powertrain force (minimum of torque limit and power limit), limit it by the tyre grip of the driven axles for the drivetrain's front share (`calculate_split_force_limit`), and subtract losses. At standstill the net force is never negative.
5. **Acceleration:** Divide net force by total mass (`calculate_acceleration`) and store the result in `CarState::acceleration` for the next step.
6. **Speed Integration:** Integrate the stored acceleration via Euler step (`get_car_speed`). If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s and acceleration to 0.0 m/s².
7. **Distance Integration:** Integrate updated speed via Euler step to advance position on the track (`get_car_distance`).
8. **Time Step Advancement:** Increment total simulation runtime by `time_step`.

---

## 3. Engineering Backlog & TODOs

1. Replace placeholder values with sourced ones: wheelbase and CoG height (`ChassisGeometry`), `mu_rolling` and `mu_breakaway` (`Tyre`), brake bias.
2. Add partial brake demand (pedal input below the threshold braking limit).
3. Add docs file about physics of roll and pitch of a vehicle.
4. Add physics module `chassis.rs` in `physics/` folder.
5. Write simple model for car pitch and roll (pitch dynamics replace the steady-state load transfer, docs/10 section 10.8).
6. Add tests for car pitch and roll.
7. Add aerodynamic downforce and tyre load sensitivity.
8. Convert codebase to a crate library and publish it at crates.io.
9. Plan out TODOs for 2D simulation integration. 

---

Licensing: This project is licensed under AGPLv3 for open-source and non-commercial use. For commercial licensing inquiries, contact [uldis.kurpnieks@proton.me].
