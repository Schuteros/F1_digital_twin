# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current versions 0.2.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone:** We have added tests to all functions and added main.rs file with simple usage example
 
---

## 2. Current Physics Architecture

We have a 1D physics model that looks at a vehicle accelerating.

We have set initial conditions and constants. Simulation is made on cycles and integration.

Every cycle (`simulation_step`) executes in this order:

1. **Track State Update:** Resolve `active_braking_zone` via `find_active_braking_zone()` and evaluate if the car is currently in a braking region using `is_braking_zone()`.
2. **Powertrain State Update:** Recalculate input engine revs (`calculate_transmission_input_revs`) and select optimal gear (`select_best_gear`).
3. **Net Force Evaluation (`calculate_net_force(car, environment, car_state)`):** Compute static axle normal forces once (`calculate_static_axle_loads` → `AxleLoads`); the rear axle is treated as the driven axle, and `AxleLoads::total()` feeds rolling/breakaway friction and brake force.
    * **If Braking:** Sum all resistive losses (rolling friction + air drag + brake force). If car speed is 0 m/s, net force is set to 0 N; otherwise net force equals `-force_losses`.
    * **If Accelerating:** Calculate powertrain force (minimum of torque limit and power limit), limit by tyre static friction limit (`calculate_force_static_friction`), and subtract rolling friction and air drag losses.
4. **Acceleration:** Calculate vehicle acceleration by dividing net force by total mass (`calculate_acceleration`).
5. **Speed Integration:** Integrate acceleration via Euler step (`get_car_speed`). If the car drops to or below 0 m/s while braking, clamp speed to 0.0 m/s.
6. **Distance Integration:** Integrate updated speed via Euler step to advance position on the track (`get_car_distance`).
7. **Time Step Advancement:** Increment total simulation runtime by `time_step`.

---

## 3. Engineering Backlog & TODOs

1. Add docs file about physics of roll and pitch of a vehicle.
2. Add physics module `chassis.rs` in `physics/` folder.
3. Write simple model for car pitch and roll.
4. Add tests for car pitch and roll.
5. Convert codebase to a crate library and publish it at crates.io.
6. Plan out TODOs for 2D simulation integration. 

---

Licensing: This project is licensed under AGPLv3 for open-source and non-commercial use. For commercial licensing inquiries, contact [uldis.kurpnieks@proton.me].
