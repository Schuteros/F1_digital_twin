# F1 Digital Twin Simulator For Design Validation

Physics simulation of a Formula 1 car in progress. In reality, it is not limited to just F1 cars, but any type of car with 4 wheels. Current versions 0.1.0.

---

## 1. Current Status

* **Goal:** We are using Rust programming language code to simulate 2D motion of F1 car.
* **Current Phase:** We have made a simulation that can simulate 1D motion acceleration with simplified models
* **Latest Milestone:** We have added tests to all functions and added main.rs file with simple usage example
 
---

## 2. Current Physics Architecture

We have a 1D physics model that looks at a vehicle accelerating.

We have set initial conditions and constants. Simulation is made on cycles and integration.

Every cycle works like this:

1. Update current revs and gear
2. We are calculating force powertrain torque limited from last state
3. We are testing if the clutch is needed (slipping)
4. If clutch needed the car is torque limited otherwise we calculate force powertrain power limited
5. If we calculate force powertrain power limited we compare it with torque limited (MIN)
6. We calculate static friction of tyres that can transmit force from powertrain which limits force.
7. We calculate rolling friction tyre and add it to loses
8. We calculate air drag lose and add it to loses
9. Then we remove powertrain force limited by friction the loses to get net force
10. We calculate acceleration knowing net force and total mass
11. From acceleration, we integrate to get speed
12. From speed, we integrate to get distance
13. Then we update the current state

---

## 3. Engineering Backlog & TODOs

1. Add docs file about physics of braking
2. Add physics module track.rs
3. Add modifiers to allow for simulation to have specifically braking zone for testing of braking
4. Add physics module braking.rs
5. Write simple model for car braking
6. Test car braking module
7. Add docs file about physics of roll and pitch of a vehicle
8. Add physics module chassis.rs at physics folder
9. Write simple model for car pitch and roll
10. Add tests for car pitch and roll
11. Convert codebase to a crate library and publish it at crates.io

---

Licensing: This project is licensed under AGPLv3 for open-source and non-commercial use. For commercial licensing inquiries, contact [uldis.kurpnieks@proton.me].
