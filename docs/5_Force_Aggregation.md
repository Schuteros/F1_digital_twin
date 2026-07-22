# 5. Longitudinal Force Aggregation Loop

Every simulation timestep, the aggregation module resolves the net longitudinal force acting on the vehicle chassis. 
This loop combines the tractive limits of the powertrain, tire friction transitions, and environmental resistances into 
a single acceleration vector.

---

## 5.1. The Timestep Evaluation Pipeline

The net force calculation follows a precise, sequential architecture during each state update:

### 1. Evaluate Tire Traction Limit
The maximum force the tires can physically transmit to the track surface is governed by the normal force currently acting 
on the driven wheels ($N_{\text{driven}}$):

$$F_{\text{traction\_max}} = N_{\text{driven}} \cdot \mu_{\text{static}} \tag{5.1}$$

### 2. Evaluate Powertrain Delivery & Cap
The powertrain computes the ideal force output by checking if the engine is torque-limited or power-limited 
(utilizing hardware-native IEEE 754 float division at zero speed). This ideal force is then immediately capped by the physical
tire traction limit:

$$F_{\text{propulsion}} = \min\left(\min\left(\frac{T_{\text{max}}}{R}, \frac{P}{v}\right), F_{\text{traction\_max}}\right) \tag{5.2}$$

### 3. Evaluate Parasitic Resistance Losses
The environment and tire interfaces exert forces that actively reduce the total forward force. The loop calculates 
these resistances based on the current vehicle velocity:

* **Standstill Threshold ($v = 0$):** The vehicle must overcome the transient breakaway force to initiate motion:
  $$F_{\text{resistance}} = F_{\text{breakaway}} \tag{5.3}$$
* **In-Motion ($v \neq 0$):** Dynamic rolling resistance and aerodynamic drag oppose the vehicle:
  $$F_{\text{resistance}} = F_{\text{rolling}} + F_{\text{drag}} \tag{5.4}$$

---

## 5.2. Net Force Calculation

Once the active propulsion and resistance forces are determined, the final net force ($F_{\text{net}}$) passed to the 
numerical integrator is:

$$F_{\text{net}} = F_{\text{propulsion}} - F_{\text{resistance}} \tag{5.5}$$

This net force is ultimately divided by the total vehicle mass ($m$) within the `integrators` module to derive the clean, 
instantaneous acceleration state ($a = \frac{F_{\text{net}}}{m}$).