# 7.Gear Ratios and Powertrain Dynamics

---

## 7.1. Why Engines Need Gearboxes

An engine cannot spin infinitely fast. It operates within a strict rotational speed range 
bounded by two mechanical limits:

* Idle Speed: The minimum rotational speed where the engine can sustain combustion without stalling.
* Redline (RPM Limit): The maximum safe operating limit before structural damage or valve float occurs.

If a vehicle had a direct, fixed connection between the engine and the wheels, you would 
face an impossible trade-off:

* A ratio sized for starting from a stop would hit the engine redline at low speeds (e.g., 40 km/h).
* A ratio sized for highway cruising would stall the vehicle when attempting to launch on an incline.

Gearboxes solve this by shifting the engine's operating RPM into its optimal torque and 
power band relative to current road speed.

---

## 7.2. Torque Multiplication and Speed Trade-off

A gear set acts as a mechanical leverage system. Assuming negligible friction losses, 
total mechanical power remains conserved:

$$\text{Power} = \text{Torque} \times \text{Rotational Speed} \tag{7.1.}$$

To increase torque at the driven wheels, you must proportionally sacrifice rotational speed,
and vice versa.

### Key Equations

When engine torque ($T_{\text{engine}}$) passes through a transmission gear ($i_{\text{gear}}$) 
and a final drive ratio ($i_{\text{final}}$): 

Wheel Torque:

$$T_{\text{wheel}} = T_{\text{engine}} \times i_{\text{gear}} \times i_{\text{final}} \tag{7.2.}$$

Wheel Rotational Speed:

$$\text{RPM}_{\text{wheel}} = \frac{\text{RPM}_{\text{engine}}}{i_{\text{gear}} \times i_{\text{final}}} \tag{7.3.}$$

### Practical Example

* First Gear (High Ratio, e.g., 3.5 : 1): Multiplies engine torque by 3.5x at the wheels 
for high initial acceleration, but reduces wheel RPM to $1/3.5$ of engine speed.
* Overdrive Gear (Low Ratio, e.g., 0.8 : 1): Reduces engine torque delivered to the 
wheels, but allows high wheel speeds at low engine RPM to maximize top speed and 
fuel efficiency.

---

## 7.3. Maximizing Tractive Force

Tractive force at the road surface determines overall longitudinal acceleration:

$$F_{\text{drive}} = \frac{T_{\text{wheel}}}{r_{\text{wheel}}}\tag{7.4.}$$

As vehicle speed increases, opposing forces like aerodynamic drag 
($F_{\text{drag}} \propto v^2$) grow significantly.

* High gear ratios provide the raw tractive force needed to overcome initial inertia at 
low speeds.

* Lower gear ratios prevent the engine from hitting its redline at high speeds, keeping power
delivery continuous across the entire velocity range.