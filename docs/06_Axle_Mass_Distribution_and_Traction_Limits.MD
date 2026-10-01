# 6. Axle Mass Distribution & Traction Limits (Static Model)

---

## 6.1. Off-Center Mass = Unequal Axle Normal Forces

If a car's Center of Gravity (CoG) is not in the exact geometric center, 
the vehicle weight ($m \cdot g$) does not split 50/50 between the front and rear wheels.
Front Axle Normal Force: 

$$F_{z,\text{front}} = m \cdot g \cdot \left(\frac{l_r}{L}\right) \tag{6.1.}$$

Rear Axle Normal Force: 

$$F_{z,\text{rear}} = m \cdot g \cdot \left(\frac{l_f}{L}\right) \tag{6.2.}$$ 

Total Normal Force: 

$$F_{z,\text{total}} = F_{z,\text{front}} + F_{z,\text{rear}} = m \cdot g, \tag{6.3.}$$

where $l_f$ and $l_r$ are distances from the CoG to the front and rear axles, and $L$ is 
the wheelbase ($L = l_f + l_r$). The code does not store $l_f$ and $l_r$: the static split is
given directly by the front and rear masses in `Mass`, so $l_r / L$ = `mass.front / mass.total`
and $l_f / L$ = `mass.rear / mass.total`.

---

## 6.2. Resistance Losses Act on ALL Axles

Rolling resistance and breakaway friction act on every single wheel touching the ground, 
regardless of whether it is connected to the engine. 

Friction Loss Rule: Uses Total Normal Force ($F_{z,\text{total}}$). All 4 wheels deform 
against the road surface and drain energy from the vehicle.3. 

Engine Traction Limits Act ONLY on Driven Axles. Engine torque can only be turned into 
forward acceleration through the wheels connected to the drivetrain.

Traction Limit Rule: Uses Driven Normal Force ($F_{z,\text{driven}}$). Front-Wheel Drive 
(FWD): Max Traction Limit 

$$= \mu_{\text{static}} \cdot F_{z,\text{front}} \tag{6.4.}$$

Rear-Wheel Drive (RWD): Max Traction Limit 

$$= \mu_{\text{static}} \cdot F_{z,\text{rear}} \tag{6.5.}$$

All-Wheel Drive (AWD): Max Traction Limit 
$$= \mu_{\text{static}} \cdot F_{z,\text{total}}\tag{6.6.}$$

Equation 6.6 is only reached when the torque split matches the axle loads. For any other
split, the first axle to saturate caps the total (see section 10.7).

The Big Takeaway: If a car has a 60% Front / 40% Rear weight bias, an RWD car can only 
use 40% of its normal force for acceleration grip, even though 100% of its normal force 
generates rolling resistance!
