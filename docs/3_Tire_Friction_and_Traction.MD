# 3. Tire Friction and Traction

The force from the powertrain is transferred directly to the tires, where contact 
with the track surface generates the longitudinal force that accelerates the vehicle. 
Consequently, the vehicle's ultimate performance is governed by tire limits. The tires 
also generate parasitic friction forces that oppose motion and must be overcome to 
accelerate.

---

## 3.1. Static Friction and Traction

The tires cannot transmit a force greater than their traction limit. If this limit is 
exceeded, the wheels spin out, causing a loss of controlled acceleration.Tire traction 
is determined by the normal force acting on the tires and the static friction coefficient,
$\mu_{\text{static}}$. This coefficient dictates how much larger the maximum 
static friction force (traction capacity) is relative to the normal force. 
This relationship is expressed as:

$$F_{static friction} = N\mu_{static friction}, \tag{3.1.}$$

where:
* $F_{static friction}$ is static friction force in Newtons (N),
* N is normal force in Newtons (N),
* $\mu_{static friction}$ is static friction coefficient which is dimensionless.

---

## 3.2. Tire breakaway friction

To initiate vehicle motion from a complete standstill, an initial threshold resistance 
must be overcome. This is defined as the tire breakaway friction. Once the vehicle 
begins to move, this transient threshold drops away, and continuous rolling resistance 
takes over. The equation for breakaway friction is:

$$F_{breakaway friction} = N\mu_{breakaway friction}, \tag{3.2.}$$

where:
* $F_{breakaway friction}$ is breakaway friction force in Newtons (N),
* N is normal force in Newtons (N),
* $\mu_{breakaway friction}$ is breakaway friction coefficient which is dimensionless.

---

## 3.3. Tire rolling friction

Once the vehicle is in motion, the tires experience a continuous resistance force that 
opposes the direction of travel. In this MVP framework, we implement the classic rolling 
friction model:

$$F_{rolling friction} = N\mu_{rolling friction}, \tag{3.3.}$$

where:
* $F_{rolling friction}$ is rolling friction force in Newtons (N),
* N is normal force in Newtons (N),
* $\mu_{rolling friction}$ is rolling friction coefficient which is dimensionless.

## 3.4. Architectural Notes and Real-World Limitations

[!WARNING]
The equations defined in this module represent a simplified, idealized friction model 
suitable for a baseline Minimum Viable Product (MVP). They assume rigid tire-road 
interfaces and instantaneous friction transitions. In high-fidelity engineering simulators,
tire dynamics are highly non-linear and depend heavily on structural deformation, 
vertical load sensitivity, temperature, and slip ratios. Real-world vehicle dynamics 
software typically replaces these linear models with empirical or semi-empirical 
equations, most notably Pacejka's Magic Formula.The Pacejka model computes longitudinal 
force $F_x$ as a non-linear function of longitudinal slip 

$\kappa$:$$F_x = D \cdot \sin\left(C \cdot \arctan\left(B \cdot \kappa - E \cdot (B \cdot \kappa - \arctan(B \cdot \kappa))\right)\right) \tag{3.4}$$

where $B, C, D,$ and $E$ are empirical coefficients derived from physical tire testing. 
While our current architecture utilizes the simplified static/rolling friction threshold 
equations for performance baseline testing, the forces module is designed modularly to 
allow dropping in a Pacejka-style slip model in future development phases.



