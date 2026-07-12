# 2. Powertrain Force Dynamics

The force that accelerates car to front is the one that comes from powertrain to the wheels
and from wheels force is pushed to tarmac. Here we will only focus on the two main phenomena - 
force limited by powertrain power and force limited by powertrain max torque.

---

## 2.1. The Mathematical Model

Force from the powertrain is limited by characteristics of the powertrain itself, 
first one is the power. Power is defined as work done per unit of time and work is defined as
force over distance, so we get that:

$$P=\frac{Fd}{t}, \tag{2.1.}$$



where:
* P is power in watts (W),
* F is force in Newtons (N),
* d is displacement in meters (m),
* t is time in seconds (s).

We additionally know that:

$$v=\frac{d}{t}, \tag{2.2.}$$

where:
* v is velocity in meters per second (m/s),
* d is displacement in meters (m),
* t is time in seconds (s).

We can substitute displacent and time from equation 2.1. using equation 2.2. to get this equation,
which can be easily used by out integrators if we substitute for force and use 2nd Newtons law:

$$F=\frac{P}{v}, \tag{2.3.}$$

where:
* F is force in Newtons (N),
* P is power in watts (W),
* v is velocity in meters per second (m/s).

For a force which is limited by max torque from powertrain, we have to look at the radius of the tires,
and we need to know also drivetrain gear ratio or look at the powertrain model as directly driven.

If we look at the model as a directly driven system, then things are easier, force is transmitted
by the tire lever arm which is given by the tire radius, if we consider tire as incompressible. 

By knowing that the torque is defined as a product of the lever arm length perpendicular to the force
and the force itself, then we can plug our known variables and substitute for force and get this equation:

$$F=\frac{T}{R},\tag{2.4.}$$

where:
* F is force in Newtons (N),
* T is torque in Newton meters (Nm),
* R is tire radius in meters (m).

If we involve geartrain ratio i, then things are not complicated, as i works as torque multiplier, 
soo the equation 2.4. resolves into this equation.

$$F=\frac{iT}{R}, \tag{2.5.}$$

where:
* F is force in Newtons (N),
* T is torque before drivetrain in Newton meters (Nm),
* i is drivetrain ratio,
* R is tire radius in meters (m).

Important thing to note is that the Force at low speeds (wheel rotates slowly) it is not power limited
but torque limited. When the speed increases (wheel rotates quickly), then the force limited by torque
is larger than the one which is power limited. At the moment when the force which is power limited
is smaller than the limited by torque, then the correct one is the one limited by power. 

## 2.2. Implementation in the Simulation Lifecycle

In our SimulationRunner lifecycle loop, the engine must evaluate equations (2.3) and (2.5) 
at every discrete time step ($dt$) to determine the current tractive force before calculating 
acceleration via Newton's Second Law.To prevent a division-by-zero error when the vehicle is 
stationary ($v = 0$), the implementation uses a conditional minimum selection:

$$F_{\text{propulsion}} = \min\left(\frac{i \cdot T}{R}, \frac{P}{v}\right) \tag{2.6}$$

## 2.3. Model Simplifications and Assumptions

While this provides a highly functional baseline, this first-iteration model relies on 
several key simplifications that deviate from a real Formula 1 car:
* Incompressible Tire Radius ($R$): Equation (2.4) assumes the tire is a perfect, 
rigid circle. In reality, F1 tires deform under downforce and high speeds, meaning the loaded 
radius changes dynamically.
* Instantaneous Peak Power: We assume the engine can deliver peak power ($P$) instantly at any 
speed above the crossover point. A real internal combustion engine has a dynamic torque/RPM band 
and requires shifting gears to stay efficient.
* Infinite Track Grip: This model assumes the tarmac can absorb 100% of the force calculated in 
Equation (2.6). At launch, the maximum torque force ($\frac{iT}{R}$) often exceeds 
the friction limits of the rubber, meaning real wheels would slip.