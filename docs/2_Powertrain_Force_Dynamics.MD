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




