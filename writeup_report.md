# Model Predictive Control (MPC)

---

## Model Predictive Control Project

### Project Basics
Using Model Predictive Control (MPC), this project involves writing a C++ program that can drive a simulated car around a virtual track using specific waypoints from the track itself. 
The simulator sends car telemetry information to the MPC using WebSocket and it receives the steering angle and throttle(actuators). 
The simulated car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

The goals / steps of this project are the following:
* Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
* Implementing the MPC calculation, including setting variables and constraints.
* Calculating actuator values from the MPC calc based on current state.
* Calculating steering angle & throttle/brake based on the actuator values.
* Tuning the timestep length and duration.
* Validate the vehicle is able to drive successfully around the track without leaving the road. 
* Summarize the reflection with a written report


### Results
A video of the simulated car driving around the track can be found [here.](https://github.com/sanchelsea/CarND-PID-Control-Project/blob/master/SDC_ND_MPC_6_2_2018_10_53_05_AM.mp4)


### Reflection

#### 1. The Model
The model used is a Kinematic model which is a simplified dynamic model ignoring the effects of gravity, tire forces and mass.
This model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration and steering angle (delta). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below: 

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```
Lf measures the distance between the center of mass of the vehicle and it's front axle. This value is provided by Udacity's project. (Lf = 2.67)
I had to tune the weights for the cost function in order to get a smoother ride. The weights used can be found in mpc.cpp from line 52 to 58.

#### 2. Timestep Length and Elapsed Duration (N & dt)
The final values used for N is 9 and for dt is 0.1. On increasing N value (like 15), the vehicle oscillates and drives off the track. With lower N values, the vehicle doesnt handle the curves and may drive straight off the track.
I experimented N values from 5 to 20. I didnt experiment much with dt, as 100 milliseconds was the latency between actuation commands. 

#### 3. Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's perspective (see main.cpp lines 107-115). This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero.


#### 4. Model Predictive Control with Latency
The model accounts for the simulator's added 100ms latency between the actuator calculation and when the simulator will actually perform that action. 
To handle actuator latency, the state values are calculated using the model and the delay interval(100ms). These values are used instead of the initial one. 
The code implementing that could be found in main.cpp from line 134 to line 143.

