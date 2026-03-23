+++
title = "Lab 7: Kalman Filter"
date = 2026-03-23
weight = 6
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Drag and Mass Estimations

To implement a Kalman Filter, we first need a mathematical model of the robot's physics. Approximating the open-loop robot system as a first-order system using Newton's second law:

$$F = ma = m\ddot{x}$$

Adding the linear drag force $d$ and motor input $u$:

$$F = -d\dot{x} + u$$

The dynamics of the system can be described in terms of the second derivative of position:

$$\ddot{x} = -\frac{d}{m} \dot{x} + \frac{u}{m}$$

We can represent the system in state-space notation with the state vector $x$:

$$x = \begin{bmatrix} x \\ \dot{x} \end{bmatrix}$$

The corresponding dynamics in the continuous state-space form $\dot{x} = Ax + Bu$ are:

$$\begin{bmatrix} \dot{x} \\ \ddot{x} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & -d/m \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \end{bmatrix} + \begin{bmatrix} 0 \\ 1/m \end{bmatrix} u$$

To find $d$ and $m$, I drove the car toward a wall using a step response of 150 PWM. Because the motor shut off at exactly 1.8 seconds (before the system reached 90% of its steady-state velocity), relying on a 90% rise time for calculations would have required extrapolating data into the braking phase. To keep the model grounded in active acceleration data, I calculated the 70% rise time ($t_{0.7}$) instead, adjusting the momentum formula to use $\ln(0.3)$.

$$d = \frac{u_{ss}}{\dot{x}_{ss}}$$

$$m = \frac{-d \cdot t_{0.7}}{\ln(0.3)}$$

*(Insert your velocity step response graph here)*

\<figcaption\>Velocity Step Response with Exponential Fit and 70% Rise Time\</figcaption\>

Using a unit step input ($u = 1$), my steady-state velocity, 70% rise time, $d$, and $m$ resulted in the following $A$ and $B$ matrices:

$$A = \begin{bmatrix} 0 & 1 \\ 0 & -d/m \end{bmatrix}$$
$$B = \begin{bmatrix} 0 \\ 1/m \end{bmatrix}$$

Because we only measure distance (ToF data) and not velocity, our observation matrix $C$ isolates the first state:

$$C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

## Kalman Python Simulation

### Initialization and Data Alignment

My raw Time-of-Flight (ToF) data was sampled at uneven intervals (roughly every 50-90ms). To accurately simulate the filter, I first interpolated the recorded distances onto a common, evenly spaced time grid.

A critical finding during simulation was the relationship between the discretization time step ($\Delta t$) and the loop execution time. Initially, calculating $\Delta t$ based on the slow ToF sensor average caused the Kalman Filter to estimate velocities 5x faster than reality when run on a faster iterative loop. To fix this, I discretized the $A$ and $B$ matrices using the exact $\Delta t$ of the simulation loop grid.

```python
# Discretization using the interpolated time grid step
delta_t = common_time[1] - common_time[0] 

Ad = np.eye(2) + (delta_t * A_matrix)
Bd = delta_t * B_matrix
```

I initialized my process noise ($\Sigma_u$) and sensor noise ($\Sigma_z$) matrices. Because I wanted the filter to slice through the highly noisy derivative velocity data, I tuned $\Sigma_z$ higher, forcing the filter to trust the physical state-space model more than the raw sensor readings during rapid acceleration changes.

```python
# Process and Sensor Noise
sig_u = np.array([[10.0**2, 0], [0, 10.0**2]]) 
sig_z = np.array([[20.0**2]])
```

### Testing the Simulation

When scaling the input $u$ for the filter, I divided the active PWM by the maximum step size (150). This ensured the filter accurately predicted physics relative to the unit step used to calculate $d$ and $m$.

*(Insert your Python Distance and Velocity KF Graphs here)*

\<figcaption\>Kalman Filter Simulation Tracking Distance and Filtering Velocity\</figcaption\>

The simulation demonstrated that the Kalman Filter successfully predicted the immediate drop in velocity the exact millisecond the motor PWM shut off (1.8s), completely bypassing the inherent latency of the ToF sensor.

## Onboard Robot Kalman Integration

### Architecture: Untethering the Control Loop

To integrate the filter onto the Artemis board using the `BasicLinearAlgebra` library, the control loop architecture had to be fundamentally redesigned. Previously, the PID controller was trapped by the speed of the ToF sensor, only executing when a new reading arrived.

To leverage the Kalman Filter's predictive power, I placed the control and prediction loop on a strict, high-speed 50Hz (20ms) timer. Every 20ms, the filter predicts the robot's state based purely on the previous PWM command. The slower ToF sensor updates act asynchronously; when a new reading arrives, it triggers the Kalman Update step to correct the prediction.

```cpp
void executeControl() {
    uint32_t now = micros();
    if (now - last_fast_loop_time >= 20000) { // 50Hz Fast Loop
        last_fast_loop_time = now;

        // 1. Predict state using applied PWM (scaled by 150.0f to prevent narrowing)
        update_kalman(new_tof_data, dist_k, current_applied_pwm);
        new_tof_data = false; // Consume flag

        // 2. Extract KF estimated distance
        float kf_distance = mu(0,0);

        // 3. Compute PID using the FILTERED distance
        float raw_control_effort = computePID(linear_pid_setpoint, kf_distance);
        
        // ... [Motor scaling and application logic] ...
        
        current_applied_pwm = actual_pwm; // Save for next loop's physics prediction
    }
}
```

By ensuring the discrete $A_d$ and $B_d$ matrices programmed into the Artemis were explicitly calculated using $\Delta t = 0.02s$, the internal physics model perfectly matched the execution speed of the microcontroller.

*(Insert your final robot performance graph/video here)*

\<figcaption\>Robot effectively utilizing KF prediction to halt at target distance\</figcaption\>

Here is the final section to complete your report. I have kept the focus strictly on the engineering decisions you made—specifically how scaling $u$ fixed the unit mismatch and how inflating $\Sigma_z$ allowed you to filter out that massive velocity derivative noise.

***

## Variable Discussion

To achieve the final, smooth predictive performance, several variables had to be strictly defined and iteratively tuned during testing:

* **$m$ and $d$ (Momentum and Drag):** These parameters ground the state-space model in physical reality. A higher $m$ represents greater inertia, meaning the mathematical model expects the robot to accelerate and decelerate sluggishly. The $d$ term governs the steady-state top speed and dictates how quickly friction and air resistance halt the robot when the motors are cut.
* **$u$ (Control Input):** Directly feeding raw PID outputs (0-255) into the filter would have completely broken the velocity predictions. Because our $m$ and $d$ values were derived from a step response performed at 150 PWM, the active control input $u$ had to be explicitly scaled (`current_pwm / 150.0f`) during the control loop to keep the physics proportional to a unit step.
* **$\Sigma_u$ and $\Sigma_z$ (Trust and Noise Covariances):** Tuning these matrices is how the filter's behavior is actually shaped.
    * $\sigma_1$ and $\sigma_2$ ($\Sigma_u$ / Process Noise): These represent how much we distrust our own physics model due to unmodeled real-world factors like wheel slip, surface changes, or battery voltage drops. Increasing these values forces the filter to rely more heavily on the raw ToF data, which tightly tracks actual distance but introduces latency.
    * $\sigma_3$ ($\Sigma_z$ / Measurement Noise): This represents our distrust of the ToF sensor. Because calculating velocity by taking the derivative of discrete distance measurements is inherently noisy, I explicitly tuned $\sigma_3$ higher. This forced the Kalman Filter to heavily trust the state-space prediction during rapid transitions (like the exact millisecond the PWM drops to 0). This predictive smoothing is what prevents the PID controller from experiencing violent derivative kick when braking.

## Collaboration

I referred to Jack Long and Aidan McNay's page for results and graph to included, as well as kalman filter implementations. I collaborated Ananya Jajodia on collecting step response data, and utilized ChatGPT to assist with data visualization. 