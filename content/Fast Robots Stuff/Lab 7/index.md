+++
title = "Lab 7: Kalman Filter"
date = 2026-03-23
weight = 6
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Drag and Mass Estimations

Approximating the robot as a first-order system using Newton's second law, factoring in linear drag $d$ and motor input $u$:

$$F = m\ddot{x} = -d\dot{x} + u$$

The dynamics in continuous state-space form ($\dot{x} = Ax + Bu$) with state vector $x = \begin{bmatrix} x \\\\ \dot{x} \end{bmatrix}$ are:

$$\begin{bmatrix} \dot{x} \\\\ \ddot{x} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\\\ 0 & -\frac{d}{m} \end{bmatrix} \begin{bmatrix} x \\\\ \dot{x} \end{bmatrix} + \begin{bmatrix} 0 \\\\ \frac{1}{m} \end{bmatrix} u$$

I recorded three open-loop step responses at 150 PWM to extract parameters $d$ and $m$. Averaging these trajectories mitigated non-Gaussian Time-of-Flight (ToF) noise. Fitting an exponential decay to this consensus trajectory yielded a 70% rise time ($t_{0.7} = 1.05$ s) and steady-state velocity ($V_{ss} = 2.45$ m/s).

Assuming a normalized unit step input ($u = 1$), the parameters are calculated as:

$$d = \frac{u_{ss}}{\dot{x}_{ss}} = \frac{1}{2.45} = 0.408$$

$$m = \frac{-d \cdot t_{0.7}}{\ln(0.3)} = \frac{-0.408 \cdot 1.05}{-1.204} = 0.356$$

Plugging these into our continuous $A$ and $B$ matrices yields:

$$A = \begin{bmatrix} 0 & 1 \\\\ 0 & -1.146 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\\\ 2.809 \end{bmatrix}$$

Measuring only distance, our observation matrix $C$ isolates the first state:

$$C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

<figure style="display: flex; justify-content: space-around; align-items: flex-start; gap: 10px; width: 100%;">
<div style="flex: 1; text-align: center;">
<img src="distance_time.jpg" alt="d" style="display:block; width:100%; max-width:400px; height:auto; margin: 0 auto;">
<figcaption style="margin-top: 5px;">Average distance vs time for open loop step response</figcaption>
</div>
<div style="flex: 1; text-align: center;">
<img src="velocity.jpg" alt="d" style="display:block; width:100%; max-width:400px; height:auto; margin: 0 auto;">
<figcaption style="margin-top: 5px;">Velocity and 70% rise time fit for open loop step response</figcaption>
</div>
<div style="flex: 1; text-align: center;">
<img src="pwm.jpg" alt="d" style="display:block; width:100%; max-width:400px; height:auto; margin: 0 auto;">
<figcaption style="margin-top: 5px;">PWM graphs for open loop step response</figcaption>
</div>
</figure>

Here is a video of one of the trajectories while I was collecting data for a step response:

<iframe width="450" height="315" src="https://youtube.com/embed/74Vm-SvuMSU" allowfullscreen></iframe>
<figcaption>Data Collection Video</figcaption>

<figure>
<img src="velocity2.jpg" alt="c" style="display:block; width:100%; max-width:600px;">
<figcaption>Velocity and 70% rise time fit for a better open loop step response</figcaption>
</figure>

*Note: Subsequent trials with a closer braking distance yielded a slightly higher steady-state velocity, but my filter performed well with the initial dataset, so I proceeded with the originally calculated A and B matrices.*

## Kalman Python Simulation

### System Timing and Discretization

The continuous model must be discretized using the fast onboard prediction loop (50 Hz, $\Delta t = 0.0209$ s) rather than the slower ToF sensor rate (\~10 Hz, $\Delta t = 0.0985$ s) to prevent artificially accelerating the physics.

Discretizing via the first-order Taylor expansion ($A_d = I + A\Delta t$ and $B_d = B\Delta t$):

$$A_d = \begin{bmatrix} 1 & 0 \\\\ 0 & 1 \end{bmatrix} + \begin{bmatrix} 0 & 1 \\\\ 0 & -1.146 \end{bmatrix}(0.0209) = \begin{bmatrix} 1 & 0.0209 \\\\ 0 & 0.976 \end{bmatrix}$$

$$B_d = \begin{bmatrix} 0 \\\\ 2.809 \end{bmatrix}(0.0209) = \begin{bmatrix} 0 \\\\ 0.0587 \end{bmatrix}$$

```python
import numpy as np

# Discretized Matrices
Ad = np.array([[1.0, 0.0209], [0.0, 0.976]])
Bd = np.array([[0.0], [0.0587]])
C = np.array([[1, 0]])

# Noise Covariances
sig_u = np.array([[10.0**2, 0], [0, 10.0**2]]) 
sig_z = np.array([[20.0**2]])

# Kalman Filter Simulation Function
def kalman_filter(mu, sigma, u, y_meas, has_new_data):
    # Predict Step
    mu_p = Ad.dot(mu) + Bd.dot(u)
    sigma_p = Ad.dot(sigma).dot(Ad.T) + sig_u

    if has_new_data:
        # Update Step
        y_m = y_meas - C.dot(mu_p)
        S = C.dot(sigma_p).dot(C.T) + sig_z
        K = sigma_p.dot(C.T).dot(np.linalg.inv(S))

        mu = mu_p + K.dot(y_m)
        sigma = (np.eye(2) - K.dot(C)).dot(sigma_p)
    else:
        mu = mu_p
        sigma = sigma_p

    return mu, sigma
```

### Simulation Debugging

<figure>
<img src="first_attempt.jpg" alt="Unit Mismatch Graph" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Simulation Attempt 1: Unit Mismatch resulting in apparent zero velocity.</figcaption>
</figure>

**1. Unit Mismatch:** The matrices initially predicted velocity in m/s, while raw data was plotted in mm/s, causing the accurate -3.1 m/s prediction to appear as a flat line on a -3000 mm/s scale. Recalculating matrices in millimeters fixed this.

<figure>
<img src="second_attempt.jpg" alt="Delta t Mismatch Graph" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Simulation Attempt 2: Discretization timing mismatch causing extreme acceleration.</figcaption>
</figure>

**2. Discretization Mismatch:** The simulated velocity reached steady-state too quickly (0.25s vs 1.06s) because $A_d$ and $B_d$ were mistakenly discretized using the 0.1s ToF rate instead of the simulation grid's interval. Using the correct $\Delta t = 0.0209$ s resolved the 5x artificial acceleration.

### Sensor Noise Characterization and Final Tuning

Stationary tests revealed distance-dependent noise: $\sigma_{start} \approx 19.2$ mm (at 3800 mm) and $\sigma_{target} \approx 1.2$ mm (at 430 mm).

<figure style="display: flex; justify-content: space-around; align-items: flex-start; gap: 10px; width: 100%;">
<div style="flex: 1; text-align: center;">
<img src="station_3800mm.jpg" alt="Stationary Noise at Far Distance" style="display:block; width:100%; max-width:400px; height:auto; margin: 0 auto;">
<figcaption style="margin-top: 5px;">Stationary Noise at Start (~3860mm): $sigma = 19.244$ mm</figcaption>
</div>
<div style="flex: 1; text-align: center;">
<img src="tof_noise_304.jpg" alt="Stationary Noise at Close Distance" style="display:block; width:100%; max-width:400px; height:auto; margin: 0 auto;">
<figcaption style="margin-top: 5px;">Stationary Noise near Target (~430mm): $sigma = 1.223$ mm</figcaption>
</div>
</figure>

To prevent violent derivative kick at high speeds, I tuned the measurement covariance ($\Sigma_z$) for the worst-case scenario.

$$
\Sigma_u = \begin{bmatrix} 100.0 & 0 \\\\ 0 & 100.0 \end{bmatrix}, \quad \Sigma_z = \begin{bmatrix} 400.0 \end{bmatrix}
$$

In the Kalman Gain equation, setting $\Sigma_z = 400.0 \gg \Sigma_u = 100.0$ drastically shrinks the gain $K$. Consequently, the update equation ($\mu = \mu_p + K(y - C\mu_p)$) minimizes the impact of noisy sensor residuals. The system heavily trusts the physical prediction ($\mu_p$), allowing the velocity estimate to slice cleanly through the initial noise cloud.

<figure>
<img src="third_attempt.jpg" alt="Properly Tuned KF Graph" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Simulation Attempt 3: Fully tuned Kalman Filter accurately tracking distance and filtering velocity.</figcaption>
</figure>

## Onboard Robot Kalman Integration

The `BasicLinearAlgebra` library was used to port the matrices directly to the Artemis board. The control loop was decoupled from the sensor, executing the prediction step on a strict 50 Hz timer and updating only when the `has_new_data` flag triggered.

```cpp
#include <BasicLinearAlgebra.h>
using namespace BLA;

BLA::Matrix<2,2> Ad = {1.0, 0.0209, 0.0, 0.976}; 
BLA::Matrix<2,1> Bd = {0.0, 0.0587};
BLA::Matrix<1,2> C  = {1.0, 0.0};

BLA::Matrix<2,2> sig_u = {100.0, 0.0, 0.0, 100.0};
BLA::Matrix<1,1> sig_z = {400.0};

BLA::Matrix<2,1> mu = {0.0, 0.0};
BLA::Matrix<2,2> sigma = {10.0, 0.0, 0.0, 10.0};

void update_kalman(bool has_new_data, float measured_distance, float current_pwm) {
    BLA::Matrix<2,1> u = {current_pwm / 150.0f}; // Scale input to unit step
    BLA::Matrix<2,1> mu_p = Ad * mu + Bd * u;
    BLA::Matrix<2,2> sigma_p = Ad * sigma * ~Ad + sig_u;

    if (has_new_data) {
        BLA::Matrix<1,1> y = {measured_distance};
        BLA::Matrix<1,1> S = C * sigma_p * ~C + sig_z;
        BLA::Matrix<1,1> S_inv = S; 
        Invert(S_inv);
        
        BLA::Matrix<2,1> K = sigma_p * ~C * S_inv;
        mu = mu_p + K * (y - C * mu_p);
        sigma = (Eye<2>() - K * C) * sigma_p;
    } else {
        mu = mu_p; sigma = sigma_p;
    }
}
```

```cpp
void executeControl() {
    uint32_t now = micros();
    if (now - last_fast_loop_time >= 20000) { // 50Hz Fast Loop
        last_fast_loop_time = now;

        update_kalman(new_tof_data, dist_k, current_applied_pwm);
        new_tof_data = false;

        float kf_distance = mu(0,0);
        float raw_control_effort = computePID(linear_pid_setpoint, kf_distance);
        int actual_pwm = applyMotorSpeed(raw_control_effort);
        
        current_applied_pwm = actual_pwm; 
    }
}
```

### PID Tuning and Derivative Kick

<figure>
<img src="high_kd.jpg" alt="High Kd Oscillation" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Phase 1: Extreme oscillation caused by Lab 5 derivative gains applied to the fast loop.</figcaption>
</figure>

**Phase 1: Timing Mismatch:** Applying Lab 5 PD gains ($K_p = 0.025$, $K_d = 20000$) to the 50 Hz loop caused violent oscillations. Because the new $\Delta t$ (0.020s) was five times smaller than the old ToF interval (0.098s), the discrete derivative ($K_d \frac{de}{dt}$) effectively multiplied the derivative kick by 5.

<figure>
<img src="no_kd_low_kp.jpg" alt="Zero Kd Coasting" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Phase 2: Proportional-only control resulting in a slow approach and failure to stop.</figcaption>
</figure>

**Phase 2: Coasting Baseline:** Disabling the derivative term ($K_d = 0$) and using $K_p = 0.015$ eliminated oscillations, but without a derivative "brake," the robot coasted through the 304 mm target and crashed.

<figure>
<img src="finetuned.jpg" alt="Tuned KF Response" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Phase 3: Final tuned response utilizing the Kalman Filter for smooth, aggressive braking.</figcaption>
</figure>

**Phase 3: Kalman Velocity Control:** To safely restore braking power, I substituted the Kalman Filter's smoothed velocity state ($\mu_1$) for the noisy discrete derivative calculation:

$$u(t) = K_p(x_{target} - \mu_0) - K_d(\mu_1)$$

This eliminated derivative chatter entirely. Approaching the 300 mm mark, $\mu_1$ applies a clean electronic braking force for a perfectly controlled stop.

<iframe width="450" height="315" src="https://youtube.com/embed/HNZxEkrY4M4" allowfullscreen></iframe>
<figcaption>Linear PID Test with Tuned Kalman Filter</figcaption>

### Final Remark

While implementation was challenging, the resulting performance was highly satisfying. I achieved rapid initial acceleration (starting at max 255 PWM and decreasing to 100 PWM within the first second), covering significant distance quickly before the filter seamlessly executed the braking maneuver.

## Collaboration

I referred to Jack Long and Aidan McNay's pages for graphing techniques and Kalman Filter C++ architectures. I collaborated with Ananya Jajodia on collecting step response data, and utilized ChatGPT to assist with data visualization and report formatting.