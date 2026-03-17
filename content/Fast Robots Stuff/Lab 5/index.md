+++
title = "Lab 5: Linear PID Control and Linear Interpolation"
date = 2026-03-15
weight = 8
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Objective

The goal is to implement a robust linear PID controller to drive the robot quickly toward a wall, stopping exactly 304mm (1 foot) away. Linear extrapolation is used to decouple the high-frequency PID control loop from the slower Time-of-Flight (ToF) sensor data rate.

-----

## Prelab: Bluetooth Architecture and Debugging

To prevent `Serial.print` or Bluetooth delays from slowing down the PID loop, debugging data is stored locally in arrays during the run.

The test sequence is controlled via Python over BLE. `START_LINEAR_PID_DATA` triggers the control loop with the 304mm setpoint. `SET_PID` allows real-time tuning of **Kp**, **Ki**, and **Kd** without recompiling. Once finished, `STOP_LINEAR_PID_DATA` halts the motors and transmits the data to Jupyter for visualization.

\<figure\>
\<img src="debug.jpg" alt="circuit" style="display:block; width:100%; max-width:600px;"\>
\<figcaption\>Here is an example of ToF data being sent over the network for debugging.\</figcaption\>
\</figure\>

-----

## Position Control Implementation

### ToF Sensor Configuration and Testing Challenges

Initial testing was frustrating. I used the Long `distanceMode` to potentially start up to 4 meters away, but the ToF sensor continuously failed to read past 1.8 meters on the ground. After hours of debugging and swapping between two cars, I found the culprits: both cars' ToF sensors pointed slightly downward, and the carpet strongly scattered the IR waves.

Moving my tests to the hallway instantly fixed the reliability issues. While this provided clean data from 3 meters out, it reduced the sampling rate to roughly 20Hz.

### Motor Deadband and Scaling Calibration

To allow micro-adjustments near the target, `driveCalibrated()` handles the motor deadband.

```cpp
void driveCalibrated(float left_speed, float right_speed) {
    if(!motors_enabled) return;

    // Constrain before scaling to prevent full-throttle drift
    left_speed = constrain(left_speed, -255.0, 255.0);
    right_speed = constrain(right_speed, -255.0, 255.0);

    left_speed *= left_motor_scaling;
    right_speed *= right_motor_scaling;

    if (fabs(left_speed) > 0.1) {
        int pwm_l = map(fabs(left_speed), 0, 255, min_pwm_left, 255);
        pwm_l = constrain(pwm_l, min_pwm_left, 255); 
        // ... (Directional logic and right motor logic follows)
```

The PID output maps an effort of "1" to the minimum PWM needed to overcome static friction (\~40 PWM). The raw input is constrained *before* scaling, ensuring straight driving even at maximum PID output.

### Proportional (P) Control Tuning

Starting 3000mm away with a 304mm setpoint creates a massive initial error (\~2696mm). A standard **Kp** (like 1.0) would dangerously saturate the motors. A viable gain range to keep commands within 0-255 bounds is between **Kp = 0.005** (conservative) and **0.009** (aggressive).

My initial tests were from 1800mm due to the early sensor issues.

\<iframe width="450" height="315" src="[https://youtube.com/embed/vyQKNND2o0w](https://youtube.com/embed/vyQKNND2o0w)" allowfullscreen\>\</iframe\>
\<figcaption\>1800mm P Test\</figcaption\>

\<figure\>
\<img src="initP.jpg" alt="circuit" style="display:block; width:100%; max-width:600px;"\>
\<figcaption\>P-only PID control, ToF Distance vs. Time \</figcaption\>
\</figure\>

While P-control was fairly accurate, higher speeds caused overshoot. Adding a Derivative (**Kd**) term acted as a brake, allowing a higher P-term.

\<iframe width="450" height="315" src="[https://youtube.com/embed/g93l-az-CnU](https://youtube.com/embed/g93l-az-CnU)" allowfullscreen\>\</iframe\>
\<figcaption\>1800mm PD Test\</figcaption\>

\<figure\>
\<img src="tuneD.jpg" alt="circuit" style="display:block; width:100%; max-width:600px;"\>
\<figcaption\>PD control, ToF Distance vs. Time \</figcaption\>
\</figure\>

My final tuned gains were **Kp = 0.0025**, **Kd = 20000.0**, and **Ki = 0**. I omitted the I-term since there was no observable steady-state error.

-----

## Linear Extrapolation

### The Sampling Rate Problem

The ToF sensor runs at 18-20Hz, but the main PID loop executes hundreds of times per second. Without extrapolation, using stale distance data for 50ms causes jerky movements and discrete derivative steps.

### Implementation

To decouple these rates, `readToF()` calculates the PID effort every loop cycle. If no new data is ready, it estimates the current distance using velocity from the last two readings:

$$d_{est} = d_{last} + (v \cdot \Delta t)$$

```cpp
    float estimated_distance = last_known_dist; 

    if (!new_data_available && prev_known_time > 0) {
        float dt_history = (last_known_time - prev_known_time) / 1000000.0f;
        if (dt_history > 0.0f) {
            float velocity = (last_known_dist - prev_known_dist) / dt_history; 
            float time_since_last = (current_time - last_known_time) / 1000000.0f;
            estimated_distance = last_known_dist + (velocity * time_since_last);
        }
    }
    
    // Execute PID immediately using the estimated distance
    if (run_linear_pid) {
        float control_effort = computePID(linear_pid_setpoint, estimated_distance);
        driveCalibrated(control_effort, control_effort);
    }
```

### Extrapolated Results

With extrapolation, the PID loop runs continuously at over 150Hz. The robot smoothly ramps down its motor speed between sensor flashes, allowing higher top speeds safely. The plots show true sensor readings at \~20Hz alongside high-frequency motor command adjustments. I recorded three trials of this extrapolated controller.

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/dOP3MGeF\_IE](https://youtube.com/embed/dOP3MGeF_IE)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 1 Video\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="trial1.jpg" alt="Trial 1 Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 1: ToF Distance vs. Time\</figcaption\>
\</figure\>
\</div\>
\</div\>

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/33lLNpuxZKE](https://youtube.com/embed/33lLNpuxZKE)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 2 Video\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="trial2.jpg" alt="Trial 2 Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 2: ToF Distance vs. Time\</figcaption\>
\</figure\>
\</div\>
\</div\>

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/v3NTideadpo](https://youtube.com/embed/v3NTideadpo)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 3 Video\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="trial3.jpg" alt="Trial 3 Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>Trial 3: ToF Distance vs. Time\</figcaption\>
\</figure\>
\</div\>
\</div\>

Motor effort is the negative of PWM, due to some sign in calculation that's hard to fix. The max speed I acheieve with my linear PID while successful is approximately **1 m/s**, this could be find via slope from above graphs. 
Finally, I shoved the car while the linear PID was active to test how well it adapted to changing states and external disturbances.

\<iframe width="100%" height="315" src="[https://youtube.com/embed/sTJlFlL0Tps](https://youtube.com/embed/sTJlFlL0Tps)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Disturbance Test Video\</figcaption\>

-----

## Discussion and Additional Implementations

To ensure the robot drove straight even when the PID demanded maximum effort, I implemented **pre-constraining** on the PID output before applying the differential **motor scaling factors**. I also kept the Bluetooth `read_data()` and `write_data()` handlers entirely **non-blocking** so the main control loop could maintain its high-frequency execution.

-----

## Collaboration

I collaborated on this project with Ananya Jajodia on troubleshooting my car. I also referenced Aiden Derocher's site for debugging and testing help. ChatGPT was used to help with some website formatting.