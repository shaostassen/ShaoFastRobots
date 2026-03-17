+++
title = "Lab 6: Orientation Control"
date = 2026-03-24
weight = 9
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Objective

The purpose of this lab is to implement a robust proportional-integral-derivative (PID) controller to manage the orientation (yaw) of the robot. This involves utilizing the IMU to perform in-place rotation using differential drive, while mitigating real-world sensor issues like gyroscope drift and derivative kick.

-----

## Prelab: Bluetooth Architecture and Dynamic Setpoints

To allow rapid tuning and mid-run setpoint changes, the Bluetooth handling was designed to be completely non-blocking. Instead of trapping the robot in a `while` loop during a turn, the main `loop()` continuously cycles through telemetry, sensor reading, and motor updating.

This architecture allowed me to implement a dedicated `UPDATE_YAW_SETPOINT` BLE command.

```cpp
case UPDATE_YAW_SETPOINT: {
    float new_yaw;
    success = robot_cmd.get_next_value(new_yaw);
    if (!success) return;
    
    rotational_setpoint = new_yaw;
    // ... [telemetry logging]
    break;
}
```

This satisfies the requirement to change the setpoint dynamically without halting the system. By passing new floats over Bluetooth, I can command the robot to snap to 90 degrees, and before it even finishes the turn, update the setpoint to -45 degrees seamlessly.

-----

## PID Input Signal and Gyroscope Bias

Initially, standard digital integration of the raw gyroscope data (`gyrZ()`) was tested to estimate orientation. However, a stationary test revealed significant hardware bias, drifting at approximately -0.24 degrees per second. Over a 3-minute combat robotics match, this bias accumulates to over 40 degrees of phantom rotation, rendering standard integration unusable for long-term orientation control.

### The Digital Motion Processor (DMP) Solution

To eliminate this bias, I bypassed manual integration and enabled the ICM-20948's onboard Digital Motion Processor (DMP). By configuring the DMP to output the 6-axis Game Rotation Vector (Quat6) at maximum speed (55Hz), the hardware's sensor fusion algorithm automatically corrects for drift in the background.

```cpp
    // Keep reading until the FIFO queue is completely empty
    while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) {
        myICM.readDMPdataFromFIFO(&data);
    }
    // ... [Quaternion to Euler conversion] ...
    yaw_g_state = raw_yaw - yaw_offset; // Zero out relative to start heading
```

To prevent the DMP's internal FIFO queue from overflowing and crashing the sensor, a `while` loop was used to completely drain the queue, ensuring the PID loop always acts on the freshest, completely drift-free quaternion data.

-----

## The Derivative Term and Anti-Kick

The lab handout asks: *"Does it make sense to take the derivative of a signal that is the integral of another signal?"* The answer is no. Taking the mathematical derivative of the error `(error - prev_error) / dt` just undoes the integration, but amplifies digital noise. More importantly, it exposes the system to **Derivative Kick**. If the setpoint is suddenly changed (e.g., commanding a turn from 0° to 90° mid-run), the error instantly spikes. Taking the derivative of this sudden step-change results in a near-infinite spike in the D-term, causing the motors to jolt violently.

Because the derivative of a constant setpoint is zero, the derivative of the error is mathematically equal to the negative of the actual rate of change. I bypassed the standard math and fed the raw gyroscope rate (`-myICM.gyrZ()`) directly into the D-term.

```cpp
    // ANTI-DERIVATIVE KICK: Use raw gyro rate instead of (error - prev_error)/dt
    float derivative = -gyro_rate; 
    float D = Kd * derivative;
```

This allows the derivative term to act smoothly as a rotational brake without inducing violent voltage spikes when the target changes.

-----

## Orientation Control and Tuning

Control relies on differential drive, passing `control_effort` to the left motor and `-control_effort` to the right motor using the calibrated motor functions from Lab 5. During initial testing, I accidentally created a positive feedback loop (the "Death Spin") because turning the robot physically right yielded a negative IMU angle, causing the error to grow rather than shrink. Flipping the motor command signs instantly fixed this.

During tuning, the integral term (**Ki**) was kept at 0 to avoid integrator wind-up at first, as steady-state error is minimal for free-spinning wheels on a smooth floor. I did implement wind-up protection (`constrain(integral_sum, -50.0, 50.0);`), later than I noticed persistent small errors between my PD controller and the setpoints, so I added the integral term. 

I increased the proportional gain (**Kp**) until the robot snapped to the target quickly, then applied the derivative gain (**Kd**) to dampen the resulting oscillations.

  * **Final Kp:** `10.0`
  * **Final Kd:** `0.5`
  * **Final Ki:** `0.5`

### System Response and Results

Below are the results of the tuned PID controller handling various setpoint commands and disturbances.

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/YOUR\_VIDEO\_ID\_1](https://www.google.com/search?q=https://youtube.com/embed/YOUR_VIDEO_ID_1)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Step Response: 0 to 90 Degrees\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="YOUR\_PLOT\_IMAGE\_1.jpg" alt="Step Response Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>Set Point, Angle, and PWM Effort over Time\</figcaption\>
\</figure\>
\</div\>
\</div\>

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/YOUR\_VIDEO\_ID\_2](https://www.google.com/search?q=https://youtube.com/embed/YOUR_VIDEO_ID_2)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Dynamic Setpoint Change (Anti-Kick Test)\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="YOUR\_PLOT\_IMAGE\_2.jpg" alt="Dynamic Setpoint Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>Notice the smooth PWM transition despite the target jumping\</figcaption\>
\</figure\>
\</div\>
\</div\>

\<div style="display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-bottom: 40px;"\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<iframe width="100%" height="315" src="[https://youtube.com/embed/YOUR\_VIDEO\_ID\_3](https://www.google.com/search?q=https://youtube.com/embed/YOUR_VIDEO_ID_3)" allowfullscreen\>\</iframe\>
\<figcaption style="text-align: center; font-style: italic;"\>Robustness Test: External Perturbations\</figcaption\>
\</div\>
\<div style="flex: 0 0 48%; min-width: 300px;"\>
\<figure style="margin: 0;"\>
\<img src="YOUR\_PLOT\_IMAGE\_3.jpg" alt="Perturbation Plot" style="display:block; width:100%;"\>
\<figcaption style="text-align: center; font-style: italic;"\>The robot actively fighting to maintain a 0-degree heading\</figcaption\>
\</figure\>
\</div\>
\</div\>


https://youtube.com/shorts/xDcn3u4P5s4
https://youtube.com/shorts/NCYYelEjF5c

https://youtube.com/shorts/wg8QhRoUuI0
https://youtube.com/shorts/aPiQhUuWHdg

https://youtube.com/shorts/MvP56cXlcOk
https://youtube.com/shorts/zwCbHPWFi0w

