+++
title = "Lab 8: Stunts"
date = 2026-04-07
weight = 5
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Introduction: The Transition from Flip to Drift

For this lab, I initially attempted Task A: The Flip. While I successfully wrote a two-phase state machine and integrated heading-hold to keep the robot straight, executing the flip reliably outside of the lab environment proved physically impossible. The stunt relies heavily on a high coefficient of friction ($\mu$) provided by the lab's sticky pads. When transitioning from full forward to full reverse (-255 PWM) on normal floors, the wheels simply broke traction and skidded, failing to generate the rotational torque necessary to vault the chassis.

Due to these hardware and environmental constraints, I pivoted to Task B: The Drift, leveraging a closed-loop PD controller to execute a high-speed 180-degree turn.

## Drift Implementation

The drift stunt is orchestrated by a three-phase state machine governed by the continuous evaluation of the Kalman Filter's distance estimate and the IMU's yaw angle.

### State Machine Architecture

1.  **Phase 0 (The Sprint):** The robot drives forward at max speed (255 PWM). A proportional heading-hold controller uses the IMU yaw to keep the trajectory perfectly straight until the Kalman Filter estimates the distance to the wall is less than or equal to 914 mm (3 feet).
2.  **Phase 1 (The 180 PD Spin):** The motors transition into a differential drive spin. I utilized a PD controller using the IMU's yaw error and derivative rate. Crucially, a **motor deadband compensator** was added. As the robot approaches the 180-degree mark, the proportional error shrinks. Without compensation, the commanded PWM drops below the physical friction threshold of the gearboxes (roughly 40 PWM), causing the robot to stall. The deadband logic forces a minimum PWM to ensure the turn completes.
3.  **Phase 2 (The Return):** Upon reaching within 10 degrees of the 180-degree target, the Kalman Filter is reset to track the *new* wall behind the robot. The robot sprints forward again with heading-hold engaged until it reaches the stop distance (600 mm).

<!-- end list -->

```cpp
// Phase 1: 180 PD Spin Logic with Deadband Compensation
float yaw_error = drift_target_yaw - yaw_g_state;
while (yaw_error > 180.0) yaw_error -= 360.0;
while (yaw_error < -180.0) yaw_error += 360.0;

float d_error = (yaw_error - previous_yaw_error) / dt;
previous_yaw_error = yaw_error;

float spin_effort = (drift_Kp_spin * yaw_error) + (drift_Kd_spin * d_error);

// Deadband Compensation
if (spin_effort > 0 && spin_effort < min_pwm_right) {
    spin_effort = min_pwm_right + 5; 
} else if (spin_effort < 0 && spin_effort > -min_pwm_left) {
    spin_effort = -min_pwm_left - 5;
}

spin_effort = constrain(spin_effort, -255.0, 255.0);
```

## Kalman Filter Integration & Debugging

Integrating the fast prediction step of the Kalman Filter with the stunt logic revealed two major implementation bugs that had to be solved.

### 1\. The Sawtooth Prediction Bug

Initially, my Kalman Filter exhibited a "sawtooth" pattern. Between sensor readings, the predicted distance increased, only to violently snap downward when a new ToF reading arrived. This was caused by a sign error in the physics model: forward PWM was being added to the distance state rather than subtracted. Fixing the input calculation to `u = {-current_pwm / 150.0f}` smoothed the trajectory entirely.

```cpp
void update_kalman(bool has_new_data, float measured_distance, float current_pwm) {
    Matrix<1,1> u = {-current_pwm / 150.0f}; // Corrected sign for forward motion
    Matrix<2,1> mu_p = Ad * mu + Bd * u;
    Matrix<2,2> sigma_p = Ad * sigma * ~Ad + sig_u; 
    // ... update logic
}
```

### 2\. The Premature Trigger Bug

When testing the drift at a 1800mm trigger distance, the robot would instantly spin at the starting line. I discovered that because the ToF sensor polling function (`readToF()`) was locked inside the active state loop, the global distance variable `dist_k` initialized at `0.0`. I moved the sensor polling directly into the main `loop()` to run continuously in the background alongside the IMU, ensuring an accurate distance was loaded into the Kalman Filter the microsecond the stunt commanded a reset.

## Results and Telemetry

\<iframe width="450" height="315" src="YOUR\_YOUTUBE\_LINK\_HERE" allowfullscreen\>\</iframe\>
\<figcaption\>Drift Execution Video\</figcaption\>

\<img src="YOUR\_GRAPH\_IMAGE\_HERE.png" alt="Drift Telemetry" style="display:block;"\>
\<figcaption\>Drift Telemetry: Kalman Distance, Yaw Angle, and Motor PWM over Time\</figcaption\>

The graphs successfully demonstrate the three phases of the stunt. The blue KF Distance curve drops linearly during the initial sprint, pauses during the differential spin (Phase 1), and drops again on the return trip. The red Yaw line cleanly tracks the 180-degree rotation, and the step changes in the PWM graph showcase the transition from linear drive to differential spin and back.

## Blooper Video

\<iframe width="450" height="315" src="YOUR\_BLOOPER\_LINK\_HERE" allowfullscreen\>\</iframe\>
\<figcaption\>Blooper: Motor stalling mid-spin before deadband compensation was added.\</figcaption\>

## Summary and Challenges

1. You can see that in two of the trials, an **added weight** is mounted to the front of the robot to help it nosedive and flip about its front. By the end I realized that the added mass did not help as much as a fully charged battery thus you can see that the trials with the added weight (1 and 2) are nearly identical to those without. The weight was made out of taped together washers and was shared between Trevor and I.

<img src="/Fast Robots Media/Lab 8/car.png" alt="Alt text" style="display:block;">
<figcaption>Robot with (right) and without (left) weight</figcaption>

2. Throughout the hours of testing, I had to use **correction factors** to straighted the car's trajectory toward the mat and also help to slow the wheels down at the same time, ensuring that when flipped, the robot was oriented straight. My correction terms scale the passed in PWM speed. When driving at the wall, a correction term of 0.95 scales the right side motor and when driving in reverse (slowing down) a correction factor of 0.90 scales the left side motor. The blooper is an example of the robot before tuning. You can see it arc left when approaching the wall and then spinning as it slowed down which made it ultimately return at the completely wrong angle.

## Collaboration

I collaborated with Ananya Jajodia on the flipping stunt mechanism. I referred to Aidan McNay on the Flip Stunt, and Jack Long on the Drift Stunt. Additionally, I utilized ChatGPT for generating the graph code for debugging.



