+++
title = "Lab 9: Mapping"
date = 2026-04-14
weight = 4
[taxonomies]
tags = ["Robotics", "C++", "Sensors", "Python", "Embedded Software", "Microcontroller" ]
+++

## Initial Hardware and Software Challenges

At the start of this lab, I encountered a cascade of system failures lingering from Lab 8. First, after redownloading my IMU library to fix a dependency issue, the DMP (Digital Motion Processor) was accidentally disabled, leaving me without reliable yaw tracking. Concurrently, an accidental reassignment of the ToF sensor's XSHUT pin in my configuration file prevented the sensors from booting entirely. Because troubleshooting these hardware and software bugs consumed a significant portion of my lab time, I collaborated with Ananya Jajodia and utilized her robot to complete a portion of the data collection for this lab.

## Orientation Control Implementation

To begin mapping, I implemented a positional PID controller utilizing the IMU's DMP. By calculating the error between our target angle and current yaw, the robot could snap to specific orientations.

Orientation control proved highly effective for reliable on-axis turns. By commanding the robot through a series of $90^\circ$ turns and driving straight, I tested its odometry by mapping a 2x3 feet rectangular path. The results below show a clean, stable path with minimal rotational drift, validating that the positional PID was well-tuned.

\<img src="/Fast Robots Media/Lab 9/Rectangle\_Test.png" alt="2x3 Rectangle Test" style="display:block;"\>
\<figcaption\>2x3 Feet Rectangle On-Axis Turn Test\</figcaption\>

### Angular Speed Control (Alternative)

While positional control worked well, I also wrote a continuous angular speed controller to compare mapping methodologies. This logic utilized a low-pass filter on the Gyro's Z-axis to maintain a constant rotational velocity (e.g., $45^\circ/s$). Due to time constraints recovering from the initial hardware bugs, I did not fully implement or tune this on the physical robot, but the architecture is shown below.

```cpp
void runSpeedPID(float target_speed) {
    float current_speed = get_gyro_z_lpf(); 
    float e = target_speed - current_speed;
    
    error_total = constrain(error_total + e, -500, 500);
    
    float p_term = Kp_spd * e;
    float i_term = Ki_spd * error_total;
    int motor_out = (int)(p_term + i_term);
    
    if (motor_out > 0) {
        motor_out = constrain(motor_out, 120, 255);
        setMotors(motor_out, -motor_out);
    } else {
        motor_out = constrain(motor_out, -255, -120);
        setMotors(-motor_out, motor_out);
    }
}
```

## Distance Data Collection

To construct the map, I wrote two distinct methods for distance data collection to see which yielded better results.

### Method 1: The Quick Scan ($360^\circ$)

The first approach was a fast, single-rotation scan using a single ToF sensor. The robot used the positional PID to step in $12^\circ$ increments, rotating a total of $360^\circ$ (30 steps). To prioritize speed, I implemented a tight $\pm 3^\circ$ error tolerance. Once the robot entered this window, it immediately recorded the distance and moved to the next setpoint without explicitly killing the motors.

```cpp
// --- QUICK SCAN 360 LOGIC ---
while (rot_counter <= 30 && BLE.central().connected()) {
    readIMUFIFO();
    get_roll_pitch_yaw(0); 
    float curr_yaw = yaw_readings[0];

    float e = curr_yaw - target_turn;
    if (e > 180.0) e -= 360.0;
    else if (e < -180.0) e += 360.0;

    // 3-degree error tolerance for fast scanning
    if (abs(e) > 3.0) {
        // Run Positional PID to reach target
        int motor_out = runPID(e); 
        setMotors(motor_out, -motor_out);
    } else {
        // Inside tolerance: immediately sample and move on
        if (distanceSensor1.checkForDataReady()) {
            tof_data[rot_counter] = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
        }
        yaw_data[rot_counter] = curr_yaw;
        
        // Increment target for the next step without fully stopping
        target_turn += 12.0;
        if (target_turn > 180.0) target_turn -= 360.0;
        
        rot_counter++;
    }
}
stopMotors();
```

While fast, this method occasionally captured noisy data because the chassis was still actively vibrating when the ToF reading was triggered.

\<img src="/Fast Robots Media/Lab 9/Old\_Polar.png" alt="Single Rotation Polar Scan" style="display:block;"\>
\<figcaption\>Polar Plot of the Quick Scan Data\</figcaption\>

### Method 2: The Dual-Sensor $720^\circ$ Sweep

To maximize accuracy, I wrote a much more robust data collection method that continuously logged PID telemetry at $20\text{Hz}$ while recording discrete map points every two seconds over a full $720^\circ$ rotation (two complete sweeps).

The logic runs the positional PID to turn the robot in $12^\circ$ increments. Every 2 seconds, the robot checks the status of *both* ToF sensors (Front and Side) and saves their distances alongside the *exact* current yaw from the IMU, rather than the target yaw. This guarantees the Python transformation matrices are mathematically perfect regardless of slight overshoots.

```cpp
// --- 2. RECORD CONTINUOUS TELEMETRY (Every 50ms / 20Hz) ---
if (current_time - last_cont_time >= 50 && cont_idx < MAX_CONT_SAMPLES) {
    time_cont[cont_idx] = current_time;
    yaw_cont[cont_idx] = curr_yaw;
    motor_cont[cont_idx] = motor_out;
    p_cont[cont_idx] = p_term;
    i_cont[cont_idx] = i_term;
    d_cont[cont_idx] = d_term;
    
    cont_idx++;
    last_cont_time = current_time;
}

// --- 3. RECORD DISCRETE ToF DATA & TURN (Every 2000ms / 2s) ---
if (current_time - last_rot_time >= 2000 && disc_idx < MAX_DISC_SAMPLES) {
    // Check Sensor 1
    if (distanceSensor1.checkForDataReady()) {
        tof1_disc[disc_idx] = distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
    } else { tof1_disc[disc_idx] = -1; }
    
    // Check Sensor 2
    if (distanceSensor2.checkForDataReady()) {
        tof2_disc[disc_idx] = distanceSensor2.getDistance();
        distanceSensor2.clearInterrupt();
    } else { tof2_disc[disc_idx] = -1; }
    
    time_disc[disc_idx] = current_time;
    yaw_disc[disc_idx] = curr_yaw; // Record exact angle!
    disc_idx++;

    // Set up next 12 degree step
    target_turn += 12.0;
    if (target_turn > 180.0) target_turn -= 360.0;
    
    rot_counter++;
    last_rot_time = current_time;
}
```

\<img src="/Fast Robots Media/Lab 9/Continuous\_PID\_Performance.png" alt="720 Sweep PID Performance" style="display:block;"\>
\<figcaption\>Continuous PID Telemetry logged during the 720° Sweep\</figcaption\>

## Data Merging and Final Mapping

Because the robot is equipped with two ToF sensors (Front and Side offset by $-90^\circ$), I executed the full $720^\circ$ rotation at five different locations in the arena. I grouped the wrapped angles and took the `.median()` of the overlapping points to mathematically filter out odometry drift and transient sensor noise.

To convert the raw ToF data into the global arena coordinate system, two transformation matrices were required. First, I accounted for the physical offset of the Front ToF sensor relative to the robot's center of rotation ($30\text{mm}$ along the $\hat{x}$ axis).

\<div id="math-p1"\>\</div\>
\<script\>
document.addEventListener("DOMContentLoaded", function () {
katex.render(String.raw`T_{sensor\_robot} = \begin{bmatrix} 1 &amp; 0 &amp; 30 \\ 0 &amp; 1 &amp; 0 \\ 0 &amp; 0 &amp; 1 \end{bmatrix}`, document.getElementById("math-p1"), {
displayMode: true
});
});
\</script\>

Next, a rotational transformation matrix was applied to convert the robot's local angular yaw coordinate into global $\hat{x}$ and $\hat{y}$ map coordinates.

\<div id="math-rotz"\>\</div\>
\<script\>
document.addEventListener("DOMContentLoaded", function () {
katex.render(String.raw`T_{robot\_world}(\theta) = \begin{bmatrix} \cos\theta &amp; -\sin\theta &amp; robot\_x \\ \sin\theta &amp; \cos\theta &amp; robot\_y \\ 0 &amp; 0 &amp; 1 \end{bmatrix}`, document.getElementById("math-rotz"), {
displayMode: true
});
});
\</script\>

### Hardware Inversions and Angle Correction

When initially applying these matrices, the resulting map was mathematically mirrored across the $y = -x$ diagonal and rotated incorrectly.

<br>
[ ***INSERT INTERMEDIATE GRAPHS HERE*** ]
<br><br>

This distortion occurred due to two physical hardware quirks:

1.  **Z-Axis Inversion:** The IMU is mounted with its Z-axis pointing toward the floor. By the right-hand rule, it reads positive values when rotating clockwise. However, standard trigonometry (`np.cos` and `np.sin`) strictly expects counter-clockwise angles to be positive. Feeding clockwise data into counter-clockwise math inverted the map inside-out.
2.  **Starting Orientation:** The robot began its scans facing the $+X$ axis ($0^\circ$), but the base transformation matrix assumed it started facing the $+Y$ axis ($90^\circ$).

To fix this globally without manually tweaking offsets for every CSV file, I inverted the raw yaw data in Python and locked the target starting angle to $0^\circ$. Because the universe was flipped to fix the math, I also had to invert the Side sensor's physical offset (from $-90^\circ$ to $+90^\circ$) to prevent it from projecting backward.

```python
# 1. Global Hardware Fix (Z-axis points down)
df['Yaw_deg'] = -df['Yaw_deg']

# 2. Lock starting angle to +X axis (0 degrees)
initial_yaw = df['Yaw_deg'].iloc[0]
yaw_bias = 0.0 - initial_yaw
df['Biased_Yaw_deg'] = df['Yaw_deg'] + yaw_bias

# 3. Apply inverted offset for Sensor 2
base_tof2_offset = 90.0 
```

By applying these hardware corrections, the dual-sensor $720^\circ$ data projected perfectly into global coordinates, yielding an incredibly dense and accurate room map. Finally, I overlaid the estimated wall segments directly onto the scatter plot to visualize the bounds of the arena.

\<img src="/Fast Robots Media/Lab 9/Final\_Wall\_Map.png" alt="Final Arena Map" style="display:block;"\>
\<figcaption\>Final Static Room Map with Filtered Dual-Sensor Data and Estimated Wall Segments\</figcaption\>

## Collaboration

I collaborated extensively on this project with Ananya Jajodia to overcome initial hardware setbacks, and with Jack Long and Trevor Dales. AI tools were utilized to assist in formatting Python plotting scripts and organizing graph subplots.