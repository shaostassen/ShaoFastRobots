+++
title = "Lab 10: Grid Localization using Bayes Filter"
date = 2026-04-20
weight = 5
[taxonomies]
tags = ["Robotics", "Python", "Sensors", "Bayes Filter", "Algorithms"]
+++

## Overview 

Because the robot does not have innate access to its exact "ground truth" coordinates, it must rely on environmental clues gathered by ToF sensors to figure out where it is. This is called localization, and it works best when we use probability. Our objective is to take that imprecise, noisy sensor data and convert it into a trustworthy calculation of the robot's state.

For this lab, I applied a Bayes filter to achieve probabilistic localization. This algorithm holds a continuous "belief" regarding the robot's coordinates. By combining initial positioning, control commands, and incoming ToF measurements, the system guesses its location. The robot constantly refines this belief through Bayesian inference every time fresh data arrives. The primary goal of this assignment is to build and test this filter in a simulated environment prior to hardware deployment.

### Grid Localization Parameters

A 3D coordinate system defines the robot's state: $(x, y, \theta)$. The operational arena spans a continuous area defined by the following boundaries: 

* **x-axis:** -5.5 ft to 6.5 ft
* **y-axis:** -4.5 ft to 4.5 ft
* **Heading (theta):** -180° to 180°

To make the math computable, this continuous arena is divided into a finite 3D grid. Each discrete block measures **0.3048 m by 0.3048 m by 20°**, creating a grid array of (12, 9, 18) cells. A probability value is assigned to every single cell to represent the likelihood of the robot occupying that space, and the entire grid must always sum to exactly 1. As the Bayes filter processes new data, it shifts these probabilities. The specific block containing the maximum probability at any given moment represents our best estimate of the robot's actual pose, and tracking this over time produces the simulated path.

---

## Bayes Filter Architecture

The Bayes filter operates in a continuous loop. It first predicts the robot's subsequent location using movement commands, then corrects that prediction by weighing it against actual sensor feedback. The core structure is as follows:

<div style="width: fit-content; margin: 0 auto; text-align: left; background-color: #f4f4f4; padding: 15px; border-radius: 5px;">

**Algorithm** $\text{Bayes\_Filter} (bel(x_{t-1}), u_t, z_t)$  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**for all** $x_t$ **do** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$\overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \ bel(x_{t-1})$  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$bel(x_t) = \eta \ p(z_t \mid x_t) \ \overline{bel}(x_t)$  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**end for** **return** $bel(x_t)$  

</div>

This logic splits into two primary phases:
1. **Prediction Step:** The system forecasts the robot's new coordinates relying purely on the previous location and the commanded motion.
2. **Update Step:** This acts as a correction mechanism. It evaluates the predicted state against the actual sensor readings to minimize positional uncertainty.

---

### Odometry Motion Model

We represent the movement input $u$ using an Odometry Motion Model, which captures the relative shift between two distinct poses. We can fully describe any transition between two states using three distinct variables: an initial turn, a forward translation, and a finishing turn.

<figure>
<img src="odometry.jpg" alt="Odometry Model Parameters" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Odometry Model Parameters detailing the initial rotation ($\delta_{rot1}$), translation ($\delta_{trans}$), and final rotation ($\delta_{rot2}$).</figcaption>
</figure>

The kinematics for these rotations and translations are calculated using the following formulas:

$$\delta_{rot1} = \text{atan2}(\bar{y}' - \bar{y}, \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{trans} = \sqrt{(\bar{y}' - \bar{y})^2 + (\bar{x}' - \bar{x})^2}$$
$$\delta_{rot2} = \bar{\theta}' - \bar{\theta} - \delta_{rot1}$$

---

## Algorithm Implementation

The Python snippets below outline my programmatic approach to the Bayes filter. I integrated several computational optimizations to improve processing speeds and handle geometrical quirks (like wrapping angles).

### Compute Control & Motion Model

The `compute_control()` function derives the expected movement kinematics from the baseline odometry model. Following that, `odom_motion_model()` evaluates the statistical probability of these movements using Gaussian distributions. It works by checking the deviation between the actual executed control and the theoretically required path.

```python
import math
import numpy as np

def compute_control(cur_pose, prev_pose):
    x_prev, y_prev, yaw_prev = prev_pose
    x_cur, y_cur, yaw_cur = cur_pose

    delta_trans = math.hypot(x_cur - x_prev, y_cur - y_prev)
    delta_rot_1 = math.degrees(math.atan2(y_cur - y_prev, x_cur - x_prev)) - yaw_prev
    delta_rot_1 = mapper.normalize_angle(delta_rot_1)

    delta_rot_2 = yaw_cur - yaw_prev - delta_rot_1
    delta_rot_2 = mapper.normalize_angle(delta_rot_2)

    return delta_rot_1, delta_trans, delta_rot_2

def odom_motion_model(cur_pose, prev_pose, u):
    rot1, trans, rot2 = u
    delta_rot1, delta_trans, delta_rot2 = compute_control(cur_pose, prev_pose)

    p1 = loc.gaussian(delta_rot1, rot1, loc.odom_rot_sigma)
    p2 = loc.gaussian(delta_trans, trans, loc.odom_trans_sigma)
    p3 = loc.gaussian(delta_rot2, rot2, loc.odom_rot_sigma)

    prob = p1 * p2 * p3
    return prob

Prediction Step

During the prediction phase, the algorithm must evaluate every possible transition from a past cell to a current cell across the 3D grid. Simply nesting these loops would result in nearly 3.8 million operations per tick. To circumvent this massive bottleneck, I added an early-exit condition (if loc.bel[cx, cy, ca] > 0.0001:). This forces the script to ignore originating cells that hold virtually zero probability, drastically speeding up execution while preserving the integrity of the math.
Python

def prediction_step(cur_odom, prev_odom):
    u = compute_control(cur_odom, prev_odom)

    loc.bel_bar = np.zeros((mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A))

    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for ca in range(mapper.MAX_CELLS_A):
                
                # Optimization: Skip negligible probabilities
                if loc.bel[cx, cy, ca] > 0.0001:
                    prev_pose = mapper.from_map(cx, cy, ca)
                    
                    for cur_cx in range(mapper.MAX_CELLS_X):
                        for cur_cy in range(mapper.MAX_CELLS_Y):
                            for cur_ca in range(mapper.MAX_CELLS_A):
                                cur_pose = mapper.from_map(cur_cx, cur_cy, cur_ca)
                                
                                prob = odom_motion_model(cur_pose, prev_pose, u)
                                loc.bel_bar[cur_cx, cur_cy, cur_ca] += prob * loc.bel[cx, cy, ca]
                                
    # Normalize to prevent underflow
    loc.bel_bar /= np.sum(loc.bel_bar)

Sensor Model & Update Step

Instead of iterating through every individual cell to apply the sensor updates, I took a fully vectorized approach for update_step(). By flattening and reshaping actual_obs, I leveraged NumPy's broadcasting capabilities against the pre-cached 4D mapper.obs_views matrix. This allows the script to simultaneously calculate the probability likelihoods for all 18 sensor measurements across the entire grid.

This methodology computes the Bayesian update formula below with exceptional efficiency:
p(zt​∣xt​,m)=k=1∏18​p(ztk​∣xt​,m)

```python
def sensor_model(obs):
    pass

def update_step():
    # Vectorized update step utilizing NumPy broadcasting
    actual_obs = loc.obs_range_data.flatten().reshape(1,1,1, mapper.OBS_PER_CELL)
    likelihoods = loc.gaussian(mapper.obs_views, actual_obs, loc.sensor_sigma)
    
    cell_likelihoods = np.prod(likelihoods, axis=3)
    
    loc.bel = cell_likelihoods * loc.bel_bar
    loc.bel /= np.sum(loc.bel)
```

## Simulation & Results

The embedded video demonstrates the Bayes filter successfully localizing the robot along a pre-programmed rectangular route. In the visualizer, the pure odometry estimate is shown in red, while the simulator's true ground truth is green. You can clearly observe how quickly the isolated odometry model drifts off course.

Conversely, the probabilistic estimation is marked in blue, and it closely mirrors the actual ground truth. The underlying white grid visually represents the probability distribution itself—brighter white cells indicate higher confidence.

<iframe width="450" height="315" src="https://youtu.be/DMEheQDtAwY" allowfullscreen></iframe>
<figcaption>Bayes Simulation tracking Ground Truth vs Odometry</figcaption>

At the very start of the run, before the ToF sensors provide useful feedback, the robot relies entirely on its odometry. However, once sensor data streams in, the algorithm rapidly corrects the drift. I noted that the filter exhibits much higher accuracy when the robot navigates close to the walls. This is because the ToF sensors produce much more stable, low-variance readings at short ranges. When navigating the center of the arena, the lack of nearby physical landmarks introduces slightly more uncertainty into the belief grid.

## Collaboration
I referred to Lucca Correia's site for the debug the Bayes filter and video demostratation. 