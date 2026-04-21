+++
title = "Lab 10: Grid Localization using Bayes Filter"
date = 2026-04-20
weight = 5
[taxonomies]
tags = ["Robotics", "Python", "Sensors", "Bayes Filter", "Algorithms"]
+++

## Overview
The robot doesn't know where it is on its own, so it has to figure out its position from ToF sensor readings — this is localization, and probability is the right tool for the job. In this lab I implemented a Bayes filter, which keeps a running "belief" about where the robot thinks it is. Every time fresh sensor data or a control input comes in, the filter updates that belief through Bayesian inference. The goal was to get everything working in simulation before putting it on real hardware.

## Grid Localization Parameters
The robot's state is 3D: (x,y,θ)(x, y, \theta)
(x,y,θ). The arena spans roughly -5.5 to 6.5 ft in xx
x, -4.5 to 4.5 ft in yy
y, and [−180°,+180°)[-180°, +180°)
[−180°,+180°) in heading. Since we can't compute over a continuous space, the arena is chopped into a 3D grid of (12,9,18)(12, 9, 18)
(12,9,18) cells, each 0.3048 m×0.3048 m×20°0.3048 \text{ m} \times 0.3048 \text{ m} \times 20°
0.3048 m×0.3048 m×20°. Every cell holds a probability of the robot being there, and the whole grid sums to 1. After each update, the cell with the highest probability is our best guess at the robot's pose.


## Bayes Filter Architecture
The filter runs in a loop with two stages. The prediction step uses control inputs to forecast where the robot moved, and the update step corrects that guess by comparing expected sensor readings against the real ones.
<div style="width: fit-content; margin: 0 auto; text-align: left; background-color: #f4f4f4; padding: 15px; border-radius: 5px;">
Algorithm Bayes_Filter(bel(xt−1),ut,zt)\text{Bayes\_Filter}(bel(x_{t-1}), u_t, z_t)
Bayes_Filter(bel(xt−1​),ut​,zt​)
    for all xtx_t
xt​
do&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;bel‾(xt)=∑xt−1p(xt∣ut,xt−1) bel(xt−1)\overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \, bel(x_{t-1})
bel(xt​)=∑xt−1​​p(xt​∣ut​,xt−1​)bel(xt−1​)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;bel(xt)=η p(zt∣xt) bel‾(xt)bel(x_t) = \eta \, p(z_t \mid x_t) \, \overline{bel}(x_t)
bel(xt​)=ηp(zt​∣xt​)bel(xt​)&nbsp;&nbsp;&nbsp;&nbsp
end for
    return bel(xt)bel(x_t)
bel(xt​)
</div>

## Odometry Motion Model
The control input uu
u is broken into three pieces — an initial rotation, a straight-line translation, and a final rotation — which together describe any transition between two poses.

<figure>
<img src="odometry.jpg" alt="Odometry Model Parameters" style="display:block; width:100%; max-width:600px; margin: 0 auto;">
<figcaption>Odometry model parameters: $\delta_{rot1}$, $\delta_{trans}$, and $\delta_{rot2}$.</figcaption>
</figure>
δrot1=atan2⁡(yˉ′−yˉ, xˉ′−xˉ)−θˉ\delta_{rot1} = \operatorname{atan2}(\bar{y}' - \bar{y},\, \bar{x}' - \bar{x}) - \bar{\theta}δrot1​=atan2(yˉ​′−yˉ​,xˉ′−xˉ)−θˉ
δtrans=(xˉ′−xˉ)2+(yˉ′−yˉ)2\delta_{trans} = \sqrt{(\bar{x}' - \bar{x})^2 + (\bar{y}' - \bar{y})^2}δtrans​=(xˉ′−xˉ)2+(yˉ​′−yˉ​)2​
δrot2=θˉ′−θˉ−δrot1\delta_{rot2} = \bar{\theta}' - \bar{\theta} - \delta_{rot1}δrot2​=θˉ′−θˉ−δrot1​

## Algorithm Implementation
The Python snippets below outline my programmatic approach to the Bayes filter. 

### Compute Control & Motion Model
`compute_control()` pulls the three kinematic parameters out of two consecutive poses. odom_motion_model() then evaluates how likely a given transition is by plugging the actual and expected controls into Gaussians.

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

    return p1 * p2 * p3
```

### Prediction Step
This is where computation blows up — a naive implementation loops over roughly 3.8 million transitions per tick. To keep things tractable, I skip any previous cell with belief below 0.00010.0001
0.0001, since those barely contribute to the sum. The tradeoff is a small accuracy hit for a massive speedup. After accumulating the probabilities I normalize so the grid stays a valid distribution.

```python
def prediction_step(cur_odom, prev_odom):
    u = compute_control(cur_odom, prev_odom)
    loc.bel_bar = np.zeros((mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A))

    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for ca in range(mapper.MAX_CELLS_A):
                if loc.bel[cx, cy, ca] > 0.0001:
                    prev_pose = mapper.from_map(cx, cy, ca)
                    for cur_cx in range(mapper.MAX_CELLS_X):
                        for cur_cy in range(mapper.MAX_CELLS_Y):
                            for cur_ca in range(mapper.MAX_CELLS_A):
                                cur_pose = mapper.from_map(cur_cx, cur_cy, cur_ca)
                                prob = odom_motion_model(cur_pose, prev_pose, u)
                                loc.bel_bar[cur_cx, cur_cy, cur_ca] += prob * loc.bel[cx, cy, ca]

    loc.bel_bar /= np.sum(loc.bel_bar)
```

### Sensor Model & Update Step
I vectorized the update step to avoid looping over every cell. By reshaping the raw sensor readings and broadcasting them against the cached mapper.obs_views array, NumPy computes all 18 sensor likelihoods across the entire grid in one shot. This was a huge speedup over the nested-loop version, and it still implements the same Bayesian update:

p(zt∣xt,m)=∏k=118p(ztk∣xt,m)p(z_t \mid x_t, m) = \prod_{k=1}^{18} p(z_t^k \mid x_t, m)p(zt​∣xt​,m)=k=1∏18​p(ztk​∣xt​,m)

```python
def update_step():
    actual_obs = loc.obs_range_data.flatten().reshape(1, 1, 1, mapper.OBS_PER_CELL)
    likelihoods = loc.gaussian(mapper.obs_views, actual_obs, loc.sensor_sigma)
    cell_likelihoods = np.prod(likelihoods, axis=3)

    loc.bel = cell_likelihoods * loc.bel_bar
    loc.bel /= np.sum(loc.bel)
```

## Simulation & Results
The video below shows the filter localizing along a pre-planned rectangular path. Red is the raw odometry estimate, green is ground truth, and blue is the Bayes filter's belief. You can see the odometry drifts badly on its own, but the filter pulls the estimate back toward ground truth as sensor data comes in. The white cells behind the robot visualize the belief grid — brighter means higher probability, and I'm ignoring anything below 0.0001.

<iframe width="450" height="315" src="https://youtu.be/DMEheQDtAwY" allowfullscreen></iframe>
<figcaption>Bayes simulation tracking ground truth vs. odometry.</figcaption>

One thing I noticed: the filter works noticeably better near walls. ToF sensors are more stable at short ranges, so readings there carry more useful information. In the open middle of the arena there aren't many nearby features to lock onto, and the belief gets a bit fuzzier. Still, across the full trajectory the Bayes estimate consistently tracked ground truth far better than odometry alone.

## Collaboration
I referenced Lucca Correia's site while debugging the filter and putting together the video demo.