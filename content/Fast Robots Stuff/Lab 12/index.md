+++
title = "Lab 12: Inverted Pendulum"
date = 2026-05-13
weight = 1
[taxonomies]
tags = ["LQR Controller", "C++", "Sensors", "Embedded Software", "Nonlinear Dynamic", "Kalman Filter"]
+++

# Overview

For this open-ended final lab I drew inspiration from two prior writeups —
[Aravind Ramaswami's wheelie report](https://anunth-r.github.io/Fast-Robots/lab_9000.html),
which uses Kalman state estimation plus pole-placement gains, and
[Stephan Wagner's Lab 12 report](https://fast.synthghost.com/lab-12-inverted-pendulum-control/),
which keeps it simple with a hand-tuned PD on pitch and no observer. With the collaboration with Dyllan Hofflich and Tina Cheng, our solution lands closer to Stephan Wagner's.

The lab broke into three pieces: modeling the car as a pendulum on a cart,
implementing a real-time balance controller at θ = 0, and building a state
machine that launches from horizontal into the nose down orientation so the
controller can take over.

## System Modeling

### Pendulum-on-cart geometry

I modeled the system the same way Aravind's group did — a wheel of mass
$M$ that translates along $x$, with a rigid rod of mass $m$ and moment of
inertia $I$ rigidly attached to its axle. The rod's center of mass sits a
distance $\ell$ above the wheel center. The state vector is

$$
\mathbf{x} = \begin{bmatrix} x \\ \dot x \\ \theta \\ \dot\theta \end{bmatrix},
$$

where $\theta$ is the rod's tilt from vertical.

![Pendulum-on-cart model](pendulum_diagram.jpg)
![Car-Pendulum](car.jpg)

### Lagrangian and equations of motion

Defining the kinetic energy of the wheel as $T_w = \tfrac{1}{2}M\dot x^2$,
the kinetic energy of the rod as

$$
T_p = \tfrac{1}{2}m\left[(\dot x + \ell\dot\theta\cos\theta)^2
                       + (\ell\dot\theta\sin\theta)^2\right]
    + \tfrac{1}{2}I\dot\theta^2,
$$

and the potential energy $V = mg\ell\cos\theta$, the Lagrangian
$\mathcal{L} = T_w + T_p - V$ yields the coupled equations:

$$
(M + m)\,\ddot x + m\ell\,\ddot\theta\cos\theta - m\ell\,\dot\theta^2\sin\theta = u
$$
$$
(I + m\ell^2)\,\ddot\theta + m\ell\,\ddot x\cos\theta - mg\ell\sin\theta = 0
$$

where $u$ is the horizontal force from the motors at the wheel contact.

### Linearization about vertical

Linearizing about $\theta = 0$ with $\sin\theta \approx \theta$,
$\cos\theta \approx 1$, $\dot\theta^2 \approx 0$:

$$
(M+m)\,\ddot x + m\ell\,\ddot\theta = u
$$
$$
(I+m\ell^2)\,\ddot\theta + m\ell\,\ddot x - mg\ell\,\theta = 0
$$

Since I have no good way to sense $x$ on the car (the ToF only measures
distance to walls, which is not the cart position in general), I followed
Aravind's group in dropping the $x$ row and keeping a 2-state model in
$[\theta,\ \dot\theta]$:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B u, \quad
A = \begin{bmatrix} 0 & 1 \\ \alpha_1 & 0 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ \alpha_2 \end{bmatrix}
$$

with $\alpha_1$ and $\alpha_2$ aggregating $g$, $\ell$, $m$, $M$, and $I$.
I matched their values of $\alpha_1 \approx 6.21$ and $\alpha_2 \approx 60$
since my chassis geometry is similar and the constants compress into a
gain scaling that I retune by hand anyway.

## Controller Design

### LQR for principled gain selection

With the linear $(A,B)$ in hand, the standard LQR cost is

$$
J = \int_0^\infty \mathbf{x}^\top Q\,\mathbf{x} + R\,u^2 \, dt,
$$

with $Q \succeq 0$ penalizing state deviation and $R > 0$ penalizing
control effort. The optimal gain is $K = R^{-1} B^\top P$, where $P$
solves the continuous-time algebraic Riccati equation:

$$
A^\top P + P A - P B R^{-1} B^\top P + Q = 0.
$$

I wrote a small Python helper to compute the gains for several
$Q,R$ choices and observe how they trade off responsiveness and effort:

```python
import numpy as np
from scipy.linalg import solve_continuous_are

alpha1, alpha2 = 6.21, 60.0
A = np.array([[0, 1], [alpha1, 0]])
B = np.array([[0], [alpha2]])

Q = np.diag([100.0, 1.0])
R = np.array([[1.0]])

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print("LQR gains [Kp, Kd]:", K.ravel())
```

This produced gains in the same family as Aravind's pole-placement result —
roughly $K_p \approx 0.04$ on $\theta$ and $K_d \approx 0.005$ on
$\dot\theta$ when the input is interpreted as a normalized force.

### Why PD with manually-tuned gains in the end

In practice, the LQR gains were a starting point that needed substantial
scaling to live in the real PWM range (0–255). The control variable in
firmware is signed PWM, not Newtons, so $\alpha_2$ embeds a transmission
factor I never measured rigorously. I followed synthghost's approach and
converged on PD gains by hand, with the LQR ratio of $K_p:K_d$ informing
the starting point:

| Parameter | Value | Why |
|---|---|---|
| $K_p$ | 15.0 | 5° error → 75 PWM, just above motor deadband |
| $K_d$ | 1.0  | Strong rate damping to avoid oscillation near target |
| $max_pwm$ | 255 | Full authority — needed to recover from 20°+ disturbances |
| target pitch | +90° | Endo orientation in my IMU mounting |

The earlier defaults of $K_p = 4$, $K_d = 0.2$ from the reference were too
gentle: a 5° error mapped to only 20 PWM, well below the motor's static-
friction deadband. The wheels physically refused to move for small
corrections, then slammed into action once the error grew enough to break
through — a recipe for limit-cycle oscillation. Tripling $K_p$ moved the
controller above deadband for almost any disturbance.

### Control law in firmware

The final balance law in `runFastLoop()` is a few lines:

```cpp
if (stunt_state == IPEND_HOLD) {
    float pitch_rate_dps = +latest_gx;
    float deviation      = latest_pitch_comp - ipend_cfg.target_pitch_deg;
    float u              = ipend_cfg.Kp * deviation
                         + ipend_cfg.Kd * pitch_rate_dps;
    int   pwm            = (int)constrain(u, -ipend_cfg.max_pwm,
                                              +ipend_cfg.max_pwm);
    setLeftMotor(pwm);
    setRightMotor(pwm);
}
```

Both wheels receive the same signed PWM — no differential, the chassis
only needs longitudinal acceleration to catch a falling pendulum. The Kd
term has the *same sign* as Kp (not the textbook `-Kd*rate` regulator
form) because we want the controller to *add* thrust when the chassis is
accelerating away from vertical. This is algebraically equivalent to the
standard PD law given the convention that error is measured relative to
the upright equilibrium.

## IMU State Estimation: Why I Skipped the Kalman Filter

I initially attempted to follow Aravind's approach and add a Kalman observer
on $[\theta,\ \dot\theta]$ with the accelerometer pitch as the measurement.
The result was unstable: my pitch estimate would drift unpredictably during
fast maneuvers and produce phantom angles that saturated the controller.
After a few sessions of tuning $\Sigma_u$ and $\Sigma_z$ to no avail, I
abandoned KF and used the **complementary filter** that was already working
well in my other labs:

```cpp
// In serviceIMU():
pitch_lpf_state = ALPHA_LPF * pitch_acc + (1 - ALPHA_LPF) * pitch_lpf_state;
pitch_comp_state = (1 - ALPHA_COMP) * (pitch_comp_state + pitch_rate * dt)
                 + ALPHA_COMP * pitch_lpf_state;
```

with `ALPHA_LPF = 0.10` and `ALPHA_COMP = 0.05`. This integrates the gyro
rate for short-timescale tracking and pulls slowly toward the accelerometer
pitch for long-timescale drift correction. For a 50 Hz balance controller
this is essentially as good as a Kalman filter, with zero hyperparameters
that need re-tuning per stunt.

### Axis-swap calibration

A separate issue ate real debugging time: my IMU breakout was mounted
rotated 90° around the chassis z-axis. The firmware computes pitch from
$\arctan_2(a_x, a_z)$, assuming IMU-x is the chassis longitudinal axis.
With my mounting, chassis pitch instead appeared on IMU-y. Verifying this
required walking the car through five positions (flat, nose-down, nose-up,
left side, right side) and noting which accel channel saturated to ±1 g in
each. The fix swaps which axes feed pitch and roll:

```cpp
latest_pitch_acc = atan2(latest_ay, latest_az) * 180.0f / M_PI;
latest_roll_acc  = atan2(latest_ax, latest_az) * 180.0f / M_PI;
float pitch_rate = +latest_gx;
float roll_rate  = +latest_gy;
```

In the new convention endo reads as **+90°**, wheelie as **−90°**, flat as
0, so I set `target_pitch_deg = +90` everywhere.

## Switching Strategies: From My Car to Dyllan's

After days of fighting motor deadband and carpet friction on my own
chassis, I shelved the balance work and pivoted to **path planning** so
I'd have a complete deliverable (own section below). Coming back to the
pendulum with Dyllan and Tina, we tested my PD code on Dyllan's car and
it balanced for a couple of seconds — my controller wasn't the limiting
factor, my chassis was. We then committed to Dyllan's full-LQR
formulation and tuned that together; the results below come from his
hardware running his controller, with both of us at the gain dials.

### Why Dyllan's LQR was a better starting point than my PD

Two concrete reasons, both visible in his
[Lab 12 report](https://spike-h.github.io/fastRobots/lab12.html). First,
**cart velocity is a state**: his 3-state model
$[\dot x,\ \theta,\ \dot\theta]$ keeps $\dot x$ in the loop (estimated
from a lowpass of commanded PWM), preserving the off-diagonal coupling
in the mass matrix — the momentum effect my 2-state model throws away.
Second, **PWM-to-force calibration divides into $K$**: his notebook
divides the LQR gain by a measured $k_f$, producing unit-correct gains
straight from `solve_continuous_are`. Without that step, my LQR output
was dimensionally inconsistent with PWM and I had to abandon it for
hand-tuned PD. His derivation also gave a physical fall-time constant
$\tau \approx 70$ ms — a bandwidth target I never had.

One physical insight from his flip-up attempt: **adding mass at the top**
makes balance easier because higher $I_{b,\text{com}}$ lengthens $\tau$.
We applied that to the final tuning runs — same gains, weights taped to
the top — and it visibly stabilized the pole.

## Endo Launch State Machine

To launch from horizontal into endo I implemented the "rip the rug"
sequence: drive forward briefly, hard brake to plant the front, then
drive backward sharply. The reaction force from the sudden direction
change pitches the chassis forward over the front axle. Once
$|\theta_\text{pitch}| > 30°$, the launch FSM hands off to the balance
controller.

![Launch + balance state machine](state_machine.png)

The FSM uses event-based exits everywhere except the launch phases: the
balance state stays active until either the chassis has lain flat for 5 s
(success) or pitched past ±135° (over-rotated abort). I kept the structure
simple so the FSM has only one "interesting" state — `BALANCE`.

```cpp
struct WheelieConfig {
    int   forward_pwm        = 250;
    int   forward_ms         = 250;
    int   brake_ms           = 100;
    int   reverse_pwm        = 250;
    int   reverse_ms         = 270;
    int   wait_max_ms        = 600;
    float balance_trigger_deg = 30.0f;
    float target_pitch_deg    = +90.0f;
    float Kp = 15.0f;
    float Kd = 1.0f;
    int   max_balance_pwm = 255;
};
```

## Results

### First attempt

The initial attempt used the inherited gains ($K_p = 4$, $K_d = 0.2$). The
chassis would briefly lift toward vertical during the launch, then over-
rotate or stall before reaching the trigger threshold. Even when I cheated
and started the chassis vertical by hand, the controller produced almost no
audible motor response to small disturbances — the gains were entirely
sub-deadband.

<!-- TODO: insert first attempt video -->
<iframe width="560" height="315" src="https://youtube.com/embed/jXx1CmFetPo"
  title="First attempt — gains too low" frameborder="0" allowfullscreen></iframe>

### After tuning the gains

Bumping $K_p$ to 15 and $K_d$ to 1.0 produced the qualitative behavior I
wanted: small disturbances drew immediate motor response, larger ones were
caught before turning into falls, and the chassis stayed within roughly
$\pm 10°$ of vertical when balanced.

<!-- TODO: insert tuned-gains video -->
<iframe width="560" height="315" src="https://www.youtube.com/embed/lLXaZBUH5W"
  title="Tuned gains" frameborder="0" allowfullscreen></iframe>

### PID result — manual placement (Flow A)

My most consistent result is Flow A: place the chassis manually in the
endo position, then engage the controller. The plot below shows pitch and
motor PWM during a balance run, including the response to a few gentle
pokes.

<!-- TODO: insert Flow A pid balance video -->
<iframe width="560" height="315" src="https://www.youtube.com/embed/Z2JP7KJSA-Y"
  title="Flow A — PID balance from manual placement" frameborder="0" allowfullscreen></iframe>

<!-- TODO: insert pitch + PWM plot from Flow A run -->
![Pitch and motor PWM during Flow A balance](TODO_flow_a_plot.png)

### Physical enhancement: launching off carpet

My biggest practical obstacle was surface. The lab carpet absorbs the
impulse during the brake-to-reverse transition — the wheels sink slightly
into the pile, fail to bite, and never generate the sharp reaction torque
to flip the chassis. On a hard surface the launch reaches the trigger
threshold and the controller takes over.

<!-- TODO: insert hard-surface launch video -->
<iframe width="560" height="315" src="https://www.youtube.com/embed/__SURFACE_FIX_ID__"
  title="Endo launch on hard surface" frameborder="0" allowfullscreen></iframe>

## Path Planning Strategy

The task is to navigate 9 waypoints across the mapped arena, from
$(-4, -3)$ to $(0, 0)$ in grid-cell coordinates. I broadly followed
[Lucca Correial's Lab 12 writeup](https://correial.github.io/LuccaFastRobots/),
making four explicit choices:

| Concern | My choice | Why |
|---|---|---|
| Global planning | None — direct turn-go-turn between adjacent waypoints | Waypoints are hand-designed, no walls between them |
| Localization | Full Bayes update (Lab 11) at every waypoint, uniform prior before each scan | No reliable odometry on the real robot |
| Motion primitives | Onboard orientation PID for turns, onboard ToF-linear PID for legs | Both already tuned from Labs 5–6 |
| Architecture | Offboard plans, onboard executes; Python `await`s `DONE` notifications | Decouples Bayes cost from the 50 Hz control loop |

The offboard `step_once()` routine, an `async` method on a `PathPlanner`
class, runs one waypoint at a time:

```python
async def step_once(self):
    # 1. Current pose = argmax of latest belief
    argmax_cell = np.unravel_index(np.argmax(self.loc.bel),
                                   self.loc.bel.shape)
    current_pose = self.loc.mapper.from_map(*argmax_cell)

    # 2. Turn-go-turn to next waypoint
    rot1, trans = self._plan_segment(current_pose, self.target_waypoint)
    await self.robot.turn_to_heading(current_pose[2] + rot1)
    await self.robot.drive_distance(trans)

    # 3. Re-localize: 360° scan + Bayes update
    await self.loc.get_observation_data()  # 18 readings, 20° apart
    self.loc.update_step()

    # 4. Advance if within tolerance
    new_pose = self.loc.mapper.from_map(
        *np.unravel_index(np.argmax(self.loc.bel), self.loc.bel.shape))
    if np.linalg.norm(new_pose[:2] - self.target_waypoint[:2]) < 0.2:
        self._advance_waypoint()
```

`turn_to_heading` sends `Kp|Ki|Kd|setpoint` to the firmware and blocks
on the completion notification:

```python
async def ble_turn_to_yaw_int(target_yaw_int_deg, timeout_s=7.0):
    state['ori_done'] = None
    payload = f'{KP_ORI}|{KI_ORI}|{KD_ORI}|{target_yaw_int_deg}'
    ble.send_command(CMD.START_ORIENT_PID, payload)
    t0 = time.time()
    while state['ori_done'] is None:
        if time.time() - t0 > timeout_s:
            ble.send_command(CMD.STOP_ORIENT_PID, '')
            raise TimeoutError
        await asyncio.sleep(0.02)
    return state['ori_done']
```

The firmware emits `ORI_DONE` once orientation error has stayed within
±1.5° for 400 ms, then brakes hard for 100 ms — the same settle scheme
as the Lab 11 mapping scan, which keeps the 18 localization readings
clean and motionless.

### Timing and reliability

A waypoint takes 8–12 seconds end-to-end, dominated by the 18
turn-and-settle cycles of the localization scan (~5–8 s). The Bayes
update is under 100 ms; I skip the prediction step since
`localization_extras.py` is $O(N^6)$ and reset to a uniform prior
instead. Reliability is limited by cumulative heading drift — ±5° per
turn even with on-axle gyro integration. Localization usually recovers
the *position* estimate, but the *next* turn restarts from the wrong
absolute reference. My best run hits the first 4–5 waypoints before the
heading error grows large enough to point the car at a wall.

<!-- Path planning run video -->
<iframe width="560" height="315" src="https://www.youtube.com/embed/__PATH_PLANNING_ID__"
  title="Path execution — best run" frameborder="0" allowfullscreen></iframe>

## Collaboration

Big thanks to **Dyllan Hofflich** for letting me run my PD code on his car
when mine couldn't generate enough launch impulse, and for the LQR
derivation and tuning sessions we did together — read his
[Lab 12 report](https://spike-h.github.io/fastRobots/lab12.html) for the
full state-space derivation, the Kalman-vs-DMP comparison, and the
top-weight trick. Thanks to **Tina Cheng** for the tuning sessions, and to
**Lucca Correial's**
[Lab 12 sample solution](https://correial.github.io/LuccaFastRobots/) for
the offboard/onboard split pattern I followed for path planning.

## Conclusion

What started as a one-person inverted-pendulum project on a stubborn
front-heavy chassis turned into two parallel deliverables: a path
execution pipeline that hits the first half of the waypoint list, and a
collaborative LQR balance controller that finally stuck — on Dyllan's
car rather than mine. The biggest lessons were practical: a 5-line
complementary filter beats a hand-tuned Kalman on a 50 Hz loop;
controller gains have to clear the motor deadband for the smallest error
you care about; and a higher center of mass actually makes balance
easier by lengthening the fall time constant. Switching to path
planning under time pressure turned out to be the right call — it gave
me a complete second deliverable and the headroom to return to balance
with a teammate's hardware that worked.