# Reversing issue on the physical robot — analysis

## 0. Brief understanding
My first suspicion was that a few ArUco frames had been lost. While ArUco is missing, the robot relies only on the motion model, and every step carries some error (which Q is meant to describe). So after a few lost frames, the robot believes it is still on the correct path, but in reality it has drifted some distance away. Because R is very small, when ArUco comes back the estimated position p_robot is pulled almost entirely onto the ArUco measurement, which could make the robot jerk or rotate sharply. However, in Test 1 R was as large as 10, so the filter should rely much more on the motion model, and there should be no large jerk or rotation. Yet the robot still reversed. So this idea is most likely wrong.

My current guess is this. After the third turn, the robot never got within the waypoint's accept_radius, and the time had not yet exceeded the limit computed in wp_progress() (twice the scheduled time to reach the waypoint). By then, the waypoint was already behind the robot, so to reach it the robot had to rotate. After rotating, the time limit may have been exceeded, so wp_id increases by 1 and the robot starts looking for the next waypoint. Because the robot has already turned around, the new waypoint is now behind it. So, from `ds = Inverse(H_eb.H_R) @ dp`
the forward/backward component in the robot frame, ds[0], is likely negative, and ds[2], which lies in [−π, π], will be close to −π or π because the robot has turned around. The linear velocity is `v = v_ref + dv = v_ref + k_s·ds[0] = v_ref + (1/τ_s)·ds[0]`.

In our code τ_s = 1 s, and ds[0] is negative. v_ref is within [0, v_traj], and our v_traj is only 0.1 m/s, which is very small. So in this situation v will very likely be negative, which makes k_n and k_g negative as well. The angular velocity is dw = k_n·ds[1] + k_g·ds[2]. I think ds[1] can be ignored here, so dw ≈ k_g·ds[2]. Since ds[2] is close to −π or π, the resulting angular velocity is saturated at −w_max or +w_max. Therefore, after turning around, the robot body wiggles left and right, and it does not turn back to face the waypoint. Instead, it goes to the next waypoint rear first.

## 1. What we observed

- On the physical robot, the first part of the square path is tracked normally.
- After roughly the **third turn**, the robot suddenly **rotates through a large angle**, then **drives backwards (rear first)** towards the following waypoints and does not turn back.
- While reversing, the body **wiggles left and right**.
- It happened in **every test**, regardless of the Q and R values:

| Test | Q[DOTX,DOTX] | Q[DOTG,DOTG] | g_std (R heading) | n_std, e_std (R position) | Start time | Reversed? |
|---|---|---|---|---|---|---|
| Test 3 | 0.1² | deg2rad(100)² | deg2rad(0.01) | 0.01 |  | Yes |
| Test 2 | 0.01² | deg2rad(10)² | deg2rad(0.01) | 0.01 | 13:59 | Yes |
| Test 2 (changed path) | 0.01² | deg2rad(10)² | deg2rad(0.01) | 0.01 | 14:03 | Yes |
| Test 1 | 0.001² | deg2rad(1)² | deg2rad(10) | **10** | 14:10 | Yes |
---
## 2. Hypothesis 1: ArUco dropout + R too small — unlikely to be the root cause
When ArUco frames are missing, the EKF only runs the motion-model prediction. The real robot drifts because of effects the model does not capture (wheel slip, wheel-radius / wheelbase errors), but the estimate does not know this. Q only describes *how fast our uncertainty grows*; it does not cause the drift. When ArUco comes back and R is very small, the estimate jumps to the measurement in a single step, the controller suddenly sees a large error, and the robot could spin.

**I think it is not the main cause**
In **Test 1**, R was large (heading 10°, position 10), so the filter trusts the motion model much more and the estimate should change smoothly, yet the robot **still reversed**. Changing Q by two orders of magnitude also made no difference. A small R may make things worse, but it is unlikely to be the root cause.


## 3. Hypothesis 2: waypoint hold + gain sign flip ( I think it's the main cause)

### 3.1 Trigger: the reference waits at a waypoint
`wp_id` only advances in `wp_progress()` when **either**:
- the estimated position is within `accept_radius` (0.2 m) of the waypoint, **or**
- time exceeds `latest_allowable = Tp[wp_id-1] + 2 × (Tp[wp_id] − Tp[wp_id-1])` (twice the scheduled segment time, based on the *planned* timetable).

So the reference stays frozen on a waypoint while **both** of these hold: the robot is **outside** the radius **and** the timeout has **not** been reached. Note that with turning arcs, the arc start and end points (0.3 m from each corner) are also waypoints, so they are only about 0.4 m apart.

1. Around the third/fourth corner, the robot passes a waypoint **outside** the 0.2 m radius.
2. The reference stays on that waypoint, which is now **behind** the robot.
3. The robot turns to go back to it. By the time it has turned, the timeout may have passed, so `wp_id` advances, and the **next** waypoint is now behind the robot, which is facing the wrong way.

### 3.2 Why v becomes negative
Pose error in the robot frame: `ds = Inverse(H_eb.H_R) @ dp`, where
`ds[0]` = forward/backward, `ds[1]` = left/right, `ds[2]` = heading difference (wrapped to [−π, π)).

Linear velocity: `v = v_ref + k_s·ds[0]`, with `k_s = 1/τ_s`.

When the target is behind the robot, `ds[0] < 0`, and v becomes negative once `ds[0] < −v_ref·τ_s`. With v_ref = 0.1 m/s and τ_s = 1 s, that is only **10 cm**. A waypoint tens of centimetres behind easily exceeds this, and the velocity clamp allows down to −v_max.

### 3.3 the gain update flips sign
In `laptop.py`, the gains are re-computed every step from the **commanded** velocity `u[0]`:
```python
self.k_n = 2 * u[0] / (self.L ** 2)
self.k_g = u[0] / self.L
```
Once `u[0] < 0`, **both k_n and k_g become negative**. Angular correction: `dw = k_n·ds[1] + k_g·ds[2] ≈ k_g·ds[2]`, since the lateral error is small.

- With **k_g > 0**, the heading error is pulled towards **0°** (front facing forward). This is the normal case.
- With **k_g < 0**, the heading error is pushed **away from 0°** towards **±180°**. At ±180°, wrapping flips the sign of `ds[2]` every time the heading crosses 180°, so `dw` flips sign and pushes it back. **±180° (rear facing forward) becomes a stable point.**
- When v is negative enough, `|k_g·ds[2]|` (e.g. 0.8 × π ≈ 2.5 rad/s) exceeds w_max (30°/s ≈ 0.52 rad/s), so ω chatters between **+w_max and −w_max**. This is the left–right wiggle we saw.
- Rear facing forward → target is behind → `ds[0] < 0` → `v < 0` → `k_g < 0` → stays rear facing forward. **Each condition keeps the other going, so it never recovers.**


