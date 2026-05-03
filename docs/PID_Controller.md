# ⚙️ PID Controller
 
## Purpose
 
The custom PID controller node (`pid_controller.py`) sits between the high-level velocity commands (from Nav2 or teleop) and the low-level Gazebo differential drive plugin. It smooths velocity transitions and prevents sudden jerks that would degrade SLAM map quality and Nav2 trajectory tracking.
 
---
 
## Control Law
 
For each control axis (linear x and angular z):
 
```
e(t)     = v_desired(t) − v_actual(t)
 
u(t)     = Kp · e(t)
         + Ki · ∫₀ᵗ e(τ) dτ
         + Kd · de(t)/dt
```
 
Where:
- `v_desired` — commanded velocity from `/cmd_vel_raw`
- `v_actual` — measured velocity from `/odom`
- `u(t)` — corrected output published to `/cmd_vel`
---
 
## Gains (Tuned Values)
 
| Parameter | Linear (x) | Angular (z) |
|---|---|---|
| **Kp** | 1.2 | 1.5 |
| **Ki** | 0.01 | 0.02 |
| **Kd** | 0.05 | 0.03 |
| **Output clamp** | ±0.5 m/s | ±1.5 rad/s |
| **Integral clamp** | ±0.2 | ±0.3 |
 
---
 
## Implementation Notes
 
### Anti-Windup
The integral term is clamped to prevent accumulation when the robot is at a physical limit (e.g., against a wall). Without anti-windup, the integral winds up and causes overshoot when the obstruction clears.
 
```python
self.integral = max(-self.integral_clamp,
                min(self.integral_clamp, self.integral))
```
 
### Derivative Filtering
Raw derivative on error causes noise spikes from discrete odometry. A low-pass filter is applied:
 
```python
filtered_derivative = 0.8 * prev_derivative + 0.2 * raw_derivative
```
 
### Update Rate
The PID loop runs at **50 Hz** (`timer_period = 0.02s`) — fast enough for smooth control without overloading the system.
 
---
 
## Node I/O
 
| I/O | Topic | Type |
|---|---|---|
| Input | `/cmd_vel_raw` | `geometry_msgs/Twist` |
| Input | `/odom` | `nav_msgs/Odometry` |
| Output | `/cmd_vel` | `geometry_msgs/Twist` |
 
---
 
## Tuning Process
 
1. **Set Ki = Kd = 0**, increase Kp until robot responds quickly without oscillation
2. **Add Kd** to dampen oscillation
3. **Add small Ki** to eliminate steady-state error
4. **Clamp outputs** to robot's physical velocity limits
5. **Test on figure-8 path** — verify no overshoot or oscillation
---
 
## Observed Behavior
 
| Scenario | Without PID | With PID |
|---|---|---|
| Sudden stop command | Wheel slip, overshoot | Smooth deceleration |
| Slow rotation | Stuttering | Smooth arc |
| SLAM mapping quality | Blurry walls | Clean wall lines |
| Nav2 path tracking | Oscillation | Steady tracking |
