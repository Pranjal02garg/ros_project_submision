 # 🗺️ SLAM & Nav2
 
## Part 1 — SLAM Mapping with slam_toolbox
 
### What is SLAM?
**Simultaneous Localization and Mapping** — the robot builds a map of an unknown environment while simultaneously tracking its own position within that map. No prior map is needed.
 
### Configuration
 
**Mode:** Online Asynchronous (best for real-time use with a moving robot)
 
Key parameters in `config/slam_params.yaml`:
 
```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    mode: mapping
 
    # Scan matching
    minimum_travel_distance: 0.5      # meters before new keyframe
    minimum_travel_heading: 0.5       # radians before new keyframe
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
 
    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
 
    # Map resolution
    resolution: 0.05                  # 5 cm per cell
    max_laser_range: 10.0
```
 
### Mapping Procedure
 
1. Launch Gazebo + SLAM nodes
2. Teleoperate robot through **all rooms** slowly
3. **Rotate in place** at room centers for 360° coverage
4. Re-visit areas if map shows gaps
5. Save map when full apartment is covered
```bash
ros2 run nav2_map_server map_saver_cli -f maps/apartment_map
```
 
Output:
- `apartment_map.pgm` — grayscale image (white = free, black = occupied, grey = unknown)
- `apartment_map.yaml` — resolution, origin, thresholds
### Map Quality Tips
 
| Tip | Why |
|---|---|
| Move at ≤ 0.15 m/s | Prevents scan distortion |
| Rotate ≤ 0.5 rad/s | Preserves scan continuity |
| Overlap paths | Enables loop closure |
| Stop at doorways | Allows scan to settle |
 
---
 
## Part 2 — Autonomous Navigation with Nav2
 
### Nav2 Stack Overview
 
```
Goal Pose (RViz or script)
        │
        ▼
   BT Navigator
   (behavior tree execution)
        │
        ▼
   NavFn Planner ──────────── Global Costmap
   (global path)                    │
        │                    Static + Obstacle layers
        ▼
   DWB Controller ─────────── Local Costmap
   (local trajectory)              │
        │                    Rolling obstacle layer
        ▼
   /cmd_vel → Robot
```
 
### AMCL — Localization
 
AMCL (Adaptive Monte Carlo Localization) uses a **particle filter** to estimate the robot's pose on the saved map:
 
- Initialized with **2D Pose Estimate** in RViz
- Particles converge around the true pose as the robot moves
- Publishes the `map → odom` TF correction
Key parameters:
```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.25        # minimum movement to update
    update_min_a: 0.2         # minimum rotation to update
    laser_max_range: 10.0
    laser_model_type: likelihood_field
```
 
### NavFn Planner — Global Path
 
Uses **Dijkstra's algorithm** (or A*) on the global costmap to compute a collision-free path from current pose to goal.
 
### DWB Controller — Local Trajectory
 
**Dynamic Window Approach** — samples velocity commands in the robot's dynamic window, simulates trajectories, and selects the best one scoring on:
- Path distance
- Goal distance  
- Obstacle clearance
- Robot kinematics
Key parameters:
```yaml
controller_server:
  ros__parameters:
    FollowPath.max_vel_x: 0.3
    FollowPath.max_vel_theta: 1.0
    FollowPath.min_turning_radius: 0.0    # diff drive = 0
    FollowPath.PathDistBias: 32.0
    FollowPath.GoalDistBias: 24.0
    FollowPath.ObstacleScalingFactor: 0.1
```
 
### Costmap Inflation
 
Obstacle cells are inflated by a configurable radius to keep the robot away from walls. With robot radius ≈ 0.2 m:
- `inflation_radius: 0.35 m` — safe clearance
- `cost_scaling_factor: 3.0` — exponential decay from obstacle
### Waypoint Navigation Script
 
`scripts/waypoint_navigator.py` sends a sequence of `NavigateToPose` action goals:
 
```python
waypoints = [
    (1.5, 0.5, 0.0),     # Hallway
    (3.0, 2.0, 1.57),    # Room 1 center
    (-1.0, 2.5, 3.14),   # Room 2 center
    (0.0, 0.0, 0.0),     # Return home
]
```
 
Each goal is sent only after the previous one succeeds (feedback monitored via action client).
 
---
 
## Common Issues & Fixes
 
| Issue | Cause | Fix |
|---|---|---|
| Robot gets stuck in doorway | Inflation radius too large | Reduce `inflation_radius` |
| AMCL doesn't converge | Bad initial pose | Use RViz 2D Pose Estimate |
| Path planner fails | No path through costmap | Check map quality near goal |
| Robot oscillates | DWB gains too aggressive | Reduce `PathDistBias` |
| SLAM map has gaps | Robot moved too fast | Remap at lower speed |
