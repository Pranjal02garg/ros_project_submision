# 🏛️ System Architecture
 
## Overview
 
The project follows the standard **ROS2 node-topic graph** architecture, where each functional unit is an isolated node communicating via typed topics, services, and actions.
 
---
 
## Node Graph
 
```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                             │
│                                                                 │
│   [Gazebo LiDAR Plugin] ──────────────► /scan  (LaserScan)     │
│   [Gazebo Diff Drive Plugin] ─────────► /odom  (Odometry)      │
│   [robot_state_publisher] ────────────► /tf    (TF2)           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       MAPPING LAYER                             │
│                                                                 │
│   /scan + /odom + /tf ──► [slam_toolbox] ──► /map (OccGrid)   │
│                                          ──► /map_metadata     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    LOCALIZATION LAYER                           │
│                                                                 │
│   /map + /scan ──► [AMCL] ──► /amcl_pose (PoseWithCovariance) │
│                           ──► /particlecloud                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PLANNING LAYER                               │
│                                                                 │
│   /amcl_pose + /map ──► [NavFn Planner] ──► /plan (Path)      │
│   /plan + /scan     ──► [DWB Controller] ──► /cmd_vel_nav      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     CONTROL LAYER                               │
│                                                                 │
│   /cmd_vel_nav ──► [PID Controller] ──► /cmd_vel               │
│   [Teleop Node] ──────────────────────► /cmd_vel               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ACTUATION LAYER                              │
│                                                                 │
│   /cmd_vel ──► [Gazebo diff_drive plugin] ──► wheel motion     │
└─────────────────────────────────────────────────────────────────┘
```
 
---
 
## TF Tree
 
```
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── left_wheel
           ├── right_wheel
           ├── front_left_caster
           ├── front_right_caster
           └── lidar_link
```
 
- **map → odom**: Published by AMCL (localization correction)
- **odom → base_link**: Published by Gazebo diff_drive plugin
- **base_link → lidar_link**: Static transform from URDF
---
 
## Key ROS2 Topics
 
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR plugin | slam_toolbox, Nav2 costmap |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff_drive | slam_toolbox, AMCL |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | Nav2 costmap, RViz |
| `/cmd_vel` | `geometry_msgs/Twist` | PID node / teleop | Gazebo diff_drive |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | Nav2 BT Navigator |
| `/plan` | `nav_msgs/Path` | NavFn planner | DWB controller, RViz |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | All nodes |
 
---
 
## Launch File Dependency Chain
 
```
all_in_one.launch.py
 ├── gazebo.launch.py          → spawns world + robot
 ├── robot_state_publisher     → URDF → TF
 ├── slam.launch.py            → slam_toolbox node
 │     OR
 ├── nav2.launch.py            → full Nav2 bringup
 │    ├── map_server
 │    ├── amcl
 │    ├── planner_server (NavFn)
 │    ├── controller_server (DWB)
 │    ├── bt_navigator
 │    └── lifecycle_manager
 └── rviz2                     → visualization
```
 
---
 
## Costmap Configuration
 
Two costmaps run simultaneously in Nav2:
 
**Global Costmap** — used for path planning
- Layer: Static map layer (from `/map`)
- Layer: Obstacle layer (from `/scan`)
- Inflation radius: 0.3 m
