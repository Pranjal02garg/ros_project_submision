# 🤖 Autonomous Indoor Mobile Robot
### ROS2 · Gazebo · SLAM · Nav2 · PID Control
 
<p align="center">
  <img src="assets/demo_nav2.gif" alt="Nav2 Autonomous Navigation Demo" width="800"/>
</p>
<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros" />
  <img src="https://img.shields.io/badge/Gazebo-Classic_11-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python" />
  <img src="https://img.shields.io/badge/Ubuntu-22.04-red?style=for-the-badge&logo=ubuntu" />
  <img src="https://img.shields.io/badge/SLAM-slam__toolbox-green?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Nav2-Enabled-blueviolet?style=for-the-badge" />
</p>
---
 
## 📌 Overview
 
A fully simulated **autonomous indoor mobile robot** built from scratch using **ROS2 Humble**, featuring a custom URDF differential-drive platform, a custom Gazebo apartment environment, real-time 2D SLAM mapping, and fully autonomous waypoint navigation via Nav2.
 
The robot can:
- **Teleoperate** through an apartment using keyboard controls
- **Build a map** of an unknown environment using SLAM Toolbox
- **Navigate autonomously** to any goal using Nav2 (AMCL + NavFn + DWB)
- **Follow PID-controlled motion** for smooth and accurate movement
> **Audience:** Academic portfolio · Robotics internships · ROS2 project reference
 
---
 
## ✨ Project Highlights
 
| Feature | Detail |
|---|---|
| 🏗️ Robot Model | 4-wheel differential drive, URDF + Gazebo plugins |
| 🗺️ Environment | Custom apartment world — rooms, furniture, walls |
| 📡 Sensing | 2D LiDAR (360°, simulated via `ray` sensor) |
| 🧭 SLAM | `slam_toolbox` — online async mapping |
| 🚀 Navigation | Nav2 full stack — AMCL, NavFn planner, DWB controller |
| 🎮 Teleoperation | Custom safe arrow-key teleop + speed control |
| ⚙️ Control | Custom PID controller node for velocity regulation |
| 💾 Map Export | Saves `.pgm` + `.yaml` occupancy grid |
| 🖥️ Visualization | Full RViz config — TF tree, robot model, LaserScan, SLAM map |
 
---
 
## 🏛️ System Architecture
 
```
┌──────────────────────────────────────────────────────────┐
│                      ROS2 Node Graph                     │
│                                                          │
│  [Keyboard Input]                                        │
│       │                                                  │
│       ▼                                                  │
│  [Teleop Node] ──► /cmd_vel ──► [PID Controller]        │
│                                       │                  │
│                                       ▼                  │
│                               [diff_drive plugin]        │
│                                       │                  │
│                          ┌────────────┴─────────────┐    │
│                          ▼                          ▼    │
│                    /odom topic              /scan topic  │
│                          │                          │    │
│               ┌──────────┘               ┌──────────┘    │
│               ▼                          ▼               │
│          [slam_toolbox] ──────────► /map topic          │
│               │                                          │
│               ▼                                          │
│          [Nav2 Stack]                                    │
│     AMCL → NavFn → DWB → /cmd_vel                       │
│                                                          │
│          [RViz2 Visualization]                           │
└──────────────────────────────────────────────────────────┘
```
 
---
## 🤖 Robot Model
 
<p align="center">
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/rviz.jpeg" alt="Robot in RViz with TF" width="48%"/>
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/robot_in_gazebo.jpeg" alt="Robot Model Close-up" width="48%"/>
 <p align="center"><em>RViz view with TF frames (left) · Robot model close-up in Gazebo (right)</em></p>
 <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/cad_model.png" alt="Robot CAD Model in Solidworks" width="48%"/>
</p>
<p align="center"><em>Robot CAD Model in Solidworks</em></p>

### 🔹 1. Robot Parameter Selection

The robot was designed specifically for an **indoor navigation application**, such as a cleaning or service robot.

Key design considerations:
- Compact size for navigating narrow doorways  
- Stable 4-wheel base for smooth motion  
- Differential drive for simple control  
- Moderate speed for safe indoor operation  

---

### 🔹 2. 3D Design and Structure

The robot consists of:
- Rectangular base chassis  
- Four wheels (2 active + 2 passive)  
- Top-mounted LiDAR sensor  

The design ensures:
- Low center of gravity  
- Balanced weight distribution  
- Clear sensor field of view  

---

### 🔹 3. URDF Modelling & Visualization

The robot was modeled using **URDF (Unified Robot Description Format)** and integrated with ROS2.

Implemented features:
- Links and joints for all components  
- Inertial properties for stable physics simulation  
- Differential drive plugin for motion  
- LiDAR sensor plugin for perception  

Visualization tools:
- **Gazebo** → physics simulation and movement  
- **RViz2** → TF tree, robot model, sensor data  

---
 
 
## 🌍 Gazebo Environment
 
<p align="center">
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/Gazebo_Apartment_Top_view.jpeg" alt="Gazebo Apartment Top View" width="48%"/>
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/Gazebo%20Apartment%20Perspective.jpeg" alt="Gazebo Apartment Perspective" width="48%"/>
</p>
<p align="center"><em>Custom apartment world — top view (left) · Robot navigating inside the custom Gazebo environment (right)</em></p>


### 🔹 4. Custom Environment Design

A **custom indoor apartment environment** was designed in Gazebo to simulate a real-world application scenario for the robot.

Design objectives:
- Represent a realistic indoor navigation environment  
- Include multiple rooms and narrow doorways  
- Introduce obstacles to test SLAM and navigation  

---

### 🔹4.1 Environment Features

The world was created using a `.world` (SDF) file and includes:

- Multi-room layout with interconnected spaces  
- Interior walls with doorway gaps (~0.9 m width)  
- Central open area for navigation transitions  
- Furniture objects:
  - Sofa, tables, chairs  
  - Cabinets and shelves  
  - Decorative elements (plants, objects)  

---

### 🔹4.2 Simulation Characteristics

- Flat ground plane for stable robot motion  
- Static obstacles for mapping and path planning  
- Realistic spacing to challenge navigation algorithms  
- Compatible with Gazebo physics engine  

---

### 🔹4.3 Purpose of Environment

This environment was specifically designed to:

- Test **teleoperation control** in confined spaces  
- Enable **SLAM mapping across multiple rooms**  
- Evaluate **Nav2 path planning through doorways and obstacles**  

---

## 🎮 Teleoperation (Manual Control)

<p align="center">
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/e8bc1ec7ff789af00180633a8ecb35b4c2c6ac4b/media/vs_code_structure.jpeg" width="70%"/>
</p>
<p align="center"><em>Manual robot control using keyboard teleoperation</em></p>

### 🔹 5. Teleoperation Implementation

Teleoperation was implemented to manually control the robot inside the Gazebo environment.  
This step was critical for:

- Verifying robot motion and kinematics  
- Testing `/cmd_vel` communication  
- Performing SLAM mapping exploration  

---

### 🔹5.1 Method Used

A custom Python-based teleoperation script was developed:

python3 scripts/arrow_teleop_safe.py

### 🔹 5.2 Control Mapping

| Key | Action |
|-----|--------|
| ↑ | Move Forward |
| ↓ | Move Backward |
| ← | Rotate Left |
| → | Rotate Right |
| SPACE | Stop |
| q | Quit |

---

### 🔹5.3 Observations

- Real-time response with negligible delay (< 50 ms)  
- Smooth forward and rotational motion  
- Stable skid-steer turning behavior  
- Accurate control in narrow indoor spaces  

---

### 🔹5.4 Role in Project

Teleoperation was essential for:

- Driving the robot during **SLAM mapping**  
- Verifying correct motion control via `/cmd_vel`  
- Testing system integration before autonomous navigation
  
## ⚙️ PID Controller

<p align="center">
  <a href="https://youtu.be/X8UEGggXbnY">
    <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/2122ef2c34be912104483fe8f5b410c2e11c479a/media/assetspid_thumbnail.png" width="60%"/>
  </a>
</p>
<p align="center"><em>PID Controller Demonstration — Click to watch video</em></p>

### 🔹 6. PID Control Implementation

A custom **PID (Proportional–Integral–Derivative) controller** was implemented as a ROS2 node to regulate robot motion using feedback from odometry.

The controller ensures:
- Smooth velocity control  
- Reduced oscillations  
- Accurate trajectory execution  

---

### 🔹6.1 Mathematical Model

The PID control law is defined as:
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)

Where:
- `e(t)` → error between desired and actual velocity  
- `Kp` → proportional gain  
- `Ki` → integral gain  
- `Kd` → derivative gain  

---

### 🔹6.2 Implementation Details

- Node: `pid_controller.py`  
- Subscribes to: `/cmd_vel`  
- Uses feedback from: `/odom`  
- Publishes corrected velocity commands  

Control loops:
- Linear velocity control  
- Angular velocity control  

---

### 🔹6.3 Gain Selection

The gains were tuned experimentally for stable indoor navigation:

- `Kp = 1.2` → responsive correction  
- `Ki = 0.01` → minimal steady-state error  
- `Kd = 0.05` → damping to reduce oscillations  

> Note: In some scenarios, `Ki` was kept very small to prevent integral windup.

---

### 🔹 6.4 Observations

- Faster convergence to desired velocity  
- Reduced overshoot compared to P-only control  
- Stable motion in straight-line travel  
- Improved turning accuracy  

---
 

> 📌 Note: While Nav2 handles high-level navigation, this PID controller demonstrates low-level control understanding.
> See [PID Controller](docs/PID_Controller.md) for full derivation and tuning notes.
 
---

## 🗺️ SLAM Mapping
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/92bd4eace07ca4a2e4d194157f3f28fad5a0dd02/media/slam_before_tracing.jpeg" alt="SLAM Mapping in RViz" width="60%"/>
</p>
<p align="center"><em>Real-time occupancy grid map being built during teleoperation</em></p>
<p align="center">
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/slam_before_tracing.jpeg" alt="Final SLAM Map" width="45%"/>
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/slam_model%20_closeup.jpeg" alt="SLAM Map 3D View" width="45%"/>
</p>
<p align="center"><em>Final saved occupancy grid map (left) · 3D RViz map overlay (right)</em></p>

### 🔹 7.1 SLAM Implementation

Simultaneous Localization and Mapping (SLAM) was implemented using the **`slam_toolbox`** package in ROS2.

This allows the robot to:
- Build a map of an unknown environment  
- Estimate its position simultaneously  
- Correct drift using scan matching  

---

### 🔹7.2 Inputs and Outputs

| Input | Source |
|------|--------|
| `/scan` | LiDAR sensor |
| `/odom` | Differential drive plugin |
| `/tf` | Robot state publisher |

| Output | Description |
|--------|------------|
| `/map` | Occupancy grid map |
| `/tf (map → odom)` | Localization correction |


---

### 🔹7.3 Map Generation

- Robot was manually driven using teleoperation  
- All rooms were explored to ensure complete coverage  
- Loop closure corrected accumulated drift  

Final output:
- `.pgm` → map image  
- `.yaml` → metadata  

ros2 run nav2_map_server map_saver_cli -f maps/apartment_map


---



## 🚀 Autonomous Navigation (Nav2)
 
<p align="center">
  <img src="https://github.com/Pranjal02garg/ros_project_submision/blob/35e55252b3b09f6b3368ac17cc82e9758f8c5303/media/nav2_trajectory.jpeg" alt="Nav2 Navigation Trajectory" width="60%"/>
</p>
<p align="center"><em>Nav2 planned path and real-time trajectory during autonomous navigation</em></p>

### 🔹 7.4 Nav2 Navigation Stack

Autonomous navigation was implemented using the **ROS2 Navigation Stack (Nav2)**.

The robot is capable of:
- Localizing itself on a known map  
- Planning a path to a target location  
- Avoiding obstacles while moving  

---

### 🔹7.5 Core Components

| Component | Role |
|---|---|
| **AMCL** | Localization using particle filter |
| **Map Server** | Provides saved occupancy grid |
| **NavFn Planner** | Global path planning |
| **DWB Controller** | Local trajectory execution |
| **Costmaps** | Obstacle representation |
| **BT Navigator** | Behavior execution |

---

### 🔹7.6 Navigation Pipeline
---
Saved Map
-->
Map Server (/map)
-->
AMCL Localization
-->
Goal Input (/navigate_to_pose)
-->
Global Planner (NavFn)
-->
Local Controller (DWB)
-->
/cmd_vel
-->
Robot Motion


---

### 🔹 7.7 Path Planning

- Global planner computes shortest collision-free path  
- Local controller adjusts motion in real-time  
- Costmaps inflate obstacles for safety margins  

---

### 🔹7.8 Autonomous Execution

A custom script sends navigation goals:

python3 scripts/waypoint_navigator.py
 
 
 See [SLAM  & nav2 ](docs/SLAM_nav2.md) for full tuning notes.
 
## 📁 Repository Structure
 
```
autonomous_indoor_robot/
│
├── urdf/
│   └── new_mobile_robot.urdf          # Full robot description
│
├── launch/
│   ├── gazebo.launch.py               # Gazebo + robot spawn
│   ├── slam.launch.py                 # SLAM mapping session
│   ├── nav2.launch.py                 # Autonomous navigation
│   ├── sim.launch.py                  # Full simulation
│   └── all_in_one.launch.py           # Complete stack launch
│
├── worlds/
│   └── apartment.world                # Custom Gazebo environment
│
├── config/
│   ├── nav2_params.yaml               # Nav2 full parameter set
│   ├── slam_params.yaml               # SLAM Toolbox config
│   ├── slam_view.rviz                 # RViz SLAM session config
│   └── nav_view.rviz                  # RViz Nav2 session config
│
├── scripts/
│   ├── arrow_teleop_safe.py           # Custom keyboard teleop
│   ├── pid_controller.py              # PID velocity controller
│   └── waypoint_navigator.py          # Autonomous waypoint script
│
├── maps/
│   ├── apartment_map.pgm              # Saved occupancy grid
│   └── apartment_map.yaml             # Map metadata
│
├── assets/                            # README media
│   ├── demo_nav2.gif
│   ├── demo_slam.gif
│   ├── gazebo_world_top.png
│   ├── gazebo_world_perspective.png
│   ├── robot_rviz.png
│   ├── robot_model_closeup.png
│   ├── slam_mapping_rviz.png
│   ├── slam_map_final.png
│   ├── slam_map_3d.png
│   ├── nav2_trajectory.png
│   └── vscode_structure.png
│
├── docs/
│   ├── architecture.md
│   ├── pid_controller.md
│   ├── slam_nav2.md
│   └── viva_questions.md
│
├── CMakeLists.txt
├── package.xml
└── README.md
```
 
---
 
## 🛠️ How to Run
 
### Prerequisites
```bash
# ROS2 Humble + Gazebo Classic 11 + Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-gazebo-ros-pkgs
```
 
### 1. Clone & Build
```bash
git clone https://github.com/YOUR_USERNAME/autonomous_indoor_robot.git
cd autonomous_indoor_robot
colcon build --symlink-install
source install/setup.bash
```
 
### 2. Launch Gazebo Simulation
```bash
ros2 launch new_mobile_robot gazebo.launch.py
```
 
### 3. SLAM Mapping (Teleoperate to Build Map)
```bash
# Terminal 1 — SLAM
ros2 launch new_mobile_robot slam.launch.py
 
# Terminal 2 — Teleop
python3 scripts/arrow_teleop_safe.py
 
# Terminal 3 — Save map when done
ros2 run nav2_map_server map_saver_cli -f maps/apartment_map
```
 
### 4. Autonomous Navigation
```bash
# Terminal 1 — Nav2 with saved map
ros2 launch new_mobile_robot nav2.launch.py map:=maps/apartment_map.yaml
 
# Terminal 2 — Waypoint navigator (optional)
python3 scripts/waypoint_navigator.py
```
 
### 5. RViz Visualization
```bash
# For SLAM
rviz2 -d config/slam_view.rviz
 
# For Nav2
rviz2 -d config/nav_view.rviz
```
 
---
 
## ⚠️ Known Limitations
 
- Map quality degrades if robot moves too fast during SLAM — use slow speeds
- AMCL requires a good initial pose estimate; use **2D Pose Estimate** in RViz
- DWB local planner may oscillate in narrow doorways — tuning needed
- No 3D obstacle detection (LiDAR is 2D only)
- Simulation runs on Gazebo Classic (EOL Jan 2025); migration to Gazebo Harmonic planned
---
 
## 🔭 Future Scope
 
- [ ] Migrate to **Gazebo Harmonic** (new API)
- [ ] Add **depth camera** (Intel RealSense sim) for 3D obstacle avoidance
- [ ] Implement **explore_lite** for fully autonomous frontier exploration
- [ ] Deploy on **real hardware** (Raspberry Pi 4 + RPLiDAR A1)
- [ ] Add **multi-robot** coordination
- [ ] Integrate **object detection** (YOLOv8) with Nav2 semantic layer
---
 
## 👤 Contributors
 
| Name | Roll Number |
|---|---|
| **Pranjal  Garg** | 102323055 |
| **Shiwang Garg** | 102323053 |
| **Saam Gupta** | 102323056 |
---
 
## 📄 License
 
This project is licensed under the **MIT License**.
 
---
 
<p align="center">
  <i>Built with ROS2 Humble · Gazebo Classic · slam_toolbox · Nav2</i><br/>
  <i>Running on WSL2 Ubuntu 22.04</i>
</p>
