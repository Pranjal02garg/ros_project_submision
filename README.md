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
 
## 🌍 Gazebo Environment
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/d5bc4467c62b1adb42656ada9ef455ef78db8f80/media/Gazebo_Apartment_Top_view.jpeg" alt="Gazebo Apartment Top View" width="48%"/>
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/4acdf44386961f832da99d38997b6b5e96aca68c/media/Gazebo%20Apartment%20Perspective.jpeg" alt="Gazebo Apartment Perspective" width="48%"/>
</p>
<p align="center"><em>Custom apartment world — top view (left) · perspective view (right)</em></p>
A hand-crafted `.world` file featuring:
- Multi-room layout with interior walls and doorways
- Furniture: sofa, tables, chairs, shelves, plants
- Realistic obstacle density for SLAM and Nav2 testing
---
 
## 🤖 Robot Model
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/16d007808f9356840d74c2991802b787050d9a30/media/rviz.jpeg" alt="Robot in RViz with TF" width="48%"/>
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/16d007808f9356840d74c2991802b787050d9a30/media/robot_in_gazebo.jpeg" alt="Robot Model Close-up" width="48%"/>
</p>
<p align="center"><em>RViz view with TF frames (left) · Robot model close-up in Gazebo (right)</em></p>
- **4-wheel chassis** — 2 active drive wheels + 2 passive casters
- **LiDAR mount** on top center
- **Gazebo differential drive plugin** for realistic wheel odometry
- Full **TF tree**: `map → odom → base_link → wheels + lidar`
---
 
## 🗺️ SLAM Mapping
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/92bd4eace07ca4a2e4d194157f3f28fad5a0dd02/media/slam_before_tracing.jpeg" alt="SLAM Mapping in RViz" width="60%"/>
</p>
<p align="center"><em>Real-time occupancy grid map being built during teleoperation</em></p>
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/92bd4eace07ca4a2e4d194157f3f28fad5a0dd02/media/slam_after_tracing.jpeg" alt="Final SLAM Map" width="45%"/>
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/92bd4eace07ca4a2e4d194157f3f28fad5a0dd02/media/slam_model%20_closeup.jpeg" alt="SLAM Map 3D View" width="45%"/>
</p>
<p align="center"><em>Final saved occupancy grid map (left) · 3D RViz map overlay (right)</em></p>
- Uses **`slam_toolbox`** in online asynchronous mode
- Robot teleoperated through all rooms to build a complete map
- Map saved as `.pgm` + `.yaml` for Nav2 localization
```bash
# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/apartment_map
```
 
---
 
## 🚀 Autonomous Navigation (Nav2)
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/dab297f323694719df57226cb3f939a4bd55b0b5/media/nav2_trajectory.jpeg" alt="Nav2 Navigation Trajectory" width="60%"/>
</p>
<p align="center"><em>Nav2 planned path and real-time trajectory during autonomous navigation</em></p>
**Nav2 Stack Components:**
 
| Component | Role |
|---|---|
| **AMCL** | Particle filter localization on saved map |
| **NavFn** | Global path planning (Dijkstra/A*) |
| **DWB Controller** | Local trajectory following |
| **Costmap2D** | Obstacle inflation and layered costmaps |
| **BT Navigator** | Behavior tree execution engine |
 
The waypoint script sends sequential goals to Nav2's action server, enabling fully autonomous multi-room navigation.
 
---
 
## ⚙️ PID Controller
 
Custom ROS2 node (`pid_controller.py`) implementing:
 
```
error = desired_velocity - actual_velocity
output = Kp*error + Ki*∫error·dt + Kd*(Δerror/Δt)
```
 
- Separate PID loops for **linear** and **angular** velocity
- Tuned gains: `Kp=1.2, Ki=0.01, Kd=0.05`
- Subscribes to `/cmd_vel`, publishes corrected velocity
- Anti-windup clamping on integral term
> See [PID Controller](docs/system.md#-pid-controller) for full derivation and tuning notes.
 
---
 
## 🖥️ Project Structure in VS Code
 
<p align="center">
  <img src="https://github.com/shiwanggarg/Ros_Project/blob/ea667ab733d3834f9899c73b82384d777309ef72/media/vs_code_structure.jpeg" alt="VS Code Project Structure" width="70%"/>
</p>
<p align="center"><em>ROS2 workspace layout in VS Code with WSL Ubuntu 22.04</em></p>
---
 
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
