# Project Workflow

## 1. Robot Selection
A wheeled mobile robot was selected for an indoor cleaning robot application.

## 2. Robot Design
The robot was designed using 3D modelling software and exported to URDF. It includes a base body, four wheels, differential drive configuration, and a 2D LiDAR sensor.

## 3. Gazebo World
A custom apartment world was created with rooms, walls, doorways, and furniture-like obstacles.

## 4. RViz Visualization
RViz was used to visualize the robot model, TF frames, LiDAR scan, SLAM map, and Nav2 navigation map.

## 5. Teleoperation
Keyboard teleoperation was implemented for manual robot movement.

## 6. PID Controller
A PID-based movement controller was implemented using odometry feedback. The robot performs forward movement, backward movement, left turn, forward movement, and right turn.

## 7. SLAM Mapping
SLAM Toolbox was used to create an occupancy grid map using LiDAR scan data.

## 8. Nav2 Navigation
The final saved SLAM map was loaded into Nav2. AMCL was used for localization and Nav2 was used for autonomous navigation.

## 9. Final Output
The project demonstrates robot modelling, simulation, teleop, PID control, SLAM mapping, and autonomous path planning.
