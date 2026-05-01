import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot URDF path
    pkg_share = FindPackageShare('new_mobile_robot').find('new_mobile_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'new_mobile_robot.urdf')
    
    # Read URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['new_mobile_robot', '1.0', '1.0', '0.1', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn
    ])