import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('turtlebot_patrol'),
                          'config', 'patrol_params.yaml')

    # turtlebot3_world already has obstacles (the pillars)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'launch', 'turtlebot3_world.launch.py'))
    )

    nodes = [
        Node(package='turtlebot_patrol', executable='lidar_obstacle_detector',
             parameters=[params], output='screen'),
        Node(package='turtlebot_patrol', executable='emergency_stop_server',
             parameters=[params], output='screen'),
        Node(package='turtlebot_patrol', executable='patrol_controller',
             parameters=[params], output='screen'),
        Node(package='turtlebot_patrol', executable='telemetry_logger',
             parameters=[params], output='screen'),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gazebo,
        # wait a few seconds so gazebo and the robot are up before our nodes start
        TimerAction(period=5.0, actions=nodes),
    ])
