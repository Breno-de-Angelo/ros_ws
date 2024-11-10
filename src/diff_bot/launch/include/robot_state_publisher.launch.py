import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    xacro_file = PathJoinSubstitution([pkg_path, 'description', 'diff_bot', 'robot.urdf.xacro'])
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'True', ' sim_mode:=', 'True'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
        ),
    ])
