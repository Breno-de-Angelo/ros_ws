import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_config_file',
            default_value=os.path.join(pkg_path, 'config', 'joystick.yaml'),
            description='Joystick YAML config file',
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[LaunchConfiguration('joystick_config_file'), {'use_sim_time': True}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[LaunchConfiguration('joystick_config_file'), {'use_sim_time': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
        ),
    ])
