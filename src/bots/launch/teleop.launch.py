import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'bots'
    pkg_path = get_package_share_directory(pkg_name)

    # Gazebo
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Joystick Teleop
    pkg_teleop = get_package_share_directory('teleop_twist_joy')
    teleop_launch_path = PathJoinSubstitution([pkg_teleop, 'launch', 'teleop-launch.py'])

    return LaunchDescription([
        
        # Gazebo
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_path, 'worlds', 'diff_drive.sdf'),
            description='SDF world file',
        ),
        DeclareLaunchArgument(
            'ros_gz_bridge_config_file',
            default_value=os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml'),
            description='ROS_GZ bridge YAML config file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ['-r ', LaunchConfiguration('world')],
                'on_exit_shutdown': 'True',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                {'config_file': LaunchConfiguration('ros_gz_bridge_config_file')}
            ]
        ),

        # Joystick Teleop
        DeclareLaunchArgument(
            'joystick_config_file',
            default_value=os.path.join(pkg_path, 'config', 'joystick.yaml'),
            description='Joystick YAML config file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_path),
            launch_arguments={
                'publish_stamped_twist': 'True',
                'config_filepath': LaunchConfiguration('joystick_config_file'),
            }.items(),
        ),
        Node(
            package='twist_stamper',
            executable='twist_unstamper',
            remappings=[('/cmd_vel_in', '/cmd_vel'),
                        ('/cmd_vel_out', '/cmd_vel_unstamped')],
        ),
    ])
