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
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_path, 'worlds', 'warehouse.world'),
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
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["/camera/image_raw"]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description',
                       '-name', 'robot',
                       'z', '0.5'],
        ),

    ])
