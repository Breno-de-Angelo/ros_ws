import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    return LaunchDescription([
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[os.path.join(pkg_path, 'config', 'twist_mux.yaml'), {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        ),
        Node(
            package='twist_stamper',
            executable='twist_unstamper',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel'),
                        ('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        ),
    ])
