import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    return LaunchDescription([
        # Choose one of the following: EKF, UKF

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml'), {'use_sim_time': True}]
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ukf_node',
        #     name='ukf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(pkg_path, 'config', 'ukf.yaml'), {'use_sim_time': True}]
        # ),
    ])
