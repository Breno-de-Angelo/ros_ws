from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config_file = LaunchConfiguration('slam_config_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )
    slam_config_file_arg = DeclareLaunchArgument(
        'slam_config_file',
        description='SLAM config YAML file.'
    )

    slam_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py']),
        launch_arguments={
            'slam_params_file': slam_config_file
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_file_arg,
        slam_launch
    ])
