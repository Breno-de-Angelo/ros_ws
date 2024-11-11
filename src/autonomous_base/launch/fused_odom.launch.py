from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_algorithm = LaunchConfiguration('localization_algorithm')
    config_file = LaunchConfiguration('config_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )
    localization_algorithm_arg = DeclareLaunchArgument(
        'localization_algorithm',
        description='YAML config file for the selected method.',
        choices=['ekf', 'ukf']
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='YAML config file for the selected method.',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(PythonExpression(["'", localization_algorithm, "' == 'ekf'"]))
    )

    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(PythonExpression(["'", localization_algorithm, "' == 'ukf'"]))
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_algorithm_arg,
        config_file_arg,
        ekf_node,
        ukf_node
    ])
