from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_xacro = LaunchConfiguration('robot_xacro')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )
    robot_xacro_arg = DeclareLaunchArgument(
        'robot_xacro',
        description='Robot xacro file.'
    )

    robot_description_config = Command(['xacro ', robot_xacro, ' use_ros2_control:=', 'True', ' sim_mode:=', 'True'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_xacro_arg,
        robot_state_publisher_node
    ])
