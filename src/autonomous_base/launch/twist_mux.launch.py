from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    twist_mux_config_file = LaunchConfiguration('twist_mux_config_file')
    stamped_cmd_vel_out_topic = LaunchConfiguration('stamped_cmd_vel_out_topic')
    unstamped_cmd_vel_out_topic = LaunchConfiguration('unstamped_cmd_vel_out_topic')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )
    twist_mux_config_file_arg = DeclareLaunchArgument(
        'twist_mux_config_file',
        description='Twist mux config YAML file.'
    )
    stamped_cmd_vel_out_topic_arg = DeclareLaunchArgument(
        'stamped_cmd_vel_out_topic',
        description='Name of the output stamped cmd_vel topic.'
    )
    unstamped_cmd_vel_out_topic_arg = DeclareLaunchArgument(
        'unstamped_cmd_vel_out_topic',
        description='Name of the output unstamped cmd_vel topic.'
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_file,
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', stamped_cmd_vel_out_topic)]
    )
    twist_unstamper_node = Node(
        package='twist_stamper',
        executable='twist_unstamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_in', stamped_cmd_vel_out_topic),
                    ('/cmd_vel_out', unstamped_cmd_vel_out_topic)],
    )

    return LaunchDescription([
        use_sim_time_arg,
        twist_mux_config_file_arg,
        stamped_cmd_vel_out_topic_arg,
        unstamped_cmd_vel_out_topic_arg,
        twist_mux_node,
        twist_unstamper_node
    ])
