from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joystick_config_file = LaunchConfiguration('joystick_config_file')
    cmd_vel_out_topic = LaunchConfiguration('cmd_vel_output_topic')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    )
    joystick_config_file_arg = DeclareLaunchArgument(
        'joystick_config_file',
        description='Joystick config file.'
    )
    cmd_vel_out_topic_arg = DeclareLaunchArgument(
        'cmd_vel_output_topic',
        description='Name of the output cmd_vel topic.',
        default_value='/cmd_vel_joy'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joystick_config_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            joystick_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/cmd_vel', cmd_vel_out_topic)]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joystick_config_file_arg,
        cmd_vel_out_topic_arg,
        joy_node,
        teleop_node
    ])
