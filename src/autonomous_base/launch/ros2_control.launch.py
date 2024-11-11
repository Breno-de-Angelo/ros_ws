from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robot_controller = LaunchConfiguration('robot_controller')
    joint_broadcaster = LaunchConfiguration('joint_broadcaster')

    robot_controller_arg = DeclareLaunchArgument(
        'robot_controller',
        description='Name of the controller used. Must be the same as the ros2_control yaml.'
    )
    joint_broadcaster_arg = DeclareLaunchArgument(
        'joint_broadcaster',
        description='Name of the joint broadcaster used. Must be the same as the ros2_control yaml.'
    )

    robot_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller]
    )
    joint_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[joint_broadcaster]
    )

    return LaunchDescription([
        robot_controller_arg,
        joint_broadcaster_arg,
        robot_controller_node,
        joint_broadcaster_node
    ])
