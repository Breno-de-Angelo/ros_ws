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
    Command,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    # Robot State Publisher
    xacro_file = PathJoinSubstitution([pkg_path, 'description', 'diff_bot', 'robot.urdf.xacro'])
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'True', ' sim_mode:=', 'True'])

    # RViz
    rviz_config_file = PathJoinSubstitution([pkg_path, 'rviz', 'fixed_robot.rviz'])

    # SLAM-Toolbox
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
        ),
        
        # Gazebo
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'gazebo.launch.py'])
        ),

        # ROS2 Control
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad']
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),

        # Joystick Teleop
        DeclareLaunchArgument(
            'joystick_config_file',
            default_value=os.path.join(pkg_path, 'config', 'joystick.yaml'),
            description='Joystick YAML config file',
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[LaunchConfiguration('joystick_config_file'), {'use_sim_time': True}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[LaunchConfiguration('joystick_config_file'), {'use_sim_time': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
        ),

        # Twist MUX
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

        # Robot Localization - choose one of the following: EKF, UKF
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

        # Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        ),
    ])
