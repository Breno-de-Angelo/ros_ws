from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    autonomous_base_path = get_package_share_directory('autonomous_base')
    robot_path = get_package_share_directory('diff_bot')

    # SLAM-Toolbox
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    return LaunchDescription([

        # Robot State Publisher
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'robot_state_publisher.launch.py']),
            launch_arguments={
                'use_sim_time': 'true',
                'robot_xacro': PathJoinSubstitution([robot_path, 'description', 'diff_bot', 'robot.urdf.xacro']),
            }.items()
        ),
        
        # Gazebo
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'gazebo.launch.py']),
            launch_arguments={
                'ros_gz_bridge_config_file': PathJoinSubstitution([robot_path, 'config', 'ros_gz_bridge.yaml']),
                'robot_spawn_height': '0.5',
                'use_image_bridge': 'true',
            }.items()
        ),

        # ROS2 Control
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'ros2_control.launch.py']),
            launch_arguments={
                'robot_controller': 'diff_cont',
                'joint_broadcaster': 'joint_broad',
            }.items()
        ),

        # Joystick Teleop
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'joystick.launch.py']),
            launch_arguments={
                'use_sim_time': 'true',
                'joystick_config_file': PathJoinSubstitution([robot_path, 'config', 'joystick.yaml']),
            }.items()
        ),

        # Twist MUX
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'twist_mux.launch.py']),
            launch_arguments={
                'use_sim_time': 'true',
                'twist_mux_config_file': PathJoinSubstitution([robot_path, 'config', 'twist_mux.yaml']),
                'stamped_cmd_vel_out_topic': '/diff_cont/cmd_vel',
                'unstamped_cmd_vel_out_topic': '/diff_cont/cmd_vel_unstamped'
            }.items()
        ),

        # Robot Localization
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'fused_odom.launch.py']),
            launch_arguments={
                'use_sim_time': 'true',
                'localization_algorithm': 'ekf',
                'config_file': PathJoinSubstitution([robot_path, 'config', 'ekf.yaml'])
            }.items()
        ),

        # Mapping
        IncludeLaunchDescription(
            PathJoinSubstitution([autonomous_base_path, 'launch', 'slam_toolbox_async_online.launch.py']),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_config_file': PathJoinSubstitution([robot_path, 'config', 'mapper_params_online_async.yaml'])
            }.items()
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([robot_path, 'rviz', 'slam_toolbox.rviz'])],
            parameters=[{'use_sim_time': True}]
        ),
    ])
