from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'diff_bot'
    pkg_path = get_package_share_directory(pkg_name)

    # SLAM-Toolbox
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    return LaunchDescription([

        # Robot State Publisher
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'robot_state_publisher.launch.py'])
        ),
        
        # Gazebo
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'gazebo.launch.py'])
        ),

        # ROS2 Control
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'ros2_control.launch.py'])
        ),

        # Joystick Teleop
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'joystick.launch.py'])
        ),

        # Twist MUX
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'twist_mux.launch.py'])
        ),

        # Robot Localization
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_path, 'launch', 'include', 'fused_odom.launch.py'])
        ),

        # Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg_path, 'rviz', 'fixed_robot.rviz'])],
            parameters=[{'use_sim_time': True}]
        ),
    ])
