import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'autonomous_base'
    pkg_path = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    world = LaunchConfiguration('world')
    ros_gz_bridge_config_file = LaunchConfiguration('ros_gz_bridge_config_file')
    robot_description_topic = LaunchConfiguration('robot_description_topic')
    robot_name = LaunchConfiguration('robot_name')
    robot_spawn_height = LaunchConfiguration('robot_spawn_height')
    image_topic = LaunchConfiguration('image_topics')
    use_image_bridge = LaunchConfiguration('use_image_bridge')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'warehouse.world'),
        description='SDF world file'
    )
    ros_gz_bridge_config_file_arg = DeclareLaunchArgument(
        'ros_gz_bridge_config_file',
        description='ROS_GZ bridge YAML config file'
    )
    robot_description_topic_arg = DeclareLaunchArgument(
        'robot_description_topic',
        default_value='robot_description',
        description='Robot description topic'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Robot name to spawn'
    )
    robot_spawn_height_arg = DeclareLaunchArgument(
        'robot_spawn_height',
        description='Robot height to spawn'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topics',
        default_value='/camera/image_raw',
        description='Image topic for the image bridge node'
    )
    use_image_bridge_arg = DeclareLaunchArgument(
        'use_image_bridge',
        default_value='false',
        description='Whether to launch the image bridge node'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'True',
        }.items(),
    )

    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config_file}]
    )

    image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[image_topic],
        condition=IfCondition(use_image_bridge)
    )

    create_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', robot_description_topic,
            '-name', robot_name,
            'z', robot_spawn_height
        ]
    )

    return LaunchDescription([
        world_arg,
        ros_gz_bridge_config_file_arg,
        robot_description_topic_arg,
        robot_name_arg,
        robot_spawn_height_arg,
        image_topic_arg,
        use_image_bridge_arg,
        gz_sim_launch,
        parameter_bridge_node,
        image_bridge_node,
        create_robot_node
    ])
