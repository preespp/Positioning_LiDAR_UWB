#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    pkg_share = get_package_share_directory("uwb_lidar")
    config_dir = os.path.join(pkg_share, "config")

    # Static transform: map -> odom
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        namespace=namespace,
    )

    # Static transform: odom -> base_link
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        namespace=namespace,
    )

    # Static transform: base_link -> laser_frame
    tf_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_to_laser',
        arguments=['0', '0', '0.144', '0', '0', '0', 'base_link', 'laser_frame'],
        namespace=namespace,
    )

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        name='rplidar_node',
        parameters=[
            os.path.join(config_dir, 'rplidar_node.yaml')
        ],
    )

    # SLAM toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        parameters=[
            os.path.join(config_dir, 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )
    # central node
    central_node = Node(
        package='uwb_lidar',
        executable='central',
        name='central_node',
        output='screen',
        namespace=namespace,
    )

    # uwb node
    uwb_node = Node(
        package='uwb_lidar',
        executable='uwb',
        name='uwb_node',
        output='screen',
        namespace=namespace,
    )

    # wheel node
    wheel_node = Node(
        package='uwb_lidar',
        executable='wheel',
        name='wheel_node',
        output='screen',
        namespace=namespace,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        tf_map_to_odom,
        tf_odom_to_base,
        tf_link_to_laser,
        rplidar_node,
        slam_node,
        central_node,
        uwb_node,
        wheel_node,
    ])

