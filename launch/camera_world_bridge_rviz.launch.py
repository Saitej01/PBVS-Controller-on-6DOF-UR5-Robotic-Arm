#!/usr/bin/env python3
"""
World-only launch: Gazebo world + camera bridge + static camera TF + detector + RViz.

Use this when you are NOT launching UR inside Gazebo from this package.
For the complete robot-in-Gazebo setup, use ur5e_pbvs_gazebo.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera_bridge_node(condition):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pbvs_camera_bridge',
        arguments=[
            '/camera/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/camera/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
        condition=IfCondition(condition),
    )


def _static_camera_tf_node(condition):
    return Node(
        package='pbvs_camera',
        executable='static_gazebo_camera_tf',
        name='static_gazebo_camera_tf',
        arguments=[
            '--parent_frame', 'base_link',
            '--camera_frame', 'camera_color_optical_frame',
            '--x', '1.00', '--y', '-1.05', '--z', '1.25',
            '--look_at', '--look_x', '1.00', '--look_y', '0.00', '--look_z', '0.40',
        ],
        output='screen',
        condition=IfCondition(condition),
    )


def _detector_node(target, condition):
    return Node(
        package='pbvs_camera',
        executable='camera_color_depth_target_node',
        name='camera_color_depth_target_node',
        arguments=[
            '--target', target,
            '--image_topic', '/camera/camera/color/image_raw',
            '--depth_topic', '/camera/camera/aligned_depth_to_color/image_raw',
            '--camera_info_topic', '/camera/camera/color/camera_info',
            '--output_topic', '/target_pose',
            '--base_frame', 'base_link',
            '--camera_frame', 'camera_color_optical_frame',
            '--offset_z', '0.10',
            '--depth_window', '12',
            '--debug',
        ],
        output='screen',
        condition=IfCondition(condition),
    )


def generate_launch_description():
    pkg_share = FindPackageShare('pbvs_camera')

    world_file = LaunchConfiguration('world_file')
    rviz_config = LaunchConfiguration('rviz_config')
    target = LaunchConfiguration('target')
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_bridge = LaunchConfiguration('start_bridge')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    run_detector = LaunchConfiguration('run_detector')
    run_markers = LaunchConfiguration('run_markers')
    run_rviz = LaunchConfiguration('run_rviz')

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])

    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'pick_and_place_pbvs.world'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'camera_pbvs.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=default_world),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('target', default_value='red_cylinder'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
        DeclareLaunchArgument('run_detector', default_value='true'),
        DeclareLaunchArgument('run_markers', default_value='true'),
        DeclareLaunchArgument('run_rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': ['-r ', world_file]}.items(),
            condition=IfCondition(start_gazebo),
        ),

        _camera_bridge_node(start_bridge),
        _static_camera_tf_node(publish_camera_tf),
        _detector_node(target, run_detector),

        Node(
            package='pbvs_camera',
            executable='rviz_world_markers',
            name='rviz_world_markers',
            arguments=['--target', target],
            output='screen',
            condition=IfCondition(run_markers),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='camera_pbvs_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(run_rviz),
        ),
    ])
