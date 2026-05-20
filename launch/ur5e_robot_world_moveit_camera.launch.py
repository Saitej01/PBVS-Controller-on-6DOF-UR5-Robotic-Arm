#!/usr/bin/env python3
"""
ur5e_robot_world_moveit_camera.launch.py

Same as ur5e_robot_world_camera.launch.py, but uses ur_sim_moveit.launch.py
so Gazebo + MoveIt are launched together by ur_simulation_gz.

If this launch works on your system, you do not need a separate MoveIt terminal.
If Servo is not started by the UR simulation MoveIt launch, start your existing
ur_moveit_config Servo terminal separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pbvs_camera')

    ur_type = LaunchConfiguration('ur_type')
    world_file = LaunchConfiguration('world_file')
    rviz_config = LaunchConfiguration('rviz_config')
    target = LaunchConfiguration('target')

    start_robot_moveit = LaunchConfiguration('start_robot_moveit')
    start_bridge = LaunchConfiguration('start_bridge')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    run_detector = LaunchConfiguration('run_detector')
    run_markers = LaunchConfiguration('run_markers')
    run_rviz = LaunchConfiguration('run_rviz')

    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'pick_and_place_pbvs.world'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'camera_pbvs.rviz'])

    ur_gz_moveit_launch = PathJoinSubstitution([
        FindPackageShare('ur_simulation_gz'),
        'launch',
        'ur_sim_moveit.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('world_file', default_value=default_world),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('target', default_value='red_cylinder'),

        DeclareLaunchArgument('start_robot_moveit', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
        DeclareLaunchArgument('run_detector', default_value='true'),
        DeclareLaunchArgument('run_markers', default_value='true'),
        DeclareLaunchArgument('run_rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_gz_moveit_launch),
            launch_arguments={
                'ur_type': ur_type,
                'world_file': world_file,
            }.items(),
            condition=IfCondition(start_robot_moveit),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pbvs_camera_bridge',
            arguments=[
                '/camera/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/camera/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            output='screen',
            condition=IfCondition(start_bridge),
        ),

        Node(
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
            condition=IfCondition(publish_camera_tf),
        ),

        Node(
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
                '--debug',
            ],
            output='screen',
            condition=IfCondition(run_detector),
        ),

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
            name='pbvs_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(run_rviz),
        ),
    ])
