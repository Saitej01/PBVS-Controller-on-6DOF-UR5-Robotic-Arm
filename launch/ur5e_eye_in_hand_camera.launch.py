#!/usr/bin/env python3
"""
ur5e_eye_in_hand_camera.launch.py  —  Eye-in-Hand PBVS (Gazebo + MoveIt + Servo)

PREREQUISITE — run this ONCE before launching:
    bash patch_ur_sim_moveit.sh
  This patches ur_sim_control.launch.py in ur_simulation_gz to wrap
  robot_description in ParameterValue(str), fixing:
    "Unable to parse the value of parameter robot_description as yaml"

FULL STACK started here:
  1. ur_sim_moveit  →  Gazebo + UR driver + MoveIt + MoveIt Servo
  2. Camera bridge  →  Gazebo wrist-cam topics → ROS 2
  3. Detector       →  RGB-D → /target_pose
  4. RViz markers + RViz

SECOND TERMINAL (after Gazebo + Servo are fully up ~20s):
  ros2 launch pbvs_camera pbvs_controller.launch.py mode:=eye_in_hand approach_z:=0.10
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type      = LaunchConfiguration('ur_type').perform(context)
    world_file   = LaunchConfiguration('world_file').perform(context)
    target       = LaunchConfiguration('target').perform(context)
    rviz_config  = LaunchConfiguration('rviz_config').perform(context)
    start_gazebo = LaunchConfiguration('start_gazebo').perform(context)
    start_bridge = LaunchConfiguration('start_bridge').perform(context)
    run_detector = LaunchConfiguration('run_detector').perform(context)
    run_markers  = LaunchConfiguration('run_markers').perform(context)
    run_rviz     = LaunchConfiguration('run_rviz').perform(context)

    pbvs_share   = get_package_share_directory('pbvs_camera')
    ur_sim_share = get_package_share_directory('ur_simulation_gz')

    xacro_file = os.path.join(pbvs_share, 'urdf', 'ur_gz_eih_camera.urdf.xacro')

    nodes = []

    if start_gazebo == 'true':
        # ur_sim_moveit = ur_sim_control (Gazebo + UR driver) + ur_moveit (MoveIt + Servo)
        # REQUIRES the upstream patch so ur_sim_control doesn't crash on robot_description.
        ur_sim_moveit_launch = os.path.join(
            ur_sim_share, 'launch', 'ur_sim_moveit.launch.py'
        )
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_moveit_launch),
            launch_arguments={
                'ur_type':          ur_type,
                'world_file':       world_file,
                'description_file': xacro_file,
                'launch_rviz':      'false',
            }.items(),
        ))

    if start_bridge == 'true':
        # Delay 8s — Gazebo needs time before camera topics exist
        nodes.append(TimerAction(period=8.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='pbvs_eih_camera_bridge',
                arguments=[
                    '/camera/camera/color/image_raw'
                        '@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera/color/camera_info'
                        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/camera/camera/aligned_depth_to_color/image_raw'
                        '@sensor_msgs/msg/Image[gz.msgs.Image',
                ],
                output='screen',
            ),
        ]))

    if run_detector == 'true':
        nodes.append(Node(
            package='pbvs_camera',
            executable='camera_color_depth_target_node',
            name='camera_color_depth_target_node',
            arguments=[
                '--target',            target,
                '--image_topic',       '/camera/camera/color/image_raw',
                '--depth_topic',       '/camera/camera/aligned_depth_to_color/image_raw',
                '--camera_info_topic', '/camera/camera/color/camera_info',
                '--output_topic',      '/target_pose',
                '--base_frame',        'base_link',
                '--camera_frame',      'camera_color_optical_frame',
                '--output_frame',      'camera',
                '--offset_z',          '0.00',
                '--debug',
            ],
            output='screen',
        ))

    if run_markers == 'true':
        nodes.append(Node(
            package='pbvs_camera',
            executable='rviz_world_markers',
            name='rviz_world_markers',
            arguments=['--target', target],
            output='screen',
        ))

    if run_rviz == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='pbvs_eih_rviz',
            arguments=['-d', rviz_config],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    pkg_share = FindPackageShare('pbvs_camera')

    default_world = PathJoinSubstitution([
        pkg_share, 'worlds', 'pick_and_place_pbvs_eih.world',
    ])
    default_rviz = PathJoinSubstitution([
        pkg_share, 'rviz', 'camera_pbvs.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type',      default_value='ur5e'),
        DeclareLaunchArgument('world_file',   default_value=default_world),
        DeclareLaunchArgument('rviz_config',  default_value=default_rviz),
        DeclareLaunchArgument('target',       default_value='red_cylinder'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('run_detector', default_value='true'),
        DeclareLaunchArgument('run_markers',  default_value='true'),
        DeclareLaunchArgument('run_rviz',     default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])
