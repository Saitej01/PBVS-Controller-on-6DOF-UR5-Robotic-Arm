#!/usr/bin/env python3
"""
pbvs_controller.launch.py

Launches the PBVS controller plus the nodes it depends on:
  - static_eih_camera_tf : publishes tool0 → camera_color_optical_frame TF
                           (eye_in_hand only; skipped for eye_to_hand)
  - pbvs_controller      : the main visual servoing controller

MoveIt Servo and the UR driver / Gazebo simulation must already be running.

Arguments
---------
mode         eye_in_hand | eye_to_hand   (default: eye_to_hand)
approach_z   float, metres above target  (default: 0.0)
cam_x        camera offset X from tool0  (default: 0.01)
cam_y        camera offset Y from tool0  (default: 0.00)
cam_z        camera offset Z from tool0  (default: 0.01)
cam_roll     camera roll  from tool0     (default: 0.0)
cam_pitch    camera pitch from tool0     (default: 0.0)
cam_yaw      camera yaw   from tool0     (default: 0.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode      = LaunchConfiguration('mode')
    approach_z = LaunchConfiguration('approach_z')

    cam_x     = LaunchConfiguration('cam_x')
    cam_y     = LaunchConfiguration('cam_y')
    cam_z     = LaunchConfiguration('cam_z')
    cam_roll  = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw   = LaunchConfiguration('cam_yaw')

    # Condition: only publish camera TF in eye_in_hand mode
    is_eih = PythonExpression(["'", mode, "' == 'eye_in_hand'"])

    return LaunchDescription([
        # ── Arguments ───────────────────────────────────────────────────
        DeclareLaunchArgument('mode',       default_value='eye_to_hand'),
        DeclareLaunchArgument('approach_z', default_value='0.0'),
        DeclareLaunchArgument('cam_x',      default_value='0.01'),
        DeclareLaunchArgument('cam_y',      default_value='0.00'),
        DeclareLaunchArgument('cam_z',      default_value='0.01'),
        DeclareLaunchArgument('cam_roll',   default_value='0.0'),
        DeclareLaunchArgument('cam_pitch',  default_value='0.0'),
        DeclareLaunchArgument('cam_yaw',    default_value='0.0'),

        # ── Static TF: tool0 → camera_color_optical_frame ───────────────
        # Only needed for eye_in_hand. If the robot URDF already includes the
        # camera joint (ur_gz_eih_camera.urdf.xacro), this node is redundant
        # but harmless — the URDF joint takes precedence in the TF tree.
        # If you are running WITHOUT the custom URDF (e.g. real robot with
        # only ur_robot_driver), this node is REQUIRED.
        Node(
            package='pbvs_camera',
            executable='static_eih_camera_tf',
            name='static_eih_camera_tf',
            arguments=[
                '--parent_frame', 'tool0',
                '--camera_frame', 'camera_color_optical_frame',
                '--x',     cam_x,
                '--y',     cam_y,
                '--z',     cam_z,
                '--roll',  cam_roll,
                '--pitch', cam_pitch,
                '--yaw',   cam_yaw,
            ],
            output='screen',
            condition=IfCondition(is_eih),
        ),

        # ── PBVS controller ──────────────────────────────────────────────
        Node(
            package='pbvs_camera',
            executable='pbvs_controller',
            name='pbvs_controller',
            arguments=['--mode', mode, '--approach_z', approach_z],
            output='screen',
        ),
    ])
