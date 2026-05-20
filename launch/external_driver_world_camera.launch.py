#!/usr/bin/env python3
"""
Launch for your required setup:
  1) UR robot is started separately by ur_robot_driver with use_mock_hardware:=true.
  2) This launch starts only the Gazebo world/table/objects/camera, camera bridge,
     camera TF, RGB-D detector, RViz markers, and RViz.

Use this with:
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 use_mock_hardware:=true launch_rviz:=true
  ros2 launch pbvs_camera external_driver_world_camera.launch.py target:=red_cylinder
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pbvs_camera'), 'launch', 'camera_world_bridge_rviz.launch.py'
                ])
            )
        )
    ])
