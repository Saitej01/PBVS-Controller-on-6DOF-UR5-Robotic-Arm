#!/usr/bin/env python3
"""
static_gazebo_camera_tf.py

Publishes the static TF for the fixed Gazebo RGB-D camera used by
pick_and_place_pbvs.world.

Important:
  The default values below match the corrected world file in this package:
    camera position = (1.00, -1.05, 1.25)
    look-at point   = (1.00,  0.00, 0.40)

The child frame is a ROS optical frame:
  +Z = camera forward direction
  +X = image right
  +Y = image down
"""

import argparse
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster


class StaticGazeboCameraTF(Node):
    def __init__(self, args):
        super().__init__('static_gazebo_camera_tf')
        self.br = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = args.parent_frame
        t.child_frame_id = args.camera_frame
        t.transform.translation.x = args.x
        t.transform.translation.y = args.y
        t.transform.translation.z = args.z

        if args.look_at:
            q = self._optical_quat_look_at(
                np.array([args.x, args.y, args.z], dtype=float),
                np.array([args.look_x, args.look_y, args.look_z], dtype=float),
            )
            mode = f'look_at=({args.look_x:.3f},{args.look_y:.3f},{args.look_z:.3f})'
        else:
            q = Rotation.from_euler('xyz', [args.roll, args.pitch, args.yaw]).as_quat()
            mode = f'rpy=({args.roll:.3f},{args.pitch:.3f},{args.yaw:.3f})'

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.br.sendTransform(t)

        self.get_logger().info(
            f'Static TF published: {args.parent_frame} -> {args.camera_frame} '
            f'xyz=({args.x:.3f},{args.y:.3f},{args.z:.3f}) {mode}'
        )

    @staticmethod
    def _optical_quat_look_at(position: np.ndarray, target: np.ndarray):
        forward = target - position
        norm = np.linalg.norm(forward)
        if norm < 1e-9:
            raise ValueError('Camera position and look-at target are identical')
        z_axis = forward / norm  # optical +Z = forward

        # Choose image right so the camera is stable and not rolled strangely.
        world_up = np.array([0.0, 0.0, 1.0])
        x_axis = np.cross(z_axis, world_up)
        if np.linalg.norm(x_axis) < 1e-9:
            x_axis = np.array([1.0, 0.0, 0.0])
        else:
            x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)  # optical +Y = image down
        y_axis = y_axis / np.linalg.norm(y_axis)

        rot = np.column_stack([x_axis, y_axis, z_axis])
        return Rotation.from_matrix(rot).as_quat()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--parent_frame', default='base_link')
    p.add_argument('--camera_frame', default='camera_color_optical_frame')
    p.add_argument('--x', type=float, default=1.00)
    p.add_argument('--y', type=float, default=-1.05)
    p.add_argument('--z', type=float, default=1.25)
    p.add_argument('--roll', type=float, default=0.0)
    p.add_argument('--pitch', type=float, default=0.0)
    p.add_argument('--yaw', type=float, default=0.0)
    p.add_argument('--look_at', action='store_true', default=True)
    p.add_argument('--no_look_at', dest='look_at', action='store_false')
    p.add_argument('--look_x', type=float, default=1.00)
    p.add_argument('--look_y', type=float, default=0.00)
    p.add_argument('--look_z', type=float, default=0.40)
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = StaticGazeboCameraTF(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
