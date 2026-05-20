#!/usr/bin/env python3
"""
static_eih_camera_tf.py

Publishes tool0 -> camera_color_optical_frame for eye-in-hand testing.
This is only the TF. True simulated eye-in-hand also needs the camera sensor
attached to the robot wrist in the Gazebo robot description.
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class StaticEyeInHandCameraTF(Node):
    def __init__(self, args):
        super().__init__('static_eih_camera_tf')
        self.br = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = args.parent_frame
        t.child_frame_id = args.camera_frame

        t.transform.translation.x = float(args.x)
        t.transform.translation.y = float(args.y)
        t.transform.translation.z = float(args.z)

        q = R.from_euler('xyz', [args.roll, args.pitch, args.yaw]).as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.br.sendTransform(t)
        self.get_logger().info(
            f'Static eye-in-hand TF published: {args.parent_frame} -> {args.camera_frame} '
            f'xyz=({args.x:.3f},{args.y:.3f},{args.z:.3f}) '
            f'rpy=({args.roll:.3f},{args.pitch:.3f},{args.yaw:.3f})'
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent_frame', default='tool0')
    parser.add_argument('--camera_frame', default='camera_color_optical_frame')
    parser.add_argument('--x', type=float, default=0.01)
    parser.add_argument('--y', type=float, default=0.00)
    parser.add_argument('--z', type=float, default=0.01)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parsed, remaining = parser.parse_known_args(args)

    rclpy.init(args=remaining)
    node = StaticEyeInHandCameraTF(parsed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
