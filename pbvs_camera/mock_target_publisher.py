#!/usr/bin/env python3
"""
mock_target_publisher.py — Simulates a camera detecting a target for PBVS testing.

Publishes a geometry_msgs/PoseStamped representing the target object pose
as if detected by a camera. You can move the target around to test PBVS.

Topics published:
    /target_pose  →  geometry_msgs/PoseStamped

Usage:
    python3 mock_target_publisher.py

Controls (keyboard):
    w/s  → target moves +/- X
    a/d  → target moves +/- Y
    q/e  → target moves +/- Z
    r    → reset to default pose
    p    → print current target pose
    Ctrl+C → exit
"""

import math
import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import numpy as np


# ── Default target pose in base_link frame ────────────────────────────────────
DEFAULT_POS  = [0.4, 0.1, 0.5]           # [x, y, z] metres
DEFAULT_RPY  = [0.0, math.pi/2, 0.0]     # [roll, pitch, yaw] radians

# ── Publisher settings ────────────────────────────────────────────────────────
PUBLISH_HZ   = 30.0        # Hz
TARGET_FRAME = 'base_link' # frame the target pose is expressed in


class MockTargetPublisher(Node):
    """
    Publishes a mock target pose for PBVS testing.
    Simulates what a real camera + pose estimator would publish.
    """

    def __init__(self):
        super().__init__('mock_target_publisher')

        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # Current target pose
        self.pos  = list(DEFAULT_POS)
        self.rpy  = list(DEFAULT_RPY)

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info('MockTargetPublisher started ✅')
        self.get_logger().info(
            f'Publishing target at {self.pos} on /target_pose')
        self.get_logger().info(
            'Controls: w/s=X  a/d=Y  q/e=Z  r=reset  p=print  Ctrl+C=exit')

    def set_position(self, x=None, y=None, z=None):
        if x is not None: self.pos[0] = x
        if y is not None: self.pos[1] = y
        if z is not None: self.pos[2] = z

    def set_rpy(self, roll=None, pitch=None, yaw=None):
        if roll  is not None: self.rpy[0] = roll
        if pitch is not None: self.rpy[1] = pitch
        if yaw   is not None: self.rpy[2] = yaw

    def move(self, dx=0.0, dy=0.0, dz=0.0):
        self.pos[0] += dx
        self.pos[1] += dy
        self.pos[2] += dz
        self.get_logger().info(
            f'Target moved to ({self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.3f})')

    def reset(self):
        self.pos = list(DEFAULT_POS)
        self.rpy = list(DEFAULT_RPY)
        self.get_logger().info('Target reset to default pose')

    def _publish(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = TARGET_FRAME

        msg.pose.position.x = self.pos[0]
        msg.pose.position.y = self.pos[1]
        msg.pose.position.z = self.pos[2]

        q = Rotation.from_euler('xyz', self.rpy).as_quat()  # [x,y,z,w]
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub.publish(msg)


def keyboard_thread(node: MockTargetPublisher):
    """Non-blocking keyboard control loop."""
    step = 0.02  # metres per keypress
    while rclpy.ok():
        try:
            key = input().strip().lower()
            if   key == 'w': node.move(dx= step)
            elif key == 's': node.move(dx=-step)
            elif key == 'a': node.move(dy= step)
            elif key == 'd': node.move(dy=-step)
            elif key == 'q': node.move(dz= step)
            elif key == 'e': node.move(dz=-step)
            elif key == 'r': node.reset()
            elif key == 'p':
                print(f'pos=({node.pos[0]:.3f},{node.pos[1]:.3f},{node.pos[2]:.3f})  '
                      f'rpy=({math.degrees(node.rpy[0]):.1f}°,'
                      f'{math.degrees(node.rpy[1]):.1f}°,'
                      f'{math.degrees(node.rpy[2]):.1f}°)')
            else:
                print('Unknown key. Use: w/s/a/d/q/e/r/p')
        except EOFError:
            break


def main(args=None):
    rclpy.init(args=args)
    node = MockTargetPublisher()

    # Start keyboard thread
    kb_thread = threading.Thread(target=keyboard_thread, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
