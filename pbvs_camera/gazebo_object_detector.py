#!/usr/bin/env python3
"""
gazebo_object_detector.py — Simulated object detector for Gazebo PBVS testing.

Since we know the exact object positions from the world file, this node:
1. Reads object poses directly from Gazebo (via /world/*/model/*/pose topic)
2. Publishes selected object pose as /target_pose

This is the simulation equivalent of realsense_object_detector.py.
Use this BEFORE you have a real camera — zero setup needed.

Objects available (from pick_and_place_pbvs.world):
    red_cylinder   : (0.65, 0.16, 0.40)  ← start here, easiest
    mustard        : (0.95, 0.13, 0.38)
    cheezit        : (1.28, 0.18, 0.405)
    cardboard_box  : (1.18, -0.18, 0.39)
    coke_can       : (0.82, -0.20, 0.36)

Usage:
    # Track red_cylinder (default)
    python3 gazebo_object_detector.py

    # Track mustard bottle
    python3 gazebo_object_detector.py --target mustard

    # Track with 10cm approach offset above object
    python3 gazebo_object_detector.py --target red_cylinder --offset_z 0.10

Topics published:
    /target_pose  → geometry_msgs/PoseStamped (in base_link frame)
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Entity
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import numpy as np


# ── Known object poses from world file ───────────────────────────────────────
# Format: name → (x, y, z, roll, pitch, yaw)
WORLD_OBJECTS = {
    'red_cylinder':  (0.65,  0.16, 0.40,  0.0, 0.0, 0.0),
    'mustard':       (0.95,  0.13, 0.38,  0.0, 0.0, 0.0),
    'cheezit':       (1.28,  0.18, 0.405, 0.0, 0.0, 0.25),
    'cardboard_box': (1.18, -0.18, 0.39,  0.0, 0.0, 0.40),
    'coke_can':      (0.82, -0.20, 0.36,  0.0, 0.0, 0.0),
}

# Drop zone (where to place the object)
DROP_ZONE = (0.62, -0.32, 0.306, 0.0, 0.0, 0.0)

PUBLISH_HZ  = 30.0
BASE_FRAME  = 'base_link'


class GazeboObjectDetector(Node):
    """
    Simulated object detector — publishes known object poses as /target_pose.

    In simulation we don't need real vision — we use ground truth poses
    from the world file. This lets you test the full PBVS control loop
    without a camera.

    When you switch to a real RealSense camera, just replace this node
    with realsense_object_detector.py — the rest of the pipeline stays identical.
    """

    def __init__(self, target_name: str, offset_z: float = 0.10):
        super().__init__('gazebo_object_detector')

        self.target_name = target_name
        self.offset_z    = offset_z   # approach height above object

        if target_name not in WORLD_OBJECTS:
            self.get_logger().error(
                f'Unknown target: {target_name}. '
                f'Available: {list(WORLD_OBJECTS.keys())}')
            raise ValueError(f'Unknown target: {target_name}')

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # Get object pose from world file
        x, y, z, roll, pitch, yaw = WORLD_OBJECTS[target_name]

        # Apply approach offset (move above object for pre-grasp)
        self.target_x = x
        self.target_y = y
        self.target_z = z + offset_z   # approach from above

        # Target orientation: tool pointing straight down (for top grasp)
        # Override with object orientation if you want to match object pose
        q = Rotation.from_euler('xyz', [roll, pitch, yaw + math.pi]).as_quat()
        self.target_qx = q[0]
        self.target_qy = q[1]
        self.target_qz = q[2]
        self.target_qw = q[3]

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info(
            f'GazeboObjectDetector: tracking [{target_name}]')
        self.get_logger().info(
            f'  World pose:    ({x:.3f}, {y:.3f}, {z:.3f})')
        self.get_logger().info(
            f'  Approach pose: ({self.target_x:.3f}, {self.target_y:.3f}, '
            f'{self.target_z:.3f})  [+{offset_z:.2f}m above]')
        self.get_logger().info(
            f'  Publishing on: /target_pose  frame: {BASE_FRAME}')

    def _publish(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        msg.pose.orientation.x = self.target_qx
        msg.pose.orientation.y = self.target_qy
        msg.pose.orientation.z = self.target_qz
        msg.pose.orientation.w = self.target_qw

        self.pose_pub.publish(msg)

    def switch_to_drop_zone(self):
        """Switch target to drop zone (call after object is grasped)."""
        x, y, z, roll, pitch, yaw = DROP_ZONE
        self.target_x = x
        self.target_y = y
        self.target_z = z
        q = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        self.target_qx = q[0]
        self.target_qy = q[1]
        self.target_qz = q[2]
        self.target_qw = q[3]
        self.get_logger().info('Switched target to DROP ZONE ✅')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Gazebo object detector for PBVS')
    parser.add_argument(
        '--target',
        default='red_cylinder',
        choices=list(WORLD_OBJECTS.keys()),
        help='Which object to track (default: red_cylinder)')
    parser.add_argument(
        '--offset_z',
        type=float,
        default=0.10,
        help='Approach height above object in metres (default: 0.10)')
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = GazeboObjectDetector(
        target_name=parsed.target,
        offset_z=parsed.offset_z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
