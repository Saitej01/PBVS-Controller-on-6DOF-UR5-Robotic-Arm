#!/usr/bin/env python3
"""
camera_visualizer.py — Publishes camera TF + RViz marker for PBVS debugging.

Supports both Eye-in-Hand and Eye-to-Hand camera configurations.

Eye-in-Hand:
    camera_link is a child of tool0 (moves with the end-effector).
    The camera TF is broadcast relative to tool0 with a fixed offset.

Eye-to-Hand:
    camera_link is a child of base_link (fixed in the world).
    The camera TF is broadcast at a fixed pose relative to base_link.

Usage:
    # Eye-to-hand (default) — camera fixed in world
    python3 camera_visualizer.py

    # Eye-in-hand — camera mounted on end-effector
    python3 camera_visualizer.py --mode eye_in_hand

RViz setup:
    Add → TF               (to see the camera_link frame axes)
    Add → MarkerArray      topic: /visualization/camera
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
import numpy as np


# ═══════════════════════════════════════════════════════════════════════════════
# Configuration — edit to match your physical camera mount
# ═══════════════════════════════════════════════════════════════════════════════

MODE_EYE_IN_HAND = 'eye_in_hand'
MODE_EYE_TO_HAND = 'eye_to_hand'

BASE_FRAME   = 'base_link'
EEF_FRAME    = 'tool0'
CAMERA_FRAME = 'camera_link'

# ── Eye-in-Hand offset: camera position relative to tool0 ────────────────────
# Adjust these to match how your camera is physically mounted on the EEF.
# Example: camera is 5cm in front of the tool, 3cm to the side, pointing forward
EIH_OFFSET_X  =  0.05   # metres forward  (+X = forward along tool)
EIH_OFFSET_Y  =  0.00   # metres sideways
EIH_OFFSET_Z  =  0.03   # metres up
EIH_ROLL      =  0.0    # radians — camera roll
EIH_PITCH     = -math.pi/2   # radians — tilt down (e.g. math.pi/6 = 30°)
EIH_YAW       =  0.0    # radians — camera yaw

# ── Eye-to-Hand pose: camera position in base_link frame ─────────────────────
# Adjust these to match where your fixed camera is placed in the world.
# Example: camera is 0.8m to the side, 0.6m up, looking at the workspace
ETH_X         =  -0.2    # metres
ETH_Y         = -0.7    # metres (to the side of the robot)
ETH_Z         =  0.6    # metres (elevated)
ETH_ROLL      =  0.0    # radians
ETH_PITCH     =  math.pi/10    # 30° tilt downward toward workspace
ETH_YAW       =  math.pi/4  # 90° to face the robot workspace

PUBLISH_HZ    = 30.0


# ═══════════════════════════════════════════════════════════════════════════════
# Camera Visualizer Node
# ═══════════════════════════════════════════════════════════════════════════════

class CameraVisualizer(Node):
    """
    Broadcasts camera_link TF and publishes a camera frustum marker in RViz.

    Eye-in-Hand: camera_link is dynamic — parented to tool0, moves with the arm.
                 Uses a regular (dynamic) TransformBroadcaster so it updates
                 every tick as the arm moves.

    Eye-to-Hand: camera_link is static — parented to base_link, never moves.
                 Uses a StaticTransformBroadcaster (latched, only sent once).
    """

    def __init__(self, mode: str = MODE_EYE_TO_HAND):
        super().__init__('camera_visualizer')
        self.mode = mode
        self.get_logger().info(f'CameraVisualizer: mode = {self.mode.upper()}')

        # ── Marker publisher ───────────────────────────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization/camera', 10)

        if self.mode == MODE_EYE_IN_HAND:
            # Dynamic broadcaster — must re-publish every frame
            self.tf_broadcaster = TransformBroadcaster(self)
            self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick_eih)
            self.get_logger().info(
                f'  camera_link → child of {EEF_FRAME} (moves with arm)')
            self.get_logger().info(
                f'  Offset from {EEF_FRAME}: '
                f'x={EIH_OFFSET_X:.3f} y={EIH_OFFSET_Y:.3f} z={EIH_OFFSET_Z:.3f}')
        else:
            # Static broadcaster — publish once, stays latched
            self.static_broadcaster = StaticTransformBroadcaster(self)
            self._broadcast_static_tf()
            self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_marker)
            self.get_logger().info(
                f'  camera_link → child of {BASE_FRAME} (fixed in world)')
            self.get_logger().info(
                f'  Position in {BASE_FRAME}: '
                f'x={ETH_X:.3f} y={ETH_Y:.3f} z={ETH_Z:.3f}')

        self.get_logger().info(
            'RViz: Add → TF (to see camera_link axes)')
        self.get_logger().info(
            'RViz: Add → MarkerArray → /visualization/camera (to see frustum)')

    # ── Eye-in-Hand: dynamic TF + marker every tick ───────────────────────
    def _tick_eih(self):
        self._broadcast_eih_tf()
        self._publish_marker()

    def _broadcast_eih_tf(self):
        """Broadcast camera_link as a child of tool0 (dynamic)."""
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = EEF_FRAME       # parent: end-effector
        t.child_frame_id  = CAMERA_FRAME    # child:  camera

        t.transform.translation.x = EIH_OFFSET_X
        t.transform.translation.y = EIH_OFFSET_Y
        t.transform.translation.z = EIH_OFFSET_Z

        q = Rotation.from_euler('xyz', [EIH_ROLL, EIH_PITCH, EIH_YAW]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # ── Eye-to-Hand: static TF once ───────────────────────────────────────
    def _broadcast_static_tf(self):
        """Broadcast camera_link as a fixed child of base_link (static)."""
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = BASE_FRAME      # parent: robot base
        t.child_frame_id  = CAMERA_FRAME    # child:  camera

        t.transform.translation.x = ETH_X
        t.transform.translation.y = ETH_Y
        t.transform.translation.z = ETH_Z

        q = Rotation.from_euler('xyz', [ETH_ROLL, ETH_PITCH, ETH_YAW]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info('Static camera_link TF broadcast ✅')

    # ── Camera marker (frustum + body) ────────────────────────────────────
    def _publish_marker(self):
        """
        Publish a camera body box + frustum lines in RViz.
        All markers are in camera_link frame so they move correctly
        in both eye-in-hand and eye-to-hand modes.
        """
        now   = self.get_clock().now().to_msg()
        array = MarkerArray()

        # ── Camera body (small grey box) ──────────────────────────────────
        body = Marker()
        body.header.frame_id = CAMERA_FRAME
        body.header.stamp    = now
        body.ns              = 'camera'
        body.id              = 0
        body.type            = Marker.CUBE
        body.action          = Marker.ADD
        body.lifetime        = Duration(sec=1, nanosec=0)
        body.pose.position.x = 0.0
        body.pose.position.y = 0.0
        body.pose.position.z = 0.0
        body.pose.orientation.w = 1.0
        body.scale.x = 0.06   # camera body width
        body.scale.y = 0.04   # camera body height
        body.scale.z = 0.03   # camera body depth
        if self.mode == MODE_EYE_IN_HAND:
            body.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)   # blue = EIH
        else:
            body.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)   # orange = ETH
        array.markers.append(body)

        # ── Camera lens (small cylinder on front face) ─────────────────────
        lens = Marker()
        lens.header.frame_id = CAMERA_FRAME
        lens.header.stamp    = now
        lens.ns              = 'camera'
        lens.id              = 1
        lens.type            = Marker.CYLINDER
        lens.action          = Marker.ADD
        lens.lifetime        = Duration(sec=1, nanosec=0)
        lens.pose.position.x = 0.032   # front of the camera body
        lens.pose.position.y = 0.0
        lens.pose.position.z = 0.0
        # Rotate cylinder to point along X (camera optical axis)
        q_lens = Rotation.from_euler('xyz', [0.0, math.pi/2, 0.0]).as_quat()
        lens.pose.orientation.x = q_lens[0]
        lens.pose.orientation.y = q_lens[1]
        lens.pose.orientation.z = q_lens[2]
        lens.pose.orientation.w = q_lens[3]
        lens.scale.x = 0.02   # diameter
        lens.scale.y = 0.02   # diameter
        lens.scale.z = 0.01   # depth of lens
        lens.color   = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)  # dark grey
        array.markers.append(lens)

        # ── View frustum lines ────────────────────────────────────────────
        # 4 lines from camera origin to the corners of a rectangle in front
        frustum_depth = 0.30   # how far the frustum extends (metres)
        half_w        = 0.12   # half-width  of frustum at depth
        half_h        = 0.09   # half-height of frustum at depth

        corners = [
            ( frustum_depth,  half_w,  half_h),
            ( frustum_depth, -half_w,  half_h),
            ( frustum_depth, -half_w, -half_h),
            ( frustum_depth,  half_w, -half_h),
        ]

        fid = 2
        for i, (fx, fy, fz) in enumerate(corners):
            # Line from origin to corner
            line = Marker()
            line.header.frame_id = CAMERA_FRAME
            line.header.stamp    = now
            line.ns              = 'camera'
            line.id              = fid
            line.type            = Marker.LINE_STRIP
            line.action          = Marker.ADD
            line.lifetime        = Duration(sec=1, nanosec=0)
            line.pose.orientation.w = 1.0
            line.scale.x = 0.003   # line width

            from geometry_msgs.msg import Point
            origin = Point(); origin.x = 0.0; origin.y = 0.0; origin.z = 0.0
            corner = Point(); corner.x = fx;  corner.y = fy;  corner.z = fz
            line.points = [origin, corner]
            line.color  = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.7)  # green-cyan
            array.markers.append(line)
            fid += 1

            # Line connecting corners (the frustum rectangle)
            next_c = corners[(i + 1) % 4]
            rect = Marker()
            rect.header.frame_id = CAMERA_FRAME
            rect.header.stamp    = now
            rect.ns              = 'camera'
            rect.id              = fid
            rect.type            = Marker.LINE_STRIP
            rect.action          = Marker.ADD
            rect.lifetime        = Duration(sec=1, nanosec=0)
            rect.pose.orientation.w = 1.0
            rect.scale.x = 0.003
            c1 = Point(); c1.x = fx;        c1.y = fy;        c1.z = fz
            c2 = Point(); c2.x = next_c[0]; c2.y = next_c[1]; c2.z = next_c[2]
            rect.points = [c1, c2]
            rect.color  = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.7)
            array.markers.append(rect)
            fid += 1

        # ── Label ─────────────────────────────────────────────────────────
        label = Marker()
        label.header.frame_id = CAMERA_FRAME
        label.header.stamp    = now
        label.ns              = 'camera'
        label.id              = fid
        label.type            = Marker.TEXT_VIEW_FACING
        label.action          = Marker.ADD
        label.lifetime        = Duration(sec=1, nanosec=0)
        label.pose.position.x = 0.0
        label.pose.position.y = 0.0
        label.pose.position.z = 0.06
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        mode_str = 'EYE-IN-HAND' if self.mode == MODE_EYE_IN_HAND else 'EYE-TO-HAND'
        label.text = f'camera\n({mode_str})'
        array.markers.append(label)

        self.marker_pub.publish(array)


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    parser = argparse.ArgumentParser(
        description='Camera TF + RViz marker for PBVS (eye-in-hand / eye-to-hand)')
    parser.add_argument(
        '--mode',
        choices=[MODE_EYE_IN_HAND, MODE_EYE_TO_HAND],
        default=MODE_EYE_TO_HAND,
        help='Camera configuration (default: eye_to_hand)')
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = CameraVisualizer(mode=parsed.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
