#!/usr/bin/env python3
"""
rviz_world_markers.py

Publishes RViz markers that match the corrected Gazebo PBVS world.

Final setup:
  1. Table x pose = 1.00 m.
  2. Table height = 0.30 m.
  3. Table centre z = 0.15 m.
  4. Table top z = 0.30 m.
  5. Objects are placed on the lowered table.
  6. Drop zone is placed on the lowered table.
  7. Marker colours match the Gazebo world colours.

Coordinate rule:
  table_top_z = table_pose_z + table_height / 2
              = 0.15 + 0.30 / 2
              = 0.30 m

  object_center_z = table_top_z + object_height / 2
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401

from scipy.spatial.transform import Rotation as R


BASE_FRAME = 'base_link'
PUBLISH_HZ = 10.0


TABLE = {
    'pose': (1.00, 0.00, 0.15, 0.0, 0.0, 0.0),
    'scale': (1.20, 0.80, 0.30),
    'color': (0.55, 0.36, 0.18, 0.55),
}


OBJECTS = {
    'red_cylinder': {
        'pose': (0.65, 0.16, 0.40, 0.0, 0.0, 0.0),
        'shape': Marker.CYLINDER,
        'scale': (0.07, 0.07, 0.20),
        'color': (0.95, 0.05, 0.05, 1.0),
        'label': 'red_cylinder',
    },

    'mustard': {
        'pose': (0.95, 0.13, 0.38, 0.0, 0.0, 0.0),
        'shape': Marker.CYLINDER,
        'scale': (0.06, 0.06, 0.16),
        'color': (1.00, 0.82, 0.05, 1.0),
        'label': 'mustard',
    },

    'cheezit': {
        'pose': (1.28, 0.18, 0.405, 0.0, 0.0, 0.25),
        'shape': Marker.CUBE,
        'scale': (0.16, 0.06, 0.21),
        'color': (1.00, 0.42, 0.05, 1.0),
        'label': 'cheezit',
    },

    'cardboard_box': {
        'pose': (1.18, -0.18, 0.39, 0.0, 0.0, 0.40),
        'shape': Marker.CUBE,
        'scale': (0.22, 0.16, 0.18),
        'color': (0.58, 0.36, 0.16, 1.0),
        'label': 'cardboard_box',
    },

    'coke_can': {
        'pose': (0.82, -0.20, 0.36, 0.0, 0.0, 0.0),
        'shape': Marker.CYLINDER,
        'scale': (0.066, 0.066, 0.12),
        'color': (0.70, 0.00, 0.00, 1.0),
        'label': 'coke_can',
    },
}


DROP_ZONE = {
    'pose': (0.62, -0.32, 0.306, 0.0, 0.0, 0.0),
    'scale': (0.22, 0.18, 0.012),
    'color': (0.0, 0.9, 0.0, 0.65),
}


class RVizWorldMarkers(Node):
    def __init__(self, target_name: str):
        super().__init__('rviz_world_markers')

        self.target_name = target_name

        # In eye-to-hand mode /target_pose is already in base_link.
        # In eye-in-hand mode /target_pose is in camera_color_optical_frame.
        # The live RViz marker must always be drawn in base_link, otherwise the
        # marker appears to drift whenever the wrist camera moves.
        self._target_pose = None
        self._target_input_frame = ''
        self._target_pose_source = '/target_pose'
        self._last_target_base_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obj_pub = self.create_publisher(
            MarkerArray,
            '/visualization/objects',
            10
        )

        self.tgt_pub = self.create_publisher(
            MarkerArray,
            '/visualization/target',
            10
        )

        self.drop_pub = self.create_publisher(
            MarkerArray,
            '/visualization/drop_zone',
            10
        )

        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self._target_cb,
            10
        )

        # Preferred display in eye-in-hand mode.  The controller publishes this
        # after transforming the camera-frame target into base_link.
        self.create_subscription(
            PoseStamped,
            '/target_pose_base',
            self._target_base_cb,
            10
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self._publish_all
        )

        self.get_logger().info(
            'RViz world markers started: table x = 1.00 m, '
            'table height = 0.30 m, table top z = 0.30 m.'
        )

    def _target_base_cb(self, msg: PoseStamped):
        # Stable base-frame target used by the controller.  Use it preferentially
        # for RViz when the PBVS controller is running.
        pose = msg
        pose.header.frame_id = BASE_FRAME
        self._target_pose = pose
        self._target_input_frame = BASE_FRAME
        self._target_pose_source = '/target_pose_base'
        self._last_target_base_time = self.get_clock().now()

    def _target_cb(self, msg: PoseStamped):
        # If /target_pose_base is fresh, do not overwrite the RViz marker with
        # raw camera-frame /target_pose.
        if self._last_target_base_time is not None:
            age = (self.get_clock().now() - self._last_target_base_time).nanoseconds / 1e9
            if age < 0.5:
                return

        in_frame = msg.header.frame_id or BASE_FRAME
        self._target_input_frame = in_frame
        self._target_pose_source = '/target_pose'

        # If the incoming target is already in base_link, keep it directly.
        if in_frame == BASE_FRAME:
            self._target_pose = msg
            return

        # Eye-in-hand case: transform camera-frame target into base_link before
        # drawing the marker. This fixes the RViz target-marker drift.
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                in_frame,
                Time(),
                RclpyDuration(seconds=0.05),
            )
            try:
                pose_base = tf2_geometry_msgs.do_transform_pose_stamped(msg, tf)
            except AttributeError:
                pose_base = self.tf_buffer.transform(
                    msg, BASE_FRAME, timeout=RclpyDuration(seconds=0.05)
                )
            pose_base.header.frame_id = BASE_FRAME
            self._target_pose = pose_base
        except (TransformException, Exception) as exc:
            self.get_logger().warn(
                f'Cannot transform /target_pose from {in_frame} to {BASE_FRAME}: {exc}',
                throttle_duration_sec=1.0,
            )

    def _quat_from_rpy(self, roll, pitch, yaw):
        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        return q

    def _set_pose(self, marker: Marker, pose):
        x, y, z, roll, pitch, yaw = pose

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        q = self._quat_from_rpy(roll, pitch, yaw)

        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])

    def _publish_all(self):
        self._publish_objects()
        self._publish_target()
        self._publish_drop_zone()

    def _publish_objects(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        mid = 0

        table = Marker()
        table.header.frame_id = BASE_FRAME
        table.header.stamp = now
        table.ns = 'table'
        table.id = mid
        table.type = Marker.CUBE
        table.action = Marker.ADD
        table.lifetime = Duration(sec=1)

        self._set_pose(table, TABLE['pose'])

        table.scale.x = float(TABLE['scale'][0])
        table.scale.y = float(TABLE['scale'][1])
        table.scale.z = float(TABLE['scale'][2])

        r, g, b, a = TABLE['color']
        table.color = ColorRGBA(
            r=float(r),
            g=float(g),
            b=float(b),
            a=float(a)
        )

        array.markers.append(table)
        mid += 1

        for name, obj in OBJECTS.items():
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.header.stamp = now
            m.ns = 'objects'
            m.id = mid
            m.type = obj['shape']
            m.action = Marker.ADD
            m.lifetime = Duration(sec=1)

            self._set_pose(m, obj['pose'])

            m.scale.x = float(obj['scale'][0])
            m.scale.y = float(obj['scale'][1])
            m.scale.z = float(obj['scale'][2])

            r, g, b, a = obj['color']

            if name == self.target_name:
                m.color = ColorRGBA(
                    r=1.0,
                    g=1.0,
                    b=0.0,
                    a=0.95
                )
            else:
                m.color = ColorRGBA(
                    r=float(r),
                    g=float(g),
                    b=float(b),
                    a=float(a)
                )

            array.markers.append(m)
            mid += 1

            t = Marker()
            t.header.frame_id = BASE_FRAME
            t.header.stamp = now
            t.ns = 'labels'
            t.id = mid
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.lifetime = Duration(sec=1)

            x, y, z, *_ = obj['pose']
            _, _, sz = obj['scale']

            t.pose.position.x = float(x)
            t.pose.position.y = float(y)
            t.pose.position.z = float(z + sz / 2.0 + 0.08)
            t.pose.orientation.w = 1.0

            t.scale.z = 0.055
            t.color = ColorRGBA(
                r=1.0,
                g=1.0,
                b=1.0,
                a=1.0
            )
            t.text = obj['label']

            array.markers.append(t)
            mid += 1

        self.obj_pub.publish(array)

    def _publish_target(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        if self._target_pose is not None:
            p = self._target_pose.pose.position
            x, y, z = p.x, p.y, p.z
            if self._target_input_frame and self._target_input_frame != BASE_FRAME:
                label_text = f'LIVE {self._target_pose_source}\n{self._target_input_frame} → {BASE_FRAME}'
            else:
                label_text = f'LIVE {self._target_pose_source}'
        else:
            if self.target_name not in OBJECTS:
                return

            x, y, z, *_ = OBJECTS[self.target_name]['pose']
            z += 0.10
            label_text = f'TARGET\n{self.target_name}'

        sphere = Marker()
        sphere.header.frame_id = BASE_FRAME
        sphere.header.stamp = now
        sphere.ns = 'target'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.lifetime = Duration(sec=1)

        sphere.pose.position.x = float(x)
        sphere.pose.position.y = float(y)
        sphere.pose.position.z = float(z)
        sphere.pose.orientation.w = 1.0

        sphere.scale.x = 0.05
        sphere.scale.y = 0.05
        sphere.scale.z = 0.05

        sphere.color = ColorRGBA(
            r=0.0,
            g=1.0,
            b=1.0,
            a=1.0
        )

        array.markers.append(sphere)

        arrow = Marker()
        arrow.header.frame_id = BASE_FRAME
        arrow.header.stamp = now
        arrow.ns = 'target'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.lifetime = Duration(sec=1)

        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = float(z + 0.18)

        q = R.from_euler(
            'xyz',
            [0.0, math.pi / 2.0, 0.0]
        ).as_quat()

        arrow.pose.orientation.x = float(q[0])
        arrow.pose.orientation.y = float(q[1])
        arrow.pose.orientation.z = float(q[2])
        arrow.pose.orientation.w = float(q[3])

        arrow.scale.x = 0.18
        arrow.scale.y = 0.025
        arrow.scale.z = 0.025

        arrow.color = ColorRGBA(
            r=0.0,
            g=1.0,
            b=1.0,
            a=1.0
        )

        array.markers.append(arrow)

        label = Marker()
        label.header.frame_id = BASE_FRAME
        label.header.stamp = now
        label.ns = 'target'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.lifetime = Duration(sec=1)

        label.pose.position.x = float(x + 0.07)
        label.pose.position.y = float(y)
        label.pose.position.z = float(z + 0.24)
        label.pose.orientation.w = 1.0

        label.scale.z = 0.055
        label.color = ColorRGBA(
            r=0.0,
            g=1.0,
            b=1.0,
            a=1.0
        )
        label.text = label_text

        array.markers.append(label)

        self.tgt_pub.publish(array)

    def _publish_drop_zone(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        x, y, z, roll, pitch, yaw = DROP_ZONE['pose']
        sx, sy, sz = DROP_ZONE['scale']
        r, g, b, a = DROP_ZONE['color']

        box = Marker()
        box.header.frame_id = BASE_FRAME
        box.header.stamp = now
        box.ns = 'drop_zone'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.lifetime = Duration(sec=1)

        self._set_pose(box, DROP_ZONE['pose'])

        box.scale.x = float(sx)
        box.scale.y = float(sy)
        box.scale.z = float(sz)

        box.color = ColorRGBA(
            r=float(r),
            g=float(g),
            b=float(b),
            a=float(a)
        )

        array.markers.append(box)

        label = Marker()
        label.header.frame_id = BASE_FRAME
        label.header.stamp = now
        label.ns = 'drop_zone'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.lifetime = Duration(sec=1)

        label.pose.position.x = float(x)
        label.pose.position.y = float(y)
        label.pose.position.z = float(z + 0.10)
        label.pose.orientation.w = 1.0

        label.scale.z = 0.055
        label.color = ColorRGBA(
            r=0.1,
            g=1.0,
            b=0.1,
            a=1.0
        )
        label.text = 'DROP ZONE'

        array.markers.append(label)

        self.drop_pub.publish(array)


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--target',
        default='red_cylinder',
        choices=list(OBJECTS.keys()),
        help='Target object name to highlight in RViz.'
    )

    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    node = RVizWorldMarkers(target_name=parsed.target)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
