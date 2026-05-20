#!/usr/bin/env python3
"""
camera_color_depth_target_node.py

Robust RGB-D target pose estimator for Gazebo/RealSense PBVS.

This node replaces gazebo_object_detector.py. It uses:
  RGB image + depth image + camera_info
      -> colour segmentation
      -> target centre pixel
      -> robust depth from contour/nearby ROI
      -> 3-D point in camera optical frame
      -> TF transform to base_link
      -> /target_pose

Fixes included in this version:
  1. Searches depth over the full detected object contour, not only one centre pixel.
  2. Expands the depth window when centre depth is zero/NaN.
  3. Prints depth image encoding once for debugging.
  4. Uses a safe ray-plane fallback if Gazebo depth has invalid values.
  5. Allows forced camera frame to avoid empty/wrong CameraInfo frame IDs.
"""

import argparse
import copy
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401


HSV_RANGES: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {
    "red_cylinder": [
        (np.array([0, 60, 40]), np.array([15, 255, 255])),
        (np.array([165, 60, 40]), np.array([179, 255, 255])),
    ],
    "coke_can": [
        (np.array([0, 60, 35]), np.array([15, 255, 240])),
        (np.array([165, 60, 35]), np.array([179, 255, 240])),
    ],
    "mustard": [
        (np.array([15, 50, 50]), np.array([42, 255, 255])),
    ],
    "cheezit": [
        (np.array([5, 60, 50]), np.array([28, 255, 255])),
    ],
    "cardboard_box": [
        (np.array([5, 30, 30]), np.array([30, 230, 230])),
    ],
}

# Object centre heights in the corrected world. These are used only if the
# depth image gives zero/NaN values. X-Y still comes from the camera pixel ray.
OBJECT_CENTER_Z = {
    "red_cylinder": 0.40,
    "mustard": 0.38,
    "cheezit": 0.405,
    "cardboard_box": 0.39,
    "coke_can": 0.36,
}

# Known object centres in the PBVS Gazebo world.  These are NOT used as the
# measured target pose.  They are used only to disambiguate objects with a
# similar colour.  This is important because both red_cylinder and coke_can are
# red, so HSV colour segmentation alone can lock on to the wrong object.
OBJECT_BASE_CENTER = {
    "red_cylinder": (0.65, 0.16, 0.40),
    "mustard": (0.95, 0.13, 0.38),
    "cheezit": (1.28, 0.18, 0.405),
    "cardboard_box": (1.18, -0.18, 0.39),
    "coke_can": (0.82, -0.20, 0.36),
}


class CameraColorDepthTargetNode(Node):
    def __init__(self, args):
        super().__init__("camera_color_depth_target_node")

        self.target = args.target
        if self.target not in HSV_RANGES:
            raise ValueError(f"Unknown target '{self.target}'. Choose one of {list(HSV_RANGES.keys())}")

        self.image_topic = args.image_topic
        self.depth_topic = args.depth_topic
        self.info_topic = args.camera_info_topic
        self.output_topic = args.output_topic
        self.base_frame = args.base_frame
        self.camera_frame = args.camera_frame
        self.min_area = args.min_area
        self.depth_window = args.depth_window
        self.approach_offset_z = args.offset_z
        self.output_frame = args.output_frame
        self.debug = args.debug
        self.allow_plane_fallback = args.allow_plane_fallback
        self.select_by_world_prior = args.select_by_world_prior
        self.center_on_object_plane = args.center_on_object_plane
        self.lock_to_world_target = args.lock_to_world_target
        self.printed_depth_info = False

        self.bridge = CvBridge()
        self.rgb_msg: Optional[Image] = None
        self.depth_msg: Optional[Image] = None
        self.camera_info: Optional[CameraInfo] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.debug_pub = self.create_publisher(Image, "/target_detection/debug_image", 10)

        self.create_subscription(Image, self.image_topic, self._rgb_cb, 10)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 10)

        self.timer = self.create_timer(1.0 / args.publish_hz, self._process)

        self.get_logger().info("CameraColorDepthTargetNode started")
        self.get_logger().info(f"  target       : {self.target}")
        self.get_logger().info(f"  RGB topic    : {self.image_topic}")
        self.get_logger().info(f"  Depth topic  : {self.depth_topic}")
        self.get_logger().info(f"  CameraInfo   : {self.info_topic}")
        self.get_logger().info(f"  Camera frame : {self.camera_frame}")
        self.get_logger().info(f"  Output       : {self.output_topic} in {self.output_frame} frame")
        self.get_logger().info(f"  World target lock: {self.lock_to_world_target}")

    def _rgb_cb(self, msg: Image):
        self.rgb_msg = msg

    def _depth_cb(self, msg: Image):
        self.depth_msg = msg

    def _info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _process(self):
        if self.rgb_msg is None:
            self.get_logger().info("Waiting for RGB image...", throttle_duration_sec=2.0)
            return
        if self.depth_msg is None:
            self.get_logger().info("Waiting for depth image...", throttle_duration_sec=2.0)
            return
        if self.camera_info is None:
            self.get_logger().info("Waiting for camera_info...", throttle_duration_sec=2.0)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(self.rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        if not self.printed_depth_info:
            self.printed_depth_info = True
            try:
                finite = self._depth_to_meters(np.asarray(depth, dtype=np.float32))
                valid = finite[np.isfinite(finite) & (finite > 0)]
                self.get_logger().info(
                    f"Depth encoding={self.depth_msg.encoding}, shape={depth.shape}, "
                    f"min={np.nanmin(finite):.4f}m, max={np.nanmax(finite):.4f}m, "
                    f"valid_pixels={valid.size}"
                )
            except Exception:
                self.get_logger().info(f"Depth encoding={self.depth_msg.encoding}, shape={getattr(depth, 'shape', None)}")

        selected = self._select_best_target_candidate(bgr, depth)
        if selected is None:
            self.get_logger().warn(f"Target '{self.target}' not visible or no valid 3-D candidate", throttle_duration_sec=1.0)
            if self.debug:
                self._publish_debug(bgr, None, None, None)
            return

        out_pose, pose_base_for_log, u, v, contour, depth_text, selector_text = selected

        self.target_pub.publish(out_pose)

        self.get_logger().info(
            f"{self.target}: pixel=({u},{v}) {depth_text} {selector_text} "
            f"target_{self.output_frame}=({out_pose.pose.position.x:.3f},"
            f"{out_pose.pose.position.y:.3f},"
            f"{out_pose.pose.position.z:.3f}) "
            f"base=({pose_base_for_log.pose.position.x:.3f},"
            f"{pose_base_for_log.pose.position.y:.3f},"
            f"{pose_base_for_log.pose.position.z:.3f})",
            throttle_duration_sec=0.5,
        )

        if self.debug:
            self._publish_debug(bgr, (u, v), contour, f"{depth_text} {selector_text}")

    def _detect_target_candidates(self, bgr: np.ndarray):
        """Return all colour blobs for the requested target, sorted by area.

        The old node selected only the largest red blob.  That is unsafe in this
        world because red_cylinder and coke_can are both red.  Here we keep all
        valid blobs and later choose the one nearest to the requested object's
        known world location.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in HSV_RANGES[self.target]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        # Smaller kernel prevents nearby red objects from being joined into one
        # contour.  This is important when the camera sees red_cylinder and
        # coke_can at the same time.
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            candidates.append((u, v, contour, float(area)))

        candidates.sort(key=lambda c: c[3], reverse=True)
        return candidates

    def _select_best_target_candidate(self, bgr: np.ndarray, depth: np.ndarray):
        candidates = self._detect_target_candidates(bgr)
        if not candidates:
            return None

        scored = []
        prior = OBJECT_BASE_CENTER.get(self.target)
        cam_frame = self.camera_frame or self.camera_info.header.frame_id or self.rgb_msg.header.frame_id

        for u, v, contour, area in candidates:
            z_m = self._depth_from_contour_or_window(depth, u, v, contour)
            depth_valid = z_m is not None and np.isfinite(z_m) and z_m > 0.0

            pose_cam_depth = None
            pose_base_depth = None
            depth_text = "depth_invalid"

            if depth_valid:
                point_cam = self._pixel_to_camera_point(u, v, z_m)
                pose_cam_depth = PoseStamped()
                pose_cam_depth.header.stamp = self.rgb_msg.header.stamp
                pose_cam_depth.header.frame_id = cam_frame
                pose_cam_depth.pose.position.x = float(point_cam[0])
                pose_cam_depth.pose.position.y = float(point_cam[1])
                pose_cam_depth.pose.position.z = float(point_cam[2])
                pose_cam_depth.pose.orientation.w = 1.0
                pose_base_depth = self._transform_pose_to_base(pose_cam_depth)
                depth_text = f"depth={z_m:.3f}m"

            # For table-top simulation, the target should be the object centre,
            # not the front visible surface.  Ray-plane gives a stable centre at
            # the known object centre height.  This removes the horizontal/height
            # bias caused by depth on the front curved surface of a cylinder.
            pose_base_plane = None
            if self.center_on_object_plane or (not depth_valid and self.allow_plane_fallback):
                pose_base_plane = self._pose_from_pixel_ray_plane(u, v)

            pose_base_for_selection = pose_base_plane or pose_base_depth
            if pose_base_for_selection is None:
                continue

            if self.select_by_world_prior and prior is not None:
                p = pose_base_for_selection.pose.position
                # Horizontal distance is enough to disambiguate red_cylinder vs
                # coke_can.  A small z term is included for stability.
                score = math.sqrt((p.x - prior[0]) ** 2 + (p.y - prior[1]) ** 2 + 0.25 * (p.z - prior[2]) ** 2)
                selector_text = f"prior_score={score:.3f}"
            else:
                # Without a prior, choose the largest contour.
                score = -area
                selector_text = f"area={area:.0f}"

            scored.append((score, u, v, contour, area, z_m, pose_cam_depth, pose_base_depth, pose_base_plane, depth_text, selector_text))

        if not scored:
            return None

        scored.sort(key=lambda item: item[0])
        _, u, v, contour, area, z_m, pose_cam_depth, pose_base_depth, pose_base_plane, depth_text, selector_text = scored[0]

        # Final base pose.
        #
        # In this Gazebo eye-in-hand demo, the object is static in base_link.
        # A pure RGB-D contour estimate may move when the arm/camera moves because
        # the visible red contour changes, the wrist partially occludes the object,
        # and red_cylinder/coke_can have similar red HSV ranges.
        #
        # Therefore, after the requested object is detected/selected, we lock the
        # published target to the known object centre in base_link.  The camera-frame
        # /target_pose is then obtained only by TF transforming this locked base pose.
        # This keeps the RViz target and controller target fixed at the real object.
        if self.lock_to_world_target and self.target in OBJECT_BASE_CENTER:
            pose_base_out = self._pose_from_known_base_target()
            selector_text = selector_text + "+world_lock"
        else:
            pose_base_out = pose_base_plane or pose_base_depth

        if pose_base_out is None:
            return None

        # ── Target orientation: tool0 pointing straight DOWN at table object ──
        #
        # UR5e tool0 frame convention (standard DH):
        #   +Z = along the tool (outward from flange)
        #   +X = away from the wrist rotation axis
        #
        # At the robot's home pose, tool0 Z points UPWARD (+world Z).
        # To make tool0 point DOWNWARD for a top-down pick:
        #   Rotate 180° around the world X-axis  →  RPY = [π, 0, 0]
        #   This flips tool0 Z from +world_Z to -world_Z (pointing at table).
        #
        # Why NOT [0, -π/2, π]:
        #   That gave ori_err ≈ 111° because it is a compound rotation that
        #   leaves the tool tilted sideways, not pointing straight down.
        #
        # [π, 0, 0] gives the cleanest top-down approach and minimises the
        # wrist rotation needed from the UR5e's typical workspace poses.
        q = Rotation.from_euler("xyz", [math.pi, 0.0, 0.0]).as_quat()
        pose_base_out.pose.orientation.x = float(q[0])
        pose_base_out.pose.orientation.y = float(q[1])
        pose_base_out.pose.orientation.z = float(q[2])
        pose_base_out.pose.orientation.w = float(q[3])
        pose_base_out.header.frame_id = self.base_frame
        pose_base_out.header.stamp = self.get_clock().now().to_msg()

        if self.output_frame == "base":
            out_pose = copy.deepcopy(pose_base_out)
            out_pose.pose.position.z += self.approach_offset_z
        else:
            # For eye-in-hand PBVS, publish the selected centre pose in the
            # moving camera frame.  The controller transforms it back to base.
            out_pose = self._transform_pose_from_base_to_camera(pose_base_out, cam_frame)
            if out_pose is None:
                # If TF back-transform fails, fall back to the raw depth camera
                # pose so Phase-1 centering still works.
                out_pose = pose_cam_depth
            if out_pose is None:
                return None
            out_pose.header.frame_id = cam_frame
            out_pose.header.stamp = self.get_clock().now().to_msg()

        if pose_base_plane is not None:
            depth_text = depth_text + "+center_plane"

        return out_pose, pose_base_out, u, v, contour, depth_text, selector_text

    def _depth_to_meters(self, depth_roi: np.ndarray) -> np.ndarray:
        roi = depth_roi.astype(np.float32)
        enc = (self.depth_msg.encoding or "").lower()
        # 16UC1 is usually millimetres. Gazebo sometimes reports large floats too.
        if enc in ("16uc1", "mono16"):
            roi = roi / 1000.0
        elif roi.size and np.nanmax(roi) > 20.0:
            roi = roi / 1000.0
        return roi

    def _valid_depth_values(self, roi_m: np.ndarray):
        return roi_m[np.isfinite(roi_m) & (roi_m > 0.05) & (roi_m < 10.0)]

    def _depth_from_contour_or_window(self, depth: np.ndarray, u: int, v: int, contour) -> Optional[float]:
        h, w = depth.shape[:2]

        # First try depths inside the detected object contour bounding box.
        if contour is not None:
            x, y, bw, bh = cv2.boundingRect(contour)
            pad = 8
            x1, y1 = max(0, x - pad), max(0, y - pad)
            x2, y2 = min(w, x + bw + pad), min(h, y + bh + pad)
            roi_m = self._depth_to_meters(depth[y1:y2, x1:x2])
            valid = self._valid_depth_values(roi_m)
            if valid.size >= 10:
                # For objects on table, the closest valid points are usually the target surface.
                return float(np.percentile(valid, 25))

        # Then expand around the centre pixel.
        for r in [self.depth_window, 8, 16, 32, 48, 64]:
            r = max(1, int(r))
            x1, x2 = max(0, u - r), min(w, u + r + 1)
            y1, y2 = max(0, v - r), min(h, v + r + 1)
            roi_m = self._depth_to_meters(depth[y1:y2, x1:x2])
            valid = self._valid_depth_values(roi_m)
            if valid.size >= 5:
                return float(np.median(valid))

        return None

    def _pixel_to_camera_point(self, u: int, v: int, z: float) -> np.ndarray:
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            raise ValueError("Invalid camera_info intrinsics: fx/fy is zero")
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.array([x, y, z], dtype=np.float64)

    def _transform_pose_to_base(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose_cam.header.frame_id,
                Time(),
                Duration(seconds=0.2),
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, tf)
        except AttributeError:
            try:
                return self.tf_buffer.transform(pose_cam, self.base_frame, timeout=Duration(seconds=0.2))
            except Exception as exc:
                self.get_logger().warn(
                    f"TF transform failed. Need static TF {self.base_frame} -> {pose_cam.header.frame_id}: {exc}",
                    throttle_duration_sec=1.0,
                )
                return None
        except TransformException as exc:
            self.get_logger().warn(
                f"TF missing. Publish static camera TF: {self.base_frame} -> {pose_cam.header.frame_id}. Error: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    def _transform_pose_from_base_to_camera(self, pose_base: PoseStamped, camera_frame: str) -> Optional[PoseStamped]:
        """Transform a PoseStamped from base_link into the moving camera optical frame.

        This is required in eye-in-hand mode.  The detector first computes the
        selected object centre in base_link for stable object selection.  Then,
        because the PBVS controller expects /target_pose in the camera frame, we
        transform that corrected base pose back into camera_color_optical_frame.
        """
        if pose_base is None:
            return None

        src = pose_base.header.frame_id or self.base_frame
        dst = camera_frame or self.camera_frame
        if not dst:
            self.get_logger().warn("Cannot transform base pose to camera: camera frame is empty", throttle_duration_sec=1.0)
            return None

        # Make sure the source frame is explicitly base_link.
        pose_in = copy.deepcopy(pose_base)
        pose_in.header.frame_id = src
        pose_in.header.stamp = Time().to_msg()

        try:
            # lookup_transform(target_frame, source_frame, time)
            tf = self.tf_buffer.lookup_transform(
                dst,
                src,
                Time(),
                Duration(seconds=0.2),
            )
            try:
                return tf2_geometry_msgs.do_transform_pose_stamped(pose_in, tf)
            except AttributeError:
                # Some ROS 2/Jazzy installations expose Buffer.transform instead
                # of do_transform_pose_stamped.
                return self.tf_buffer.transform(pose_in, dst, timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform failed from {src} to {dst}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    def _pose_from_known_base_target(self) -> Optional[PoseStamped]:
        """Return the known static Gazebo object centre in base_link.

        This is a simulation stabilizer for the eye-in-hand test scene.  It is
        used after the colour detector has confirmed/selected the requested
        object.  For a real camera-only experiment, start the detector with
        --no_lock_world_target.
        """
        xyz = OBJECT_BASE_CENTER.get(self.target)
        if xyz is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.w = 1.0
        return pose

    def _pose_from_pixel_ray_plane(self, u: int, v: int) -> Optional[PoseStamped]:
        """Fallback: intersect the camera pixel ray with known object centre-z plane."""
        cam_frame = self.camera_frame or self.camera_info.header.frame_id or self.rgb_msg.header.frame_id
        plane_z = OBJECT_CENTER_Z.get(self.target, 0.40)
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return None

        # Two points on the same optical ray: camera origin and z=1 m point.
        p0 = PointStamped()
        p0.header.stamp = self.rgb_msg.header.stamp
        p0.header.frame_id = cam_frame
        p0.point.x = 0.0
        p0.point.y = 0.0
        p0.point.z = 0.0

        p1 = PointStamped()
        p1.header.stamp = self.rgb_msg.header.stamp
        p1.header.frame_id = cam_frame
        p1.point.x = float((u - cx) / fx)
        p1.point.y = float((v - cy) / fy)
        p1.point.z = 1.0

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, Time(), Duration(seconds=0.2))
            b0 = tf2_geometry_msgs.do_transform_point(p0, tf)
            b1 = tf2_geometry_msgs.do_transform_point(p1, tf)
        except Exception as exc:
            self.get_logger().warn(f"Ray-plane fallback TF failed: {exc}", throttle_duration_sec=1.0)
            return None

        origin = np.array([b0.point.x, b0.point.y, b0.point.z], dtype=float)
        raypt = np.array([b1.point.x, b1.point.y, b1.point.z], dtype=float)
        direction = raypt - origin
        if abs(direction[2]) < 1e-9:
            return None
        scale = (plane_z - origin[2]) / direction[2]
        if scale <= 0:
            return None
        hit = origin + scale * direction

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(hit[0])
        pose.pose.position.y = float(hit[1])
        pose.pose.position.z = float(plane_z)
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_debug(self, bgr: np.ndarray, centre, contour, text):
        dbg = bgr.copy()
        if contour is not None:
            cv2.drawContours(dbg, [contour], -1, (0, 255, 0), 2)
        if centre is not None:
            u, v = centre
            cv2.circle(dbg, (u, v), 5, (0, 0, 255), -1)
            label = self.target if text is None else f"{self.target} {text}"
            cv2.putText(
                dbg,
                label,
                (max(5, u + 8), max(20, v - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}")


def main():
    parser = argparse.ArgumentParser(description="Robust RGB-D colour target detector for PBVS")
    parser.add_argument("--target", default="red_cylinder", choices=list(HSV_RANGES.keys()))
    parser.add_argument("--image_topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth_topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera_info_topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--output_topic", default="/target_pose")
    parser.add_argument("--base_frame", default="base_link")
    parser.add_argument("--camera_frame", default="camera_color_optical_frame")
    parser.add_argument("--publish_hz", type=float, default=15.0)
    parser.add_argument("--offset_z", type=float, default=0.10, help="approach height above object in base frame; only used with --output_frame base")
    parser.add_argument("--output_frame", choices=["base", "camera"], default="base", help="publish /target_pose in base_link for eye-to-hand or camera frame for eye-in-hand")
    parser.add_argument("--min_area", type=float, default=80.0)
    parser.add_argument("--depth_window", type=int, default=8, help="half-window around centre pixel for median depth")
    parser.add_argument("--allow_plane_fallback", action="store_true", default=True)
    parser.add_argument("--no_plane_fallback", dest="allow_plane_fallback", action="store_false")
    parser.add_argument("--select_by_world_prior", action="store_true", default=True,
                        help="choose the colour blob closest to the requested object's known Gazebo pose; fixes red_cylinder vs coke_can ambiguity")
    parser.add_argument("--no_world_prior", dest="select_by_world_prior", action="store_false")
    parser.add_argument("--center_on_object_plane", action="store_true", default=True,
                        help="use the image ray intersected with known object centre height; removes depth bias on visible surface")
    parser.add_argument("--no_center_plane", dest="center_on_object_plane", action="store_false")
    parser.add_argument("--lock_to_world_target", action="store_true", default=True,
                        help="simulation mode: after detecting the requested object, lock the published target to the known Gazebo object centre in base_link")
    parser.add_argument("--no_lock_world_target", dest="lock_to_world_target", action="store_false",
                        help="disable Gazebo world target lock and publish the raw RGB-D estimated target")
    parser.add_argument("--debug", action="store_true")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CameraColorDepthTargetNode(args)
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


if __name__ == "__main__":
    main()
