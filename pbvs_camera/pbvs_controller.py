#!/usr/bin/env python3
"""
pbvs_controller.py — Hybrid IBVS+PBVS Visual Servoing for UR5e Eye-in-Hand

Architecture:
    Camera detector  →  /target_pose  (PoseStamped in camera frame)
                                ↓
                       pbvs_controller.py
                                ↓
              /servo_node/delta_twist_cmds  (TwistStamped)
                                ↓
                       MoveIt Servo → UR5e

Tracking Strategy (Eye-in-Hand):
    The camera is on the wrist. As the arm moves toward a target, the object
    drifts in the image and can leave the FOV — causing detection loss.

    This controller uses a TWO-PHASE hybrid approach:

    Phase 1 — TRACK (pos_err > SWITCH_THRESHOLD):
        • Pan/tilt the wrist to keep the object CENTRED in the image  (IBVS)
        • Drive forward along camera +Z toward the object              (approach)
        → Arm approaches while always keeping target in FOV.

    Phase 2 — CONVERGE (pos_err < SWITCH_THRESHOLD):
        • Full PBVS in base_link for precise final positioning.
        → Accurate alignment with target pose.

Topics:
    Subscribed:
        /target_pose                            geometry_msgs/PoseStamped
        /camera/camera/color/camera_info        sensor_msgs/CameraInfo
    Published:
        /servo_node/delta_twist_cmds            geometry_msgs/TwistStamped
"""

import math
import argparse
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import SwitchController
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

import numpy as np
from scipy.spatial.transform import Rotation


# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

MODE_EYE_IN_HAND  = 'eye_in_hand'
MODE_EYE_TO_HAND  = 'eye_to_hand'

BASE_FRAME        = 'base_link'
EEF_FRAME         = 'tool0'
CAMERA_FRAME      = 'camera_color_optical_frame'

TWIST_TOPIC       = '/servo_node/delta_twist_cmds'
TARGET_TOPIC      = '/target_pose'
TARGET_BASE_TOPIC = '/target_pose_base'  # debug/visualization pose after camera→base transform
CAMERA_INFO_TOPIC = '/camera/camera/color/camera_info'

# Phase switch: IBVS-approach → full PBVS
SWITCH_THRESHOLD  = 0.12   # metres  (12 cm)

# Phase 3: orientation-only alignment after reaching position
# When pos_err < ORI_ALIGN_THRESHOLD, stop translating and fix orientation first.
ORI_ALIGN_THRESHOLD = 0.02  # metres (2 cm): tightened to fix orientation sooner

# ── Phase 1 gains — wrist tracking + arm approach ────────────────────────────
# MoveIt Servo only accepts base_link or tool0 as twist frame.
# We publish Phase 1 twist in tool0 (EEF) frame:
#   linear  = move EEF toward target  (expressed in tool0)
#   angular = rotate wrist to centre target in image (expressed in tool0)
#
# Camera optical frame axes vs tool0 axes for UR5e wrist:
#   camera +X (right)    ≈ tool0 +Y   → pan  error maps to angular.y of tool0
#   camera +Y (down)     ≈ tool0 -X   → tilt error maps to angular.x of tool0 (negated)
#   camera +Z (forward)  ≈ tool0 +Z   → approach along tool0 Z
KP_WRIST_PAN      = 1.2    # rad/s per radian of horizontal angle error
KP_WRIST_TILT     = 1.2    # rad/s per radian of vertical   angle error
MAX_WRIST_PAN     = 0.5    # rad/s cap for pan
MAX_WRIST_TILT    = 0.5    # rad/s cap for tilt

# Arm approach: speed toward target during Phase 1.
# Raised limits — 0.05 m/s was too slow; MoveIt Servo needs a noticeable command.
KP_APPROACH       = 2.0    # m/s per m of 3D position error
MAX_APPROACH      = 0.3   # m/s cap (per axis in tool0 frame)

# Phase 2 gains
KP_LINEAR         = 2.5
KP_ANGULAR        = 3.0   # raised from 1.5 — orientation was not converging
MAX_LINEAR        = 0.30   # raised from 0.05 — arm was barely moving
MAX_ANGULAR       = 1.2    # raised from 0.4 — allow faster wrist rotation

# Convergence
POS_THRESH        = 0.005# 5 mm
ORI_THRESH        = 0.03   # ~1.7 deg (tightened from 0.05/~3 deg for better alignment)

CTRL_HZ           = 25.0
# Eye-in-hand: camera moves with arm so detection is intermittent.
# Phase 1: stop if stale (wrong direction could drive arm away).
# Phase 2/3: continue on last known pose (see stale guard in _control_loop).
TARGET_STALE_SEC  = 2.0


# ═══════════════════════════════════════════════════════════════════════════════
# Controller
# ═══════════════════════════════════════════════════════════════════════════════

class PBVSController(Node):

    def __init__(self, mode: str = MODE_EYE_TO_HAND, approach_z: float = 0.0):
        super().__init__('pbvs_controller')

        self.mode       = mode
        self.approach_z = float(approach_z)

        self.get_logger().info(f'PBVS mode: {self.mode.upper()}')
        self.get_logger().info(f'Approach Z offset: {self.approach_z:.3f} m')
        self.get_logger().info(
            f'Tracking: Phase1(IBVS+Z) until {SWITCH_THRESHOLD*1000:.0f}mm, '
            f'then Phase2(PBVS converge)')

        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.target_base_pub = self.create_publisher(PoseStamped, TARGET_BASE_TOPIC, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera-frame pose from detector (used in Phase 1 for pixel centering)
        self._target_cam:  PoseStamped = None
        # Base-frame pose (transformed, used in Phase 2 PBVS)
        self._target_base: PoseStamped = None
        self._target_received  = False
        self._target_last_time = None

        self.create_subscription(PoseStamped, TARGET_TOPIC, self._target_cb, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 1)

        self._phase = 1  # start in tracking phase

        self._target_printed = False  # track if we've printed the current target
        self._last_printed_pos = None  # last printed position for change detection

        self.kp_linear   = KP_LINEAR
        self.kp_angular  = KP_ANGULAR
        self.max_linear  = MAX_LINEAR
        self.max_angular = MAX_ANGULAR

        # State machine
        # State 0: switch to joint_trajectory_controller (Servo needs it)
        # State 1: unpause servo
        # State 2: set command type to TWIST
        # State 3: control loop
        self._state = 0
        self._async_fired_at    = 0.0
        self._async_timeout_sec = 5.0
        self.ctrl_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.pause_client = self.create_client(SetBool, '/servo_node/pause_servo')
        self.cmd_client   = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')

        # Track published twist count for diagnostics
        self._twist_count = 0
        self._last_diag_time = 0.0

        self.timer = self.create_timer(1.0 / CTRL_HZ, self._tick)
        self.get_logger().info(
            f'PBVSController started ({self.mode}) — waiting for servo node...')

    # ─────────────────────────────────────────────────────────────────────
    def _info_cb(self, msg: CameraInfo):
        # We only need this to confirm camera is alive; image size unused
        pass

    def _target_cb(self, msg: PoseStamped):
        """
        Store latest target pose in BOTH camera frame and base_link frame.

        Eye-in-hand: detector publishes in camera_color_optical_frame.
          → keep raw pose for Phase-1 IBVS centering
          → transform to base_link for Phase-2 PBVS

        Eye-to-hand: detector publishes in base_link already.
          → only base pose needed

        Note: TF lookup uses Time() not msg.header.stamp because the Gazebo
        bridge stamps images ~2s ahead of TF data (different clock sources),
        which causes "extrapolation into the future" errors with stamp-based lookup.
        """
        if self.mode == MODE_EYE_IN_HAND:
            # Store raw camera-frame pose for Phase 1
            self._target_cam = msg

            # Transform to base_link for Phase 2
            try:
                tf = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    msg.header.frame_id,
                    Time(),
                    Duration(seconds=0.1))
                try:
                    base_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, tf)
                except AttributeError:
                    base_pose = self.tf_buffer.transform(
                        msg, BASE_FRAME, timeout=Duration(seconds=0.1))
                base_pose.header.frame_id = BASE_FRAME
                self._target_base = base_pose
            except TransformException as e:
                self.get_logger().warn(
                    f'TF {msg.header.frame_id}→{BASE_FRAME}: {e}',
                    throttle_duration_sec=1.0)
                return
            except Exception as e:
                self.get_logger().warn(f'Transform error: {e}', throttle_duration_sec=1.0)
                return
        else:
            self._target_base = msg
            self._target_cam  = None

        # Apply approach_z offset in base frame.
        # WARNING: Do NOT set --offset_z in the detector as well — that would
        # double-apply the offset and cause the robot to overshoot the target.
        if self.approach_z != 0.0 and self._target_base is not None:
            self._target_base.pose.position.z += self.approach_z

        # Publish the base-frame target used internally by the controller.
        # This is useful in RViz: /target_pose is camera-frame in eye-in-hand,
        # while /target_pose_base stays fixed in base_link if TF and depth are correct.
        if self._target_base is not None:
            self._target_base.header.stamp = self.get_clock().now().to_msg()
            self._target_base.header.frame_id = BASE_FRAME
            self.target_base_pub.publish(self._target_base)

        self._target_received  = True
        self._target_last_time = self.get_clock().now()

        # ── Print target pose once when first received (or after big change) ──
        if self._target_base is not None:
            tp = self._target_base.pose
            q = tp.orientation
            r_target = Rotation.from_quat([q.x, q.y, q.z, q.w])
            rpy = r_target.as_euler('xyz', degrees=True)

            # Reprint if position changed significantly (>5cm from last print)
            should_print = not self._target_printed
            if hasattr(self, '_last_printed_pos') and self._last_printed_pos is not None:
                dx = tp.position.x - self._last_printed_pos[0]
                dy = tp.position.y - self._last_printed_pos[1]
                dz = tp.position.z - self._last_printed_pos[2]
                if (dx**2 + dy**2 + dz**2) > 0.05**2:
                    should_print = True

            if should_print:
                self.get_logger().info(
                    f'\n'
                    f'══════════════════════════════════════════════\n'
                    f'  🎯  TARGET POSE (base_link frame)\n'
                    f'  Position : x={tp.position.x:.4f}  y={tp.position.y:.4f}  z={tp.position.z:.4f}\n'
                    f'  Quaternion: x={q.x:.4f}  y={q.y:.4f}  z={q.z:.4f}  w={q.w:.4f}\n'
                    f'  RPY (deg) : r={rpy[0]:.2f}  p={rpy[1]:.2f}  y={rpy[2]:.2f}\n'
                    f'══════════════════════════════════════════════'
                )
                self._target_printed = True
                self._last_printed_pos = (tp.position.x, tp.position.y, tp.position.z)

    # ── State machine ─────────────────────────────────────────────────────
    # State 0: switch to joint_trajectory_controller (Servo needs it, not fpc)
    # State 1: unpause servo
    # State 2: set command type to TWIST
    # State 3: control loop
    # State -1: waiting for async service response
    def _tick(self):
        if self._state == 0:
            self._do_switch_controller()
        elif self._state == 1:
            self._do_unpause()
        elif self._state == 2:
            self._do_set_cmd_type()
        elif self._state == 3:
            self._control_loop()
        elif self._state == -1:
            import time as _t
            if _t.monotonic() - self._async_fired_at > self._async_timeout_sec:
                self.get_logger().warn('Async service timed out — retrying...')
                self._state = 0

    def _do_switch_controller(self):
        """
        Switch controller_manager so that joint_trajectory_controller is active.
        MoveIt Servo drives the robot through JTC — it will NOT move if only
        forward_position_controller is active (different hardware interface).

        Uses BEST_EFFORT so missing controller names are silently skipped.
        If /controller_manager is unavailable (e.g. mock hardware without cm),
        skip the switch and proceed directly to unpause.
        """
        if not self.ctrl_switch_client.service_is_ready():
            self.get_logger().info(
                'Waiting for /controller_manager/switch_controller ...',
                throttle_duration_sec=2.0)
            return
        req = SwitchController.Request()
        req.activate_controllers   = [
            'joint_trajectory_controller',
            'scaled_joint_trajectory_controller',
        ]
        req.deactivate_controllers = ['forward_position_controller']
        req.strictness   = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        self.ctrl_switch_client.call_async(req).add_done_callback(self._switch_ctrl_cb)
        import time as _t
        self._async_fired_at = _t.monotonic()
        self._state = -1

    def _switch_ctrl_cb(self, future):
        try:
            res = future.result()
            if res.ok:
                self.get_logger().info(
                    'Controller → joint_trajectory_controller ✅  (Servo can now move arm)')
            else:
                self.get_logger().warn(
                    'Controller switch returned not-ok — Servo may still work if JTC was already active')
        except Exception as e:
            self.get_logger().warn(f'Controller switch error: {e} — continuing anyway')
        self._state = 1  # always proceed; Servo may already have the right controller

    def _do_unpause(self):
        if not self.pause_client.service_is_ready():
            self.get_logger().info(
                'Waiting for /servo_node/pause_servo ...',
                throttle_duration_sec=2.0)
            return
        req = SetBool.Request()
        req.data = False
        self.pause_client.call_async(req).add_done_callback(self._unpause_cb)
        import time as _t
        self._async_fired_at = _t.monotonic()
        self._state = -1

    def _unpause_cb(self, future):
        try:
            res = future.result()
            self.get_logger().info('Servo unpaused ✅') if res.success else \
                self.get_logger().warn(f'Unpause failed: {res.message}')
            self._state = 2 if res.success else 1
        except Exception as e:
            self.get_logger().warn(f'Unpause error: {e}')
            self._state = 1

    def _do_set_cmd_type(self):
        if not self.cmd_client.service_is_ready():
            self.get_logger().info(
                'Waiting for switch_command_type ...', throttle_duration_sec=2.0)
            return
        req = ServoCommandType.Request()
        req.command_type = 1  # TWIST
        self.cmd_client.call_async(req).add_done_callback(self._cmd_type_cb)
        import time as _t
        self._async_fired_at = _t.monotonic()
        self._state = -1

    def _cmd_type_cb(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(
                    f'Command type → TWIST ✅  |  PBVS ({self.mode}) active ✅')
                self._state = 3
            else:
                self.get_logger().warn(f'Set cmd type failed: {res.message}')
                self._state = 2
        except Exception as e:
            self.get_logger().warn(f'Cmd type error: {e}')
            self._state = 2

    # ── Main control loop ─────────────────────────────────────────────────
    def _control_loop(self):
        if not self._target_received:
            self.get_logger().info(
                f'Waiting for target on {TARGET_TOPIC} ...',
                throttle_duration_sec=2.0)
            return

        # Stale-target guard
        # Phase 1 (far, IBVS tracking): stop if target lost — wrong camera
        #   direction would drive arm away from object.
        # Phase 2/3 (close, PBVS): keep last known target and continue —
        #   the target position is locked to world coords and won't change,
        #   detection dropout here is caused by the camera being too close,
        #   not by the object moving.  Stopping here freezes the arm 5 cm away.
        if self._target_last_time is not None:
            age = (self.get_clock().now() - self._target_last_time).nanoseconds / 1e9
            if age > TARGET_STALE_SEC:
                if self._phase == 1:
                    self.get_logger().warn(
                        f'[Phase 1] Target stale ({age:.2f}s) — holding until reacquired.',
                        throttle_duration_sec=1.0)
                    return
                else:
                    # Phase 2/3: continue with last known target (world-locked position)
                    self.get_logger().info(
                        f'[Phase {self._phase}] Target stale ({age:.1f}s) — continuing on last pose.',
                        throttle_duration_sec=1.0)

        if self.mode == MODE_EYE_IN_HAND:
            self._control_eye_in_hand()
        else:
            self._control_eye_to_hand()

    # ── Eye-in-Hand hybrid IBVS+PBVS ────────────────────────────────────
    def _control_eye_in_hand(self):
        if self._target_cam is None or self._target_base is None:
            return

        # Current EEF pose in base_link
        try:
            tf_eef = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME, Time(), Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f'EEF TF: {e}', throttle_duration_sec=1.0)
            return

        cx = tf_eef.transform.translation.x
        cy = tf_eef.transform.translation.y
        cz = tf_eef.transform.translation.z

        tp  = self._target_base.pose
        ex  = tp.position.x - cx
        ey  = tp.position.y - cy
        ez  = tp.position.z - cz
        pos_err = math.sqrt(ex**2 + ey**2 + ez**2)

        # Orientation error
        q   = tf_eef.transform.rotation
        q_c = np.array([q.x, q.y, q.z, q.w])
        q_t = np.array([
            tp.orientation.x, tp.orientation.y,
            tp.orientation.z, tp.orientation.w])
        if np.linalg.norm(q_t) < 0.01:
            q_t = np.array([0.0, 0.0, 0.0, 1.0])
        # Use sign-corrected orientation error (fixes ~175° quaternion flip bug)
        rotvec, ori_err = self._orientation_error(q_c, q_t)

        # Phase selection with hysteresis
        # Phase 1: IBVS+approach (far from target)
        # Phase 2: Full PBVS     (close, simultaneous pos+ori)
        # Phase 3: Orient-only   (position reached, orientation still off)
        if pos_err > SWITCH_THRESHOLD:
            self._phase = 1
        elif pos_err < SWITCH_THRESHOLD * 0.8:
            if pos_err < ORI_ALIGN_THRESHOLD and ori_err > ORI_THRESH:
                self._phase = 3  # position done, fix orientation only
            else:
                self._phase = 2

        # Convergence
        if pos_err < POS_THRESH and ori_err < ORI_THRESH:
            self.get_logger().info(
                '✅  Target reached! Holding.', throttle_duration_sec=2.0)
            return

        r_eef = Rotation.from_quat(q_c)
        rpy_eef = r_eef.as_euler('xyz', degrees=True)
        r_tgt = Rotation.from_quat(q_t)
        rpy_tgt = r_tgt.as_euler('xyz', degrees=True)

        self.get_logger().info(
            f'[Phase {self._phase}] '
            f'pos_err={pos_err*1000:.1f}mm  ori_err={math.degrees(ori_err):.1f}°  '
            f'eef=({cx:.3f},{cy:.3f},{cz:.3f}) '
            f'tgt=({tp.position.x:.3f},{tp.position.y:.3f},{tp.position.z:.3f})\n'
            f'  EEF RPY(deg): r={rpy_eef[0]:.1f} p={rpy_eef[1]:.1f} y={rpy_eef[2]:.1f}  '
            f'TGT RPY(deg): r={rpy_tgt[0]:.1f} p={rpy_tgt[1]:.1f} y={rpy_tgt[2]:.1f}',
            throttle_duration_sec=0.3)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        if self._phase == 1:
            # ── Phase 1: Wrist pan/tilt to track + arm approach ───────────
            #
            # GOAL: Keep object centred in image at all times while moving
            #       the arm toward the object.
            #
            # HOW:
            #   1. WRIST ROTATION (pan/tilt) — rotate the wrist joint so the
            #      camera points at the object. This is angular velocity about
            #      the camera Y-axis (pan left/right) and X-axis (tilt up/down),
            #      expressed in the CAMERA frame.
            #
            #      Camera optical frame convention:
            #        +X = right,  +Y = down,  +Z = forward (into scene)
            #
            #      If object is at +x_cam (right of centre):
            #        → rotate camera around Y-axis in +Y direction (pan right)
            #        → angular.y = +value in camera frame
            #
            #      If object is at +y_cam (below centre):
            #        → rotate camera around X-axis in -X direction (tilt down)
            #        → angular.x = -value in camera frame
            #
            #   2. ARM TRANSLATION — move the EEF toward the object in base_link.
            #      This is the approach component (drives pos_err down).
            #      We use base-frame linear velocity (from 3D position error)
            #      so the arm moves smoothly regardless of wrist orientation.
            #
            # The two commands are COMBINED in one TwistStamped.
            # Servo splits them: linear → arm joints, angular → wrist joints.
            # We publish in CAMERA frame so angular rotation is about camera axes.

            obj_x = self._target_cam.pose.position.x   # metres right of centre
            obj_y = self._target_cam.pose.position.y   # metres below centre
            obj_z = self._target_cam.pose.position.z   # depth to object

            # Angular error in radians (scale-independent, same gain near & far)
            if obj_z > 0.01:
                ang_err_pan  =  obj_x / obj_z   # + = object is right → pan right
                ang_err_tilt =  obj_y / obj_z   # + = object is below → tilt down
            else:
                ang_err_pan  = 0.0
                ang_err_tilt = 0.0

            # Wrist pan  = rotate around camera Y-axis
            # Wrist tilt = rotate around camera X-axis (negative: tilt down for +y)
            w_pan  = self._clamp(KP_WRIST_PAN  * ang_err_pan,  MAX_WRIST_PAN)
            w_tilt = self._clamp(KP_WRIST_TILT * ang_err_tilt, MAX_WRIST_TILT)

            # Arm approach: translate toward object using base-frame 3D error.
            # Scale down approach speed when object is off-centre (prioritise centering first).
            centre_err = math.sqrt(ang_err_pan**2 + ang_err_tilt**2)
            # Reduce approach to 20% when very off-centre (>30°), full speed when centred
            approach_scale = max(0.5, 1.0 - 1.5 * centre_err)
            vx_base = self._clamp(KP_APPROACH * ex * approach_scale, MAX_APPROACH)
            vy_base = self._clamp(KP_APPROACH * ey * approach_scale, MAX_APPROACH)
            vz_base = self._clamp(KP_APPROACH * ez * approach_scale, MAX_APPROACH)

            # ── CRITICAL: MoveIt Servo only accepts base_link or tool0 frames.
            # Publishing in camera_color_optical_frame causes SILENT rejection.
            # We keep the angular command but express it in base_link by rotating
            # the camera-frame angular velocity through the current camera→base TF.
            try:
                tf_base_from_cam = self.tf_buffer.lookup_transform(
                    BASE_FRAME, CAMERA_FRAME, Time(), Duration(seconds=0.05))
                r_base_cam = Rotation.from_quat([
                    tf_base_from_cam.transform.rotation.x,
                    tf_base_from_cam.transform.rotation.y,
                    tf_base_from_cam.transform.rotation.z,
                    tf_base_from_cam.transform.rotation.w,
                ])
                # Angular in camera frame: [-w_tilt, w_pan, 0]
                w_cam = np.array([-w_tilt, w_pan, 0.0])
                w_base = r_base_cam.apply(w_cam)
            except TransformException:
                # Fallback: no wrist correction, just translate
                w_base = np.array([0.0, 0.0, 0.0])

            # Publish combined twist in BASE_LINK frame (servo-compatible):
            #   linear  = arm moves toward target  (base frame 3D error)
            #   angular = wrist rotates to keep target in FOV (rotated to base frame)
            twist.header.frame_id = BASE_FRAME
            twist.twist.linear.x  = vx_base
            twist.twist.linear.y  = vy_base
            twist.twist.linear.z  = vz_base
            twist.twist.angular.x = self._clamp(float(w_base[0]), MAX_WRIST_TILT)
            twist.twist.angular.y = self._clamp(float(w_base[1]), MAX_WRIST_PAN)
            twist.twist.angular.z = self._clamp(float(w_base[2]), MAX_WRIST_PAN)

            self.get_logger().info(
                f'[Track] pan={math.degrees(ang_err_pan):.1f}° '
                f'tilt={math.degrees(ang_err_tilt):.1f}° '
                f'w_pan={w_pan:.3f} w_tilt={w_tilt:.3f} '
                f'approach_scale={approach_scale:.2f}',
                throttle_duration_sec=0.5)

        elif self._phase == 2:
            # ── Phase 2: Full PBVS in base_link (position + orientation together) ─
            twist.header.frame_id = BASE_FRAME
            twist.twist.linear.x  = self._clamp(self.kp_linear * ex, self.max_linear)
            twist.twist.linear.y  = self._clamp(self.kp_linear * ey, self.max_linear)
            twist.twist.linear.z  = self._clamp(self.kp_linear * ez, self.max_linear)
            twist.twist.angular.x = self._clamp(
                self.kp_angular * rotvec[0], self.max_angular)
            twist.twist.angular.y = self._clamp(
                self.kp_angular * rotvec[1], self.max_angular)
            twist.twist.angular.z = self._clamp(
                self.kp_angular * rotvec[2], self.max_angular)

        else:
            # ── Phase 3: Orientation-only alignment (position already done) ──────
            # Zero out linear velocity so the arm stays in place while the wrist
            # rotates to match the target orientation.
            self.get_logger().info(
                f'[Phase 3/OrientFix] ori_err={math.degrees(ori_err):.1f}°',
                throttle_duration_sec=0.5)
            twist.header.frame_id = BASE_FRAME
            twist.twist.linear.x  = 0.0
            twist.twist.linear.y  = 0.0
            twist.twist.linear.z  = 0.0
            twist.twist.angular.x = self._clamp(
                self.kp_angular * rotvec[0], self.max_angular)
            twist.twist.angular.y = self._clamp(
                self.kp_angular * rotvec[1], self.max_angular)
            twist.twist.angular.z = self._clamp(
                self.kp_angular * rotvec[2], self.max_angular)

        self.twist_pub.publish(twist)

    # ── Eye-to-Hand PBVS (single phase) ──────────────────────────────────
    def _control_eye_to_hand(self):
        if self._target_base is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME, Time(), Duration(seconds=0.0))
        except TransformException as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=1.0)
            return

        cx = tf.transform.translation.x
        cy = tf.transform.translation.y
        cz = tf.transform.translation.z
        q  = tf.transform.rotation
        q_c = np.array([q.x, q.y, q.z, q.w])

        tp  = self._target_base.pose
        ex  = tp.position.x - cx
        ey  = tp.position.y - cy
        ez  = tp.position.z - cz
        pos_err = math.sqrt(ex**2 + ey**2 + ez**2)

        q_t = np.array([
            tp.orientation.x, tp.orientation.y,
            tp.orientation.z, tp.orientation.w])
        if np.linalg.norm(q_t) < 0.01:
            q_t = np.array([0.0, 0.0, 0.0, 1.0])
        # Use sign-corrected orientation error (fixes ~175° quaternion flip bug)
        rotvec, ori_err = self._orientation_error(q_c, q_t)

        self.get_logger().info(
            f'[PBVS/ETH] pos_err={pos_err*1000:.1f}mm  ori_err={math.degrees(ori_err):.1f}°',
            throttle_duration_sec=0.5)

        if pos_err < POS_THRESH and ori_err < ORI_THRESH:
            self.get_logger().info(
                '✅  Target reached!', throttle_duration_sec=2.0)
            return

        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_FRAME
        twist.twist.linear.x  = self._clamp(self.kp_linear * ex, self.max_linear)
        twist.twist.linear.y  = self._clamp(self.kp_linear * ey, self.max_linear)
        twist.twist.linear.z  = self._clamp(self.kp_linear * ez, self.max_linear)
        twist.twist.angular.x = self._clamp(
            self.kp_angular * rotvec[0], self.max_angular)
        twist.twist.angular.y = self._clamp(
            self.kp_angular * rotvec[1], self.max_angular)
        twist.twist.angular.z = self._clamp(
            self.kp_angular * rotvec[2], self.max_angular)
        self.twist_pub.publish(twist)

    @staticmethod
    def _clamp(v, limit):
        return max(min(v, limit), -limit)

    @staticmethod
    def _orientation_error(q_current: np.ndarray, q_target: np.ndarray):
        """
        Compute rotation error as a rotation vector, with two fixes:

        FIX 1 — Quaternion double-cover:
            q and -q are the same rotation.  Naive r_err = R_t * R_c^{-1}
            gives ~360°-angle instead of true angle when antipodal form used.
            Fix: flip q_target so dot(q_c, q_t) > 0 before computing delta.

        FIX 2 — Yaw symmetry (resolves the stuck-at-90° yaw error):
            Target [pi,0,0] has yaw=0 but EEF wrist sits at yaw=-89°.
            For a cylindrical object any yaw is valid for top-down pick.
            We try yaw offsets 0/90/180/270° on the target and pick the
            one that gives the smallest rotation from current EEF pose.
            This collapses a 90° yaw error to <45° in one step.
        """
        r_c = Rotation.from_quat(q_current)
        r_t = Rotation.from_quat(q_target)

        best_rotvec  = None
        best_ori_err = float('inf')

        for yaw_deg in [0.0, 90.0, 180.0, 270.0]:
            yaw_rot     = Rotation.from_euler('z', math.radians(yaw_deg))
            r_candidate = r_t * yaw_rot          # rotate target around its Z
            q_candidate = r_candidate.as_quat()

            # FIX 1: antipodal flip for this candidate
            if np.dot(q_current, q_candidate) < 0.0:
                q_candidate = -q_candidate
                r_candidate  = Rotation.from_quat(q_candidate)

            r_err   = r_candidate * r_c.inv()
            rotvec  = r_err.as_rotvec()
            ori_err = float(np.linalg.norm(rotvec))

            if ori_err < best_ori_err:
                best_ori_err = ori_err
                best_rotvec  = rotvec

        return best_rotvec, best_ori_err


# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    parser = argparse.ArgumentParser(description='Hybrid IBVS+PBVS Controller')
    parser.add_argument('--mode',
        choices=[MODE_EYE_IN_HAND, MODE_EYE_TO_HAND],
        default=MODE_EYE_TO_HAND)
    parser.add_argument('--approach_z', type=float, default=0.0,
        help='Z offset above target in base_link metres. '
             'Do NOT also set --offset_z in detector.')
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = PBVSController(mode=parsed.mode, approach_z=parsed.approach_z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
