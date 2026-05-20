# PBVS Camera — UR5e Visual Servoing (ROS 2 Jazzy)

Position-Based Visual Servoing (PBVS) for the UR5e arm using RGB-D camera perception and MoveIt Servo.
This repository contains two camera configurations, each supported within the `pbvs_camera` package.

---

## ✅ Results — Both Modes Achieved

| Mode | Position Error | Orientation Error | Status |
|---|---|---|---|
| Eye-to-Hand (Fixed Camera) | **4 mm** | **0°** | ✅ Converged |
| Eye-in-Hand (Wrist Camera) | **5 mm** | **1.7°** | ✅ Converged |

---

## 📦 Package

> **Create the ROS 2 workspace and package before building:**

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src

# Clone or copy the package
cp -r pbvs_camera ~/ur_ws/src/

# The package name is: pbvs_camera
# Verify package.xml exists:
ls ~/ur_ws/src/pbvs_camera/package.xml
```

The single package `pbvs_camera` supports both eye-to-hand and eye-in-hand modes via a `mode` launch argument.

---

## 🏗️ System Architecture

### Eye-to-Hand (Fixed World Camera)

```
Gazebo camera sensor
  → ros_gz_bridge
  → /camera/camera/color/image_raw
  → /camera/camera/aligned_depth_to_color/image_raw
  → camera_color_depth_target_node
  → /target_pose  (PoseStamped in base_link)
        ↓
  pbvs_controller  (eye_to_hand mode)
  • TF lookup: base_link → tool0  (current EEF pose)
  • Cartesian error × KP gain → intermediate waypoint
  • /compute_ik  (MoveIt GetPositionIK service)
  → joint angles
        ↓
  /joint_trajectory_controller/joint_trajectory
        ↓
  UR5e arm moves  →  pos_err = 4 mm,  ori_err = 0°  ✅
```

### Eye-in-Hand (Wrist-Mounted Camera)

```
Wrist-mounted Gazebo camera
  → /target_pose  (PoseStamped in camera_color_optical_frame)
        ↓
  pbvs_controller  (eye_in_hand mode — TWO-PHASE hybrid)

  Phase 1 — TRACK  (pos_err > 12 cm):
    • IBVS: Pan/tilt wrist to keep target centred in FOV
    • Approach: drive forward along camera +Z

  Phase 2 — CONVERGE  (pos_err < 12 cm):
    • Full PBVS in base_link for precise final positioning
        ↓
  /servo_node/delta_twist_cmds  (TwistStamped)
        ↓
  MoveIt Servo → UR5e  →  pos_err = 5 mm,  ori_err = 1.7°  ✅
```

---

## 📋 Prerequisites

- ROS 2 Jazzy
- UR5e robot (real or `use_mock_hardware:=true`)
- Gazebo (Harmonic or compatible)
- Python 3 with `opencv`, `scipy`

---

## 📥 Install Dependencies

### Eye-to-Hand

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ur-robot-driver \
  ros-jazzy-ur-moveit-config \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rviz2 \
  python3-opencv \
  python3-scipy
```

### Eye-in-Hand (additional)

```bash
sudo apt install \
  ros-jazzy-ur-simulation-gazebo
```

---

## 🔨 Build

```bash
cd ~/ur_ws
colcon build --packages-select pbvs_camera --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/ur_ws/install/setup.bash
```

---

## 🚀 Running — Eye-to-Hand

Uses 4 terminals. The controller talks to `joint_trajectory_controller` via MoveIt IK — **no MoveIt Servo needed**.

### Terminal 1 — Robot driver

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true
```

Wait for: `Controller Manager running`

### Terminal 2 — MoveIt (IK only)

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  launch_rviz:=false
```

Wait for: `MoveGroup running`

> MoveIt Servo is **not** used in eye-to-hand mode. No controller switching needed.

### Terminal 3 — Gazebo world + camera

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch pbvs_camera external_driver_world_camera.launch.py \
  target:=red_cylinder
```

This starts Gazebo with table and objects, camera bridge (color + depth), static camera TF, RGB-D target detector → `/target_pose`, and RViz markers.

Available targets:

```bash
target:=red_cylinder   # default
target:=mustard
target:=cheezit
target:=coke_can
target:=cardboard_box
```

### Terminal 4 — PBVS controller

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch pbvs_camera pbvs_controller.launch.py mode:=eye_to_hand
```

Expected output:

```
[PBVS/EYE] eef=(0.183,-0.483,0.420)  tgt=(0.450,0.160,0.500)  pos=312.4mm  ori=2.3°
→ JTC: [0.523, -1.201, 1.803, -2.197, -1.198, 0.302]
...
✅ Target reached! Holding pose.
[PBVS/EYE] pos_err=4mm  ori_err=0.0°
```

---

## 🚀 Running — Eye-in-Hand

Uses 4 terminals. The controller uses MoveIt Servo with a two-phase hybrid strategy.

### Terminal 1 — Gazebo + wrist camera

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch pbvs_camera ur5e_eye_in_hand_camera.launch.py target:=red_cylinder
```

> ⚠️ Do **not** run `ur_robot_driver ur_control.launch.py` at the same time. This launch already starts the UR5e inside Gazebo.

### Terminal 2 — MoveIt Servo

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_servo:=true \
  launch_rviz:=false \
  servo_config_package:=ur_moveit_config \
  servo_config_file:=$HOME/ur_ws/ur_servo_fixed.yaml
```

### Terminal 3 — Controller switch + Servo tuning

```bash
source /opt/ros/jazzy/setup.bash
ros2 control switch_controllers \
  --activate forward_position_controller \
  --deactivate scaled_joint_trajectory_controller

ros2 param set /servo_node moveit_servo.lower_singularity_threshold 100.0
ros2 param set /servo_node moveit_servo.hard_stop_singularity_threshold 200.0
```

### Terminal 4 — PBVS controller

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur_ws/install/setup.bash
ros2 launch pbvs_camera pbvs_controller.launch.py \
  mode:=eye_in_hand \
  approach_z:=0.10
```

Expected output:

```
[pbvs_controller] [Phase 1] pos_err=380.2mm  ori_err=106°  ...
[pbvs_controller] [Track] pan=-12.3° tilt=18.1° ...
[pbvs_controller] [Phase 2] pos_err=95.1mm  ori_err=85°  ...
...
✅ Target reached! Holding pose.
[pbvs_controller] pos_err=5mm  ori_err=1.7°
```

---

## ⚙️ Tuning Parameters

### Eye-to-Hand (`pbvs_controller.py`)

| Parameter | Default | Effect |
|---|---|---|
| `KP` | 0.5 | Position gain — lower = smoother, higher = faster |
| `MAX_LINEAR_STEP` | 0.04 m | Max Cartesian movement per cycle |
| `MAX_ANGULAR_STEP` | 0.15 rad | Max rotation per cycle |
| `TRAJ_DT` | 0.4 s | Time for JTC to reach each waypoint |
| `CTRL_HZ` | 5.0 Hz | Control loop frequency |
| `POS_THRESH` | 0.006 m | Position convergence threshold (6 mm) |
| `ORI_THRESH` | 0.05 rad | Orientation convergence threshold (~3°) |

If the arm **oscillates**: lower `KP` or raise `TRAJ_DT`.
If the arm is **too slow**: raise `KP` or lower `TRAJ_DT`.

### Eye-in-Hand (`pbvs_controller.py`)

| Parameter | Default | Effect |
|---|---|---|
| `KP_WRIST_PAN` | 1.2 | Wrist pan rate (horizontal tracking) |
| `KP_WRIST_TILT` | 1.2 | Wrist tilt rate (vertical tracking) |
| `MAX_WRIST_PAN` | 0.4 rad/s | Pan speed cap |
| `SWITCH_THRESHOLD` | 0.12 m | Phase 1 → Phase 2 switch distance |
| `KP_LINEAR` | 0.8 | Position gain (Phase 2) |
| `KP_ANGULAR` | 1.0 | Orientation gain (Phase 2) |
| `MAX_LINEAR` | 0.05 m/s | Max linear speed |
| `MAX_ANGULAR` | 0.3 rad/s | Max angular speed |
| `POS_THRESH` | 0.005 m | Position convergence (5 mm) |
| `ORI_THRESH` | 0.05 rad | Orientation convergence (~3°) |

---

## 🐛 Debug

```bash
# Check joint_trajectory_controller is active
ros2 control list_controllers

# Check IK service is available (eye-to-hand)
ros2 service list | grep compute_ik

# Check target is being detected
ros2 topic echo /target_pose --once

# Check stable base-frame target (eye-in-hand)
ros2 topic echo /target_pose_base --once

# Check TF (eye-in-hand)
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame

# View camera images
ros2 run rqt_image_view rqt_image_view
# Select: /camera/camera/color/image_raw
#     or: /target_detection/debug_image

# Check topic rates
ros2 topic hz /camera/camera/color/image_raw
```

---

## ❗ Common Errors

**`IK failed — MoveIt error code: -31`**
Target pose is out of reach or in collision. Move the target object closer to the robot workspace: `x: 0.3–0.8, y: ±0.4, z: 0.3–0.8`.

**`TF lookup base_link→tool0 failed`**
Terminal 1 (ur_robot_driver) is not running or not yet fully started.

**`Waiting for /compute_ik`**
Terminal 2 (ur_moveit_config) is not running yet.

**`Waiting for /joint_states`**
Terminal 1 is not running or `joint_state_broadcaster` is not active.

**`pos_err` frozen / arm not moving (eye-in-hand)**
MoveIt Servo is not running. Run `ros2 node list | grep servo` — if `/servo_node` is missing, relaunch Terminal 2.

**Camera does not see the table (eye-in-hand)**
Adjust the camera mount in `urdf/ur_gz_eih_camera.urdf.xacro`:
```xml
<xacro:arg name="eih_camera_xyz" default="0.00 0.00 0.08"/>
<xacro:arg name="eih_camera_rpy" default="0 0 0"/>
```
Tune `eih_camera_rpy` until the wrist camera points toward the objects.

---

## 📁 Package Structure

```
pbvs_camera/
├── launch/
│   ├── pbvs_controller.launch.py               # Main controller launcher (mode arg)
│   ├── external_driver_world_camera.launch.py  # Eye-to-hand: world + camera bridge
│   └── ur5e_eye_in_hand_camera.launch.py       # Eye-in-hand: Gazebo + wrist camera
├── pbvs_camera/
│   ├── pbvs_controller.py                      # Main PBVS controller (both modes)
│   ├── camera_color_depth_target_node.py       # RGB-D target detector
│   ├── static_gazebo_camera_tf.py              # Fixed camera TF broadcaster (eye-to-hand)
│   ├── static_eih_camera_tf.py                 # Wrist camera TF broadcaster (eye-in-hand)
│   └── rviz_world_markers.py                   # RViz marker publisher
├── urdf/
│   ├── realsense_camera.urdf.xacro             # RealSense camera URDF
│   ├── realsense_gazebo_plugin.gazebo          # Gazebo sensor plugin
│   └── ur_gz_eih_camera.urdf.xacro            # UR5e + wrist camera (eye-in-hand)
├── worlds/
│   ├── pick_and_place_pbvs.world               # Eye-to-hand Gazebo world
│   └── pick_and_place_pbvs_eih.world           # Eye-in-hand Gazebo world
├── rviz/
│   └── camera_pbvs.rviz                        # RViz configuration
├── resource/pbvs_camera
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 🔭 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/target_pose` | `PoseStamped` | Sub | Target pose from camera detector |
| `/target_pose_base` | `PoseStamped` | Pub | Debug: stable target in base_link frame |
| `/servo_node/delta_twist_cmds` | `TwistStamped` | Pub | Velocity command to MoveIt Servo (eye-in-hand) |
| `/joint_trajectory_controller/joint_trajectory` | `JointTrajectory` | Pub | Joint command (eye-to-hand) |
| `/camera/camera/color/image_raw` | `Image` | Sub | RGB camera stream |
| `/camera/camera/aligned_depth_to_color/image_raw` | `Image` | Sub | Depth camera stream |
| `/target_detection/debug_image` | `Image` | Pub | Annotated detection image |

---

## Author
Pavada Pavan Kumar
