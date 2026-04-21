# D435i Depth Camera Integration — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate an Intel RealSense D435i depth camera into the ROScar1 ROS2 stack on the RPi5, publishing depth + color + point cloud under the existing TF tree, and feed its point cloud into Nav2's local costmap as a `voxel_layer` so the robot can see 3D obstacles the 2D lidar misses.

**Architecture:**
- `realsense2_camera` node runs on the Pi5, publishes `/camera/camera/*` topics. Its internal frames (depth, color, IR L/R, accel, gyro) are built by `realsense2_description`'s xacro macro under a parent `d435i_link` that we attach to the chassis.
- The existing Logitech webcam stays — we rename its URDF link from `camera_link` → `webcam_link` to avoid a collision with the RealSense driver's frame.
- D435i's built-in IMU is **not fused into EKF** in this pass. STM32 IMU stays primary. Keeping the scope tight.
- A new `use_depth:=true|false` launch arg in `robot.launch.py` gates the depth camera. Default `false` so existing launch flows are unchanged until we explicitly enable it.
- Nav2 local costmap gets a `voxel_layer` observation source pointing at the D435i point cloud, with `obstacle_max_range` tuned to match the D435i's useful range (~3m).

**Tech Stack:** ROS2 Jazzy, `realsense2_camera` 4.56, `realsense2_description` xacro macros, `pointcloud_to_laserscan` (optional fallback), xacro, Nav2 voxel_layer.

**Branch:** `feat/d435i-depth-camera` off `master`. Single-instance working directory per CLAUDE.md.

---

## Prerequisites & Critical Finding

**USB bandwidth problem detected.** Running `lsusb -t` on the Pi shows the D435i on a 480M (USB 2.0) bus. The Pi5 has two USB 3.0 ports (blue connectors, Bus 003 and Bus 005 per `lsusb`) that are currently empty. D435i at USB 2.0 is capped at ~6 FPS depth at 640×480 and will throw MIPI errors under load.

**Action:** Physically move the D435i USB-C cable to a USB 3.0 (blue) port on the Pi5 before Task 1. All downstream work assumes USB 3.0 (5000M) connection. Use the short Intel-supplied USB-C → USB-A cable if the one in use is a charging cable. Tuck the lidar's CP210x serial cable into a USB 2.0 port if USB 3.0 ports are scarce — serial only needs 12M.

---

## File Structure

| File | Action | Purpose |
|---|---|---|
| `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro` | Modify | Rename existing `camera_link` → `webcam_link` (and its optical joint). Add xacro include + instantiation of `realsense2_description`'s `_d435i.urdf.xacro` macro attached at the measured mount pose. |
| `roscar_ws/src/roscar_bringup/launch/depth_camera.launch.py` | Create | Launches `realsense2_camera_node` with our params file. Gated internally by launch-arg `use_depth`. |
| `roscar_ws/src/roscar_bringup/config/realsense_params.yaml` | Create | Streams, FPS, pointcloud filters, IMU disabled. Tuned for USB 3.0 with headroom. |
| `roscar_ws/src/roscar_bringup/launch/robot.launch.py` | Modify | Add `use_depth_arg` (default `false`). IncludeLaunchDescription the depth camera launch conditionally. |
| `roscar_ws/src/roscar_bringup/config/nav2_params.yaml` | Modify | Add `voxel_layer` to `local_costmap.plugins`, with a `d435i_depth` observation source subscribed to the D435i point cloud. |
| `roscar_ws/src/roscar_bringup/CMakeLists.txt` | Modify | Ensure new launch and config files are installed. |
| `roscar_ws/src/roscar_description/package.xml` | Modify | Add `realsense2_description` as exec_depend. |
| `roscar_ws/src/roscar_bringup/package.xml` | Modify | Add `realsense2_camera` as exec_depend. |
| `tasks/todo.md` | Modify | Track progress on this plan. |
| `CLAUDE.md` | Modify | Document the new package dependency, launch arg, and Nav2 voxel_layer. |

---

## Chunk 1: Hardware + Driver Install on Pi

### Task 1: Move the D435i to a USB 3.0 port and verify

**Files:** none (hardware).

- [ ] **Step 1: Physically move the cable**

Unplug the D435i from its current USB 2.0 port. Plug it into one of the Pi5's USB 3.0 ports (blue plastic in the connector, furthest from the USB-C power jack). Use the Intel-supplied USB-C cable — a charging-only cable will force USB 2.0 or fail entirely.

- [ ] **Step 2: Verify USB 3.0 enumeration**

Run: `ssh brian@192.168.1.170 'lsusb -t'`

Expected: the RealSense device (Intel, vendor `8086`) should appear on a `5000M` bus (Bus 003 or Bus 005), not 480M.

Also run: `ssh brian@192.168.1.170 'lsusb | grep 8086'`

Expected: `ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i`

If still 480M: try the other USB 3.0 port, and confirm the cable is the Intel-supplied one.

---

### Task 2: Install realsense2_camera on the Pi

**Files:** none (system apt install).

- [ ] **Step 1: Install the driver and description packages**

Run:
```bash
ssh brian@192.168.1.170 'sudo apt update && sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description ros-jazzy-pointcloud-to-laserscan'
```

Expected: installs complete without errors. `realsense2_camera` version should be `4.56.x` per `apt-cache policy`.

- [ ] **Step 2: Verify the driver can see the device**

Run:
```bash
ssh brian@192.168.1.170 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix realsense2_camera && ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_depth:=true -p enable_color:=false -p enable_infra1:=false -p enable_infra2:=false -p enable_gyro:=false -p enable_accel:=false' &
sleep 8
ssh brian@192.168.1.170 'source /opt/ros/jazzy/setup.bash && ros2 topic list | grep camera'
ssh brian@192.168.1.170 'pkill -f realsense2_camera_node'
```

Expected: topics include `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info`. No errors like "No RealSense devices were found" or "MIPI error".

- [ ] **Step 3: Commit nothing yet** — this is environment setup; nothing to commit.

---

## Chunk 2: URDF

### Task 3: Rename existing webcam link and add D435i xacro

**Files:**
- Modify: `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro`
- Modify: `roscar_ws/src/roscar_description/package.xml`

The existing URDF has a `camera_link` and `camera_optical_frame` for the Logitech webcam. `realsense2_description` publishes a `camera_link` + internal frames by default, which would collide. We rename the Logitech's link first.

- [ ] **Step 1: Rename `camera_link` → `webcam_link` and update its optical frame**

Edit `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro`:

Replace lines 112–135 (the Camera Link section) with:

```xml
  <!-- ==================== Logitech Webcam Link ==================== -->
  <link name="webcam_link">
    <visual>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="webcam_joint" type="fixed">
    <parent link="base_link"/>
    <child link="webcam_link"/>
    <origin xyz="${camera_x} 0 ${camera_z}" rpy="0 0 0"/>
  </joint>

  <!-- Optical frame: z forward, x right, y down (REP 103) -->
  <link name="webcam_optical_frame"/>

  <joint name="webcam_optical_joint" type="fixed">
    <parent link="webcam_link"/>
    <child link="webcam_optical_frame"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>
```

- [ ] **Step 2: Search the repo for any other reference to `camera_link` or `camera_optical_frame`**

Run Grep on `camera_link` and `camera_optical_frame` across the whole repo. For each match outside `realsense2_description`'s own files and outside this URDF's new `realsense` section (which legitimately uses them), update to `webcam_link` / `webcam_optical_frame`.

Expected hit list (verify):
- `roscar_ws/src/roscar_driver/config/landmark_params.yaml` — if the landmark localizer is configured to look up `camera_optical_frame`, it needs updating to match whichever camera is doing ArUco detection. Leave as-is for now if ArUco runs on the Logitech; otherwise rename.
- `roscar_ws/src/roscar_bringup/config/camera_calibration.yaml` — this is the Logitech's calibration. Update its `frame_id` field from `camera_link` → `webcam_link`.

Hits inside the existing webcam's `camera.launch.py` should also be updated.

**Guidance:** for anything you're not sure about, stop and ask — do not blanket-rename.

- [ ] **Step 3: Add parameters for D435i mount pose**

In the "Robot Dimensions" section of the URDF, after the `lidar_z` property, add:

```xml
  <!-- D435i position (relative to base_link) — front-facing, tilted slightly down -->
  <!-- TODO: measure after physical mount — placeholder values below -->
  <xacro:property name="d435i_x" value="0.09"/>    <!-- m, forward of center -->
  <xacro:property name="d435i_y" value="0.0"/>     <!-- m, centered -->
  <xacro:property name="d435i_z" value="0.07"/>    <!-- m, above base_link -->
  <xacro:property name="d435i_pitch" value="0.175"/>  <!-- rad, ~10deg down tilt -->
```

- [ ] **Step 4: Include the RealSense xacro macro and instantiate it**

At the top of `roscar.urdf.xacro` (after the opening `<robot>` tag), add:

```xml
  <xacro:include filename="$(find realsense2_description)/urdf/_d435i.urdf.xacro"/>
```

At the bottom of the URDF, **before** the closing `</robot>`, add:

```xml
  <!-- ==================== D435i Depth Camera ==================== -->
  <!-- realsense2_description creates:
         camera_link (body), camera_depth_frame, camera_color_frame,
         camera_accel_frame, camera_gyro_frame, and their _optical variants.
       We parent `camera_link` to base_link at the mount pose. -->
  <xacro:sensor_d435i parent="base_link" name="camera" topics_ns="camera"
                      use_nominal_extrinsics="true" add_plug="false" use_mesh="true">
    <origin xyz="${d435i_x} ${d435i_y} ${d435i_z}" rpy="0 ${d435i_pitch} 0"/>
  </xacro:sensor_d435i>
```

Note: the macro's `name` param becomes the prefix for the body link (`camera_link`), and `topics_ns` should match the `camera_name` param we'll put in the driver launch so TF frames and topic namespaces agree.

- [ ] **Step 5: Add exec_depend in package.xml**

Edit `roscar_ws/src/roscar_description/package.xml`, add inside the dependencies block:

```xml
  <exec_depend>realsense2_description</exec_depend>
```

- [ ] **Step 6: Build and verify URDF parses**

On the Pi:
```bash
ssh brian@192.168.1.170 'cd ~/roscar_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select roscar_description --symlink-install'
ssh brian@192.168.1.170 'cd ~/roscar_ws && source install/setup.bash && xacro src/roscar_description/urdf/roscar.urdf.xacro > /tmp/roscar.urdf && check_urdf /tmp/roscar.urdf'
```

Expected: `Successfully Parsed XML`, and the tree should list links including `base_link`, `base_footprint`, `webcam_link`, `camera_link`, `camera_depth_frame`, `camera_color_frame`, `camera_accel_frame`, `camera_gyro_frame`, all four wheels, `imu_link`, `laser`.

- [ ] **Step 7: Commit**

```bash
git add roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro \
        roscar_ws/src/roscar_description/package.xml \
        roscar_ws/src/roscar_bringup/config/camera_calibration.yaml
git commit -m "description: rename webcam link, add D435i via realsense2_description xacro

- rename camera_link/camera_optical_frame to webcam_link/webcam_optical_frame
  to free the 'camera_link' name for realsense2_description's D435i macro
- add sensor_d435i macro mounted at front of chassis, tilted 10deg down
- d435i_x/y/z/pitch placeholders; measure and update after physical mount"
```

---

## Chunk 3: Driver config + launch

### Task 4: Create realsense_params.yaml

**Files:**
- Create: `roscar_ws/src/roscar_bringup/config/realsense_params.yaml`

Tuned for USB 3.0 with the CV pipeline in mind. Depth + color enabled (color supports Nav2's ObstacleLayer-style visualization and the dashboard). IMU disabled (STM32 IMU is primary). Align-depth-to-color enabled so the point cloud has per-point RGB.

- [ ] **Step 1: Write the file**

```yaml
/camera/camera:
  ros__parameters:
    # --- Device selection ---
    # Serial can be left empty if only one device is connected.
    serial_no: ""
    usb_port_id: ""
    device_type: ""

    # --- Frame-of-reference naming (matches the xacro macro above) ---
    camera_name: "camera"
    base_frame_id: "camera_link"

    # --- Streams ---
    enable_depth: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    enable_sync: true

    # Resolutions and framerates — balanced for USB 3.0 headroom on a Pi5.
    # Pi5 CPU is the other limit; 15 FPS keeps load reasonable.
    depth_module.profile: "848x480x15"
    rgb_camera.profile: "640x480x15"

    # --- Point cloud ---
    pointcloud.enable: true
    pointcloud.stream_filter: 2         # 2 = RS2_STREAM_COLOR (aligned point cloud)
    pointcloud.stream_index_filter: 0
    pointcloud.ordered_pc: false        # unordered is lighter
    pointcloud.allow_no_texture_points: false

    # Align depth to color so point cloud has RGB and uses color intrinsics.
    align_depth.enable: true

    # --- Filters (post-processing) ---
    # Decimation drops pointcloud density — big CPU saver on Pi5.
    decimation_filter.enable: true
    decimation_filter.filter_magnitude: 2

    # Spatial + temporal filters clean noise. Leave defaults enabled.
    spatial_filter.enable: true
    temporal_filter.enable: true
    hole_filling_filter.enable: false

    # --- IMU: DISABLED for now ---
    enable_gyro: false
    enable_accel: false
    unite_imu_method: 0

    # --- Misc ---
    publish_tf: false         # we own TF via URDF; driver's internal TF would duplicate
    tf_publish_rate: 0.0
    initial_reset: false
    diagnostics_period: 1.0
```

**Why `publish_tf: false`:** the URDF's `realsense2_description` xacro already creates `camera_depth_frame`, `camera_color_frame`, etc., with nominal extrinsics. Letting the driver re-publish the same transforms causes "extrapolation into the future" warnings.

- [ ] **Step 2: Install-via-CMake**

Edit `roscar_ws/src/roscar_bringup/CMakeLists.txt`. Find the `install(DIRECTORY config ...)` line. Confirm that directory-install is already in place (it is, since `ekf.yaml` etc. get installed). No change needed if the whole `config/` dir is installed.

- [ ] **Step 3: Don't commit yet** — commit with launch file in next task.

---

### Task 5: Create depth_camera.launch.py

**Files:**
- Create: `roscar_ws/src/roscar_bringup/launch/depth_camera.launch.py`

- [ ] **Step 1: Write the launch file**

```python
"""Launch Intel RealSense D435i depth camera."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('roscar_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        realsense_node,
    ])
```

- [ ] **Step 2: Ensure launch dir install is present**

In `roscar_ws/src/roscar_bringup/CMakeLists.txt`, confirm there's an `install(DIRECTORY launch ...)` entry. There is — all launch files go in.

- [ ] **Step 3: Add exec_depend to package.xml**

Edit `roscar_ws/src/roscar_bringup/package.xml`, add:

```xml
  <exec_depend>realsense2_camera</exec_depend>
```

- [ ] **Step 4: Build**

```bash
ssh brian@192.168.1.170 'cd ~/roscar_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select roscar_bringup --symlink-install'
```

Expected: clean build.

- [ ] **Step 5: Launch standalone and verify topics**

```bash
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 launch roscar_bringup depth_camera.launch.py' &
sleep 10
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 topic list | grep camera' | tee /tmp/d435i_topics.txt
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 topic hz /camera/camera/depth/color/points --window 50'
```

Expected topics include (at least):
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`
- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/color/points`

Expected hz on `depth/color/points`: ~15 Hz after decimation (±5 Hz tolerance). CPU on Pi5 should stay below 70% on the cores running the driver — check with `ssh brian@192.168.1.170 'top -bn1 | head -20'`.

Kill the launch:
```bash
ssh brian@192.168.1.170 'pkill -f realsense2_camera_node'
```

- [ ] **Step 6: If topics missing or hz below 5 — diagnose before proceeding**

Common failure modes:
  - No topics at all: driver failed to open device. Check `dmesg | tail -20` for USB errors. Re-verify USB 3.0.
  - hz=0 on pointcloud but image topics fine: `pointcloud.enable` or `align_depth.enable` mis-set. Re-check params.
  - hz spiky / MIPI errors in log: USB bandwidth. Reduce color profile to `424x240x15`.

- [ ] **Step 7: Commit**

```bash
git add roscar_ws/src/roscar_bringup/config/realsense_params.yaml \
        roscar_ws/src/roscar_bringup/launch/depth_camera.launch.py \
        roscar_ws/src/roscar_bringup/package.xml
git commit -m "bringup: add D435i depth camera launch + params

- realsense_params.yaml: depth+color@15fps on USB 3.0, decimation filter,
  IMU disabled, publish_tf=false (URDF owns TF)
- depth_camera.launch.py: standalone launch of realsense2_camera_node"
```

---

### Task 6: Wire depth camera into robot.launch.py behind a flag

**Files:**
- Modify: `roscar_ws/src/roscar_bringup/launch/robot.launch.py`

- [ ] **Step 1: Add `use_depth` launch argument**

After the `use_camera_arg` block (around line 31), add:

```python
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='false',
        description='Launch the Intel RealSense D435i depth camera',
    )
```

- [ ] **Step 2: Add conditional include**

After the `camera_launch` block (around line 73), add:

```python
    # -- D435i depth camera --
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'depth_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_depth')),
    )
```

- [ ] **Step 3: Add both to the returned LaunchDescription list**

Update the `return LaunchDescription([...])` block to include `use_depth_arg` and `depth_camera_launch`.

- [ ] **Step 4: Build + launch with use_depth:=true**

```bash
ssh brian@192.168.1.170 'cd ~/roscar_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select roscar_bringup --symlink-install'
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 launch roscar_bringup robot.launch.py use_depth:=true' &
sleep 15
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 run tf2_ros tf2_echo base_link camera_depth_frame --timeout 3'
ssh brian@192.168.1.170 'pkill -f ros2'
```

Expected: `tf2_echo` prints a transform with roughly the XYZ + pitch matching our URDF placeholder (x≈0.09, z≈0.07, pitch≈0.175).

- [ ] **Step 5: Default-off smoke test**

Launch without the flag and confirm nothing depth-related spins up:
```bash
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 launch roscar_bringup robot.launch.py' &
sleep 10
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 topic list | grep camera/camera | wc -l'
ssh brian@192.168.1.170 'pkill -f ros2'
```

Expected: `0` (no `/camera/camera/*` topics when `use_depth=false`).

- [ ] **Step 6: Commit**

```bash
git add roscar_ws/src/roscar_bringup/launch/robot.launch.py
git commit -m "bringup: gate D435i depth camera behind use_depth launch arg (default false)"
```

---

## Chunk 4: Nav2 voxel_layer integration

### Task 7: Add voxel_layer observation source to local costmap

**Files:**
- Modify: `roscar_ws/src/roscar_bringup/config/nav2_params.yaml`

We add a `voxel_layer` plugin to the local costmap only (global costmap stays on lidar). The D435i stream gets registered as an observation source. Because the point cloud is in `camera_depth_frame`, Nav2 uses TF to project it into `base_link` and then into the costmap.

- [ ] **Step 1: Locate the local_costmap plugin list**

In `nav2_params.yaml`, search for `local_costmap:` → `ros__parameters:` → `plugins:`. The existing list currently includes at least `obstacle_layer` and `inflation_layer`.

- [ ] **Step 2: Add voxel_layer to the plugin list**

Change:
```yaml
      plugins: ["obstacle_layer", "inflation_layer"]
```

to:
```yaml
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
```

(`inflation_layer` must remain last.)

- [ ] **Step 3: Add voxel_layer config block under local_costmap.ros__parameters**

Insert this block after the `obstacle_layer` block and before `inflation_layer`:

```yaml
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        footprint_clearing_enabled: True
        max_obstacle_height: 1.5
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.1
        z_voxels: 16            # 16 * 0.1 = 1.6 m vertical range
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: d435i_depth
        d435i_depth:
          topic: /camera/camera/depth/color/points
          max_obstacle_height: 1.5
          min_obstacle_height: 0.05    # ignore ground-plane noise
          obstacle_max_range: 3.0      # D435i useful depth range
          obstacle_min_range: 0.2
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

- [ ] **Step 4: Launch SLAM+Nav with depth on, verify voxel layer receives data**

```bash
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 launch roscar_bringup slam_nav.launch.py use_depth:=true' &
sleep 25
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 topic hz /local_costmap/voxel_marked_cloud --window 20'
ssh brian@192.168.1.170 'source ~/roscar_ws/install/setup.bash && ros2 node info /local_costmap/local_costmap | grep -A2 Subscribers | grep points'
```

Note: `slam_nav.launch.py` needs to support `use_depth` — if it passes args through via `LaunchConfiguration`, it should just work. If it doesn't, patch it the same way as `robot.launch.py`.

Expected:
- `/local_costmap/voxel_marked_cloud` publishing at >1 Hz
- Node info shows a subscription to `/camera/camera/depth/color/points`

- [ ] **Step 5: Visual verification in rviz (from WSL2)**

In WSL2:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2 -d /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_description/rviz/roscar.rviz
```

Add displays (if not already present):
- PointCloud2 → `/camera/camera/depth/color/points`
- PointCloud2 → `/local_costmap/voxel_marked_cloud`
- Map → `/local_costmap/costmap`

Place a chair or box in front of the robot. Confirm:
- Depth point cloud shows the object.
- `voxel_marked_cloud` lights up cells where the object is.
- Local costmap (2D projection) shows inflation around the object.

- [ ] **Step 6: Commit**

```bash
git add roscar_ws/src/roscar_bringup/config/nav2_params.yaml
git commit -m "nav2: add voxel_layer with D435i observation source to local costmap

Lets Nav2 see 3D obstacles the 2D lidar misses (tables, chair seats,
cables). 10cm vertical voxel resolution, 1.6m vertical range, 3m horizontal."
```

---

## Chunk 5: Documentation + tracking

### Task 8: Update CLAUDE.md and tasks/todo.md

**Files:**
- Modify: `CLAUDE.md`
- Modify: `tasks/todo.md`

- [ ] **Step 1: Update CLAUDE.md**

Add the D435i to the top "Project Overview" hardware list: `Intel RealSense D435i depth camera (USB 3.0)`.

Add a new row to the "Key Topics" table:
| `/camera/camera/depth/color/points` | PointCloud2 | realsense2_camera | D435i depth point cloud with RGB |
| `/camera/camera/depth/image_rect_raw` | Image | realsense2_camera | D435i depth image |
| `/camera/camera/color/image_raw` | Image | realsense2_camera | D435i color image |
| `/local_costmap/voxel_marked_cloud` | PointCloud2 | nav2_costmap_2d | 3D obstacles in local costmap |

Add to the "Critical Files" table:
| `roscar_ws/src/roscar_bringup/launch/depth_camera.launch.py` | D435i depth camera launch |
| `roscar_ws/src/roscar_bringup/config/realsense_params.yaml` | D435i driver config |

Add a short "Depth Camera (D435i)" section after the Nav2 velocity limits table, covering:
- Launch: `ros2 launch roscar_bringup robot.launch.py use_depth:=true`
- Mount pose placeholders in URDF (TODO to measure)
- USB 3.0 requirement
- STM32 IMU remains primary (D435i IMU disabled in config)

Tick the existing TODO entries if any apply.

- [ ] **Step 2: Add review entry to tasks/todo.md**

Append a "D435i Integration" section summarizing:
- What landed (URDF, driver launch, Nav2 voxel_layer)
- What didn't (dashboard depth toggle — deferred)
- Known placeholders (mount XYZ + pitch — needs physical measurement)

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md tasks/todo.md
git commit -m "docs: document D435i depth camera integration"
```

---

### Task 9: Open a PR (or merge) — decide at end

**Files:** none.

- [ ] **Step 1: Push the branch**

```bash
git push -u origin feat/d435i-depth-camera
```

- [ ] **Step 2: Decide merge strategy**

Offer the user two options:
1. **Merge to master now** — if all Chunk 1–4 verifications passed and they're ready to use it live.
2. **Keep as branch** — if any placeholder values (mount pose) haven't been measured yet or voxel_layer needs tuning with real obstacles.

---

## Out of scope (deferred)

The following were considered and left for a follow-up plan. Not blocking for basic depth-in-costmap functionality.

- **D435i IMU → EKF fusion** — adds a second IMU to robot_localization. Useful for VIO later but requires careful extrinsic calibration. Explicitly skipped.
- **Dashboard depth stream toggle** — add a `DEPTH` mode to `aio-camera.js` that routes to web_video_server's `/camera/camera/depth/image_rect_raw?type=mjpeg` stream. Straightforward but cosmetic.
- **pointcloud_to_laserscan fallback** — if voxel_layer perf is too heavy on the Pi5, we could flatten the depth cloud to a pseudo-laserscan at a specific height. Only add if needed.
- **Physical mount measurement** — placeholder `d435i_x/y/z/pitch` values in the URDF. Update with calipers once the camera is physically affixed to the chassis.
- **Firmware update** — if the Viewer earlier flagged an old firmware, update via Windows before re-deploying.
