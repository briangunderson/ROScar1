# D435i Obstacle Avoidance & Other Enhancements

**Date:** 2026-04-30
**Status:** Draft — implementation phased (Phase A.1 first)
**Scope:** Use the Intel RealSense D435i for things beyond just being the color camera — primarily 3D obstacle detection in Nav2 costmaps, plus future cliff detection and distance-keyed object detection.

---

## 1. Problem Statement

Today the robot navigates with a single 2D laser plane (RPLIDAR C1 at z ≈ 0.295 m above the floor). The D435i streams depth at 640×480 × 15 Hz and a colored point cloud is published to `/camera/camera/depth/color/points`, but **nothing consumes the depth data for navigation**. The Nav2 local and global costmaps both list `observation_sources: scan` only.

This means the robot is blind to:

| Obstacle class | Why lidar misses it | Real-world examples |
|---|---|---|
| **Below lidar plane (< 0.295 m)** | Beam passes overhead | Coffee table edges, chair seats, low boxes, dog beds, cables on the floor |
| **Above lidar plane but with low base** | Beam passes underneath at the height it's sampled | Bar-stool seats with thin legs, hanging shelves with narrow brackets |
| **Negative space (cliffs/stairs)** | Lidar only marks reflections, not absence of floor | Stairs, ledges, ramps |
| **Glass / specular** | Lidar passes through or scatters | Glass tables, polished steel, mirrors |

The D435i can see all of these. Putting depth data into the costmap is the single biggest navigation reliability win available right now.

## 2. Design Goals

| Priority | Goal | Success Criteria |
|---|---|---|
| P0 | Detect sub-lidar-plane obstacles in local costmap | Robot stops / replans when approaching a 25 cm tall box |
| P0 | No CPU regression on Pi5 | Load avg stays < 3.0 in nav mode; controller @ 10 Hz, local costmap @ 5 Hz |
| P0 | Fail safe — disabling depth must keep nav working on lidar alone | `use_depth:=false` boots cleanly with no costmap errors |
| P1 | Stay within USB 3.0 bandwidth headroom | Depth + color don't wedge the D435i (already verified on master) |
| P1 | Keep map quality intact | SLAM mapping stays lidar-only (depth is local-costmap-only) |
| P2 | Catch cliffs / dropoffs | Robot refuses to drive off a stairway edge |
| P3 | Distance-tagged YOLO bounding boxes | Dashboard shows "person 1.4 m" instead of just "person" |

## 3. Why a phased approach

The D435i can deliver in many ways, but each option has a different CPU cost and integration risk on the Pi5. Starting cheap and verifying we have headroom is the right move:

| Approach | CPU | Integration risk | Captures |
|---|---|---|---|
| **A.1 — `pointcloud_to_laserscan` virtual scan + 2nd source on local costmap obstacle_layer** | Low | Low — existing plugin, single new param block | Single-height slice of depth volume; configurable height range |
| **A.2 — Cliff detection custom node** | Low | Medium — new ROS node, simple geometry | Negative-space (missing floor) hazards |
| **B — `voxel_layer` plugin on local costmap** | Medium | Low — first-class Nav2 plugin | Full 3D obstacle volume; one config does multi-height |
| **C — RTAB-Map RGBD-D odometry / dense mapping** | High | High — adds an EKF input + dense map publishing | Visual odometry redundancy + 3D occupancy |
| **D — Distance-keyed YOLO** | Low | Low — post-processing on existing detections | Better dashboard UX, person/object range |

**Phase A.1 first.** It delivers the P0 win at minimum CPU and minimum risk. Phase A.2 is a small follow-up that reuses the same depth pipeline. B is for when we have CPU headroom. C and D are nice-to-haves.

## 4. Phase A.1 — Implementation

### 4.1 Topology

```
realsense2_camera ─── /camera/camera/depth/color/points ───┐
(already running)                                          │
                                                           v
                          pointcloud_to_laserscan ─── /scan_depth (sensor_msgs/LaserScan)
                          (new node, NEW launch in        │
                           depth_camera.launch.py)        │
                                                          v
                                          local_costmap.obstacle_layer
                                                  ├─ scan        (lidar, existing)
                                                  └─ scan_depth  (new)
```

### 4.2 New node: `pointcloud_to_laserscan_node`

Added inside `depth_camera.launch.py` so it lifecycles with the depth camera.

```python
Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='depth_to_scan',
    parameters=[{
        'target_frame': 'base_link',
        'transform_tolerance': 0.1,
        'min_height': 0.05,        # 5 cm above floor — skip floor noise
        'max_height': 0.50,        # up to top of robot body (below mast/lidar)
        'angle_min': -0.76,        # -43.5 deg, matches D435i depth FOV
        'angle_max':  0.76,        # +43.5 deg
        'angle_increment': 0.0087, # ~0.5 deg, ~175 rays
        'scan_time': 0.0667,       # 1/15 Hz
        'range_min': 0.20,         # D435i min usable depth
        'range_max': 3.0,          # D435i useful depth range
        'use_inf': False,          # Mark invalid as range_max + 1 (out-of-range)
        'inf_epsilon': 1.0,
    }],
    remappings=[
        ('cloud_in', '/camera/camera/depth/color/points'),
        ('scan', '/scan_depth'),
    ],
)
```

### 4.3 Costmap config change

Only the local costmap gets the new source. Global stays lidar-only (long-range planning needs lidar's 12 m, D435i is 3 m max).

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      ...
      obstacle_layer:
        observation_sources: "scan scan_depth"
        scan:
          topic: /scan
          # ... unchanged ...
        scan_depth:
          topic: /scan_depth
          max_obstacle_height: 1.0    # ignore anything > 1m (above the robot)
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.0
          inf_is_valid: false         # match use_inf: False above
```

### 4.4 `use_depth:=false` fail-safe

When `use_depth:=false`, the depth camera launch is skipped, which means `/scan_depth` is never published. We need the obstacle_layer to behave gracefully:

- **Default behavior:** Nav2's obstacle_layer simply has no observations from a missing source — the layer doesn't crash, it just doesn't mark anything from that source.
- **Verification:** Boot `slam_nav use_depth:=false` and confirm `local_costmap` lifecycle activates and the robot can still navigate using lidar alone.

### 4.5 Bug fix carry-over: `slam_nav.launch.py` doesn't forward `use_depth`

While auditing the depth pipeline I noticed `slam_nav.launch.py` hard-codes `use_lidar: 'true'` but never forwards `use_depth` to `robot.launch.py`. Today this is fine because `robot.launch.py` defaults to `use_depth: 'true'`, but it's inconsistent with `slam.launch.py` and `navigation.launch.py` which both do forward it. This PR fixes that for consistency.

## 5. Phase A.2 — Cliff detection (next PR)

A small ROS node `cliff_detector_node`:

- Subscribe to `/camera/camera/depth/image_rect_raw`
- Project a fan of rays at the floor plane in the camera frame (using camera intrinsics + camera_link → base_footprint TF)
- For each ray, check if there's a return from the depth image close to the expected ground-plane distance
- If a ray comes back significantly *farther* than expected (or is invalid), that's a cliff
- Publish a virtual obstacle (custom topic, e.g. `/cliff_obstacles` as a `PointCloud2`) — feed it into the obstacle_layer as a third source

Expected size: ~150 lines of Python. Can be tested by pointing the camera at a step or table edge.

## 6. Phase B — Voxel layer (later, if A.1 doesn't cover enough)

Switch the local costmap from a flat obstacle_layer to a voxel_layer that consumes the full point cloud directly. Captures multi-height obstacles (a chair with seat + legs + back) in one config block. The feat/d435i-depth-camera branch already has a draft of this config we can lift.

## 7. Phase C/D — Visual odometry + distance-keyed YOLO

Future. Visual odometry is high-CPU; distance-keyed YOLO is a small post-processing tweak in `roscar_cv` that pairs each YOLO bounding box with a depth median over the box's pixel region.

## 8. Test plan (Phase A.1)

- [ ] **`use_depth:=true` boots cleanly** — `slam_nav.launch.py` brings up the full stack, `/scan_depth` publishes at ~15 Hz
- [ ] **Virtual scan looks reasonable in RViz** — overlay `/scan` (red) and `/scan_depth` (blue), drive the robot near a low table and confirm `/scan_depth` registers obstacles where `/scan` doesn't
- [ ] **Local costmap shows fused obstacles** — costmap displays markings from both lidar and depth
- [ ] **Robot avoids a low obstacle** — place a 25 cm cardboard box in front of the robot, send a goal beyond it, verify the controller plans around it
- [ ] **`use_depth:=false` boots cleanly** — same launch with depth disabled; lifecycle activates, robot navigates lidar-only without errors
- [ ] **Pi5 load** — `top` shows load avg < 3.0 with full nav + depth running
- [ ] **No depth wedging** — D435i depth stream stays at 15 Hz for 5+ minutes (no ASIC-temp errors in journalctl)

## 9. Rollback

Revert the PR. The depth camera, lidar, costmaps, and SLAM all keep working — they're additive layers.

## 10. Risks

| Risk | Mitigation |
|---|---|
| `pointcloud_to_laserscan` adds too much CPU | Keep min/max_height tight; angle_increment moderate (~175 rays); decimation already applied upstream |
| Mast extrusion in camera FOV | Camera mounted on front face of mast (mast at -X relative to camera); shouldn't appear in depth |
| Depth noise at long range | `obstacle_max_range: 3.0` matches D435i useful range, blocks bad data beyond |
| Costmap inflation makes everything red | Existing `inflation_radius: 0.10` is already snug; verify visually |
| Floor noise marked as obstacle | `min_height: 0.05` (5 cm above floor) skips floor returns |
