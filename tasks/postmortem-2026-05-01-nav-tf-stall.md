# Post-mortem: nav goal failure → infinite stale-TF spam (2026-05-01)

## TL;DR

Three things compounded into a navigation failure on 2026-05-01 at 14:26 EDT:

1. **The first nav goal failed** because slam_toolbox's `map → odom` TF
   was 524 ms old when the controller tried to use it — barely past our
   `transform_tolerance: 0.5 s`.
2. **slam_toolbox lagged** because the Pi5 was over-committed: load avg
   8.92, all 4 cores saturated 2.2×. Today's PRs added ~21 % CPU on top
   of an already-tight nav stack.
3. **After the goal aborted, nav2_container got stuck** retrying the
   same stale pose timestamp (14:26:07.546) every second indefinitely
   — even after the BT and ActionServer reported "Goal failed". The
   spam only stopped when we tore down the whole stack via `set_mode
   idle`.

## Timeline

| Time | Event |
|------|-------|
| 14:19:30 | slam_nav came up cleanly |
| 14:23:17 | dashboard reconnected, nav stack idle |
| 14:26:05.973 | dashboard sent goal `(-0.45, -0.97)` |
| 14:26:05.989 | launch_manager: "Nav goal accepted" |
| 14:26:06.000 | controller_server received goal |
| 14:26:07.103 | controller_server: "Passing new path to controller" |
| 14:26:08.103 | controller_server replanned again (1 s later) |
| 14:26:08.103 | **`tf_help: Transform data too old when converting from odom to map`** |
| 14:26:08.103 | tf_help: `Data time 1777659968.07, Transform time 1777659967.546` (524 ms gap) |
| 14:26:08.104 | controller_server: "Unable to transform robot pose into global plan's frame" |
| 14:26:08.104 | controller_server: aborting follow_path action |
| 14:26:08.136 | bt_navigator: aborting navigate_to_pose; "Goal failed" |
| 14:26:09.030 | launch_manager: status=6 (ABORTED) |
| 14:26:25 | dashboard's nav-tuning sliders re-set FollowPath.* params (innocuous) |
| 14:26:28 | second goal `(0.32, -0.65)` — accepted, **aborted within 100 ms** |
| 14:26:55 | third goal `(-0.26, 0.39)` — accepted, **aborted within 100 ms** |
| 14:26:55 | **infinite spam starts**: `transformPoseInTargetFrame: Extrapolation Error … Requested time 1777659967.546176` |
| 14:35:15 | `set_mode idle` finally tore down nav2_container, spam stopped |

The spam was constant: ~1 error/sec, requesting the SAME stale pose
timestamp 14:26:07.546 — the time of the LAST GOOD `map→odom` TF
before the original goal failed. 9 minutes of identical errors.

## Root causes

### 1. CPU pressure on the Pi5

`top` snapshot during the spam:

| Process | CPU % | Note |
|---------|-------|------|
| nav2_container | 49 % | controller + planner + costmaps + bt_navigator + recovery |
| realsense2_camera | 28 % | depth at 640×480×15, color at 424×240×30, decimation_filter=2 |
| **detection_world** | **15 %** | **NEW today (PR #33)** |
| driver_node | 14 % | mecanum FK + 50 Hz cmd_vel ramp + IMU bias correction |
| launch_manager | 10 % | spawn / monitor children |
| web_video_server | 7 % | MJPEG encode |
| **depth_to_scan** | **6 %** | **NEW today (PR #28)** |
| ekf_node | 4 % | |
| _other_ | ~30 % | |

Total: ~163 % across 4 cores. Load avg 8.92.

**The two new D435i nodes added ~21 % CPU.** That's the tipping point —
slam_toolbox's TF publishing got starved enough to miss the controller's
500 ms tolerance window by 24 ms.

### 2. Tight transform_tolerance

`controller_server.FollowPath.transform_tolerance: 0.5` (in
`nav2_params.yaml`, line 142). At 10 Hz controller frequency on a
heavily loaded system, scheduling jitter alone can push tf age past
500 ms. The first failure was 524 ms — we were sitting right on the
edge.

### 3. nav2 doesn't fully clean up on goal abort

Even after `bt_navigator: Goal failed` and the action server reported
ABORTED status, SOMETHING inside `nav2_container` kept calling
`transformPoseInTargetFrame` with a fixed 14:26:07 timestamp — for
NINE minutes, 1 Hz, until we killed the container.

The exact callsite is unidentified. `transformPoseInTargetFrame` is in
`nav2_util` and used by:
- `nav2_costmap_2d::CostmapLayer` clearing logic
- `nav2_amcl` (not us — we use slam_toolbox)
- Several behavior plugins (spin, backup)
- The DWB controller's `getPathLength`/`projectPose` helpers

The persistent same-timestamp pattern suggests a SOURCE_DATA pose
cached from the last successful transform attempt, then retried in
some recovery loop that nobody told to stop.

This may be a nav2 upstream bug or a misconfiguration in our recovery
behavior tree. It deserves a separate investigation.

## Fixes shipped in this PR

1. **`nav2_params.yaml`**: bump `controller_server.FollowPath.transform_tolerance` from `0.5` → `1.0`. Gives 2× headroom on Pi5 jitter.
2. **`detection_world_node.py`**: rate-limit input processing to ~3 Hz max. The output already publishes at 2 Hz; processing every 15 Hz YOLO frame just wasted CPU for no UI gain.

Combined effect: ~10 % of Pi CPU returned, and nav has 2× more tolerance for slam_toolbox jitter.

## What's NOT fixed in this PR

- The nav2_container stuck-retry-on-aborted-goal bug. Worth a separate dive — either find the callsite or upstream issue. For now the workaround is: when nav gets stuck spamming TF errors, `set_mode idle` will clear it (but kills the whole stack).
- The pi5 fundamentally being under-resourced for the full nav + D435i + CV pipeline. Long-term answer is to move some load off-board (CV already is) or scale back features.

## Recovery procedure (if it happens again)

```bash
# Stop the stack — clears the stuck nav2_container state
ros2 service call /web/set_mode roscar_interfaces/srv/SetMode \
  '{mode: idle, map_path: "", use_depth: false}'

# Wait for load to settle (~30 s)
ssh brian@<pi> uptime

# Re-launch
ros2 service call /web/set_mode roscar_interfaces/srv/SetMode \
  '{mode: slam_nav, map_path: "", use_depth: true}'
```

## How to confirm the fixes are working

After this PR merges + Pi pulls + restart:
- Send a nav goal from the dashboard
- Check `journalctl -u roscar-web | grep -c 'Transform data too old'` → should be 0 in normal operation
- Watch `top` — `detection_world` should be ≤ 5 % CPU (down from 15 %)
- Pi load avg in nav mode should be < 7 (down from 8–9)
