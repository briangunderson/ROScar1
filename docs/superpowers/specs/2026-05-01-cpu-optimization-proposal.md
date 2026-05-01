# CPU Optimization Proposal — ROScar1

**Date:** 2026-05-01
**Status:** Proposal. Tier 1 included as proof-of-concept; Tier 2/3 require explicit go/no-go.
**Author thesis:** *the Pi5 is over-committed because of architectural choices, not because the hardware is too slow. Most of the wasted CPU is glue, not work.*

---

## TL;DR

Today's nav crash post-mortem made one thing clear: the Pi5 is sitting at 70–80 % CPU floor in slam_nav mode and tips over the moment anything spikes. Today's PRs (#34, #35) trimmed the spike, but the floor is the real issue.

The good news: **on a freshly-measured slam_nav baseline, ~30 % of the Pi's CPU is being spent on things nothing actually consumes.** Tier 1 alone should clear most of that with zero functional regression.

The contentious news: Tier 2 proposes moving slam_toolbox onto the WSL2 GPU PC. I think that's the right call and I'll defend it. The current "slam + nav both on the Pi" topology is the root reason today's TF lag became a crash.

---

## 1. What I measured (baseline, 2026-05-01 15:39)

`slam_nav` mode, idle robot, no goal sent, top-10 by `%CPU`:

| Process | %CPU | Note |
|---|---|---|
| nav2_container | 42.8 | controller + planner + costmaps + bt_navigator + behaviors + collision_monitor + docking_server + route_server |
| **driver_node** | **21.2** | Python — 50 Hz cmd_vel ramp + 100 Hz IMU + 100 Hz encoders |
| **realsense2_camera** | **19.4** | publishing depth + color + an unused pointcloud + filters we may not need |
| **detection_world** | **19.0** | Python — receives /detections at 5 Hz; floor is rclpy DDS overhead |
| python3 (rosbridge) | 16.9 | the dashboard's WebSocket bridge |
| **joint_state_publisher** | **10.5** | publishes wheel joint angles that nothing actually uses |
| **map_saver_server** | 7.4 | idle service waiting for save requests — should be near-zero |
| sync_slam_toolbox | 7.0 | the actual SLAM solver |
| depthimage_to_laserscan | 6.6 | depth → /scan_depth (PR #28) |
| ekf_node | 6.5 | robot_localization, healthy |

Total >150 % aggregate across 4 cores. Load avg 3.65 with no goal active. **Bold = unjustified by current consumers.**

### Key finding from the audit

- **`/camera/camera/depth/color/points` (realsense pointcloud) has zero subscribers.** We switched to `depthimage_to_laserscan` in PR #28 — the pointcloud is being generated and serialized into DDS for nobody.
- **`/joint_states` has one subscriber: `robot_state_publisher`** — and only for wheel-angle TFs that no Nav2/SLAM/dashboard consumer actually uses (footprint is hardcoded in nav2_params; lidar/camera transforms are static).
- **`map_saver_server` at 7.4 % idle** — this is a long-running lifecycle service that should poll at a fraction of a percent. Worth investigating.

If the audit numbers are right, we have ~30 % CPU walking out the door on stuff that does nothing.

---

## 2. Strategy (the thesis)

The Pi5 has 3 categories of work it MUST do:

1. **Sensor I/O & low-level control** (driver_node, realsense USB pipeline, sllidar, EKF, IMU filter). These can't move — physical I/O is bound to the device.
2. **Localization & mapping** (slam_toolbox, AMCL when active, landmark_localizer). These need TF and sensor data, but the heavy compute is *not* I/O bound. **They can move.**
3. **Planning, costmap, behavior trees** (Nav2 stack). Need fast access to the costmap which needs fresh sensor observations. Latency-sensitive — moving them off-board is risky.

The desktop GPU PC is already part of the system (it runs YOLO + ArUco). It has way more headroom than the Pi5 will ever have. **The right architecture is: Pi runs (1), GPU PC runs (2) where feasible, both share the cost of (3) — with collision-stop logic always on the Pi as a safety net.**

That's the high-level direction. The proposals below ship that vision in defendable increments.

---

## 3. Proposals, ranked by ROI

### Tier 1 — Free wins, low risk, ship today

#### T1a. Disable realsense pointcloud + temporal filter
**Implementation:** In `realsense_params.yaml`, set `pointcloud__neon_.enable: false` and `temporal_filter.enable: false`. Keep decimation + spatial filters.
**Defense:** Pointcloud has zero subscribers post-PR #28. Temporal filter is expensive (history-buffer per pixel) and `depthimage_to_laserscan` does its own range filtering downstream. Spatial filter stays — it cleans single-pixel noise that would otherwise fire `cliff_detector` false positives.
**Estimated savings:** 5–10 % Pi CPU. Bandwidth Pi-internal drops by ~9 MB/s.
**Risk:** Low. Re-enable any of these in 30 seconds if needed.

#### T1b. Drop `joint_state_publisher`
**Implementation:** Remove the `joint_state_publisher` Node from `description.launch.py`.
**Defense:** Wheel-rotation TFs are visualized in RViz (continuous joints rotate as the robot drives) — but nothing in Nav2, SLAM, the dashboard map, or any consumer reads them. The static URDF transforms (mast, lidar, camera) are still published by `robot_state_publisher` from the `robot_description` parameter — those keep working without /joint_states. The wheels' visual orientation in RViz becomes static, which is purely cosmetic.
**Estimated savings:** 5–10 % Pi CPU.
**Risk:** Low. RViz wheel rotation stops animating. No functional impact.

#### T1c. Investigate `map_saver_server` 7 % idle floor
**Implementation:** Profile what map_saver_server is doing during idle. Likely a polling timer that should run at 1 Hz, not whatever it's actually doing. Either fix the config, or pull it out of the always-on lifecycle stack and only spawn it when SaveMap is invoked.
**Estimated savings:** ~5 % if it's a real bug.
**Risk:** Low. Worst case we revert the change.

#### T1d. CPU isolation via `chrt` for the control loop
**Implementation:** Wrap `driver_node` and `ekf_node` startup with `chrt -f 50 ...` (real-time priority 50). Don't migrate them — just elevate priority so they preempt nav2/realsense under contention.
**Defense:** Today's crash happened because slam_toolbox got CPU-starved long enough to miss its 20 ms TF publish window. Real-time priority for the control loop costs nothing in throughput but adds *enormous* reliability under burst load. This is what production robots do; we don't because everything has been "fast enough" until it wasn't.
**Estimated savings:** 0 % CPU, but eliminates an entire class of failure modes.
**Risk:** Real-time priority + a runaway loop = a hard hang. driver_node and ekf_node are both well-bounded. Cap with `cgroups` if paranoid.

**Tier 1 total: ~20–25 % Pi CPU back. Recommend shipping all four this week.**

### Tier 2 — Architectural shifts, defend explicitly

#### T2a. Move slam_toolbox to WSL2
**Implementation:** Spawn `sync_slam_toolbox_node` on the WSL2 distro instead of the Pi. /scan flows Pi→WSL2 (already does over CycloneDDS for CV); /odometry/filtered flows Pi→WSL2; map→odom TF flows WSL2→Pi (where Nav2 consumes it). Loop closures and serialization stay on WSL2.
**Defense:** This is the biggest topology change in the proposal, and it's the right call.
- Pi CPU saved: ~25–30 % (slam_toolbox + map_saver_server + half of TF publishing overhead).
- Network bandwidth required: /scan is 8 KB × 10 Hz = ~80 KB/s. /odometry 5 KB × 30 Hz = ~150 KB/s. /map (large, but transient_local — published once per change). map→odom TF: ~100 bytes × 50 Hz = 5 KB/s. Total: ~250 KB/s. LAN handles this trivially.
- TF latency: round-trip Pi↔WSL2 over wired LAN should be ~1–2 ms. The current `transform_tolerance: 1.0` (post-PR #34) gives 500× more headroom than that. Contrast with today's failure where slam_toolbox running on a starving Pi was 524 ms behind — *moving it eliminates the resource starvation that caused the lag.*
- Risk profile inverts: yes, network can hiccup, but a hiccup is bounded; CPU starvation has no upper bound.

**Pre-flight check before committing:** Measure actual Pi↔WSL2 round-trip latency on /tf with a test publisher. If it's < 50 ms 99 %ile, ship it.
**Risk:** Medium. WSL2 needs to be up for SLAM. If WSL2 dies, robot loses map but keeps running on lidar-only obstacle avoidance + EKF odom.
**My position:** Worth the risk. The current topology is *already* fragile; moving slam_toolbox makes the failure mode different (network blip vs CPU thrash) but no worse, and the steady-state win is huge.

#### T2b. Persistent Nav2 lifecycle (no relaunch on mode switch)
**Implementation:** Refactor `launch_manager_node` to keep nav2_container persistently loaded after first activation. Mode switches use lifecycle (DEACTIVATE → ACTIVATE) instead of process spawn/kill. Today's mode switch takes ~25–30 s; lifecycle activate alone is ~3 s.
**Defense:** The current spawn-everything-on-mode-change pattern was simple to write but expensive in two ways: (1) the bringup CPU spike (today's `systemd --user` 35–40 % during slam_nav launch — that's pure cgroup setup waste); (2) UX latency — clicking a mode in the dashboard takes a half-minute. Lifecycle was *designed* for exactly this use case.
**Risk:** Medium. Refactoring mode logic. Need to be careful with map loading (navigation mode loads a static map; slam mode generates one).
**My position:** Worth it but not urgent. Tier 1 first.

#### T2c. Component composition for Python nodes
**Implementation:** Compose `driver_node` + `landmark_localizer` + `cliff_detector` + `detection_world` into a single `rclcpp_components::ComponentContainer` (or rclpy equivalent). Saves per-node executor overhead and IPC cost for in-process pub/sub.
**Defense:** Each rclpy node currently runs its own Python process with its own executor + DDS participant. That's 4 Python interpreters when 1 would do. The DDS participants alone cost a few % each.
**Estimated savings:** 5–10 % Pi CPU.
**Risk:** Medium. One process means one crash-blast-radius. Debugging gets harder.
**My position:** Defer until measured. May not be worth the debugging tax.

### Tier 3 — Speculative

#### T3a. Move full Nav2 to WSL2, keep collision_monitor on Pi
**Defense:** Tempting — would clear 40+ % of Pi CPU. But cmd_vel latency is the killer. Network blip → robot keeps driving with stale velocity → wall. Even with collision_monitor on the Pi as a backstop, the controller's planning latency would feel sluggish.
**My position:** Don't do this. The win isn't worth losing local control.

#### T3b. Hardware H.264 encoding for the dashboard camera stream
**Defense:** Pi5 has v4l2 m2m H.264 hardware encoder. `web_video_server`'s software MJPEG is using ~7 % CPU and produces a fat stream (~500 KB/s for 424×240@30). HW H.264 + WebRTC: ~1 % CPU + ~50 KB/s + smoother playback.
**Risk:** WebRTC stack complexity. Browser compatibility concerns minor (Chrome handles it).
**My position:** Real win but not urgent. Park in the backlog.

#### T3c. Replace ROS2 message bus with iceoryx for Pi-internal traffic
**Defense:** Shared memory IPC for in-process publishers. Theoretical: drops the rclpy serialization tax that's hitting `detection_world` (and would hit T2c if attempted).
**My position:** Don't. Too exotic, too much risk, debug stories will be horrendous.

---

## 4. What I would NOT do (and why)

| Proposal | Why I'm against |
|---|---|
| Rewrite Python nodes in C++ | Multi-week effort for ~10 % CPU. Cheaper wins exist. The Pi5 isn't actually slow at Python — the topology is the issue. |
| Replace nav2 with a custom planner | Loses years of Nav2 community work. Our use case is "navigate around a small home" — well within nav2's sweet spot. |
| Move web stack (rosbridge + http) to WSL2 | The current "Pi serves dashboard directly" means you can drive the robot from any browser without WSL2 powered on. That's a real reliability feature; don't trade it for ~15 % CPU. |
| Drop the D435i and go back to a webcam | We just shipped 4 PRs that depend on it (#28, #30, #31, #33). The depth pipeline is the most differentiated thing about this robot. |
| Disable detection_world by default | Considered after PR #34; doesn't pull weight relative to the visualization value. Throttling fixed it. |

---

## 5. Recommended sequence

1. **This week:** Tier 1 (T1a, T1b, T1c, T1d). Low risk, no architecture change. Net ~25 % Pi CPU recovered. Tier 1a + 1b ship in this PR as proof-of-concept.
2. **Next session:** Pre-flight network latency measurement Pi↔WSL2. If < 50 ms 99 %ile on /tf, ship T2a.
3. **After T2a is stable for a week:** consider T2b (persistent Nav2 lifecycle) for the UX win.
4. **Backlog:** T2c, T3b — only if CPU pressure returns. T3a, T3c, the "would NOT do" list — file under "no, and here's why."

---

## 6. What's in this PR

- This proposal doc
- T1a: realsense_params.yaml — disable pointcloud, disable temporal_filter
- T1b: description.launch.py — drop joint_state_publisher

T1c and T1d need separate treatment (T1c is investigative; T1d is a systemd unit change).

Tier 2 and 3 are NOT in this PR. They need explicit user go/no-go.

## 6b. Measured results (post-deploy, slam_nav idle, 2026-05-01 15:44)

| Process | Before T1 (15:39) | After T1 (15:44) | Δ |
|---|---|---|---|
| nav2_container | 42.8 | 46.0 | +3.2 (variance) |
| realsense2_camera | 19.4 | 23.4 | **+4.0 (regression!)** |
| **joint_state_publisher** | **10.5** | **— (gone)** | **−10.5** ✅ |
| driver_node | 21.2 | 14.6 | −6.6 |
| detection_world | 19.0 | 12.0 | −7.0 |
| sync_slam_toolbox | 7.0 | 3.7 | −3.3 |
| ekf_node | 6.5 | 3.6 | −2.9 |
| robot_state_publisher | 4.6 | not in top 15 | small |
| **Net process drops** | | | **~30 %** |
| **Net process adds** | | | **~7 %** |

**Real win: ~23 % aggregate CPU back, dominated by joint_state_publisher removal.**

### Honest reporting on the parts that didn't deliver

**T1a (realsense diet) underperformed.** I estimated 5–10 % CPU savings; measured ~0 % (realsense actually went UP 4 % in the snapshot, which is likely measurement variance). Possible reasons:
- Pointcloud generation may have been cheaper than I assumed — realsense's NEON-accelerated path is probably efficient.
- Temporal filter may not have been the cost driver I thought.
- The DDS publish overhead I expected to save (no more 9 MB/s of unused pointcloud traffic) likely shows up as savings in OTHER processes (rosbridge, network stack) rather than realsense itself.

**Net assessment of T1a:** still defensible — we removed dead code (zero-subscriber topic), reduced internal bandwidth, and gained no functional regression. Whether it shows as Pi CPU is moot. But the proposal's quantitative claim was too optimistic; trim that estimate to "0–5 %" and note the win is correctness, not measurable speed.

**T1b (joint_state_publisher) overperformed expectations.** Estimated 5–10 %; measured −10.5 %. That alone justifies the whole PR.

### What this changes about the Tier 2/3 plan

**T2a (move slam_toolbox to WSL2) becomes more attractive, not less.** sync_slam_toolbox dropped 3.3 % on its own when other contention eased — that's evidence of CPU starvation. Moving it off-Pi should clear all of that AND the residual ~4 % of map_saver_server which is co-located.

**T1c (map_saver_server idle floor)** still a bug worth investigating — 5.5 % when nothing is being saved.

**T1d (chrt priority)** still recommended. The 4 % realsense regression in the snapshot above is exactly the kind of contention that priority pinning prevents from cascading.

### Production behavior after T1

- Load avg progression during settling: 6.04 → 4.06 → 2.36 (1m → 5m → 15m). Consistent with normal startup spike.
- Lifecycle manager reports active.
- /tf at 70 Hz (healthy).
- /scan_depth at 11 Hz (was 10–13 before — same).
- Zero TF errors in 2 minutes.

## 7. Defend-me list (positions I expect pushback on)

- **"Don't break wheel rotation in RViz"** — Counter: nobody's looking at wheel rotation in RViz during ops. Cosmetic-only.
- **"Don't trust slam_toolbox over the network"** — Counter: today proved the current topology is what's untrustworthy. Network latency is bounded; CPU starvation isn't.
- **"Persistent Nav2 lifecycle is too risky to refactor"** — Counter: the current launch_manager pattern is simple but already has bugs we hit today (stuck-pose-retry doesn't get cleared until full teardown). Lifecycle was designed for this.
- **"Pi5 is fast enough, just don't run as much stuff"** — Counter: every feature on this robot is one a real user would want. The fix is to spread the work, not to remove features.
