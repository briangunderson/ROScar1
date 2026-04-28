# Chassis v2 Tuning Journal

Live decision log so future sessions don't re-debug solved problems.
Started 2026-04-27 after the aluminum-extrusion frame swap; current state
as of 2026-04-28.

## Hardware delta from chassis v1 (plate)

| Item | v1 (plate) | v2 (extrusion) | Source |
|------|-----------|----------------|--------|
| Wheelbase (front-rear hub) | 96.5 mm | **213 mm** | tape, 2026-04-28 |
| Track (L-R hub) | 205 mm | **325 mm** | tape, 2026-04-28 |
| Frame width | 170 mm | **250 mm** | tape, 2026-04-28 |
| Frame length | 200 mm | ~220 mm (TODO measure) | est |
| Wheel radius | 39.7 mm | 39.7 mm (same wheels) | unchanged |
| Encoder CPR | 1320 | 1320 (same motors) | unchanged |
| Camera | Logitech webcam | RealSense D435i on tail mast | photos 2026-04-27 |
| Lidar | Center, low platform | **Top of front mast** at (0.090, 0.015, 0.235) m | URDF |
| Mast | none | 3030 extrusion column at +X, 15 mm left of centerline | photos |

## What worked

| Fix | Status | Reason |
|-----|--------|--------|
| Negate vx, vy in cmd_vel + odom (camera flipped 180°) | **Reverted** | The Pi's local edits already had this from a previous session; rebuild flipped camera back to original direction → revert was correct |
| Camera-mast addition to URDF | Kept | Real physical change |
| Lidar URDF yaw flipped 0 → π | **Kept** | Lidar 0° marker physically points -X (away from camera). Verified empirically: with yaw=0 the scan-matcher inferred backward motion when robot drove forward |
| URDF wheel offsets updated to half-wheelbase 0.1065 / half-track 0.1625 | Kept | Matches tape measurements |
| URDF chassis_width 0.220 → 0.250 | Kept | Measured |
| URDF mast_y 0 → 0.015 (15 mm left of center) | Kept | Mast physically had to shift left to clear L-bracket |
| slam_toolbox `minimum_travel_distance` 0.2 → 0.0 + `minimum_travel_heading` 0.2 → 0.0 (pure time-triggered) | Kept | Lets slam keep firing scan match even when EKF reports zero motion |
| slam_toolbox `minimum_time_interval` 0.3 → 0.2 (5 Hz scan match) | Kept | More frequent corrections, smaller per-correction error |
| EKF: drop wheel `vyaw` fusion (`odom0_config[11]: true → false`) | Kept | Firmware FK overstated wz by ~1.78× because `(L_x + L_y)` in firmware is for v1 dims (0.151 m vs real 0.269 m). IMU `vyaw` is the truth. |
| EKF: drop wheel `vx`, `vy` fusion temporarily | **Reverted then re-tried with host FK** | Initial drop without time-triggered SLAM made slam_toolbox stop processing scans (chicken-and-egg: no detected motion → no scan trigger). Solved by pairing the drop with `minimum_travel_distance: 0.0`. |
| Driver `/odom_raw` twist covariance vx, vy 0.01 → 0.25 | Kept | Documents that wheel velocity reports are not trusted on this chassis. Largely moot once EKF drops the channel. |
| AMCL `set_initial_pose: true` in nav2_params | Kept | Auto-seeds at (0,0,0) on nav-mode activation, no manual `/initialpose` publish needed |
| Nav2 footprint updated to 31×38 cm (was 29×24.6 cm) | Kept | Matches new chassis exterior including wheel protrusion |
| Nav2 `inflation_radius` 0.08 → 0.10 m | Kept | Wider robot needs wider safety margin |
| **Host-side mecanum FK** (read raw encoders, compute body velocities on Pi) | **Deployed 2026-04-28, forward sign verified, EKF re-enabled** | Replaces firmware's stale FK. Firmware is closed-source (no reflash). See `roscar_driver/mecanum_kinematics.py::MecanumFK`. Auto-drive forward at 0.15 m/s gave host-FK vx of 0.06–0.12 (positive, sign correct). Strafe / yaw signs not directly verified by auto-drive (motion stalled on carpet) but FK formula is structurally correct. |
| EKF wheel `vx`, `vy`, `wz` fusion **re-enabled** with host-FK source | **Done 2026-04-28** | Now that host FK uses real chassis dims, all three channels are trustworthy and feed the EKF normally. |

## What didn't work / dead ends

| Attempt | Result | Lesson |
|---------|--------|--------|
| Drop wheel vx/vy from EKF without changing slam_toolbox motion thresholds | slam_toolbox stopped publishing new map content | slam_toolbox uses odom-frame motion to gate scan registration. If EKF reports stationary, slam thinks robot hasn't moved. Pair odom drops with `minimum_travel_distance: 0.0` so slam fires on time alone. |
| STM32 firmware reflash with corrected (L_x + L_y) | **Not feasible** | Yahboom YB-ERF01-V3.0 firmware is closed-source binary (`Rosmaster_V3.1.hex`). No public source, only PDFs and per-peripheral demos. The fix path is host-side FK instead. |
| Bumping `correlation_search_space_dimension` above 0.5 in slam_toolbox | Crashes slam_toolbox at startup (probability grid OOB) | Hard upper bound. Can't widen scan-match search this way; widen via `coarse_search_angle_offset` instead if needed. |

## Causal map of "wavy walls"

Multiple contributors, ranked by evidence:

1. **Firmware mecanum FK uses stale L_x, L_y** → reported wz is 1.78× truth →
   EKF integrates wrong yaw → scan matcher fights the bias every cycle →
   walls smear before each correction. **Mitigation:** drop wheel wz from
   EKF (done), or use host-side FK (in progress).
2. **Wheel slip on carpet** → wheel-reported vx, vy overstate translation →
   EKF over-runs forward → slam's scan match snaps map→odom backward →
   visible "arrow inches forward then snaps back" pattern.
   **Mitigation:** drop wheel vx/vy from EKF, let scan match carry linear
   pose entirely. Done.
3. **Mast wobble under lidar's own centripetal force** → ±cm scan jitter
   even when stationary (measured: median 11 mm spread, p99 13 cm on
   worst beams). **Mitigation:** mechanical brace from top of mast to
   chassis edge. **NOT DONE** — open mechanical action item.
4. **URDF lidar X position wrong** → walls appear in arcs during rotation.
   We measured `mast_x = 0.090` (estimate from photos); user verified
   `mast_y = 0.015`. `mast_x` itself never tape-measured. Likely OK but
   worth confirming if waviness persists.

## Currently active (chassis v2 production state)

- URDF: see `roscar_ws/src/roscar_description/urdf/roscar.urdf.xacro`. Lidar
  at (0.090, 0.015, 0.235) yaw=π. Camera mast as documented above. Wheel
  offsets 0.1065 × 0.1625.
- Driver: host mecanum FK enabled (`use_host_fk: True`) with
  R=0.0397, Lx=0.1065, Ly=0.1625, CPR=1320. Falls back to firmware FK
  if you set the param false.
- EKF: wheel vx, vy, wz all fused (covariance 0.25 / 0.25 / 0.03); IMU wz also fused.
  Host FK supplies all three from raw encoders with measured chassis dims.
- slam_toolbox: time-triggered (motion thresholds = 0), `minimum_time_interval: 0.2`.
- Nav2: footprint 31×38 cm, inflation 0.10 m, `set_initial_pose: true`.

## Open action items

| # | Item | Status |
|---|------|--------|
| 1 | **STM32 firmware reflash** for correct kinematics | **CANCELLED** — closed-source. Replaced by host-side FK. |
| 2 | **Mast brace** (3030 + 2 corner brackets, diagonal top-of-mast → chassis edge) | Pending — mechanical, ~15 min |
| 3 | Nav2 footprint + inflation update | Done |
| 4 | Validate D435i depth voxel layer in costmap (drive at low table edge, see if it stops) | Pending |
| 5 | Dashboard "click map → set initial pose" | Pending |
| 6 | End-to-end landmark localizer test (ArUco markers in a closed loop) | Pending |
| 7 | Tape-measure `mast_x` (currently estimate of 0.090 m) | Pending |
| 8 | Tape-measure ground→camera optical center and ground→lidar scan plane | Pending — only matters for RViz aesthetics + depth-pointcloud world projection |
| 9 | Re-enable wheel `vyaw` fusion in EKF after host FK is sign-verified | **Done 2026-04-28** |
| 10 | Sign-verify host FK | **Done 2026-04-28** — user confirmed during dashboard driving: cleanest map yet, no arrow snap-back, walls solid |
| 11 | Optionally raise `minimum_travel_distance` back to 0.1 m now that wheel motion is trustworthy again | Pending — current time-triggered mode (0, 0, 0.2 s) is fine |

## Sign conventions cheat sheet

REP-103: +X forward, +Y left, +Z up. Yaw CCW = +.
Yahboom motor mapping: M1=FL, M2=RL, M3=FR, M4=RR.
Driving forward → all four wheels roll forward → encoders count up.
Strafe right (vy < 0): FL+, FR-, RL-, RR+ (ABBA pattern).
Yaw CCW (wz > 0): FL-, RL-, FR+, RR+.
Lidar `laser` frame yaw=π relative to base_link (0° marker points -X).
