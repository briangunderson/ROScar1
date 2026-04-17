# Chassis v2 Design — Session Handoff

**Date:** 2026-04-05
**Status:** Fusion 360 model rev8b ready to test, design spec complete

## What We're Doing

Designing a new aluminum extrusion chassis (v2) to replace the robot's current metal plate chassis. The current chassis is top-heavy and tips over. The new design is a two-deck sandwich with 3030 extrusion rails, vertical posts, and a lidar mast.

**This work is PARALLEL to the main codebase — no existing files are modified.**

## Key Documents

| Document | Path | Status |
|----------|------|--------|
| Design spec | `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md` | Complete, reviewed |
| Implementation plan | `docs/superpowers/plans/2026-04-04-fusion360-chassis-mockup.md` | Written (needs update for rev7+ changes) |
| Fusion 360 script | `docs/chassis/fusion360/roscar_v2_chassis.py` | **rev8b — NEEDS TESTING** |
| Fusion 360 README | `docs/chassis/fusion360/README.md` | Complete |
| 3D models (STEP/F3D) | `docs/chassis/models/` | Added 2026-04-11 — extrusions, brackets, T-nuts |

## Current State: Fusion 360 Model

### What Works (verified on screen)
- **rev4**: XY-plane-only boxes — confirmed correct positioning (assembled, not exploded)
- **rev5**: T-slot profiles on Z-direction rails, revolve cylinders for wheels/motors — looks good
- **rev6**: Colors working (blue battery, green PCB, orange brackets, translucent blue plates, dark wheels)
- **rev7**: Motor mounting corrected — motors hang BELOW frame on L-brackets (confirmed from robot photos)

### What Needs Testing
- **rev8b**: Full polish pass — NOT YET RUN IN FUSION 360. Features:
  - Segmented mecanum wheels (6 diagonal box segments per wheel)
  - Two-section motor body (motor can + gearbox, different colors)
  - Motor encoder disc
  - RPi5 port indicators (USB-A, Ethernet, HDMI)
  - Motor board connector headers
  - Battery wire leads
  - Camera mount clip
  - RPLIDAR cable exit
  - Deck plate standoffs (8 total)
  - Wire routing hints (USB, power, lidar cables)
  - Mounting bolts on L-brackets
  - Sub-components for browser tree organization (7 numbered groups + per-motor sub-components)
  - **RISK**: Sub-components are back. They caused exploded layouts in rev1-3 BUT that was due to XZ/YZ plane axis mapping bugs. Since rev8b is XY-plane-only, sub-components SHOULD work. If model explodes again, remove `_comp()` calls and revert to flat bodies.

### To Run the Script
1. Open Fusion 360
2. UTILITIES → Scripts and Add-Ins (or Shift+S)
3. **Delete** old script entry, click **+**, navigate to `D:\localrepos\ROScar1\docs\chassis\fusion360\`
4. Select `roscar_v2_chassis.py`, click **Run**
5. Message box should say `rev8b`
6. Fusion 360 CACHES scripts — must delete and re-add after external edits

## All Collected Measurements

| Component | Measurement | Value |
|-----------|-------------|-------|
| Motor body diameter | caliper | 24mm |
| Motor body length (excl shaft) | caliper | 62mm |
| Motor total length (incl shaft) | caliper | 75mm |
| Motor mounting screw spacing | caliper | 17mm |
| Motor shaft protrusion | derived | 13mm |
| Motor shaft diameter | caliper | 4mm |
| Wheel radius | URDF (measured 2026-03-16) | 39.7mm |
| Wheel width | URDF | 37.3mm |
| Motor board dimensions | caliper | 56 × 85mm |
| Motor board mounting holes | caliper | 50 × 60mm spacing |
| RPLIDAR C1 body | datasheet | 55.6 × 55.6 × 41.3mm |
| RPLIDAR C1 mounting holes | datasheet | 43mm square, M2.5 screws, max 4mm depth |
| RPLIDAR C1 weight | datasheet | 110g |
| Battery dimensions | Amazon listing | 75 × 55 × 66mm |
| Battery weight | Amazon listing | 574g |
| RPi5 in Argon NEO 5 case | Amazon listing | 73 × 97 × 40mm, 180g |
| Webcam | not yet measured | Logitech, generic webcam shape |

## Critical Design Decisions

1. **3030 extrusion** as primary structural material (matches inventory)
2. **Two-deck sandwich** — lower deck for battery/motors/board, upper deck for RPi5/camera
3. **Motors hang BELOW frame** on L-brackets (confirmed from robot photos) — NOT inside the frame
4. **Lidar on short mast** (120mm) at center-rear of upper deck
5. **~250mm square frame** from 3030 extrusion
6. **All Fusion 360 geometry uses XY plane ONLY** — XZ/YZ planes have axis mapping bugs that cause bodies to scatter

## Known Risks (from spec)

1. **STM32 firmware has hardcoded wheel geometry** — changing track/wheelbase means odometry errors
2. **IMU axis corrections** may need updating if motor board orientation changes
3. **Laser filter threshold** may need adjustment for new lidar height
4. **Frame weight ~2.5kg** of extrusion → total robot ~4.9kg (heavier but more stable)

## Extrusion Inventory (confirmed from Amazon orders, Sept 2025+)

- 4× 500mm black 3030 (for frame rails)
- 4× 400mm black 3030 (for posts + mast)
- 4× 600mm black 3030 (spare)
- 4× 1000mm silver 3030 (spare)
- Tons of 3030 brackets: 3-way corners, L-brackets, T-plates, T-nuts

## Fusion 360 Lessons Learned

1. **NEVER use XZ/YZ construction planes** — axis mapping is unreliable, causes bodies to scatter
2. **XY plane ONLY** — sketch X = world X, sketch Y = world Y, extrude = +Z. Zero ambiguity.
3. **Horizontal cylinders**: use `revolve` on XY plane with construction axis line along Y
4. **Sub-components** caused exploded layouts in rev1-3 (combined with XZ/YZ bugs). May be safe now with XY-only geometry — needs verification in rev8b.
5. **Fusion caches scripts** — must delete and re-add script entry after external edits
6. **Add VERSION string** to every script revision — show in message box to verify correct version ran
7. **Windows MCP tools** are unreliable for clicking Fusion 360 UI — have user run scripts manually

## What To Do Next

1. **Test rev8b** in Fusion 360 — verify sub-components don't explode
2. If model looks good: **commit the script and spec** to git
3. **Update the design spec** with corrected motor mounting (motors below frame, not inside)
4. **Design 3D-printable motor mount brackets** in Fusion 360 (parametric, based on caliper measurements)
5. **Cut extrusion stock** per the cut plan in the spec
6. **Start physical build**
