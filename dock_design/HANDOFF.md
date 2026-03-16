# Charging Dock Design — Handoff to Chassis Design Session

> **Context**: This document was written by a Claude Code session that designed a
> 3D-printable charging dock for ROScar1. It's intended to be read by the Claude
> session working on "Design robot chassis with data-driven approach" so that the
> dock and chassis designs are compatible and integrated.

## What Was Done

A complete charging dock design was created in `dock_design/` on branch
`claude/robot-charging-dock-xRrTF`. The design includes:

1. **Parametric OpenSCAD 3D model** (`dock_design/scad/charging_dock.scad`)
   - V-funnel entrance dock with pogo-pin charging contacts
   - ArUco marker post for Nav2 autonomous docking
   - 5 printable parts: base plate, left wall, right wall, contact wall, marker post

2. **Robot-side contact plate** (`dock_design/scad/robot_contacts.scad`)
   - Mounts on the robot's REAR face
   - Two raised pads for copper tape, spaced 60mm center-to-center
   - Contact center height: 35mm from floor (i.e., 35mm above the dock base plate surface)

3. **Full build guide** (`dock_design/BUILD_GUIDE.md`)
   - BOM, wiring diagrams, step-by-step assembly with ASCII mockups

4. **Nav2 docking config** (`dock_design/docs/nav2_dock_config.yaml`)
   - Functional docking_server config to replace the placeholder in nav2_params.yaml

## CRITICAL: What the Chassis Design MUST Accommodate

### 1. Rear Contact Plate Mounting

The robot needs a **flat mounting surface on the rear face** for the contact plate:
- Plate dimensions: 100mm wide x 50mm tall x 3mm thick
- Mounted centered on the rear of the chassis
- **Contact pad centers**: 60mm apart horizontally, at 35mm height from the floor
- Needs 4x M3 mounting holes (or flat surface for double-sided tape)
- Two 4mm wire routing holes behind each contact pad

```
    ROBOT REAR FACE (required mounting area)

    ┌─────────────────────────────────────┐
    │            CHASSIS REAR             │
    │                                     │
    │      ┌──── 100mm ────┐             │
    │      │  CONTACT PLATE │  ↕ 50mm    │  ← needs flat surface here
    │      │  [−]  60mm  [+]│             │     35mm center from floor
    │      └────────────────┘             │
    │                                     │
    │   ○ wheel                  wheel ○  │
    └─────────────────────────────────────┘
                                    ↑
                              floor line
```

**The 35mm contact height was chosen because**:
- Robot ground clearance is ~50mm (URDF placeholder)
- Contacts sit between the floor and the bottom of the chassis
- This puts them at wheel-hub height, below the chassis plate
- If the chassis design changes ground clearance, the dock's `contact_height`
  parameter must be updated to match (in both `charging_dock.scad` and
  `robot_contacts.scad`)

### 2. Wire Routing from Rear Contacts to Battery

The contact plate needs two wires routed from the rear face to the battery's
charging port (or BMS input). The chassis design should include:
- A **wire channel or gap** from the rear face to wherever the battery sits
- Access to the battery's charge input (XT60, JST balance connector, or dedicated charge port)
- The TP5100 charger module lives in the DOCK, not on the robot — the robot
  side is just passive copper contacts wired directly to the battery charge input

### 3. Dock Bay Clearance

The dock's interior bay dimensions depend on the robot's outer envelope:

| Robot Dimension | Current URDF (PLACEHOLDER) | Dock Parameter | Current Value |
|-----------------|---------------------------|----------------|---------------|
| Chassis width | 200mm | — | — |
| Track width (wheel outer-to-outer) | 280mm | `bay_width` | 310mm (+30mm clearance) |
| Chassis length | 250mm | `bay_length` | 300mm (+50mm clearance) |
| Ground clearance | 50mm | `wall_height` | 55mm (walls just above wheel tops) |
| Wheel radius | 33mm | — | walls clear the wheels |

**If the chassis design changes ANY of these dimensions**, update the corresponding
dock parameters:
- `bay_width` should be `track_width + 30mm` (15mm clearance per side)
- `bay_length` should be `chassis_length + 50mm`
- `wall_height` should be `ground_clearance + 5mm` (walls guide wheels, stay below chassis)
- `contact_height` should be approximately `ground_clearance - 15mm` (mid-height between floor and chassis underside)

### 4. ArUco Marker Visibility

The dock has an ArUco marker on a 120mm post at the rear, tilted 15° forward.
The robot's **front-facing camera** must be able to see this marker during approach.
The camera is currently at:
- Position: 120mm forward of base_link center, 100mm high (URDF placeholder)
- The marker top is ~200mm from floor

**Chassis design consideration**: Don't block the camera's forward line-of-sight
at the marker height (~100-200mm). If the camera position changes, the marker
post height (`marker_post_height`) may need adjustment.

### 5. Funnel Entrance Compatibility

The dock has angled funnel walls that guide the robot in. The funnel assumes:
- The robot's **outermost protrusions** are the wheels (280mm track)
- No bumpers, sensors, or accessories stick out beyond the wheel line
- If the chassis has protruding elements (e.g., a front bumper, side-mounted
  sensors), increase `funnel_flare` and `bay_width` accordingly

### 6. Weight and Center of Gravity

The dock's base has grip grooves to prevent the robot from sliding, but does NOT
have a ramp or tilt mechanism. The robot must:
- Be able to drive onto the flat 4mm base plate without getting stuck
- The 4mm step-up should be trivial for 33mm mecanum wheels, but if wheel
  diameter changes significantly (e.g., < 20mm), consider adding a chamfered
  ramp to the base plate entrance

## File Map

```
dock_design/
├── BUILD_GUIDE.md              ← Full assembly instructions (read this for the big picture)
├── HANDOFF.md                  ← THIS FILE
├── docs/
│   └── nav2_dock_config.yaml   ← Nav2 docking_server config (replaces placeholder)
└── scad/
    ├── charging_dock.scad      ← Dock model (5 parts, fully parametric)
    └── robot_contacts.scad     ← Robot-side contact plate (must mount on rear)
```

## Parametric Dimensions Quick Reference

All dimensions that couple the dock to the robot are defined as named variables
at the top of each SCAD file. Here's the mapping:

| Parameter (in SCAD) | Current Value | Depends On |
|---------------------|---------------|------------|
| `robot_length` | 250mm | Chassis length |
| `robot_width` | 200mm | Chassis body width |
| `robot_track` | 280mm | Outer wheel-to-wheel width |
| `robot_ground_clr` | 50mm | Floor to chassis underside |
| `wheel_radius` | 33mm | Wheel outer radius |
| `bay_width` | 310mm | track + 30mm |
| `bay_length` | 300mm | length + 50mm |
| `wall_height` | 55mm | ground_clr + 5mm |
| `contact_height` | 35mm | ~ground_clr - 15mm |
| `contact_spacing` | 60mm | Fixed (both SCAD files must match) |

**When chassis dimensions are finalized**: Update the `robot_*` variables at the top
of `charging_dock.scad`, re-export STLs, and verify with `part="assembled"` view.
Also update `contact_spacing` and `contact_height` in `robot_contacts.scad` if needed.

## Design Decisions & Rationale

1. **V-funnel over magnetic/IR guide**: Mechanical alignment is simpler, more reliable,
   and doesn't need additional sensors. Mecanum strafing + funnel = very forgiving docking.

2. **Pogo pins over spring contacts**: Pogo pins are cheap ($1.50/pair), self-cleaning,
   and tolerate ±5mm misalignment. Spring contacts require tighter tolerances.

3. **ArUco marker over IR beacon**: We already have ArUco detection in roscar_cv.
   No new hardware needed. Nav2 docking_server supports external pose detection natively.

4. **Charger in dock, not on robot**: Keeps robot weight down. The robot side is just
   two copper tape pads + wires to battery. All charging intelligence is in the dock.

5. **Rear-docking (drive forward into dock)**: Simpler than backing in. The camera
   faces the ArUco marker during the entire approach. `dock_backwards: false` in config.

6. **60mm contact spacing**: Wide enough that minor lateral misalignment (±10mm) still
   makes contact. Narrow enough to fit on the robot rear without interfering with wheels.

## What's NOT Done Yet (Future Work for Either Session)

- [ ] **Battery voltage bridge node**: The docking_server wants `sensor_msgs/BatteryState`
  to detect charging, but roscar_driver publishes `std_msgs/Float32` on `/battery_voltage`.
  Need a small node that subscribes to `/battery_voltage` and publishes `/battery_state`.

- [ ] **Web dashboard dock/undock buttons**: Add DOCK/UNDOCK buttons to aio-status.js
  that call the `/dock_robot` and `/undock_robot` actions via rosbridge.

- [ ] **Dock pose calibration**: After placing the dock in the real environment and
  mapping, update `home_dock.pose` in the nav2 config with real coordinates.

- [ ] **Auto-dock on low battery**: A behavior tree node that monitors battery voltage
  and triggers docking when below a threshold (e.g., 7.0V for 2S).

- [ ] **Physical measurement**: ALL robot dimensions in the SCAD files are from the
  URDF placeholders. Measure the real robot and update before printing.
