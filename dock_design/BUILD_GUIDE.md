# ROScar1 Charging Dock — Build Guide

A 3D-printable V-funnel charging dock with pogo-pin contacts and ArUco marker
for Nav2 autonomous docking.

## Design Overview

```
                    TOP VIEW
    ┌─────────────────────────────────────────────────────┐
    │                                                     │
    │  ╔══════════════════════════════════════════════╗    │
    │  ║              CONTACT WALL                    ║    │
    │  ║           [−] ●      ● [+]                   ║    │  ← Pogo pins
    │  ╚══════════════════════════════════════════════╝    │
    │  ║                                              ║    │
    │  ║           ┌─────────────────┐                ║    │
    │  ║           │                 │                ║    │
    │  ║           │   ROBOT GHOST   │                ║    │
    │  ║           │   (250 x 280)   │                ║    │
    │  ║           │                 │                ║    │
    │  ║           └─────────────────┘                ║    │
    │  ║                                              ║    │
    │  ║  LEFT WALL                        RIGHT WALL ║    │
    │   ╲                                            ╱     │
    │    ╲          FUNNEL ENTRANCE                  ╱      │
    │     ╲                                        ╱       │
    │      ╲══════════════════════════════════════╱        │
    │                                                     │
    │                   BASE PLATE                        │
    └─────────────────────────────────────────────────────┘
                         ↑
                   Robot drives in
```

```
                    SIDE VIEW

         ArUco marker (tilted 15° forward)
              ┌───────┐
              │ ┌───┐ │
              │ │ID7│ │  ← 80x80mm marker plate
              │ └───┘ │
              └───┬───┘
                  │        marker post (120mm tall)
                  │
    ══════════════╪════════════════════════════════════
    ║  CONTACT    │              BASE PLATE           ║
    ║  WALL       │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  ║
    ║  ● pogo     │  cable channel (underneath)       ║
    ║  ● pogo     │                                   ║
    ╚═════════════╧═══════════════════════════════════╝
         ↑                                         ↑
       rear                                     entrance
    (35mm high contact point)
```

## Dimensions Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bay interior | 300 x 310 mm | Robot sits here |
| Funnel flare | 60 mm/side | Guides robot in |
| Funnel depth | 120 mm | Taper length |
| Total footprint | ~450 x 430 mm | Including funnel + base overhang |
| Wall height | 55 mm | Just above wheel tops |
| Contact height | 35 mm | From base plate top |
| Contact spacing | 60 mm | Center-to-center |
| Marker height | 120 mm post + 80 mm plate | ~200mm total from floor |

## Bill of Materials

### 3D Printed Parts (5 pieces)

| Part | File | Est. Print Time | Filament |
|------|------|-----------------|----------|
| Base plate | `charging_dock.scad` → part="base" | ~3-4 hrs | ~80g PLA/PETG |
| Left wall | `charging_dock.scad` → part="left_wall" | ~1.5 hrs | ~30g |
| Right wall | `charging_dock.scad` → part="right_wall" | ~1.5 hrs | ~30g |
| Contact wall | `charging_dock.scad` → part="contact_wall" | ~1 hr | ~20g |
| Marker post | `charging_dock.scad` → part="marker_post" | ~1 hr | ~25g |
| Robot contact plate | `robot_contacts.scad` | ~30 min | ~10g |

**Print settings**: 0.2mm layer height, 3 perimeters, 20% infill, no supports needed
(all parts print flat). PETG recommended for durability, PLA works fine.

### Electronics & Hardware

| Item | Qty | Est. Cost | Notes |
|------|-----|-----------|-------|
| Pogo pins, 3mm dia, spring-loaded | 2 | $3 | P75-B1 type, 16.5mm long, Amazon/AliExpress |
| Copper tape, 15mm wide, adhesive | 1 roll | $5 | For robot-side contact pads |
| TP5100 Li-ion charger module (2S) | 1 | $3 | 2A charge rate, fits 2S/3S packs |
| DC barrel jack, panel mount | 1 | $1 | To connect wall adapter to dock |
| DC power supply (8.4V 2A for 2S) | 1 | $8 | Match your battery chemistry! |
| 22 AWG silicone wire, red + black | 1m each | $2 | Flexible, heat resistant |
| M3 x 8mm screws | 12 | $2 | For wall-to-base assembly |
| M3 nuts | 12 | — | (often bundled with screws) |
| ArUco marker printout (ID 7, 6x6) | 1 | — | Print on paper, glue to plate |
| CA glue (super glue) or epoxy | 1 | $3 | Backup adhesion |
| Heat shrink tubing | asst. | $2 | Wire insulation |

**Estimated total: ~$30**

> **IMPORTANT**: Match the charger module to YOUR battery. The YB-ERF01-V3.0 board
> typically runs on 2S or 3S LiPo/Li-ion. Check your battery voltage:
> - 2S (7.4V nominal): Use TP5100 set to 2S mode, 8.4V supply
> - 3S (11.1V nominal): Use TP5100 set to 3S mode, 12.6V supply

## Step-by-Step Assembly

### Step 1: Print All Parts

Export STL files from OpenSCAD:

```bash
cd ~/ROScar1/dock_design/scad

# Export each part (or use OpenSCAD GUI: Design → Render → Export STL)
openscad -D 'part="base"' -o ../stl/base_plate.stl charging_dock.scad
openscad -D 'part="left_wall"' -o ../stl/left_wall.stl charging_dock.scad
openscad -D 'part="right_wall"' -o ../stl/right_wall.stl charging_dock.scad
openscad -D 'part="contact_wall"' -o ../stl/contact_wall.stl charging_dock.scad
openscad -D 'part="marker_post"' -o ../stl/marker_post.stl charging_dock.scad
openscad -o ../stl/robot_contacts.stl robot_contacts.scad
```

Slice with your preferred slicer (Cura, PrusaSlicer, etc.) and print.

```
    Print orientation — all parts print flat, no supports:

    ┌────────────────────────────┐
    │  BASE PLATE                │  ← Flat on bed, groove side up
    │  (largest piece, may need  │
    │   diagonal on 220mm bed)   │
    └────────────────────────────┘

    ┌──────────┐  ┌──────────┐
    │ LEFT WALL│  │RIGHT WALL│     ← On their side (wall_height = layer dir)
    │ (mirror) │  │          │
    └──────────┘  └──────────┘

    ┌──────┐  ┌──────┐  ┌────┐
    │CONTCT│  │MARKER│  │ROBT│    ← Contact wall flat, post upright,
    │ WALL │  │ POST │  │CONT│      robot plate flat
    └──────┘  └──────┘  └────┘
```

> **Bed size note**: The base plate is ~450mm long. If your printer bed is smaller
> (e.g., 220x220mm), you have two options:
> 1. Cut the base plate in half in the slicer and join with M3 bolts
> 2. Edit `bay_length` in the SCAD file to shorten it (250mm minimum)
> 3. Use a flat piece of plywood/acrylic as the base and just print the walls

### Step 2: Assemble the Dock Frame

```
    ASSEMBLY — EXPLODED VIEW

                     ┌─ArUco plate
                     │
              ┌──────┴──────┐
              │  marker post │
              └──────┬──────┘
                     │
    ┌────────────────┼────────────────┐
    │  CONTACT WALL  │                │
    │  [−] ●    ● [+]│                │
    └────┬───────────┼────────────┬───┘
         │           │            │
    ┌────┴──┐   ┌────┴────┐  ┌───┴────┐
    │ LEFT  │   │  BASE   │  │ RIGHT  │
    │ WALL  │   │  PLATE  │  │  WALL  │
    └───────┘   └─────────┘  └────────┘
         ↑           ↑            ↑
      M3 screws through base into wall
```

1. Place the **base plate** on a flat surface, groove-side up
2. Align the **left wall** along the left edge — screw holes should line up
3. Insert **M3 x 8mm screws** from below the base plate, through the wall, secured with nuts on top
4. Repeat for the **right wall**
5. Attach the **contact wall** at the rear — it sits on top of the base plate between the two side walls
6. Glue or screw the **marker post** to the center-rear of the contact wall

```
    ASSEMBLED (front view, looking into the dock)

        ┌──────────────────────────────────┐
        │          ArUco ID 7              │
        │         ┌──────────┐             │
        │         │ ██  ██ █ │             │
        │         │ █ ██ █ █ │             │
        │         │ ██ █  ██ │             │
        │         └──────────┘             │
        └──────────────┬───────────────────┘
                       │  (marker post)
    ╔══════════════════╪══════╗
    ║  ●          ●    │      ║ ← pogo pins at 35mm height
    ╠══════════════════╧══════╣ ← contact wall
    ║                         ║
    ║      (bay interior)     ║ ← 310mm wide, 300mm deep
    ║                         ║
    ╚═╗                     ╔═╝
       ╲                   ╱    ← funnel walls diverge
        ╲                 ╱
         ╲_______________╱      ← entrance: 430mm wide
```

### Step 3: Install Pogo Pins

```
    CONTACT WALL — REAR VIEW (wiring side)

    ┌───────────────────────────────┐
    │                               │
    │     ○ wire       wire ○       │  ← wire routing holes
    │     │ hole       hole │       │
    │     ●─────────────────●       │  ← pogo pin holes (3.2mm)
    │    (−)    60mm c-c   (+)      │
    │                               │
    │      ═══cable channel═══      │
    └───────────────────────────────┘
```

1. Push each **pogo pin** through the 3.2mm hole from the FRONT (bay-facing) side
2. The spring tip should protrude ~5mm into the bay
3. Secure with a drop of CA glue on the rear side (don't glue the spring!)
4. Solder wires to the rear pins, route through the wire holes

### Step 4: Wire the Charging Circuit

```
    WIRING DIAGRAM

    DC Power  ──→  TP5100 Module  ──→  Pogo Pins  ──→  Robot Contacts
    Supply         (charger)           (in dock)        (on robot)
    8.4V/2A                                              │
                    ┌─────────┐                          │
    DC Jack ──(+)──→│ VIN+    │                          │
              (−)──→│ VIN−    │         Dock             │     Robot
                    │         │    ┌────────────┐   ┌────┴─────────┐
                    │ VOUT+ ──│──→│ (+) pogo ●──│──→│──● copper(+) │
                    │ VOUT− ──│──→│ (−) pogo ●──│──→│──● copper(−) │
                    │         │    └────────────┘   └──────────────┘
                    │  LED    │                          │
                    └─────────┘                     to battery via
                     charging                       charge port or
                     indicator                      balance connector
```

1. Mount the **TP5100 module** to the rear of the dock (double-sided tape or screws)
2. Wire the **DC barrel jack** → TP5100 VIN+ and VIN−
3. Wire TP5100 VOUT+ → right pogo pin (+)
4. Wire TP5100 VOUT− → left pogo pin (−)
5. Route wires through the cable channel in the base plate

> **TP5100 setup**: The module has a solder jumper to select 2S or 3S mode.
> Set it BEFORE powering on. Wrong setting can damage your battery!

### Step 5: Prepare the Robot Contact Plate

```
    ROBOT REAR VIEW (looking at the back of the robot)

    ┌─────────────────────────────────┐
    │          ROBOT CHASSIS          │
    │                                 │
    ├─────────────────────────────────┤
    │     ┌─CONTACT PLATE──────┐      │
    │     │                    │      │
    │     │  ▓▓▓▓         ▓▓▓▓│      │  ← copper tape on raised pads
    │     │  (−)    60mm   (+) │      │
    │     │                    │      │
    │     └────────────────────┘      │
    ├─────────────────────────────────┤
    │  ○ wheel              wheel ○   │
    └─────────────────────────────────┘
```

1. Print the **robot contact plate** (`robot_contacts.scad`)
2. Apply **copper tape** over the two raised pads — press firmly, smooth out bubbles
3. Solder a wire to each copper pad (tin the pad first, quick touch with iron)
4. Mount the plate to the robot's rear face with M3 screws or strong double-sided tape
5. Route wires to the battery charging port

> **Contact alignment**: The contact pads are at the same height and spacing as the
> dock's pogo pins (35mm high, 60mm apart). When the robot is fully docked, the
> pogo pins press against the copper tape pads.

### Step 6: Print and Mount the ArUco Marker

Generate and print an ArUco marker:

```bash
# Generate ArUco marker (ID 7, 6x6_250 dictionary, 70mm)
python3 -c "
import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
marker_img = cv2.aruco.generateImageMarker(aruco_dict, 7, 700)  # 700px = 70mm at 254dpi
cv2.imwrite('aruco_dock_marker_id7.png', marker_img)
print('Saved aruco_dock_marker_id7.png — print at 70mm x 70mm')
"
```

1. Print the marker on white paper at **exactly 70mm x 70mm**
2. Glue it centered on the marker plate at the top of the post
3. Optionally laminate or cover with clear tape for durability

```
    MARKER POST — SIDE VIEW

              ┌───────────┐
             ╱│  ArUco #7 │   ← tilted 15° forward for
            ╱ │  (70x70mm)│     camera visibility during
           ╱  └───────────┘     approach
          │
          │   post (120mm)
          │
    ══════╧════════════════
      contact wall / base
```

### Step 7: Test Fit

Before wiring, do a dry test:

1. Place dock on the floor against a wall
2. Manually push the robot into the dock
3. Check that:
   - Robot fits in the bay with ~15mm clearance per side
   - Funnel guides the robot when approaching at an angle
   - Pogo pins contact the copper pads at the right height
   - Robot wheels don't ride up on any edges
   - ArUco marker is visible to the robot's camera

```
    TEST FIT — TOP VIEW

    ══════════════════════════════════
    ║  CONTACT WALL ● ●             ║
    ║  ┌─────────────────────┐      ║
    ║  │     R O B O T       │      ║
    ║  │                     │      ║  ← ~15mm gap each side
    ║  │  (250 x 280 track)  │      ║
    ║  │                     │      ║
    ║  └─────────────────────┘      ║
    ║                               ║
    ╚═╗         GAP OK?           ╔═╝
       ╲        ↕ 15mm           ╱
        ╲                       ╱
         ╲_____________________╱
```

**If the fit is too tight or too loose**, edit these parameters in `charging_dock.scad`:
- `bay_width`: Increase for more clearance (default 310mm for 280mm track)
- `funnel_flare`: Increase for wider entrance catchment
- `contact_spacing`: Must match robot contact plate (default 60mm)

### Step 8: Final Wiring & Safety

```
    COMPLETE SYSTEM

    ┌──Wall outlet──┐
    │   AC power    │
    └──────┬────────┘
           │
    ┌──────┴────────┐
    │  DC Adapter   │  8.4V (2S) or 12.6V (3S)
    │  (matched to  │
    │   battery)    │
    └──────┬────────┘
           │ DC barrel jack
    ┌──────┴────────────────────────────────────────┐
    │  DOCK                                         │
    │  ┌─────────┐     ┌──────┐                     │
    │  │ TP5100  ├────→│ POGO │←─spring contact─→   │
    │  │ charger │     │ PINS │                 │   │
    │  └─────────┘     └──────┘                 │   │
    └───────────────────────────────────────────┼───┘
                                                │
    ┌───────────────────────────────────────────┼───┐
    │  ROBOT                                    │   │
    │                              ┌────────────┘   │
    │                              │ copper pads    │
    │                              ↓                │
    │  ┌──────────┐     ┌─────────────┐            │
    │  │ Battery  │←────│ charge port │            │
    │  │ (2S/3S)  │     │ or BMS      │            │
    │  └──────────┘     └─────────────┘            │
    └───────────────────────────────────────────────┘
```

Safety checklist:
- [ ] TP5100 set to correct cell count (2S vs 3S solder jumper)
- [ ] Power supply voltage matches charger input spec
- [ ] Polarity correct on ALL connections (use a multimeter!)
- [ ] No exposed wire — heat shrink all solder joints
- [ ] Pogo pins can't short together (60mm apart, should be fine)
- [ ] Test with multimeter: correct voltage at pogo tips before connecting robot
- [ ] TP5100 charge LED works (red = charging, green/off = done)

## Software Integration

### Nav2 Docking Server Config

The dock uses Nav2's built-in `opennav_docking` framework. Your `nav2_params.yaml`
already has a placeholder config. To make it functional, update the
`simple_charging_dock` section to reference the ArUco marker and battery topic.

See the separate config update in `dock_design/docs/nav2_dock_config.yaml`.

### Autonomous Dock Command

Once configured, trigger docking from the command line:

```bash
# Send robot to dock (dock_id and dock_pose must be configured)
ros2 action send_goal /dock_robot opennav_docking/DockRobot \
  "{dock_id: 'home_dock', max_staging_time: 30.0}"

# Undock
ros2 action send_goal /undock_robot opennav_docking/UndockRobot \
  "{dock_type: 'simple_charging_dock'}"
```

Or from the web dashboard (future integration): add DOCK / UNDOCK buttons that
call the action via rosbridge.

## Tuning & Troubleshooting

| Problem | Fix |
|---------|-----|
| Robot misses the funnel | Increase `funnel_flare` (try 80mm) or `funnel_length` (try 150mm) |
| Pogo pins don't reach | Increase `contact_wall_thick` or use longer pogo pins |
| Contact too high/low | Adjust `contact_height` in both SCAD files, reprint contact wall + robot plate |
| Robot bumps rear wall hard | Reduce Nav2 approach speed in docking config |
| ArUco not detected | Check marker size, lighting, camera exposure; tilt marker more (increase 15° angle) |
| Charging doesn't start | Check polarity, measure voltage at contacts, verify TP5100 jumper |
| Robot rocks on base grooves | Sand down grip grooves or reduce groove depth in SCAD |
| Dock slides on floor | Add rubber feet or anti-slip mat underneath |

## Customization

All dimensions are parametric in the OpenSCAD files. Key parameters to tweak:

```
charging_dock.scad:
  bay_width         = 310    ← Widen if robot is wider than expected
  bay_length        = 300    ← Shorten if you need a smaller dock
  wall_height       = 55     ← Lower if walls interfere with chassis
  funnel_flare      = 60     ← Wider = more forgiving entrance
  contact_height    = 35     ← Must match robot contact plate
  contact_spacing   = 60     ← Must match robot contact plate
  marker_post_height = 120   ← Adjust for camera field of view

robot_contacts.scad:
  contact_spacing   = 60     ← Must match dock
  pad_width         = 15     ← Wider = more forgiving alignment
  plate_width       = 100    ← Size of rear-mounted plate
```

**After changing dimensions**: Re-export STLs and verify with the `part="assembled"`
view in OpenSCAD before reprinting.
