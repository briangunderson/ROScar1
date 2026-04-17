# ROScar1 Extrusion Chassis Design Spec

**Date:** 2026-04-04
**Status:** Draft — awaiting user review
**Scope:** New aluminum extrusion chassis to replace the existing metal plate chassis
**3D Scan:** [Motor bracket scan (Polycam)](https://poly.cam/capture/47BED671-7E69-465D-95AC-22A8A67D7DB1)

---

## 1. Problem Statement

The current ROScar1 chassis is a flat metal plate with all components stacked on top. This creates several problems:

1. **Top-heavy / tips over** — The robot frequently tips forward or backward during rapid acceleration/deceleration. This is already severe enough that the driver firmware includes a deceleration limiter (1.0 m/s² linear, 3.0 rad/s² angular) specifically to prevent tipping.
2. **Poor component mounting** — Components are mounted with zip ties, double-sided tape, and a basic 3D-printed tray. Nothing is secure or serviceable.
3. **Narrow stance** — 205mm track width and 96.5mm wheelbase provide a small stability polygon. The robot's center of gravity sits well above this polygon.
4. **No expandability** — Adding new sensors, a robot arm, or a bigger battery requires redesigning the entire mounting scheme.
5. **Fragile aesthetics** — It looks like a prototype, not a robot.

## 2. Design Goals

| Priority | Goal | Success Criteria |
|----------|------|------------------|
| P0 | Fix tipping | CoG below 50% of total height; >80% of mass on lower deck |
| P0 | Wider stability polygon | Track width ≥ 280mm, wheelbase ≥ 180mm |
| P1 | Proper component mounting | Every component bolted or clamped; tool-free battery swap desirable |
| P1 | Use available stock | Primary structure from existing 3030 inventory |
| P2 | Expandability | T-slot rails allow mounting new components anywhere |
| P2 | Looks good | Clean, intentional industrial robot aesthetic |
| P3 | Parametric design | Fusion 360 API script for adjustable dimensions |

### Non-Goals
- Production-ready design (this is a learning/hobby project)
- Weatherproofing or IP rating
- Minimum weight optimization
- Compliance with any competition rules

## 3. Approaches Considered

### Approach A: Single-Deck Flat Frame
A single 3030 rectangle at ground level, everything mounted on top or on the sides.

**Pros:** Simplest build, lowest possible CoG for the frame itself.
**Cons:** Cramped component layout, no vertical separation between drive and compute layers, wiring nightmare. Camera and lidar still end up high, negating CoG advantage.

**Verdict:** Rejected — doesn't solve the organization problem, and the lidar/camera height means the CoG advantage is marginal.

### Approach B: Two-Deck Sandwich + Lidar Mast ✅ SELECTED
Lower deck for heavy/drive components, upper deck for compute/sensors, short mast for lidar.

**Pros:** Natural separation of concerns (power/drive below, compute/sensors above). Battery and motors at lowest level drops CoG dramatically. T-slot rails on both decks provide unlimited mounting options. Lidar mast keeps the upper deck low while giving lidar its 360° sightline.

**Cons:** More cuts and hardware than single-deck. Slightly taller overall. More complex build.

**Verdict:** Selected — best balance of stability, organization, and expandability.

### Approach C: Enclosed Box Frame
Full 3030 skeleton with panels on all sides, components mounted inside.

**Pros:** Maximum rigidity, best protection for components, most professional look.
**Cons:** Overkill for a learning robot, hardest to access components for tinkering, heaviest, uses the most material, blocks airflow for cooling.

**Verdict:** Rejected — over-engineered for a hobby robot where frequent access to components is important.

## 4. Selected Design: Two-Deck Sandwich

### 4.1 Architecture Overview

```
        ┌─────────┐
        │  LIDAR  │  ← 3D-printed mount plate
        └────┬────┘
             │       ← 3030 mast post (~120mm)
    ┌────────┴────────┐
    │   UPPER DECK    │  ← RPi5, camera (front)
    │  ┌──3030──┐     │
    │  │        │     │
    │  └────────┘     │
    └──┬──┬────┬──┬───┘
       │  │    │  │      ← 4× vertical posts (3030, ~100mm)
    ┌──┴──┴────┴──┴───┐
    │   LOWER DECK    │  ← Battery (center), motor board
    │  ┌──3030──┐     │
    │  │        │     │
    │  └────────┘     │
    └──┬──────────┬───┘
       │          │
   ┌───┴───┐  ┌──┴────┐
   │ MOTOR │  │ MOTOR │  ← 3D-printed brackets on frame exterior
   │ WHEEL │  │ WHEEL │
   └───────┘  └───────┘
```

### 4.2 Frame Dimensions

**Note:** These are target dimensions. Final values depend on motor measurements (pending 3D scan and caliper data from the user). The design is parametric — dimensions can be adjusted in the Fusion 360 script.

| Dimension | Value | Rationale |
|-----------|-------|-----------|
| Frame width (Y, outer) | 250mm | Provides ~320mm track width with outboard motors (measured: ~35mm motor offset per side) |
| Frame length (X, outer) | 250mm | Square frame for symmetric omnidirectional performance |
| Lower deck height (bottom of rail) | TBD — depends on motor shaft height | See cross-section diagram below |
| Vertical post height | 100mm | Clears battery (66mm) + motor board (~20mm) + wiring |
| Upper deck height (top of rail) | Lower deck + 100mm + 30mm | Post height + upper rail profile |
| Lidar mast height | 120mm above upper deck | Clears all components, provides 360° sightline |
| Total robot height | ~320-340mm (to top of lidar) | 30mm lower rail + 100mm posts + 30mm upper rail + 120mm mast + 41mm lidar + ground clearance |
| Ground clearance | TBD — set by motor bracket geometry | Target ≥ 15mm |

**Lower deck height cross-section (side view):**
```
Ground clearance and deck height depend on how the motor bracket positions
the axle relative to the frame rail. This will be finalized after motor
caliper measurements in Phase 1.

                    ┌──30mm──┐
                    │  3030  │ ← Lower deck rail
                    │  rail  │
                    └────────┘
                        │
              ┌─────────┴─────────┐
              │  3D-printed motor  │ ← Bracket clamps to outer face
              │     bracket        │
              └────────┬──────────┘
                       │
                    ───●─── ← Motor shaft at 39.7mm above ground
                    │     │
                   (wheel)    39.7mm radius

The bracket positions the motor shaft such that the wheel contacts the
ground. The frame rail height follows from the bracket geometry. Expected:
rail bottom at ~15-25mm above ground (15-25mm ground clearance).
```

**Stability polygon (updated with motor measurements):**

Motor offset per side: ~35mm (bracket ~3mm + motor face clearance + 13mm shaft + 18.7mm half-wheel-width).
- Track width: **~320mm** (250mm frame + 2×35mm) vs. current 205mm → **+56% wider**
- Wheelbase: **~200mm** (motors at ±100mm from center along frame rails) vs. current 96.5mm → **+107% longer**
- Stability polygon area: **~64,000mm²** vs. current ~19,800mm² → **3.2× larger**

The adjustable T-slot motor brackets allow fine-tuning these values without reprinting.

### 4.3 Center of Gravity Analysis

**3030 extrusion weight:** Standard 3030 aluminum extrusion weighs approximately **0.9–1.1 kg/m**. We use 1.0 kg/m for estimates. Verify with your specific stock (should be printed on packaging or available from supplier).

**Note:** The current robot (v1) weighs ~3.0kg total (2.2kg chassis + 0.8kg wheels per CLAUDE.md). The new extrusion frame is heavier than the original plate, adding roughly 1.5-2kg. This is acceptable — the motors handle 5kg+ loads easily, and the weight is concentrated low.

| Component | Weight (g) | Height (mm) | Moment (g·mm) | Notes |
|-----------|-----------|-------------|----------------|-------|
| 4× motors + wheels | 800 | 40 | 32,000 | 200g each (from URDF) |
| Battery | 574 | 55 | 31,570 | Lowest deck position |
| Motor board (56×85mm) | 80 (est.) | 55 | 4,400 | On lower deck plate, 50×60mm mounting holes |
| Lower frame (4× 250mm rails) | 1,000 | 35 | 35,000 | 4×0.25m × 1.0kg/m |
| Lower deck plate (3mm acrylic) | 100 (est.) | 50 | 5,000 | ~190×190mm |
| 4× vertical posts (100mm) | 400 | 90 | 36,000 | 4×0.1m × 1.0kg/m |
| Upper frame (4× 250mm rails) | 1,000 | 140 | 140,000 | 4×0.25m × 1.0kg/m |
| Upper deck plate (3mm acrylic) | 100 (est.) | 145 | 14,500 | ~190×190mm |
| RPi5 (in Argon case) | 180 | 160 | 28,800 | Measured |
| RPLIDAR C1 (55.6×55.6×41.3mm) | 110 | 295 | 32,450 | On mast top, 43mm square M2.5 mount |
| Lidar mast (120mm) | 120 | 200 | 24,000 | 0.12m × 1.0kg/m |
| Camera + mount | 100 (est.) | 150 | 15,000 | Front upper rail |
| Brackets & hardware | 300 (est.) | 70 | 21,000 | Distributed, mostly lower |
| **Total** | **~4,874** | | **422,670** |

**Estimated CoG height:** 422,670 / 4,874 ≈ **87mm** above ground

**CoG as % of total height:** 87 / 295 ≈ **29%** — well within the P0 target of <50%.

**Mass distribution:**
- Lower deck and below (motors, battery, board, lower frame, plate): 2,554g (**52%**)
- Mid-structure (posts, brackets): 700g (**14%**)
- Upper deck and above (upper frame, plate, RPi5, camera, lidar, mast): 1,620g (**33%**)

The majority of mass is at the lowest level. The heavier extrusion frame actually *helps* — 1.0kg of lower frame rails at 35mm height anchors the CoG down. Compare to the current robot where the CoG sits at ~50-60% of total height on the narrow plate.

### 4.4 Structural Detail

#### Corner Joint Strategy
Each of the 8 frame corners (4 lower, 4 upper) also has a vertical post. The **3-way corner bracket** handles all three connections at each corner (two horizontal rails + one vertical post), eliminating the need for separate L-brackets at these joints. This means:
- **8× 3-way corner brackets** at the 8 post-to-frame junctions (primary structure)
- **0× L-brackets** at frame corners (3-way brackets do double duty)
- L-brackets from inventory are available for future additions (cross bracing, shelf mounting, etc.)

#### Lower Deck Frame
- 4× 3030 rails forming a 250×250mm square perimeter
- Connected at corners by 3-way brackets (shared with vertical posts)
- Acrylic or aluminum deck plate (3mm) resting on top of rails, bolted using T-nuts
- Mounting holes in deck plate for battery strap and motor board standoffs

#### Vertical Posts
- 4× 3030 posts at each corner, 100mm tall
- Connected to lower and upper deck frames via 3-way corner brackets (one at each end = 8 total)
- Provides 100mm of internal clearance between decks

#### Upper Deck Frame
- 4× 3030 rails forming a matching 250×250mm square perimeter
- Connected at corners by 3-way brackets (shared with vertical posts)
- Acrylic or aluminum deck plate for RPi5 mounting
- Camera mount on front rail (3D-printed, adjustable tilt)

#### Lidar Mast
- 1× 3030 post, 120mm tall, mounted at center-rear of upper deck
- Uses a 3030 T-plate bracket to attach to the rear upper rail
- 3D-printed RPLIDAR C1 mount plate on top (circular, matches lidar mounting holes)
- Centered on X-axis for symmetric scan geometry

#### Motor Mounts
- 4× 3D-printed brackets (PLA or PETG)
- Each bracket clamps to the outside face of a lower deck frame rail
- Motor bolts through the bracket
- Bracket design allows sliding along the T-slot for track/wheelbase adjustment
- Wheels face outward from the frame

### 4.5 Component Placement

```
              FRONT
    ┌──────────────────────┐
    │  ┌──CAMERA──┐        │  ← Upper deck, front rail
    │  └──────────┘        │
    │                      │
    │     ┌────────┐       │
    │     │  RPi5  │       │  ← Upper deck, center
    │     │ Argon  │       │
    │     └────────┘       │
    │              [MAST]  │  ← Lidar mast, center-rear
    └──────────────────────┘

    ┌──────────────────────┐
    │                      │
    │     ┌────────┐       │
    │     │BATTERY │       │  ← Lower deck, center
    │     │ 574g   │       │
    │     └────────┘       │
    │   ┌──────────┐       │
    │   │MTR BOARD │       │  ← Lower deck, behind battery
    │   └──────────┘       │
    └──────────────────────┘
              REAR

    M1(FL)──────────M3(FR)    ← Motors on exterior brackets
       │              │
       │              │
    M2(RL)──────────M4(RR)
```

**Rationale for placement:**
- **Battery center-low:** Heaviest component at the geometric center of the stability polygon, as low as possible.
- **Motor board behind battery:** Short motor wires, close to battery power input. USB cable routes up a rear post to the RPi5.
- **RPi5 upper-center:** Central location, USB ports face rear (toward motor board cable route). NVMe underneath (Argon case design).
- **Camera front-upper:** Forward-facing, unobstructed view. 3D-printed tilt bracket allows angle adjustment.
- **Lidar mast center-rear:** Avoids camera obstruction, center-X gives symmetric scans. Rear placement keeps front clear for camera FOV. Height provides unobstructed 360° above all other components.

### 4.6 Cut Plan

> **Verified against the Fusion 360 CAD model (`docs/chassis/fusion360/roscar_v2_chassis.py`, rev11+) on 2026-04-17.** Rail lengths and positions in the CAD match this table exactly.

**Joinery style:** Box-joint with 3-way corner brackets. Front and rear rails (along Y axis) span the **full frame width** with their cross-sections at the outer corners. Left and right rails (along X axis) are **shorter** and fit *between* the front and rear rails. Corner brackets sit at the inside of the frame corners, clamping the two horizontal rails + the vertical post at each junction.

```
Looking down at the frame (top view):
    ┌─────── FRONT (248mm) ───────┐    
    │                             │    The F/R rails span the
    │                             │    full 248mm frame side.
    │   LEFT           RIGHT      │    
    │  (188mm)        (188mm)     │    The L/R rails are 188mm
    │                             │    (= 248 − 2×30 for the
    │                             │    front and rear rail
    └─────── REAR  (248mm) ───────┘    cross-sections).
```

**Frame outer dimension:** 248mm × 248mm (equal to the Front/Rear rail length).

**Kerf allowance:** A miter saw blade removes ~2–3mm per cut. All cut lengths below account for this.

**Stock used:**

| Stock in | Cut Into | Qty out | Kerf loss | Remainder | Purpose |
|----------|----------|---------|-----------|-----------|---------|
| 2× 500mm black 3030 | 2× 248mm each | **4× 248mm** | ~4mm/stick | ~0mm waste | **Front/Rear rails** (2 lower + 2 upper) |
| 2× 400mm black 3030 | 2× 188mm each | **4× 188mm** | ~4mm/stick | ~20mm waste | **Left/Right rails** (2 lower + 2 upper) |
| 1× 400mm black 3030 | 3× 100mm | **3× 100mm** | ~9mm (3 cuts) | ~91mm spare | Vertical corner posts (3 of 4) |
| 1× 400mm black 3030 | 1× 100mm + 1× 120mm | **1× 100mm + 1× 120mm** | ~6mm (2 cuts) | ~174mm spare | 4th vertical post + lidar mast |

**Tolerance:** Target ±1mm on all cuts. After cutting, measure each piece and match pairs of equal length to opposing sides of the frame. During assembly, measure both diagonals on each deck — they should be equal within 2mm to verify squareness.

**Label every piece as you cut it.** Suggested marking (Sharpie on the end that won't be visible):

| Label | Qty | Length | Destination |
|-------|-----|--------|-------------|
| `F-Lo` | 1 | 248mm | Lower deck, front rail (low X) |
| `F-Hi` | 1 | 248mm | Upper deck, front rail (low X) |
| `R-Lo` | 1 | 248mm | Lower deck, rear rail (high X) |
| `R-Hi` | 1 | 248mm | Upper deck, rear rail (high X) |
| `L-Lo` | 1 | 188mm | Lower deck, left rail (low Y) |
| `L-Hi` | 1 | 188mm | Upper deck, left rail (low Y) |
| `Ri-Lo` | 1 | 188mm | Lower deck, right rail (high Y) |
| `Ri-Hi` | 1 | 188mm | Upper deck, right rail (high Y) |
| `P-FL` / `P-FR` / `P-RL` / `P-RR` | 4 | 100mm | Vertical corner posts |
| `M` | 1 | 120mm | Lidar mast |

**Stock preserved (untouched):**

| Stock | Qty | Notes |
|-------|-----|-------|
| 500mm black 3030 | 2 | Future expansion |
| 400mm black 3030 | 0 | All 4× 400mm sticks consumed above |
| 600mm black 3030 | 4 | Future expansion (cross bracing, arm mount, etc.) |
| 1000mm silver 3030 | 4 | Future expansion (larger robot, test fixtures) |

**Total cuts:** 13 pieces from 6 sticks. All cuts are straight 90° on the miter saw.

### 4.7 Hardware Bill of Materials

#### From Existing Inventory
| Item | Qty Needed | Source (from Amazon orders) |
|------|-----------|---------------------------|
| 3030 3-way corner brackets | 8 | 12+12+4+4 sets available (Nov-Dec 2025) — plenty |
| 3030 T-plate connectors | 1 | 8pcs available (Nov 2025) — for lidar mast |
| 3030 L-bracket connectors | 0 (spare) | 24-set + 8pcs available — not needed for initial build but useful for future cross bracing |
| M6 T-slot nuts | ~30 | Included with bracket sets |
| M6 bolts | ~30 | Included with bracket sets |
| 3030 tapping jig | 1 | Dec 2025 order |

#### To Purchase or Fabricate
| Item | Qty | Notes |
|------|-----|-------|
| Acrylic/aluminum deck plates | 2 | ~190×190mm (inside frame dimension: 248 - 30 - 30 = ~188mm, round up), 3mm thick. Cut from existing sheet stock. If plates sit ON TOP of rails instead: ~248×248mm. |
| M3 standoffs (for RPi5) | 4 | If not using Argon case screw holes directly |
| Velcro battery strap | 1 | Or 3D-print a battery cradle |
| Nylon cable clips for T-slot | ~10 | Cable management along frame rails |

#### 3D-Printed Parts (Prusa MK4S)
| Part | Qty | Material | Notes |
|------|-----|----------|-------|
| Motor mount bracket | 4 | PETG | Clamps to 3030 rail exterior, motor bolts through. Design depends on motor measurements. |
| RPLIDAR C1 mount plate | 1 | PLA | Square plate (~60×60mm), sits atop mast post, 43mm square M2.5 hole pattern, max 4mm screw depth |
| Camera tilt bracket | 1 | PLA | Adjustable-angle mount, clamps to front upper rail |
| Battery cradle (optional) | 1 | PETG | Holds battery in place on lower deck plate, with strap cutout |
| Cable routing clips | 4-6 | PLA | Snap into T-slot channels |

## 5. Electrical / Wiring Routing

No electrical changes to the robot's ROS2 stack or motor board. All existing wiring is preserved — only the physical routing changes.

```
Battery ──(power leads)──→ Motor board (T-plug, lower deck)
Motor board ──(4× motor cables)──→ 4× Motors (route along lower frame rails)
Motor board ──(USB-C)──→ RPi5 (route up rear vertical post)
RPLIDAR C1 ──(USB)──→ RPi5 (route down mast, along upper frame, down post)
Webcam ──(USB)──→ RPi5 (short cable, both on upper deck)
RPi5 ←──(USB-C power)── Motor board 5V out (Type-C supports Pi5 power protocol)
```

**Cable management:** T-slot channels in the 3030 profiles double as cable conduits. The 8mm slot opening accepts most USB cables. Use snap-in cable clips (3D-printed or purchased) to secure cables along the frame rails.

## 6. Software Impact

### 6.1 What Changes
- **URDF:** New `roscar_v2.urdf.xacro` file with updated dimensions. Created alongside existing URDF — no breaking changes.
  - `base_link` dimensions change (250×250×100mm body)
  - Wheel joint positions change (wider track, longer wheelbase)
  - Camera and lidar link positions change (new mount points)
  - New visual geometry (box approximations or STL meshes from Fusion 360)
- **driver_params.yaml:** No changes needed — the driver doesn't use physical dimensions (the STM32 handles motor control internally)
- **driver_node.py IMU corrections:** If the motor board mounting orientation changes (currently 180° rotated), the accelerometer/gyroscope sign negations must be updated to match. Verify during Phase 5.
- **laser_filter.yaml:** The current filter drops scan returns < 0.15m to remove chassis self-reflections. The new frame geometry is different — the lidar mast may place the scan plane well above all frame members (eliminating the need for the filter) or the threshold may need adjustment. Test during Phase 6.
- **EKF config:** May need minor covariance tuning for the new dynamics
- **Nav2 config:** Update robot footprint polygon to match new frame dimensions
- **Deceleration limiter:** Can likely be relaxed (higher `max_decel_linear`) since the robot will be much more stable

### 6.2 What Doesn't Change
- `driver_node.py` — no changes
- `mecanum_kinematics.py` — no changes (the STM32 does FK internally)
- All launch files — no changes
- Web dashboard — no changes
- CV pipeline — no changes
- SLAM/Nav2 stack — no changes (just the footprint parameter)

### 6.3 Parallel Development Strategy
The new chassis design is developed in a `docs/chassis/` directory. No existing source files are modified until the physical build is complete and ready for the software switchover. At that point:
1. Create `roscar_v2.urdf.xacro` alongside the existing URDF
2. Add a launch argument `chassis_version:=v1|v2` to `robot.launch.py`
3. Update Nav2 footprint parameter
4. Test and tune

## 7. Fabrication Plan

### Phase 1: Measure (before any cutting)
1. ~~Complete 3D scan of motor+bracket assembly~~ **DONE:** [Polycam scan](https://poly.cam/capture/47BED671-7E69-465D-95AC-22A8A67D7DB1)
2. ~~Caliper motor dimensions~~ **DONE:** 24mm dia, 62mm body length (75mm total w/ shaft), 17mm mounting screw spacing, 13mm shaft protrusion
3. ~~Caliper the motor board~~ **DONE:** 56×85mm, mounting holes 50×60mm spacing
4. ~~Measure RPLIDAR C1 mounting hole pattern~~ **DONE:** 43mm square, 4× M2.5 screws (max 4mm depth per datasheet), body 55.6×55.6×41.3mm, 110g
5. Measure webcam mounting options
6. ~~Measure motor shaft diameter~~ **DONE:** 4mm shaft diameter

### Phase 2: Design 3D-Printed Parts (Fusion 360)
1. Motor mount bracket — parametric, based on measurements
2. RPLIDAR mount plate
3. Camera tilt bracket
4. Battery cradle
5. Cable routing clips

### Phase 3: Cut and Assemble Frame
1. Cut 3030 stock per cut plan (miter saw). Measure each piece after cutting.
2. Deburr all cuts (file or sandpaper)
3. Match opposing rail pairs by length (measure all 8 rails, pair the closest matches for opposite sides)
4. Assemble lower deck frame (4 rails + 3-way brackets at corners, hand-tight only)
5. **Square the frame:** Measure both diagonals — they should be equal within 2mm. Adjust corners until square, then tighten all brackets.
6. Attach vertical posts with 3-way brackets (use a square against the deck to verify 90°)
7. Assemble upper deck frame on top of posts, square it the same way
8. Install deck plates
9. Attach lidar mast

### Phase 4: Print and Install Brackets
1. Print motor mount brackets (PETG, 30% infill minimum for strength)
2. Print lidar mount plate
3. Print camera bracket
4. Print battery cradle
5. Install all 3D-printed parts
6. Test-fit all components before final tightening

### Phase 5: Component Transfer
1. Remove components from current chassis (or use spares for robot #2)
2. Install battery on lower deck
3. Install motor board on lower deck
4. Mount motors with mecanum wheels
5. Mount RPi5 on upper deck
6. Mount RPLIDAR on mast
7. Mount webcam on front
8. Route and secure all cables

### Phase 6: Software Switchover
1. Create `roscar_v2.urdf.xacro` with measured dimensions
2. Add chassis version launch argument
3. Update Nav2 footprint
4. Verify motor board mounting orientation — update IMU axis corrections in `driver_node.py` if changed
5. Calibrate gyro bias (new mounting orientation)
6. Test laser filter — check if chassis reflections are still present with new lidar height, adjust `laser_filter.yaml` threshold if needed
7. Test teleop, SLAM, navigation
8. Tune deceleration limits (probably can increase significantly)
9. Tune EKF covariances if needed
10. (Optional) Investigate reflashing STM32 with updated wheel geometry constants

## 8. Mecanum Wheel Geometry Considerations

For optimal omnidirectional performance, the track-to-wheelbase ratio matters. With mecanum wheels (ABBA pattern), the ratio of (track_width / 2 + wheelbase / 2) affects the coupling between translation and rotation.

- **Ratio near 1.0:** Equal turning and strafing authority. The robot can strafe and rotate independently without parasitic coupling.
- **Ratio >> 1.0 (wide and short):** Strong strafing, weak rotation coupling. Good for lateral stability but rotation feels sluggish.
- **Ratio << 1.0 (narrow and long):** Strong rotation, weak strafing coupling.

**Current robot:** (205/2 + 96.5/2) = 150.75mm → ratio = 205/96.5 ≈ 2.12 — significantly wide-biased.

**New design (measured):** Track ~320mm, wheelbase ~200mm → ratio = 320/200 = 1.6 — closer to ideal while still favoring stability.

The exact values will be tuned once motor measurements are in. The adjustable motor bracket design (slides along T-slot) allows fine-tuning this ratio without any reprinting.

## 9. Future Expansion Opportunities

The T-slot extrusion frame naturally supports future additions:

| Addition | How It Mounts | Stock Needed |
|----------|--------------|--------------|
| Robot arm | Upper deck rail, T-nuts | May need cross bracing (use 400mm spares) |
| Second camera (stereo) | Upper front rail, mirror of first | Just print another bracket |
| Bigger battery | Lower deck, wider cradle | May need longer battery strap |
| Ultrasonic sensors | Lower frame rails, facing outward | 3D-printed snap-in mounts |
| LED light strip | T-slot channel on any rail | Already have LED channels from Amazon |
| Bumper sensors | 3D-printed bumper on front/rear rails | Print + microswitches |
| Depth camera (RealSense) | Upper front rail, dedicated bracket | Print bracket + verify USB bandwidth |
| Compute upgrade (Jetson) | Upper deck, replaces RPi5 | New standoff pattern |
| OLED status display | Any upper frame rail | Uses motor board OLED header |

## 10. Open Questions (to resolve during build)

1. **Motor wire length:** Are the existing motor cables long enough for the wider stance? If not, need to extend or replace.
2. **USB cable lengths:** Motor board USB to RPi5 will be a longer route. May need a longer USB-C cable.
3. **Deck plate material:** Acrylic (lighter, easier to cut, can see components) vs. aluminum (more rigid, better grounding). User has both on hand.
4. **Battery retention:** Velcro strap vs. 3D-printed cradle with latch. Cradle is more secure but strap is faster for swaps.
5. **Color scheme:** Black 3030 frame with black 3D prints? Or accent color for printed parts? (The user has an MMU3 for multi-color prints.)
6. **Robot #2 vs. transfer:** User may build a second robot rather than strip the original. If so, need to order a second motor board, Pi5, motors, etc.
7. **STM32 firmware geometry constants:** The STM32 firmware has hardcoded mecanum wheel geometry (L_x, L_y parameters) used for both inverse and forward kinematics. Changing the physical track width and wheelbase means these constants are wrong. Options: (a) reflash with updated constants (Yahboom source is in `D:\localrepos\ROS-robot-expansion-board`), (b) live with the error and let EKF+SLAM compensate, (c) move kinematics to the ROS2 layer (major refactor, not recommended for now).
8. **IMU axis corrections:** The motor board (containing the IMU) is currently mounted 180° rotated. If the mounting orientation changes in the new chassis, the accelerometer/gyroscope sign corrections in `driver_node.py` must be updated. Verify board orientation during Phase 5.
9. **RPi5 power delivery:** The Pi5 requires 5V/5A (25W) for full performance. Verify that the motor board's USB-C 5V output (which supports Pi5 power protocol per Yahboom docs) can sustain this under load with the longer cable routing.

## 11. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Motor measurements don't fit bracket design | Medium | Low | Brackets are 3D-printed — just reprint. Parametric design makes this fast. |
| STM32 firmware has hardcoded wheel geometry | High | Medium | The STM32 firmware contains hardcoded mecanum kinematics (track width, wheelbase) for both IK (cmd_vel → wheel speeds) and FK (encoders → odom). Changing physical geometry means firmware odometry will be wrong and motion commands will have cross-coupling errors. **Mitigations:** (1) EKF + IMU fusion partially compensates, (2) SLAM provides map-based corrections, (3) Yahboom provides STM32 firmware source in the reference repo — we may be able to update the geometry constants and reflash, (4) for a learning robot, small kinematic errors are tolerable. See Open Question #7. |
| Frame heavier than current chassis | High | Low | 3030 at ~1.0 kg/m means ~2.5kg of extrusion. Total robot ~4.9kg vs current ~3.0kg. Motors handle this easily (rated for heavier loads). Battery life may decrease ~20-30%. The extra weight is mostly low, which *improves* stability. |
| Frame too tall after lidar mast | Low | Medium | Mast height is adjustable. Can shorten if CoG analysis shows it's fine. |
| Motor wires too short | Medium | Low | Extend with solder + heat shrink, or order replacement cables. |
| Deck plate vibration/resonance | Low | Low | Add rubber standoffs or damping pads if needed. |
| Laser filter threshold needs adjustment | Medium | Low | Current filter drops scan returns < 0.15m (chassis reflections). New frame geometry is different — the lidar mast may place the scan plane above all frame members (eliminating the problem), or the threshold may need tuning. Test during Phase 6. |

---

*Design by Claude + Brian Gunderson, 2026-04-04*
*Part of the ROScar1 project: github.com/bgunderson/ROScar1*
