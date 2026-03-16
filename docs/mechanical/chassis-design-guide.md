# ROScar V2: Data-Driven Chassis & Mechanical Design

## 1. Current Robot Audit — What We Know and What's Wrong

### 1.1 Known Problems (from real-world operation)

| Problem | Root Cause | Impact |
|---------|-----------|--------|
| **Tips forward on braking** | High CoG, short wheelbase, heavy top (Pi5+LIDAR+camera up high) | Requires software decel limiter (1.0 m/s²) — wastes dynamic capability |
| **Tips backward on acceleration** | Same — rear weight transfer exceeds rear axle moment arm | Can't use full motor torque |
| **Wires exposed everywhere** | No cable management, no enclosure design | Snag hazard, unreliable connections, looks terrible |
| **Cockeyed webcam** | Hot-glued to 3D-printed disc, no adjustment mechanism | Bad camera calibration, inconsistent CV results |
| **Fragile sensor mounts** | RPLIDAR and camera are afterthoughts, not structural | Vibration causes scan artifacts and blurry images |
| **Unknown dimensions** | Never measured — URDF is all placeholders | Nav2 footprint wrong, costmap wrong, SLAM drift |

### 1.2 Current Placeholder Dimensions (from URDF)

```
                 250mm (placeholder)
          ┌──────────────────────────┐
          │                          │
          │  ●FL              FR●    │ 200mm (placeholder)
          │      200mm wheelbase     │
          │                          │
          │  ●RL              RR●    │
          │                          │
          └──────────────────────────┘
               240mm track width

Wheel radius: 33mm (placeholder)
Wheel width: 40mm (placeholder)
Ground clearance: 50mm (chassis bottom to ground, placeholder)
Total height: ~180mm (chassis + LIDAR on top platform)
Mass: ~2.8 kg estimated (2.0 chassis + 4×0.2 wheels)
```

### 1.3 What the STM32 Firmware Assumes

The YB-ERF01-V3.0 board's firmware has hardcoded wheel geometry for its inverse kinematics (cmd_vel → individual motor speeds) and forward kinematics (encoder → body velocity). **These values are baked into the STM32 and cannot be changed without reflashing firmware.** The ROS layer trusts the board's velocity output — `mecanum_kinematics.py` just integrates.

This means the physical wheel radius and track/wheelbase dimensions should ideally match what the firmware expects, OR we need to calibrate a correction factor in software.

**Action item**: Determine the firmware's assumed geometry. Options:
1. Drive a known distance, compare encoder-reported vs actual → derive effective wheel radius
2. Spin in place, compare gyro-measured vs encoder-measured yaw rate → derive effective track width
3. Check Yahboom documentation for their assumed dimensions

---

## 2. Physics of Tipping — The Core Problem

### 2.1 Static Stability Analysis

A robot tips when the moment from deceleration about the wheel contact exceeds the moment from gravity:

```
                    ──── decel ────►

          CoG ──── ● ◄── m·a·h (tipping moment)
               h   │
               │   │
               │   m·g (restoring moment)
          ─────┼───┼───────── ground
               │   │
              rear  L/2  front contact
              axle       point
```

**Tipping condition (braking, tips forward):**
```
m · a_decel · h_cog  >  m · g · (L/2)

Simplifies to:
a_max = g · L / (2 · h_cog)

Where:
  g = 9.81 m/s²
  L = wheelbase (front-to-rear axle distance)
  h_cog = center of gravity height above ground
```

**For the current robot (estimated):**
```
L ≈ 0.20 m (200mm wheelbase, placeholder)
h_cog ≈ 0.12 m (120mm — guess: wheels+chassis+battery low, Pi+LIDAR high)

a_max = 9.81 × 0.20 / (2 × 0.12) = 8.2 m/s²
```

That seems fine for 1.0 m/s² decel limit... but the real numbers are probably worse. The actual CoG is likely higher than 120mm once you account for the RPLIDAR on a top platform, the Pi5 stack, and cable mess adding height. And the actual wheelbase may be shorter than 200mm.

### 2.2 Design Target

For a robot that doesn't need software decel limiting to stay upright:

| Parameter | Target | Rationale |
|-----------|--------|-----------|
| **a_max (no tip)** | ≥ 5 m/s² | 5× safety margin over 1.0 m/s² decel limit |
| **L/h_cog ratio** | ≥ 1.0 | Wheelbase ≥ CoG height (bare minimum for stability) |
| **L/h_cog ratio** | ≥ 1.5 | Comfortable — robot handles aggressive maneuvers |

**Rearranging: L ≥ 2 · h_cog · a_target / g**
```
For a_target = 5.0 m/s² and h_cog = 0.10 m:
L ≥ 2 × 0.10 × 5.0 / 9.81 = 0.102 m (102mm minimum wheelbase)

For h_cog = 0.15 m (tall stack):
L ≥ 2 × 0.15 × 5.0 / 9.81 = 0.153 m (153mm minimum wheelbase)
```

**Key insight**: Lowering the CoG is more effective than lengthening the wheelbase. Every mm you lower the CoG buys you the same stability as ~1mm of wheelbase, but without making the robot bigger.

### 2.3 Lateral Tipping (Strafing)

Mecanum robots also tip sideways during aggressive strafing. The same formula applies but with track width W instead of wheelbase L:

```
a_max_lateral = g · W / (2 · h_cog)
```

Since track width is usually wider than wheelbase on mecanum robots, lateral tipping is less of a concern — but it needs to be checked.

---

## 3. Design Strategy: Low & Wide

### 3.1 Guiding Principles

1. **CoG as low as possible** — heaviest components (battery, motors, driver board) at the bottom
2. **Wheelbase as long as practical** — extrusion frame makes this easy to adjust
3. **Sensors on short, rigid mounts** — not towering above the chassis
4. **Fully enclosed** — 3D-printed panels on aluminum extrusion frame
5. **Modular** — easy to reconfigure, swap components, access internals
6. **Parametric** — every dimension driven by a design spreadsheet, not eyeballed

### 3.2 Aluminum Extrusion Frame Concept

Your 2020/2040/3030 extrusions are ideal for this. The basic structure:

```
  TOP VIEW (extrusion frame)

  ══════════════════════════════════  ← 2040 cross-member (front)
  ║                                ║
  ║  3030 longitudinal rail (L)    ║  3030 longitudinal rail (R)
  ║                                ║
  ║         ┌─────────┐           ║
  ║         │ Pi5 + HAT│           ║  ← mounted LOW, inside frame
  ║         └─────────┘           ║
  ║                                ║
  ║  ┌───┐            ┌───┐      ║
  ║  │BAT│            │STM│      ║  ← battery and driver board LOW
  ║  └───┘            └───┘      ║
  ║                                ║
  ══════════════════════════════════  ← 2040 cross-member (rear)

  SIDE VIEW
                    LIDAR    Camera
                      ●────────◄ (short rigid mount, not a tower)
  ═══════════════════════════════  ← top plate (3D printed or thin aluminum)
  │ Pi5  │  STM32  │  Battery │  ← components INSIDE, mounted low
  ═══════════════════════════════  ← bottom plate (extrusion frame level)
  ●──────────────────────────●   ← wheels at frame corners
       wheelbase (300mm+)
```

### 3.3 Component Stacking Strategy

Layer the components from bottom to top by mass:

| Layer | Height | Components | Est. Mass |
|-------|--------|------------|-----------|
| **0 — Ground** | 0mm | Wheel contact patches | — |
| **1 — Motor plane** | ~33mm | 4× motors + wheels | 400g+ |
| **2 — Battery/board** | 40–60mm | LiPo battery, YB-ERF01-V3.0 | 300–500g |
| **3 — Compute** | 60–80mm | Pi5 + active cooler + USB hub | 150g |
| **4 — Sensor deck** | 80–120mm | RPLIDAR C1, webcam | 200g |

**Target CoG**: 60–70mm above ground (battery and motors dominate mass).

---

## 4. What You Need to Measure (Current Robot)

Before designing V2, measure the V1 to calibrate our models and understand the motors.

### 4.1 Measurement Checklist

Run the Python tool in `tools/measure_robot.py` (created alongside this doc) to get a guided interactive checklist with calculations, or measure manually:

**Geometry:**
- [ ] Wheel outer diameter (measure across mecanum rollers at widest point)
- [ ] Wheel width (hub face to hub face, including rollers)
- [ ] Wheelbase (center-to-center, front axle to rear axle)
- [ ] Track width (center-to-center, left wheel to right wheel)
- [ ] Ground clearance (floor to lowest point of chassis)
- [ ] Overall dimensions: length × width × height
- [ ] Axle height (floor to wheel center — should equal wheel radius)

**Mass:**
- [ ] Total robot mass (kitchen scale or luggage scale)
- [ ] Individual component masses if possible:
  - Motors+wheels (one assembly)
  - Battery
  - Pi5 + cooler
  - YB-ERF01-V3.0 board
  - RPLIDAR C1
  - Webcam
  - Chassis/plates/fasteners (everything else)

**Center of Gravity:**
- [ ] Longitudinal CoG: balance robot on a rod across the width, find the front-back balance point
- [ ] Vertical CoG (tilt test): tilt robot on rear wheels until it just tips forward, measure the angle — `h_cog = (L/2) × tan(90° - tilt_angle)`
- [ ] Or: hang robot from a string at two different points, intersection of plumb lines = CoG

**Motor Performance:**
- [ ] Stall current (per motor, from battery voltage / motor resistance)
- [ ] No-load RPM (run motor_test.py, measure encoder rate with no ground contact)
- [ ] Max speed on ground (drive straight at full throttle, read encoder velocity)
- [ ] Battery voltage under load vs idle

### 4.2 Firmware Calibration (Critical)

Run these tests to determine what the STM32 firmware assumes:

```bash
# Test 1: Effective wheel radius
# Drive forward at constant cmd_vel for exactly 2 meters (tape measure on floor)
# Compare actual distance to odom_raw reported distance
# effective_radius_correction = actual_distance / odom_distance

# Test 2: Effective track geometry
# Command pure rotation (vx=0, vy=0, wz=0.5) for 10 full rotations
# Compare gyro-counted rotations to odom-counted rotations
# track_correction = odom_rotations / gyro_rotations

# Test 3: Strafing accuracy
# Command pure strafe (vx=0, vy=0.2, wz=0) for 1 meter
# Measure actual lateral displacement vs odom
```

---

## 5. Parametric Design Calculator

See `tools/chassis_designer.py` — a Python script that takes your measurements and constraints as inputs and outputs:

1. **Stability analysis** — tipping thresholds for accel/decel/strafe
2. **Recommended dimensions** — wheelbase, track width, CoG height target
3. **Extrusion cut list** — exact lengths for your aluminum extrusions
4. **Mass budget** — component placement to hit CoG target
5. **Motor adequacy check** — can the motors handle the new (possibly heavier) chassis?
6. **3D print list** — brackets, mounts, panels needed
7. **Updated URDF values** — ready to paste into `roscar.urdf.xacro`

---

## 6. Motor & Drivetrain Analysis

### 6.1 What We Know About the Motors

| Spec | Value | Source |
|------|-------|--------|
| Encoder CPR | 1320 (30:1 × 11 PPR × 4x) | CLAUDE.md |
| Gear ratio | 30:1 | CLAUDE.md |
| Firmware max vx | ±1.0 m/s | driver_params.yaml |
| Firmware max wz | ±5.0 rad/s | driver_params.yaml |
| PID loop rate | 100 Hz | STM32 firmware |

### 6.2 What We Need to Determine

| Unknown | How to Measure | Why It Matters |
|---------|----------------|----------------|
| **Motor stall torque** | Current measurement at stall | Determines max acceleration and hill climbing |
| **No-load RPM** | Encoder at full voltage, wheels off ground | Determines max speed ceiling |
| **Motor voltage rating** | Datasheet or battery voltage | Over/under voltage affects lifetime |
| **Motor resistance** | Multimeter across terminals | Efficiency and thermal limits |
| **Wheel-ground friction** | Tipping test, push force | Determines actual usable torque |
| **Mecanum efficiency** | Strafe vs forward speed at same power | Mecanum wheels lose ~30-40% in strafe |

### 6.3 Speed-Torque Tradeoff

For a DC motor: `τ = τ_stall × (1 - ω/ω_no_load)`

The 30:1 gear ratio means these motors trade speed for torque. At the wheel:
```
τ_wheel = τ_motor × gear_ratio × efficiency
ω_wheel = ω_motor / gear_ratio

v_linear = ω_wheel × wheel_radius
F_drive = τ_wheel / wheel_radius
a_max = F_drive / m_robot  (per wheel, ×4 for all wheels)
```

If the new chassis is heavier (more aluminum extrusion), we need to verify the motors can still accelerate adequately. The calculator tool does this check.

---

## 7. Fabrication Plan (Using Your Tools)

### 7.1 Aluminum Extrusion (Frame)

| Part | Extrusion | Qty | Purpose |
|------|-----------|-----|---------|
| Longitudinal rails | 3030 | 2 | Main side rails, carry load |
| Front/rear cross | 2040 | 2 | Connect rails, mount motors |
| Sensor cross | 2020 | 1 | LIDAR + camera mount bar |

**Joining**: T-nuts and M5/M6 bolts, or corner brackets. No welding needed.

**Advantage**: Infinitely adjustable. Slide components along slots to tune CoG. Change wheelbase by swapping cross-member position.

### 7.2 3D Printed Parts (Prusa MK4S)

| Part | Material | Notes |
|------|----------|-------|
| Motor mounting brackets | PETG or ASA | Bridge motor to extrusion, need dimensional accuracy |
| Pi5 mounting tray | PLA or PETG | Snap-fit or screw to extrusion slot |
| Camera mount (adjustable) | PETG | Tilt mechanism with set screw — no more hot glue |
| RPLIDAR mount plate | PETG | Vibration-isolated with rubber grommets |
| Side panels | PLA | Cosmetic + cable containment |
| Battery tray | PETG | Secure retention, easy swap |
| Cable routing clips | PLA | Snap into extrusion T-slots |

### 7.3 CNC Parts (Genmitsu 4030)

| Part | Material | Notes |
|------|----------|-------|
| Top plate | 3mm aluminum or acrylic | Sensor deck, clean flat surface |
| Bottom plate | 3mm aluminum | Structural base if extrusions alone aren't rigid enough |
| Motor adapter plates | 3mm aluminum | If 3D printed brackets flex too much under load |

### 7.4 Genmitsu 4030 CNC (Plywood)

Work area: 400×300mm — plenty for robot chassis plates.

| Part | Material | Notes |
|------|----------|-------|
| Base plate prototype | 6mm plywood or MDF | Test fit before committing to aluminum |
| Top deck prototype | 3mm plywood | Sensor deck trial fit |
| Motor mount test jig | 6mm plywood | Verify motor spacing before printing PETG brackets |

**Feeds & speeds for plywood (1/8" single-flute upcut):**
- Feed: 800–1000 mm/min, plunge: 300 mm/min
- Depth per pass: 2mm, spindle: 10000 RPM
- Use tabs (2mm) to prevent parts shifting during cutout

---

## 8. Sensor Placement Design

### 8.1 RPLIDAR C1 Constraints

- Needs **360° unobstructed view** in the scan plane
- Current problem: chassis self-reflections below 0.15m (filtered in software)
- **Solution**: Mount at the highest point, but on a short post, not a tower
- If LIDAR scan plane is just 20-30mm above the top surface, chassis reflections disappear
- Keep LIDAR **centered** on the robot (0, 0 in XY relative to base_link) for best SLAM results

### 8.2 Camera Constraints

- Needs **forward-facing, level view**
- Current problem: hot-glued at an angle, vibrates
- **Solution**: Adjustable tilt mount with set screw
- Mount at front of chassis, slightly above LIDAR scan plane to avoid being in the laser's shadow
- Camera optical axis should intersect the ground ~1-2m ahead at default tilt

### 8.3 IMU Location

- IMU is on the YB-ERF01-V3.0 board — wherever the board goes, the IMU goes
- Ideally near the center of rotation (minimize centripetal acceleration errors)
- Current: board is 180° rotated, software-corrected. New chassis should mount board in correct orientation to eliminate sign corrections.

---

## 9. Charging Dock Considerations

A separate session produced an initial dock concept in `dock_design/` (branch `claude/robot-charging-dock-xRrTF`). That design was a quick first pass, so treat it as a starting point rather than a hard constraint. The chassis should be designed first, then the dock adapted to fit.

**Design decisions that flow from chassis → dock** (not the other way around):

| Chassis Decision | Dock Impact |
|-----------------|-------------|
| Ground clearance | Determines dock wall height and contact plate position |
| Track width | Determines dock bay width (track + ~30mm clearance) |
| Overall length | Determines dock bay depth |
| Rear face geometry | Determines where/how charging contacts mount |
| Camera position | ArUco marker post height for docking approach |

**What the chassis should provide for ANY charging dock design**:
1. **Flat rear surface** — some area to mount charging contacts (pogo pads, magnetic contacts, etc.)
2. **Wire routing path** — from rear face to battery (channel in extrusion or clip path)
3. **Battery access** — ability to connect charge wires to battery/BMS input
4. **No rear protrusions** — keep the rear clean for docking alignment

The dock's parametric SCAD files can be regenerated once chassis dimensions are finalized. The `chassis_designer.py` tool outputs the dock parameters that need updating.

---

## 10. Cable Management

### 9.1 Current Mess

Wires everywhere: USB cables (Pi→STM32, Pi→RPLIDAR, Pi→webcam), motor wires, battery wires, power cables. No strain relief, no routing.

### 9.2 V2 Approach

| Cable | Routing |
|-------|---------|
| Motor wires (×4) | Through extrusion channel or clips along inner rail |
| USB cables (×3) | Bundled, short custom-length cables, routed along one rail |
| Power | Battery → board, short direct path at lowest layer |
| RPi power | From STM32 board's 5V output, short jumper |

**Key parts**:
- 3D-printed cable clips that snap into extrusion T-slots
- Short right-angle USB cables (eliminate loops)
- Zip-tie anchor points at extrusion intersections

---

## 11. Design Iteration Workflow

1. **Measure V1** → fill in `tools/chassis_designer.py` inputs
2. **Run calculator** → get recommended dimensions and stability analysis
3. **CAD model** → even a simple OpenSCAD or FreeCAD model to verify clearances
4. **Prototype in plywood** → cut base plate on Genmitsu 4030, test fit components
5. **Aluminum frame** → cut extrusions to length, assemble with T-nuts
6. **3D print brackets** → motor mounts, sensor mounts, Pi tray
7. **CNC plates** → top deck and any adapter plates
8. **Assemble and measure** → weigh, find CoG, verify stability
9. **Update URDF** → real measurements into `roscar.urdf.xacro`
10. **Recalibrate** → encoder tests, EKF tuning, Nav2 footprint update
