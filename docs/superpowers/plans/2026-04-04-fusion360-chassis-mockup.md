# Fusion 360 Chassis Mockup Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a parametric Fusion 360 Python API script that generates a 3D mockup of the ROScar1 v2 extrusion chassis, allowing the user to visualize and iterate on the design before cutting any material.

**Architecture:** A single Python script run inside Fusion 360's Scripts & Add-Ins panel. All dimensions are defined as Python constants at the top of the script, making it easy to tweak values and re-run. The script creates named components for each subassembly (lower deck, upper deck, motors, etc.) so they can be toggled, colored, and measured independently in Fusion's browser tree.

**Tech Stack:** Fusion 360 Python API (`adsk.core`, `adsk.fusion`), Python 3.x (Fusion's embedded interpreter)

**Spec:** `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md`

---

## File Structure

| File | Purpose |
|------|---------|
| `docs/chassis/fusion360/roscar_v2_chassis.py` | Main Fusion 360 API script — generates the complete chassis assembly |
| `docs/chassis/fusion360/README.md` | How to load and run the script in Fusion 360 |

This is a single-file deliverable. No tests (Fusion 360 scripts are verified visually by running them). No dependencies beyond Fusion 360.

---

## Chunk 1: Script Skeleton + Parameters + 3030 Profile Helper

### Task 1: Create the script file with parameters and entry point

**Files:**
- Create: `docs/chassis/fusion360/roscar_v2_chassis.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p docs/chassis/fusion360
```

- [ ] **Step 2: Write the script skeleton with all parameters**

Create `docs/chassis/fusion360/roscar_v2_chassis.py`:

```python
import adsk.core
import adsk.fusion
import traceback
import math

# =============================================================================
# ROScar1 v2 Chassis — Parametric Fusion 360 Mockup
#
# Run via: Fusion 360 → UTILITIES → Scripts & Add-Ins → + → Select this file
#
# All dimensions in cm (Fusion 360 internal unit).
# Multiply mm values by 0.1 to convert.
#
# Coordinate convention (matches ROS):
#   X = front-to-back (X=0 is front of robot, X=FRAME_SIDE is rear)
#   Y = left-to-right (Y=0 is left side, Y=FRAME_SIDE is right side)
#   Z = up (Z=0 is ground plane)
# =============================================================================

# --- Parametric Dimensions (all in cm) ---

# Frame
FRAME_SIDE = 24.8        # 248mm — rail length (250mm nominal minus kerf)
PROFILE_SIZE = 3.0        # 30mm — 3030 extrusion cross-section
POST_HEIGHT = 10.0        # 100mm — vertical post between decks
MAST_HEIGHT = 12.0        # 120mm — lidar mast above upper deck

# Ground clearance (bottom of lower rail to ground)
GROUND_CLEARANCE = 2.0    # 20mm — TBD, depends on motor bracket

# Motor
MOTOR_DIA = 2.4           # 24mm motor body diameter
MOTOR_BODY_LEN = 6.2      # 62mm motor body (excl shaft)
MOTOR_SHAFT_LEN = 1.3     # 13mm shaft protrusion
MOTOR_MOUNT_SPACING = 1.7 # 17mm between mounting screws
MOTOR_SHAFT_DIA = 0.4     # 4mm shaft diameter

# Wheel
WHEEL_RADIUS = 3.97       # 39.7mm mecanum wheel radius
WHEEL_WIDTH = 3.73        # 37.3mm wheel width

# Bracket offset: distance from frame rail outer face to wheel center
BRACKET_DEPTH = 0.3       # 3mm bracket wall thickness
MOTOR_OFFSET = 3.5        # 35mm total: bracket + shaft + half wheel

# Components (bounding boxes for placeholders)
BATTERY_W = 7.5           # 75mm
BATTERY_D = 5.5           # 55mm
BATTERY_H = 6.6           # 66mm

BOARD_W = 8.5             # 85mm
BOARD_D = 5.6             # 56mm
BOARD_H = 2.0             # 20mm estimate

RPI_W = 9.7               # 97mm (Argon NEO 5 case)
RPI_D = 7.3               # 73mm
RPI_H = 4.0               # 40mm

LIDAR_SIZE = 5.56         # 55.6mm square
LIDAR_H = 4.13            # 41.3mm tall
LIDAR_MOUNT_SPACING = 4.3 # 43mm square hole pattern

# Deck plates
DECK_THICKNESS = 0.3      # 3mm acrylic

# Derived dimensions
FRAME_INNER = FRAME_SIDE - 2 * PROFILE_SIZE  # inner clearance
DECK_PLATE_SIZE = FRAME_INNER               # plate sits inside frame
LOWER_DECK_Z = GROUND_CLEARANCE             # bottom of lower rail
UPPER_DECK_Z = LOWER_DECK_Z + PROFILE_SIZE + POST_HEIGHT  # bottom of upper rail
MAST_BASE_Z = UPPER_DECK_Z + PROFILE_SIZE   # top of upper rail
MAST_TOP_Z = MAST_BASE_Z + MAST_HEIGHT      # top of mast
TOTAL_HEIGHT = MAST_TOP_Z + LIDAR_H

# Colors (R, G, B, opacity 0-255)
COLOR_FRAME = (80, 80, 80, 255)          # dark gray — 3030 extrusion
COLOR_POST = (80, 80, 80, 255)           # same as frame
COLOR_DECK = (180, 220, 255, 128)        # translucent blue — acrylic
COLOR_BATTERY = (40, 100, 200, 255)      # blue — battery pack
COLOR_BOARD = (30, 30, 30, 255)          # black — PCB
COLOR_RPI = (50, 50, 50, 255)           # dark gray — Argon case
COLOR_MOTOR = (160, 160, 160, 255)       # silver — motor body
COLOR_WHEEL = (60, 60, 60, 255)          # dark — mecanum wheel
COLOR_LIDAR = (40, 40, 40, 255)          # black — lidar
COLOR_CAMERA = (100, 100, 100, 255)      # gray — webcam
COLOR_BRACKET = (200, 100, 50, 255)      # orange — 3D printed PETG


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Create new document
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        rootComp = design.rootComponent

        # Build the chassis
        build_lower_deck(rootComp)
        build_vertical_posts(rootComp)
        build_upper_deck(rootComp)
        build_lidar_mast(rootComp)
        build_deck_plates(rootComp)
        build_battery(rootComp)
        build_motor_board(rootComp)
        build_rpi5(rootComp)
        build_lidar(rootComp)
        build_camera(rootComp)
        build_motor_assemblies(rootComp)

        # Fit the view
        app.activeViewport.fit()

        ui.messageBox('ROScar1 v2 chassis mockup created successfully!\n\n'
                       f'Frame: {FRAME_SIDE*10:.0f}mm square\n'
                       f'Track width: ~{(FRAME_SIDE + 2*MOTOR_OFFSET)*10:.0f}mm\n'
                       f'Total height: ~{TOTAL_HEIGHT*10:.0f}mm\n'
                       f'CoG target: ~29% of height')

    except:
        if ui:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')


# =============================================================================
# Helper functions
# =============================================================================

def create_component(parent, name):
    """Create a new component under parent and return (component, occurrence)."""
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp, occ


def set_color(body, r, g, b, a=255):
    """Apply an RGBA color to a body."""
    color_prop = adsk.core.ColorProperty.cast(None)
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    appearance = design.appearances.itemByName(f'Custom_{r}_{g}_{b}_{a}')
    if not appearance:
        lib = app.materialLibraries.itemByName('Fusion 360 Appearance Library')
        if lib:
            base = lib.appearances.itemByName('Paint - Enamel Glossy (Yellow)')
            if base:
                appearance = design.appearances.addByCopy(base, f'Custom_{r}_{g}_{b}_{a}')
                color_prop = adsk.core.ColorProperty.cast(
                    appearance.appearanceProperties.itemByName('Color')
                )
                if color_prop:
                    color_prop.value = adsk.core.Color.create(r, g, b, a)

    if appearance:
        body.appearance = appearance


def create_box(comp, name, x, y, z, sx, sy, sz, color=None):
    """Create a box body at (x,y,z) with size (sx,sy,sz). Origin is at the box center-bottom."""
    sketches = comp.sketches
    planes = comp.constructionPlanes

    # Create offset plane at z height
    plane_input = planes.createInput()
    offset = adsk.core.ValueInput.createByReal(z)
    plane_input.setByOffset(comp.xYConstructionPlane, offset)
    plane = planes.add(plane_input)

    sketch = sketches.add(plane)
    rect = sketch.sketchCurves.sketchLines
    p1 = adsk.core.Point3D.create(x - sx/2, y - sy/2, 0)
    p2 = adsk.core.Point3D.create(x + sx/2, y + sy/2, 0)
    rect.addTwoPointRectangle(p1, p2)

    prof = sketch.profiles.item(0)
    ext_input = comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    dist = adsk.core.ValueInput.createByReal(sz)
    ext_input.setDistanceExtent(False, dist)
    ext = comp.features.extrudeFeatures.add(ext_input)
    body = ext.bodies.item(0)
    body.name = name

    if color:
        set_color(body, *color)

    return body


def create_rail(comp, name, start_x, start_y, start_z, length, direction='x', color=None):
    """Create a 3030 extrusion rail (simplified as 30x30mm box).
    direction: 'x', 'y', or 'z' — the axis along which the rail runs."""
    s = PROFILE_SIZE
    if direction == 'x':
        return create_box(comp, name, start_x + length/2, start_y + s/2, start_z, length, s, s, color)
    elif direction == 'y':
        return create_box(comp, name, start_x + s/2, start_y + length/2, start_z, s, length, s, color)
    elif direction == 'z':
        return create_box(comp, name, start_x + s/2, start_y + s/2, start_z, s, s, length, color)


# =============================================================================
# Assembly builders — each creates a named component
# =============================================================================

def build_lower_deck(root):
    comp, _ = create_component(root, 'Lower Deck Frame')
    s = PROFILE_SIZE
    z = LOWER_DECK_Z

    # Front rail (along Y)
    create_rail(comp, 'lower_front', 0, 0, z, FRAME_SIDE, 'y', COLOR_FRAME)
    # Rear rail (along Y)
    create_rail(comp, 'lower_rear', FRAME_SIDE - s, 0, z, FRAME_SIDE, 'y', COLOR_FRAME)
    # Left rail (along X, between front and rear)
    create_rail(comp, 'lower_left', s, 0, z, FRAME_SIDE - 2*s, 'x', COLOR_FRAME)
    # Right rail (along X, between front and rear)
    create_rail(comp, 'lower_right', s, FRAME_SIDE - s, z, FRAME_SIDE - 2*s, 'x', COLOR_FRAME)


def build_upper_deck(root):
    comp, _ = create_component(root, 'Upper Deck Frame')
    s = PROFILE_SIZE
    z = UPPER_DECK_Z

    create_rail(comp, 'upper_front', 0, 0, z, FRAME_SIDE, 'y', COLOR_FRAME)
    create_rail(comp, 'upper_rear', FRAME_SIDE - s, 0, z, FRAME_SIDE, 'y', COLOR_FRAME)
    create_rail(comp, 'upper_left', s, 0, z, FRAME_SIDE - 2*s, 'x', COLOR_FRAME)
    create_rail(comp, 'upper_right', s, FRAME_SIDE - s, z, FRAME_SIDE - 2*s, 'x', COLOR_FRAME)


def build_vertical_posts(root):
    comp, _ = create_component(root, 'Vertical Posts')
    s = PROFILE_SIZE
    z = LOWER_DECK_Z + s  # posts sit on top of lower rails

    # Four corner posts
    create_rail(comp, 'post_FL', 0, 0, z, POST_HEIGHT, 'z', COLOR_POST)
    create_rail(comp, 'post_FR', 0, FRAME_SIDE - s, z, POST_HEIGHT, 'z', COLOR_POST)
    create_rail(comp, 'post_RL', FRAME_SIDE - s, 0, z, POST_HEIGHT, 'z', COLOR_POST)
    create_rail(comp, 'post_RR', FRAME_SIDE - s, FRAME_SIDE - s, z, POST_HEIGHT, 'z', COLOR_POST)


def build_lidar_mast(root):
    comp, _ = create_component(root, 'Lidar Mast')
    s = PROFILE_SIZE
    z = MAST_BASE_Z

    # Centered on Y, at rear of frame (X = frame_side - s - s/2... let's place at center-rear)
    mast_x = FRAME_SIDE - s - s/2  # rear, inset from rear rail
    mast_y = FRAME_SIDE / 2 - s / 2  # centered

    create_rail(comp, 'mast', mast_x, mast_y, z, MAST_HEIGHT, 'z', COLOR_POST)


def build_deck_plates(root):
    comp, _ = create_component(root, 'Deck Plates')
    s = PROFILE_SIZE
    inner = DECK_PLATE_SIZE
    offset = s  # plates sit inside the frame perimeter

    # Lower deck plate — sits on top of lower rails
    lower_z = LOWER_DECK_Z + s  # top of lower rail
    create_box(comp, 'lower_plate', FRAME_SIDE/2, FRAME_SIDE/2, lower_z,
               inner, inner, DECK_THICKNESS, COLOR_DECK)

    # Upper deck plate — sits on top of upper rails
    upper_z = UPPER_DECK_Z + s
    create_box(comp, 'upper_plate', FRAME_SIDE/2, FRAME_SIDE/2, upper_z,
               inner, inner, DECK_THICKNESS, COLOR_DECK)


def build_battery(root):
    comp, _ = create_component(root, 'Battery')
    z = LOWER_DECK_Z + PROFILE_SIZE + DECK_THICKNESS  # sits on lower plate
    cx = FRAME_SIDE / 2  # centered
    cy = FRAME_SIDE / 2

    create_box(comp, 'battery', cx, cy, z, BATTERY_W, BATTERY_D, BATTERY_H, COLOR_BATTERY)


def build_motor_board(root):
    comp, _ = create_component(root, 'Motor Board')
    z = LOWER_DECK_Z + PROFILE_SIZE + DECK_THICKNESS
    cx = FRAME_SIDE * 0.7  # behind battery (toward rear)
    cy = FRAME_SIDE / 2

    create_box(comp, 'motor_board', cx, cy, z, BOARD_W, BOARD_D, BOARD_H, COLOR_BOARD)


def build_rpi5(root):
    comp, _ = create_component(root, 'RPi5')
    z = UPPER_DECK_Z + PROFILE_SIZE + DECK_THICKNESS  # sits on upper plate
    cx = FRAME_SIDE / 2
    cy = FRAME_SIDE / 2

    create_box(comp, 'rpi5_argon', cx, cy, z, RPI_W, RPI_D, RPI_H, COLOR_RPI)


def build_lidar(root):
    comp, _ = create_component(root, 'RPLIDAR C1')
    z = MAST_TOP_Z
    mast_x = FRAME_SIDE - PROFILE_SIZE - PROFILE_SIZE / 2
    mast_y = FRAME_SIDE / 2

    create_box(comp, 'rplidar_c1', mast_x, mast_y, z,
               LIDAR_SIZE, LIDAR_SIZE, LIDAR_H, COLOR_LIDAR)


def build_camera(root):
    comp, _ = create_component(root, 'Camera')
    z = UPPER_DECK_Z + PROFILE_SIZE / 2  # mounted to front upper rail face
    cx = PROFILE_SIZE / 2  # front face of frame
    cy = FRAME_SIDE / 2    # centered

    create_box(comp, 'webcam', cx, cy, z, 3.0, 8.0, 3.0, COLOR_CAMERA)


def build_motor_assemblies(root):
    comp, _ = create_component(root, 'Motors & Wheels')
    s = PROFILE_SIZE

    # Motor X positions for ~200mm wheelbase (spec target)
    # Motors at +/-100mm from frame center along X-axis
    half_wheelbase = 10.0  # 100mm
    motor_x_front = FRAME_SIDE / 2 - half_wheelbase
    motor_x_rear = FRAME_SIDE / 2 + half_wheelbase

    # Axle height = wheel radius (wheels touch ground at Z=0)
    axle_z = WHEEL_RADIUS

    # Motor + wheel assemblies at 4 corners
    # my = Y position of wheel center (outboard of frame)
    # frame_face_y = Y position of the frame rail outer face on that side
    positions = [
        # (name, motor_x, wheel_center_y, frame_face_y, outward_direction)
        ('FL', motor_x_front, -MOTOR_OFFSET,              0,          -1),  # left side
        ('FR', motor_x_front, FRAME_SIDE + MOTOR_OFFSET,  FRAME_SIDE, +1),  # right side
        ('RL', motor_x_rear,  -MOTOR_OFFSET,              0,          -1),
        ('RR', motor_x_rear,  FRAME_SIDE + MOTOR_OFFSET,  FRAME_SIDE, +1),
    ]

    for name, mx, wheel_cy, frame_face, outdir in positions:
        # --- Wheel (cylinder approximated as box) ---
        wheel_z = 0  # bottom of wheel at ground
        create_box(comp, f'wheel_{name}', mx, wheel_cy, wheel_z,
                   WHEEL_RADIUS * 2, WHEEL_WIDTH, WHEEL_RADIUS * 2, COLOR_WHEEL)

        # --- Motor body (centered on axle, inboard of wheel) ---
        # Motor shaft points outward; motor body is inboard of the wheel
        motor_cy = wheel_cy - outdir * (WHEEL_WIDTH / 2 + MOTOR_SHAFT_LEN + MOTOR_BODY_LEN / 2)
        motor_z = axle_z - MOTOR_DIA / 2
        create_box(comp, f'motor_{name}', mx, motor_cy, motor_z,
                   MOTOR_DIA, MOTOR_BODY_LEN, MOTOR_DIA, COLOR_MOTOR)

        # --- Bracket (connects frame rail face to motor) ---
        # Spans from frame face to motor body, at lower deck height
        motor_inner_y = motor_cy - outdir * (MOTOR_BODY_LEN / 2)  # inboard face of motor
        bracket_cy = (frame_face + motor_inner_y) / 2  # center of bracket
        bracket_len = abs(motor_inner_y - frame_face)  # length of bracket
        create_box(comp, f'bracket_{name}', mx, bracket_cy,
                   LOWER_DECK_Z, s, bracket_len,
                   s, COLOR_BRACKET)
```

- [ ] **Step 3: Verify script syntax**

Run: `python -c "import ast; ast.parse(open('docs/chassis/fusion360/roscar_v2_chassis.py').read()); print('Syntax OK')"`
Expected: `Syntax OK`

- [ ] **Step 4: Create README with instructions**

Create `docs/chassis/fusion360/README.md`:

```markdown
# ROScar1 v2 Chassis — Fusion 360 Mockup

## How to Run

1. Open Fusion 360
2. Go to **UTILITIES** → **Scripts and Add-Ins** (or press Shift+S)
3. Click the **+** button next to "My Scripts"
4. Navigate to this directory and select `roscar_v2_chassis.py`
5. Click **Run**

The script creates a new document with the complete chassis assembly.

## What It Creates

- **Lower Deck Frame** — 4× 3030 rails forming a 248mm square
- **Upper Deck Frame** — matching 4× 3030 rails
- **Vertical Posts** — 4× 100mm corner posts connecting the decks
- **Lidar Mast** — 120mm post at center-rear of upper deck
- **Deck Plates** — translucent blue acrylic plates on each deck
- **Battery** — blue box on lower deck (75×55×66mm, 574g)
- **Motor Board** — black box behind battery (56×85mm)
- **RPi5** — dark gray box on upper deck (Argon NEO 5 case)
- **RPLIDAR C1** — black box on top of mast
- **Camera** — gray box on front upper rail
- **4× Motor + Wheel assemblies** — at frame corners with orange brackets

## Modifying Dimensions

All dimensions are defined as Python constants at the top of the script. Edit the value, save, and re-run the script (it creates a new document each time).
Key constants:
- `FRAME_SIDE` — rail length (controls overall frame size)
- `POST_HEIGHT` — vertical spacing between decks
- `MAST_HEIGHT` — lidar mast height
- `MOTOR_OFFSET` — how far motors sit from the frame

## After Running

- Use Fusion 360's **Inspect → Measure** tool to verify dimensions
- Toggle component visibility in the browser tree to see individual layers
- Use **Section Analysis** to check internal clearances
- Export STL meshes from individual components if needed for 3D printing
```

- [ ] **Step 5: Commit**

```bash
git add docs/chassis/fusion360/roscar_v2_chassis.py docs/chassis/fusion360/README.md
git commit -m "feat: add Fusion 360 parametric chassis mockup script

Generates a complete 3D mockup of the ROScar1 v2 two-deck extrusion
chassis from the design spec. All dimensions parametric."
```

---

## Chunk 2: Run, Iterate, Refine

### Task 2: Run script in Fusion 360 and fix issues

This task is interactive — the user runs the script, reports what looks wrong, and we iterate.

**Files:**
- Modify: `docs/chassis/fusion360/roscar_v2_chassis.py`

- [ ] **Step 1: User runs script in Fusion 360**

Follow README instructions. Expected: a new document opens with all components visible.

- [ ] **Step 2: Verify component positions**

Use Fusion 360's Inspect → Measure tool to check:
- Lower deck rail heights match `GROUND_CLEARANCE` (20mm)
- Upper deck is `POST_HEIGHT` (100mm) above lower deck
- Lidar mast extends `MAST_HEIGHT` (120mm) above upper deck
- Frame is approximately 248mm square
- Wheels touch the ground plane (Z=0)
- Motors are offset from frame by `MOTOR_OFFSET` (35mm)

- [ ] **Step 3: Fix any positioning or dimension issues**

Common problems:
- Bodies clipping through each other → adjust offsets
- Components floating or buried → check Z coordinates
- Colors not applying → Fusion 360 appearance library may differ
- Profile errors → check that sketches close properly

- [ ] **Step 4: Screenshot and review with user**

Take a screenshot from Fusion 360 showing the complete assembly from an isometric view.

- [ ] **Step 5: Commit fixes**

```bash
git add docs/chassis/fusion360/roscar_v2_chassis.py
git commit -m "fix: adjust chassis mockup based on Fusion 360 visual review"
```

### Task 3: Enhance with proper 3030 T-slot profile (optional)

If the user wants more visual fidelity beyond simple boxes, replace the `create_rail` function with one that draws the actual 3030 T-slot cross-section (square with 4 T-slots and center bore).

**Files:**
- Modify: `docs/chassis/fusion360/roscar_v2_chassis.py`

- [ ] **Step 1: Add 3030 T-slot profile sketch function**

Replace the simple box with a sketch that draws:
- 30×30mm outer square
- 8mm T-slot opening on each face (4 slots)
- 6.5mm center bore
- Corner radii

- [ ] **Step 2: Re-run and verify**

- [ ] **Step 3: Commit**

```bash
git add docs/chassis/fusion360/roscar_v2_chassis.py
git commit -m "feat: add realistic 3030 T-slot cross-section profile"
```

---

## Execution Notes

**This is NOT a typical software project.** The "test" for each task is running the script in Fusion 360 and visually inspecting the result. There is no pytest, no CI, no automated verification.

**The user needs to:**
1. Have Fusion 360 installed (free hobby license available)
2. Copy/paste the script into Fusion's script editor OR point it at the file
3. Run it and report back what looks right/wrong
4. Iterate with dimension tweaks

**Expected iteration cycle:** 2-3 rounds of run → fix → re-run to get positions and proportions right. The parametric constants make each iteration fast (change a number, re-run).
