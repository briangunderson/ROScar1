# ROScar1 v2 Chassis — Fusion 360 Mockup

Parametric Fusion 360 script that builds a complete CAD model of the v2
aluminum extrusion chassis. Combines imported real STEP models for
structural components (3030 extrusions, corner brackets, T-plate, Pi5,
lidar) with procedural geometry for secondary components (motors, wheels,
deck plates, battery, camera).

## How to Run

1. Open Fusion 360
2. Go to **UTILITIES** → **Scripts and Add-Ins** (or press Shift+S)
3. First run: click **+** → "Script or add-in from device" → navigate to
   `D:\localrepos\ROScar1\docs\chassis\fusion360\` and **Select Folder**
4. Hit the **▶ Run** button next to `roscar_v2_chassis`
5. Script creates a new untitled document each time it runs (~20–30s for
   the ~24 STEP imports)

**Re-runs pick up file changes automatically** — no need to unlink and
re-add after editing the script. A message box at the end shows the
current version and key dimensions.

## What It Creates

### Structural (from real STEP files in `docs/chassis/models/`)
- **8× frame rails** — imported 3030 T-slot extrusion, scaled to length
  (4× 248mm front/rear + 4× 188mm left/right)
- **4× vertical posts** — 100mm 3030 extrusions at frame corners
- **1× lidar mast** — 120mm 3030 extrusion at center-rear of upper deck
- **8× 3-way corner brackets** — brass-colored hardware at every rail/post
  junction (decorative stub extrusions hidden automatically)
- **1× T-plate** — for mast attachment to rear upper rail

### Non-structural (procedural or component-STEP placeholders)
- **Lower & upper deck plates** — tan acrylic, 188×188×3mm, bolt heads at corners
- **4× motor assemblies** — cylindrical can + gearbox + shaft, hangs below frame
- **4× mecanum wheels** — solid tire + central hub + 8 slanted rollers at 45°
- **4× motor L-brackets** — orange, clamps motor to outer rail face, with bolts
- **Battery** — blue box, 75×55×66mm
- **Motor board** — green PCB block, 56×85mm
- **RPi5** — real Raspberry Pi 5 STEP (from raspberrypi.com), on upper deck
- **RPLIDAR C1** — real Slamtec STEP, on mast top mount plate
- **Camera** — black box placeholder
- **Ground plane** — sized to wheel contact patches

## Modifying Dimensions

All key dimensions are Python constants near the top of the script
(all in **cm**, multiply mm by 0.1):

| Constant | Default | Meaning |
|----------|--------:|---------|
| `FRAME` | 24.8 | Outer frame side length (248mm) |
| `S` | 3.0 | 3030 profile width (30mm) |
| `POST_H` | 10.0 | Deck-to-deck post height (100mm) |
| `MAST_H` | 12.0 | Lidar mast height (120mm) |
| `M_DIA` / `M_CAN` / `M_GBOX` | motor body dimensions |
| `WHL_R` / `WHL_W` | 3.97 / 3.73 | Wheel radius (39.7mm) and width (37.3mm) |
| `HWB` | 10.0 | Half-wheelbase (200mm total) |

## Architecture Notes

**STEP imports go into root component, not sub-components.**
Fusion 360's `occurrence.transform` setter silently fails when the
occurrence's parent isn't the active edit target (only root is, at
script start). Frame parts, corner brackets, T-plate, RPi5, and
RPLIDAR are all imported into root. Procedural placeholder shapes
(battery, camera, motor board) go into the `4 - Electronics`
sub-component for browser tree organization.

**Bracket stub extrusions are auto-hidden.** The 3030 3-way corner
bracket STEP assembly from GrabCAD includes decorative 100mm pieces
of profile showing how the bracket connects to rails. These overlap
with our real structural posts/rails. `_hide_bracket_stubs()` uses
body bounding-box size (> 8cm in any dim → hide) plus component-name
matching to toggle them off after import.

**Transform convention**: see `_mat(c0, c1, c2, t)` helper. Each
column vector describes where the body's local +X/+Y/+Z axis points
in world space; `t` is the translation. Explicit column vectors
avoid ambiguity about rotation composition order (which bit us in
rev10 with `transformBy` chaining).

## After Running

- **Fit view**: click the Home icon on the navigation cube (top-right) or
  pick any named face
- **Inspect dimensions**: Inspect → Measure tool; select two faces/edges
- **Hide components individually**: click the eye icon in the browser
  tree to toggle any occurrence
- **Export STL for 3D printing**: right-click a component → Save As Mesh
- **Generate 2D drawings**: switch to the Drawing workspace and pick
  views from the assembled model

## Revision History

See the `REVISION LOG` header comment at the top of `roscar_v2_chassis.py`
for the full change history. Notable milestones:

- **rev8b**: original procedural-only model (pre-STEP imports)
- **rev9**: first STEP import pass (frame extrusions + brackets + T-plate)
- **rev11**: individual-import pattern (22 STEP imports, reliable)
- **rev12**: mecanum wheels with real slanted rollers; visible fasteners
- **rev13**: structural focus — non-structural components simplified to
  placeholder shapes; cut plan reconciled with CAD geometry
- **rev14–15**: real RPi5 and RPLIDAR C1 STEPs integrated; fixed
  sub-component transform bug
- **rev16–20**: palette + visibility polish (silver rails, brass brackets)
- **rev21**: ground plane shrunk to chassis footprint
- **rev22**: fixed the critical `importToTarget2` bug where the function
  returned the first-ever imported occurrence on each call instead of
  the newly created one — caused all 21 rails/posts/brackets after the
  first to stay at identity (stacked at origin). Now uses a
  `count_before`/`count_after` sandwich to grab the freshly appended
  occurrence.
- **rev23**: engineering polish + observability — motor L-brackets now
  have a reinforcement gusset in the inside corner; a placement
  verification summary is printed in the completion dialog
  (`22 STEP imports, 22 unique positions OK`) so any regression of the
  rev22 bug is caught instantly; dead-code cleanup in `_import_step`.

## Related Documentation

- **Cut sheet (saw-ready)**: `tasks/chassis-v2-cut-sheet.md`
- **Full design spec**: `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md`
- **Session handoff**: `tasks/chassis-v2-handoff.md`
- **STEP model sources**: `docs/chassis/models/*/README.md`
