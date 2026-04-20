# Chassis v2 rev42 — real 25GA20E260 motor dimensions + printable bracket

## TL;DR

Rev42 replaces the placeholder motor dimensions with the real
**25GA20E260 datasheet values** (Brian's motor: 7.4V, 1:20 gearbox,
67.4g). Motor-offset math and track width both change. Cut sheet
(`tasks/chassis-v2-cut-sheet.md`) is UNCHANGED — it lists rail lengths,
not track width.

Also ships `docs/chassis/fusion360/bridge/motor_bracket_print.py`
rewritten as a **Design 3 dart-gusset** printable bracket
(monolithic PETG, STL exports to `D:\tmp\motor_bracket.stl`).

## Motor dimensions

| Constant | rev41 (cm) | rev42 (cm) | mm | Source |
|---|---:|---:|---:|---|
| `M_DIA` | 2.4 | **2.5** | Ø25 | datasheet |
| `M_CAN` | 3.8 | **1.95** | 19.5 | datasheet (big change — motor can was mis-sized 2×) |
| `M_GBOX_DIA` | 2.2 | **2.1** | Ø21 | datasheet |
| `M_GBOX` | 2.4 | **1.95** | 19.5 | datasheet |
| `SHAFT_HUB_DIA` | — | **0.7** | Ø7 | datasheet (new constant) |
| `SHAFT_HUB` | — | **1.15** | 11.5 | datasheet (new constant) |
| `SHAFT` | 1.3 | **1.1** | 11 | datasheet |
| `SHAFT_D` | 0.4 | **0.3** | Ø3 | datasheet |

`M_OFF` (rail face → wheel center) was 9.37cm, now
`M_CAN + M_GBOX + SHAFT_HUB + SHAFT/2 = 1.95 + 1.95 + 1.15 + 0.55 = 5.60cm`.

Resulting chassis footprint:
- **Track width**: 435mm → **360mm** (248 + 2×56)
- Wheelbase: 200mm unchanged
- Outer wheel span (w/ 37.3mm wheel width): 472mm → **397mm**

Rails still cut at the same lengths; the cut sheet is correct.

## `_motor_assy()` changes

Added `MHub_<tag>` cylinder (Ø7×11.5mm) between the gearbox and the
thin shaft. Visually represents the section the mecanum wheel's hex
adapter grips. Shaft position math updated to account for the hub.

## Printable motor bracket (Design 3 — dart gusset)

Lives at `docs/chassis/fusion360/bridge/motor_bracket_print.py`. Run
via the bridge:

```bash
python docs/chassis/fusion360/bridge/fctl.py close
python docs/chassis/fusion360/bridge/fctl.py run \
       docs/chassis/fusion360/bridge/motor_bracket_print.py --timeout 120
```

Produces `D:\tmp\motor_bracket.stl`, a single-body PETG part ready to
slice.

### Geometry (printable orientation)

- **MtrFlange** — 40×40×4mm plate, lies FLAT on the build plate at
  z=[0, 4mm]. Bolts to the gearbox shaft-end face via 2× M3.
- **RailFlange** — 20×4×30mm vertical plate, elevated to z=[61.2,
  91.2mm], perpendicular to Y. Bolts to the rail's outer T-slot via
  2× M5.
- **Dart gusset** — triangular rib bridging the two flanges,
  approximated as 20 XY-plane layers forming a staircase (slicer
  smooths it into a clean 45°-ish ramp). 4mm thick rib, centered
  on X=0.

### Print settings

- Orient with the big 40×40mm face DOWN on the build plate (the
  exported STL is already in this orientation — Z+ is up).
- PETG @ 240°C, bed 75°C.
- 4 perimeters, 40% gyroid infill, 0.2mm layers.
- Tree supports under the rail flange's underside.
- After printing: drill 2× Ø3.4mm M3 clearance holes through the
  motor flange (17mm apart, vertical axis through the flange
  center); drill 2× Ø5.5mm M5 clearance holes through the rail
  flange for the T-slot. The STL skips the bolt cuts to keep the
  script on standard XY-plane sketches (rev40 established that
  non-XY plane circle sketches have fragile API behavior).

### Mirroring for the opposite side

The motor flange is asymmetric relative to the dart. Print 2 bracket
copies as-is for one side of the robot (FL + RL); print 2 more with
the slicer's MIRROR option for the other side (FR + RR).

## Bracket script — why the rev41 attempt produced no STL

Root cause: the old `motor_bracket_print.py` used `_cut_cyl_y()` to
drill M3/M5 bolt holes through the flanges. That helper sketched
circles on an XZ construction plane and extruded `CutFeatureOperation`.
Fusion's sketch coordinate convention on non-XY planes is fragile —
the extrude silently failed in `DirectDesignType`, and the script's
outer `try/except` caught the error with only a `messageBox` dialog
(no stdout print). I dismissed that dialog with Return during the
last run, the script exited, and the bridge response showed empty
stdout and no STL on disk.

Rev42 script fixes:
1. Use only XY-plane sketches (`_xy_plane()` helper mirrors the
   main chassis script's `_zp()`).
2. Approximate the dart gusset as a staircase of XY-plane layers
   instead of extruding a triangle through a non-XY plane.
3. Skip the bolt clearance cuts entirely; drill post-print.
4. `print()` a step marker BEFORE every major operation. Any silent
   exception now leaves a trace in the captured stdout of where it
   happened.
5. `print()` the traceback to stdout on failure, not just the
   modal messageBox.

## Testing checklist (run after bridge access is restored)

1. Run the bracket script via bridge. Expected stdout:
   ```
   STEP 1: get Fusion application
   STEP 2: open a fresh document
   STEP 3: MtrFlange 40x40x4mm, flat on XY at z=0
   STEP 4: RailFlange 20x4x30mm at y=7.90cm, z=6.12cm
   STEP 5: dart staircase (20 XY-plane layers)
   STEP 6: combine 22 bodies into one
   STEP 7: export STL
   DONE: wrote D:\tmp\motor_bracket.stl
         export ok=True file_exists=True size=<~50-200kB> bytes
   ```
   If you see STEP N and then FAIL, the failure is AT step N — the
   traceback that follows tells you exactly which API call choked.

2. Open `D:\tmp\motor_bracket.stl` in a slicer. Expected: L-shape
   with the big flange on the bed, rail flange elevated ~60mm above,
   a diagonal staircase dart between them.

3. Re-run the full chassis script (`roscar_v2_chassis.py`). Expected:
   9/9 sanity pass (motor bracket placement check may report tighter
   tolerances now due to the new, shorter bracket arm — if so
   update the threshold in `motors_touch_bracket` from 2.0 to ~1.0
   cm). Dialog should report track width ~360mm.

4. Copy the final STL into the repo at
   `docs/chassis/fusion360/bracket_stls/motor_bracket_25GA20E260.stl`
   so it's in git alongside the source script.
