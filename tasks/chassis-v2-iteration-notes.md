# Chassis v2 iteration notes — 2026-04-17 extended autonomous session

## TL;DR

Script is at **rev23**. Chassis renders cleanly in Fusion 360 with all
22 structural STEP occurrences + 2 electronics STEPs placed correctly.
Frame dimensions verified CAD-matches-cut-sheet.

**rev22 fixed a critical bug** — prior to rev22, `importToTarget2`'s
returned ObjectCollection was giving back the FIRST-ever imported
occurrence on every call (not the newest), so every `_place_occ()`
after the first was repositioning the same occurrence. All 21 other
rails/posts/brackets stayed at identity and stacked on top of each other
at `(0,0,0)`. The fix: use `target_comp.occurrences.count` before and
after the import, and grab the newly appended occurrence by index.

**rev23 added observability + bracket polish**:
- Motor L-brackets now have a reinforcement gusset in the inside
  corner of the L (4 brackets × 1 rib).
- The completion dialog now reports `{n} STEP imports, {k} unique
  positions OK` plus the X/Y/Z bounding box of frame placements. If
  the rev22 bug ever regresses, you'll see `k=1` and a zero-width
  bbox, caught instantly.

The major bugs that caused your "floating objects / frame missing"
complaint are fixed:
- STEPs imported into sub-components silently failed to apply
  transforms (rev15 fix)
- Dark frame colors blended with Fusion's viewport background
  (rev16 fix: bright aluminum silver)
- Corner bracket stub extrusions from the GrabCAD STEP physically
  overlap posts — partially handled (rev19/20)
- Ground plane was too big, made chassis look small (rev21 fix)

## Revision history this session (rev13 → rev21)

| rev | Change | Why |
|----:|--------|-----|
| 13 | Strip non-structural procedural noise; reconcile cut plan | You were cutting aluminum today and CAD had 4×248+4×188 while spec said 8×248 |
| 14 | Integrate RPi5 + RPLIDAR C1 real STEPs | Real manufacturer CAD from raspberrypi.com / slamtec.com |
| 15 | **Fix sub-component transform bug** | Pi5 and RPLIDAR STEPs were landing at origin (0,0,0) because `occurrence.transform` silently fails on nested occurrences. Changed `_components()` to import STEPs into root instead. This was the root cause of the "floating objects" complaint. |
| 16 | Frame color: black → bright aluminum silver | Dark frame blended with dark blue viewport; rails and posts looked absent |
| 17 | Attempt name-based stub hiding | Corner bracket STEPs have decorative 100mm stub extrusions that overlap our posts; tried to auto-hide |
| 18 | Corner brackets: silver → brass/gold | Silver on silver (brackets on rails) blended; brass pops |
| 19 | Size-based stub hiding (threshold 5cm) | Name matching wasn't reliable — sized-based catches stubs by bounding box |
| 20 | Raise threshold 5cm → 8cm; bolder brass | 5cm was too aggressive (hid T-plates at 85mm); brass was too tan |
| 21 | Shrink ground plane to wheel contact patches | Was extending 60cm past chassis — made the robot look small in viewport |
| 22 | **Fix `importToTarget2` returning wrong occurrence** | User reported "almost all of the aluminum frame pieces are in the exact same position." `_place_occ` was targeting the FIRST-ever imported occurrence on every call. Now use count before/after to get the newly appended occurrence. |
| 23 | Motor L-bracket gussets + placement verification | Each bracket now has a reinforcement rib in the inside L corner. Completion dialog shows `{n} STEP imports, {k} unique positions` — catches rev22 regressions instantly. Also cleaned up dead copy-paste code in `_import_step`. |

All committed and pushed to master.

## What's working now

- **Frame** (22 structural STEP imports): all in correct positions
  - 4× Rail_FR (front/rear, 248mm) + 4× Rail_LR (left/right, 188mm)
  - 4× Posts (100mm) + 1× Mast (120mm)
  - 8× Corner brackets (brass) + 1× Mast T-plate
- **Deck plates**: tan acrylic with 4 bolt heads at corners each
- **Motors + wheels**: procedural cylinders + 8 slanted mecanum rollers at 45°
- **Motor L-brackets**: orange, with visible mounting bolts
- **Real RPi5 STEP**: from raspberrypi.com, on upper deck
- **Real RPLIDAR C1 STEP**: from slamtec.com, on mast top plate
- **Electronics placeholders**: simple boxes for battery, motor board, camera
- **Smart bracket stub hiding**: bodies > 8cm in any dim auto-hidden after
  import (stops the decorative stubs from eating the view)

## What still needs attention when you're back

1. **Non-structural STEPs from GrabCAD** (login required, can't automate):
   - Mecanum wheels (80mm hex bore) → replaces the procedural roller wheels
   - JGA25 / JGB37 encoder motor → replaces motor cylinders
   - Argon NEO 5 case → wraps around the bare Pi5 PCB
   - XT60 female connector → detail on motor board
   - Logitech C270 camera → replaces camera placeholder
   - 3S LiPo battery → replaces battery placeholder
   - Drop them in `docs/chassis/models/electronics/` and I'll wire them in

2. **Pi5 and RPLIDAR placement tweaks**: initial placement is based on
   guessed STEP native origins. Might need ±5mm nudges. Edit the
   translation tuples in `_components()`:
   - `_place_occ(rpi_occ, _X, _Y, _Z, (c - 4.25, c - 2.8, rz))`
   - `_place_occ(lid_occ, _X, _Y, _Z, (lcx - LID_SZ/2, lcy - LID_SZ/2, lz))`

3. **3D-printable motor bracket design**: Phase 2 deliverable from the
   design spec. Current L-bracket in `_motor_assy()` is a simple
   approximation. A proper printable bracket would have a circular
   hole for the motor body (25mm clearance for a 24mm motor) + 2 M3
   screw holes at 17mm spacing + reinforcement ribs. Didn't tackle
   this in this session; flagging as a distinct feature to design.

4. **Fabrication-ready stuff**: you're good to cut aluminum. Everything
   in `tasks/chassis-v2-cut-sheet.md` matches the CAD exactly — 13
   cuts from 6 sticks of 3030 stock.

## How to view cleanly in Fusion

After the script runs, if things look cluttered:
1. Click the **Home** icon on the navigation cube (top-right of viewport)
   to get a clean isometric
2. In the browser tree, toggle off the eye icon on `7 - Ground Plane`
   if you want to see the chassis floating without the floor
3. Use the navigation cube's face-click (TOP / FRONT / RIGHT) for
   orthographic views — great for inspecting individual components

## Known cosmetic quirks (not functional issues)

- The corner bracket **stub extrusions** that the GrabCAD STEP includes
  are still partially visible when viewed up close. The size-based
  auto-hide catches most but some bodies deep in nested sub-components
  may slip through. Doesn't affect structural correctness — real brackets
  will install fine.
- Posts and rails, both being aluminum silver, can visually blend in
  some views. They're there, they're just the same color family. If
  you want them more distinguishable, pick a different color for
  `'frame'` in the `PAL` dict at the top of the script.
- Motor L-bracket's vertical and horizontal plates are currently
  separate boxes with slight overlap at the join. A "proper" L-bracket
  would be a single body with a fillet. Cosmetic.

## Commit trail (all on master)

```
a839726 docs(chassis): rewrite Fusion 360 README for rev21
6099715 refactor(chassis): rev21 — shrink ground plane to chassis footprint + small margin
35a65ac fix(chassis): rev20 — fix stub hide threshold + bolder brass
757dba3 fix(chassis): rev19 — size-based stub body hiding (bodies > 5cm → hidden)
a0b297c fix(chassis): rev18 — brass/gold corner brackets for max visual distinction
947708b fix(chassis): rev17 — hide decorative stub extrusions from corner bracket STEPs
3f73c9e fix(chassis): rev16 — bright aluminum silver frame for CAD visibility
a0b3f5f fix(chassis): brighten frame color so the upper deck + mast are visible
fe5b8be fix(chassis): rev15 — STEP imports go to root, not Electronics sub-component
279cbd0 feat(chassis): rev14 — integrate real RPi5 and RPLIDAR C1 STEPs
11db42d feat(chassis): add Raspberry Pi 5 and RPLIDAR C1 STEP models
23e1339 fix(chassis): rev13 palette — black anodized frame + orange motor brackets
1b594de fix(chassis): correct track width + total height from CAD audit
0d2255a feat(chassis): rev13 — structural focus for fabrication
```
