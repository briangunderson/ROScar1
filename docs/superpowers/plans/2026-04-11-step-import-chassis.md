# STEP Import for Chassis Fusion 360 Script — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status:** Implemented as rev9, revised as rev10 (see rev10 addendum at bottom).

**Goal:** Replace procedural geometry in `roscar_v2_chassis.py` with real STEP model imports for 3030 extrusions, 3-way corner brackets, and T-plate brackets.

**Architecture:** Add a STEP import helper that loads models from `docs/chassis/models/`, positions them with Matrix3D transforms, and optionally scales extrusion bodies to the correct length. The existing procedural primitives (B, CZ, CY) remain for electronics, motors, wheels, wiring, and ground plane. The `_frame()` function is rewritten to use imported geometry.

**Tech Stack:** Fusion 360 Python API (adsk.core, adsk.fusion), STEP files from `docs/chassis/models/`

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `docs/chassis/fusion360/roscar_v2_chassis.py` | Modify | Add STEP import helpers, rewrite `_frame()` |

Single-file change. All new code goes into the existing script.

---

## Chunk 1: STEP Import Infrastructure + Frame Rewrite

### Task 1: Add model paths and import helper functions

**Files:**
- Modify: `docs/chassis/fusion360/roscar_v2_chassis.py` (top of file, after constants)

- [ ] **Step 1: Add STEP model path constants**

Add after the `_cc = {}; _clog = []` line (line 89):

```python
# ═══════════════════════════════════════════════════════════════════════════
# STEP model paths (absolute — this script runs locally in Fusion 360)
# ═══════════════════════════════════════════════════════════════════════════
import os
_MODELS = r'D:\localrepos\ROScar1\docs\chassis\models'
STEP_EXTRUSION = os.path.join(_MODELS, 'extrusions', '30X30 - T-slot - Aluminium Profile.step')
STEP_CORNER_BRACKET = os.path.join(_MODELS, 'brackets', '3030 - 3-way corner bracket.step')
STEP_TPLATE_M6 = os.path.join(_MODELS, 'brackets', 'T-Plate 3030 5 Holes - M6.step')
```

- [ ] **Step 2: Add `_import_step()` helper**

Add after the path constants. This function imports a STEP file into a target component and returns the occurrence. Uses `importToTarget2` to get a handle on the imported occurrence.

```python
def _import_step(target_comp, step_path, name=None):
    """Import a STEP file into target_comp. Returns the Occurrence."""
    app = adsk.core.Application.get()
    mgr = app.importManager
    opts = mgr.createSTEPImportOptions(step_path)
    opts.isViewFit = False
    results = mgr.importToTarget2(opts, target_comp)
    occ = adsk.fusion.Occurrence.cast(results.item(0))
    if name:
        occ.component.name = name
    return occ
```

- [ ] **Step 3: Add `_place()` helper for positioning occurrences**

Composes rotation + translation into a Matrix3D and assigns it to the occurrence. Fusion 360 API uses cm internally, matching our existing constants.

```python
def _place(occ, tx=0, ty=0, tz=0, rot_axis=None, rot_angle=0, rot_origin=None):
    """Position an occurrence via rotation then translation."""
    m = adsk.core.Matrix3D.create()
    if rot_axis and rot_angle != 0:
        origin = rot_origin or adsk.core.Point3D.create(0, 0, 0)
        m.setToRotation(rot_angle, rot_axis, origin)
    t = adsk.core.Matrix3D.create()
    t.translation = adsk.core.Vector3D.create(tx, ty, tz)
    m.transformBy(t)
    occ.transform = m
```

- [ ] **Step 4: Add `_scale_y()` helper for stretching extrusions**

The source STEP extrusion is 100mm (10.0 cm) along Y. This scales it to the desired length along Y only.

```python
def _scale_y(occ, target_len_cm):
    """Non-uniform scale an occurrence's bodies along Y to target length.

    The source extrusion STEP is 10.0 cm (100mm) along Y.
    Scale factor = target_len_cm / 10.0.
    Scale is applied from the component origin, so position AFTER scaling.
    """
    factor = target_len_cm / 10.0
    comp = occ.component
    bodies = adsk.core.ObjectCollection.create()
    for i in range(comp.bRepBodies.count):
        bodies.add(comp.bRepBodies.item(i))
    inp = comp.features.scaleFeatures.createInput(
        bodies, comp.originConstructionPoint,
        adsk.core.ValueInput.createByReal(1.0))
    inp.setToNonUniform(
        adsk.core.ValueInput.createByReal(1.0),
        adsk.core.ValueInput.createByReal(factor),
        adsk.core.ValueInput.createByReal(1.0))
    comp.features.scaleFeatures.add(inp)
```

- [ ] **Step 5: Add `_clr_occ()` helper for coloring imported bodies**

Reuses the existing `_clr()` function but applies to all bodies in an occurrence's component.

```python
def _clr_occ(occ, key):
    """Apply color to all bodies in an occurrence's component."""
    comp = occ.component
    for i in range(comp.bRepBodies.count):
        _clr(comp.bRepBodies.item(i), key)
```

### Task 2: Rewrite `_frame()` to use STEP imports

**Files:**
- Modify: `docs/chassis/fusion360/roscar_v2_chassis.py` — replace `_frame()` function (lines 245-267)

The STEP extrusion model has its cross-section in the XZ plane and length along Y. The existing frame uses:
- **Lower/upper deck front/back rails**: run along Y (no rotation needed, just scale+translate)
- **Lower/upper deck left/right rails**: run along X (rotate 90deg around Z, then scale+translate)
- **Vertical posts**: run along Z (rotate 90deg around X, then scale+translate)
- **Lidar mast**: runs along Z (same as posts, different length)

The corner bracket STEP is oriented with arms along +X, +Y, +Z and needs rotation per corner.

- [ ] **Step 1: Write the new `_frame()` function**

Replace the existing `_frame()` (lines 245-267) with:

```python
def _frame(rc):
    """Frame structure using imported STEP models."""
    _frame_rails(rc)
    _frame_posts(rc)
    _frame_mast(rc)
    _frame_corners(rc)
    _frame_tplate(rc)


def _import_rail(rc, name, length_cm, tx, ty, tz, rot_axis=None, rot_angle=0):
    """Import a 3030 extrusion, scale to length, rotate, translate, color."""
    occ = _import_step(rc, STEP_EXTRUSION, name)
    _scale_y(occ, length_cm)
    _place(occ, tx, ty, tz, rot_axis, rot_angle)
    _clr_occ(occ, 'frame')
    return occ


def _frame_rails(rc):
    """8 horizontal frame rails (4 lower deck + 4 upper deck).

    STEP extrusion: cross-section in XZ, length along +Y.
    Front/rear rails run along Y — no rotation needed.
    Left/right rails run along X — rotate -90deg around Z.

    Rail positions (corner at x=0,y=0):
      Front (x=0):     origin at (0, 0, z)          along +Y for FRAME length
      Rear  (x=FRAME-S): origin at (FRAME-S, 0, z)  along +Y
      Left  (y=0):     rotated -90Z, origin maps to (S, 0, z) along +X
      Right (y=FRAME-S): rotated -90Z, origin maps to (S, FRAME-S, z) along +X

    The extrusion cross-section is centered on X=0,Z=0 (±1.5cm from center).
    After import it spans X=[-1.5,+1.5], Y=[0,length], Z=[-1.5,+1.5].
    We need the corner at (0,0) so translate X+1.5 and Z+1.5 (relative to placement z).
    """
    hs = S / 2  # 1.5 cm — half of 3030 profile

    z_axis = adsk.core.Vector3D.create(0, 0, 1)

    for deck, z in [('Lo', LO), ('Hi', HI)]:
        # Front/Rear rails — along Y, length = FRAME
        # Cross-section centered at origin, so offset by +hs in X and Z
        _import_rail(rc, f'{deck}_Front', FRAME, hs, 0, z + hs)
        _import_rail(rc, f'{deck}_Rear', FRAME, FRAME - hs, 0, z + hs)

        # Left/Right rails — along X (rotate -90 around Z)
        # After -90Z rotation: original +Y maps to -X, original +X maps to +Y
        # So rail extends in -X direction from origin. We want it from x=S to x=FRAME-S.
        # Place origin at (FRAME - S, hs, z + hs) so it extends in -X toward x=S.
        # Rail length = FRAME - 2*S (fits between front and rear rails)
        inner_len = FRAME - 2 * S
        _import_rail(rc, f'{deck}_Left', inner_len, FRAME - S, hs, z + hs,
                     z_axis, -math.pi / 2)
        _import_rail(rc, f'{deck}_Right', inner_len, FRAME - S, FRAME - hs, z + hs,
                     z_axis, -math.pi / 2)


def _frame_posts(rc):
    """4 vertical corner posts.

    STEP extrusion length along +Y. Rotate +90 around X to point along +Z.
    After +90X: original +Y maps to +Z. Cross-section stays in XZ but now
    the rail extends upward.
    """
    x_axis = adsk.core.Vector3D.create(1, 0, 0)
    hs = S / 2
    pz = LO + S  # posts start at top of lower deck rail

    for name, px, py in [('FL', hs, hs), ('FR', hs, FRAME - hs),
                          ('RL', FRAME - hs, hs), ('RR', FRAME - hs, FRAME - hs)]:
        _import_rail(rc, f'Post_{name}', POST_H, px, py, pz,
                     x_axis, math.pi / 2)


def _frame_mast(rc):
    """Lidar mast — single vertical post at center-rear of upper deck."""
    x_axis = adsk.core.Vector3D.create(1, 0, 0)
    hs = S / 2
    _import_rail(rc, 'Mast', MAST_H, FRAME - S, FRAME / 2, MST,
                 x_axis, math.pi / 2)


def _frame_corners(rc):
    """8 corner brackets (imported STEP) at frame-post junctions.

    The STEP bracket is an assembly (bracket + 2 stub extrusions + 2 grub screws)
    oriented with arms along +X, +Y, +Z and the corner near the origin.
    The bracket body is ~3cm cube centered at the corner.

    For each corner we need a specific rotation so the bracket arms align with
    the two horizontal rails and one vertical post meeting at that corner.

    Corner rotation strategy (viewed from above, posts go up):
      FL (x=0, y=0):         arms toward +X, +Y, +Z — identity (no rotation)
      FR (x=0, y=FRAME):     arms toward +X, -Y, +Z — rotate 90 around Z (swaps Y→-X)
                              Actually: mirror Y. Use -90 around Z.
      RL (x=FRAME, y=0):     arms toward -X, +Y, +Z — rotate 90 around Z
      RR (x=FRAME, y=FRAME): arms toward -X, -Y, +Z — rotate 180 around Z

    We place the bracket at each corner for both lower and upper deck junctions.
    Lower brackets: at z = LO + S (top of lower rail, bottom of post)
    Upper brackets: at z = HI (bottom of upper rail, top of post)
    """
    z_axis = adsk.core.Vector3D.create(0, 0, 1)

    # (corner_name, x, y, z_rotation_angle)
    corners = [
        ('FL', 0, 0, 0),
        ('FR', 0, FRAME, -math.pi / 2),
        ('RL', FRAME, 0, math.pi / 2),
        ('RR', FRAME, FRAME, math.pi),
    ]

    lo_z = LO + S   # lower junction
    hi_z = HI        # upper junction

    for name, cx, cy, angle in corners:
        for deck, z in [('lo', lo_z), ('hi', hi_z)]:
            occ = _import_step(rc, STEP_CORNER_BRACKET, f'CB_{deck}_{name}')
            if angle != 0:
                _place(occ, cx, cy, z, z_axis, angle)
            else:
                _place(occ, cx, cy, z)
            _clr_occ(occ, 'corner')


def _frame_tplate(rc):
    """T-plate bracket for lidar mast attachment to rear upper rail."""
    occ = _import_step(rc, STEP_TPLATE_M6, 'Mast_TPlate')
    # Position at rear rail center, just below mast base
    _place(occ, FRAME - S, FRAME / 2, MST - 0.1)
    _clr_occ(occ, 'corner')
```

- [ ] **Step 2: Remove the old `TZ()` function**

Delete the `TZ()` function (lines 217-240). It was only used by the old `_frame()` for posts and mast. The new code imports real STEP extrusions instead.

Note: keep `B()`, `CZ()`, `CY()` — they're still used by electronics, motors, wheels, wiring, plates, and ground.

- [ ] **Step 3: Update VERSION constant**

Change line 26:
```python
VERSION = 'rev9'
```

- [ ] **Step 4: Update revision log**

Add to the revision log comment block (after line 24):
```python
#   rev9    2026-04-11  STEP model imports:
#                       - Real 3030 extrusion profiles (imported + scaled)
#                       - Real 3-way corner brackets at all 8 frame joints
#                       - T-plate bracket for lidar mast
#                       - Removed procedural TZ() function
```

### Task 3: Test in Fusion 360

- [ ] **Step 1: Open Fusion 360, delete any old script entry**

UTILITIES -> Scripts and Add-Ins (Shift+S). Delete old `roscar_v2_chassis` entry if present.

- [ ] **Step 2: Add the script from the worktree path**

Click +, navigate to `D:\localrepos\ROScar1\.claude\worktrees\relaxed-chandrasekhar\docs\chassis\fusion360\`, select `roscar_v2_chassis.py`, click Run.

- [ ] **Step 3: Verify**

Expected:
- Message box shows `ROScar1 v2 (rev9)` with frame/track/wheelbase/height stats
- 13 imported extrusion rails (8 horizontal + 4 posts + 1 mast) with real T-slot cross-sections
- 8 imported corner brackets at frame-to-post junctions
- 1 T-plate bracket at mast base
- All imported bodies colored correctly (frame=dark gray, corner=gray)
- Electronics, motors, wheels, plates, wiring, ground plane unchanged from rev8b
- No exploded/scattered bodies — everything positioned correctly

If non-uniform scale fails on the extrusion bodies (the bracket STEP is an assembly with multiple body types), fall back to scaling only the BRep bodies and skip sketches/surfaces.

### Task 4: Commit

- [ ] **Step 1: Commit the updated script**

```bash
cd D:\localrepos\ROScar1\.claude\worktrees\relaxed-chandrasekhar
git add docs/chassis/fusion360/roscar_v2_chassis.py
git commit -m "feat(chassis): import real STEP models into Fusion 360 script (rev9)

Replace procedural T-slot polygon extrusions with imported STEP files:
- 3030 extrusion profiles (non-uniform scaled to rail/post/mast lengths)
- 3-way corner brackets at all 8 frame-to-post junctions
- T-plate bracket for lidar mast attachment
- Remove TZ() procedural function (no longer needed)

Electronics, motors, wheels, plates, wiring remain procedural."
```

---

## Risk Mitigations

1. **Non-uniform scale fails on assembly STEP**: The corner bracket STEP is an assembly (bracket + stub extrusions + grub screws). `_scale_y()` only runs on extrusion imports, not brackets, so this is safe. If the extrusion STEP has multiple bodies, `_scale_y()` collects all BRep bodies and scales them together.

2. **Transform positioning off**: The extrusion cross-section is centered at X=0, Z=0 (spans ±1.5cm). All position calculations account for this half-size offset. If any rail is mispositioned, the fix is adjusting the tx/ty/tz values — no structural changes needed.

3. **Bracket rotation wrong**: The 4 corner rotations (0, -90, +90, 180 around Z) are derived from which direction the bracket arms need to point. If a bracket is rotated wrong, swap the angle. This is a quick iteration.

4. **Performance**: 13 STEP imports + 8 bracket imports = 21 import operations. Each takes ~0.5-1s in Fusion 360. Total import time ~10-20s, acceptable for a setup script.

---

## Rev10 Addendum — Fixes and Optimizations

After rev9 was committed, re-review surfaced three real issues plus one unverified assumption. Rev10 addresses all of them in a single follow-up commit.

### 1. Upper-deck bracket Z-arm was pointing UP (bug)

**Problem:** `_frame_corners()` used the same rotation matrix for lower and upper deck brackets. The STEP bracket has arms along +X, +Y, +Z. At upper corners the Z arm should point **down** (into the post below), not up.

**Fix:** Replaced the angle-based `corners` table with an 8-entry placement table that gives each corner an explicit 3x3 rotation (column vectors). Upper-deck entries swap X/Y assignment in addition to flipping Z, preserving determinant +1 (proper rotation) while getting the arms to the right world directions.

### 2. Transform composition via `transformBy` was ambiguous (potential bug)

**Problem:** `_place()` built the transform as `rotation.transformBy(translation)`. Fusion 360's API docs don't clearly specify whether this pre-multiplies or post-multiplies, which would flip the order of rotate-then-translate vs. translate-then-rotate.

**Fix:** Replaced the chained `transformBy` with an explicit row-major matrix build via `setWithArray`. The helper `_mat(c0, c1, c2, t)` takes the three rotation column vectors and a translation tuple, producing an unambiguous transform matrix. No composition order to get wrong.

### 3. 22 STEP imports was wasteful (performance)

**Problem:** Each rail/bracket/tplate was imported as a fresh STEP. 22 imports at ~1-2s each = 30-45s per script run.

**Fix:** Use `rootComp.occurrences.addExistingComponent(component, transform)` for instance reuse. Each unique part variant (long rail 248mm, short rail 188mm, post 100mm, mast 120mm, bracket, T-plate) is imported ONCE as a template. Additional placements are created as instances sharing the same geometry. Total STEP imports drop from 22 to 6 (~4x speedup).

### 4. `occurrence.transform` was being set on nested occurrences (latent bug)

**Problem:** rev9 imported into a `'1 - Frame'` sub-component, then set `occurrence.transform`. Fusion 360 API docs say `transform` can only be set when the occurrence's parent is the active edit target — root at script start. Nested occurrences would silently fail to position.

**Fix:** Import frame elements directly into root. The `'1 - Frame'` sub-component is gone; rails, posts, mast, brackets, and T-plate appear at root level in the browser tree. Other sub-components (plates, electronics, drive, wiring, ground) keep their sub-component grouping since they use procedural primitives rather than imports.

### 5. Robustness improvements

- **STEP file existence check** at the start of `_frame()`: fails fast with a clear error message if any model file is missing, rather than producing a partially-built frame.
- **Bounding-box sanity check** via `_check_centered()`: after importing the first rail, verifies the cross-section is centered at origin (±0.1cm). If off-center, logs a warning — all the hs offset math would be wrong and every rail/post misaligned.

### Rev10 code structure changes

- `_place()` replaced by `_place_occ(occ, c0, c1, c2, t)` and `_mat(c0, c1, c2, t)`
- `_scale_y(occ, ...)` renamed to `_scale_body_y(comp, factor)` (takes component directly)
- New helpers: `_instance()`, `_check_centered()`
- Frame sub-functions (`_frame_rails`, `_frame_posts`, etc.) collapsed into one `_frame()` with commented sections — easier to follow the import-scale-place flow
- New standard-basis constants `_X, _Y, _Z, _NX, _NY, _NZ` for readable transform tables
