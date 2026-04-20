"""Standalone motor-bracket printable STL generator (rev42 / Design 3).

Produces a single-body PETG dart-gusset bracket for the 25GA20E260
motor:
  - Motor flange (40x40x4mm, flat on the build plate)
  - Rail flange (20x4x30mm, elevated to rail-bottom height)
  - Diagonal dart (triangular rib, 4mm thick, connects the two flanges
    as a 45°-ish truss — stiffest geometry per gram)

All sketches are on the STANDARD XY plane (rev40 lesson: sketches on
XZ/YZ construction planes cause silent failures via CutFeatureOperation).
For the dart's triangular profile we use a sketch on an OFFSET XY plane
and extrude in +Z; the triangle itself lies flat in XY space.

Bolt clearance holes (M3 through motor flange, M5 through rail flange)
are SKIPPED in this script — drill them post-print with a 3.4mm and
5.5mm bit respectively. Keeps the script simple and avoids cutting
through non-XY plane sketches.

Usage via the bridge:
    python docs/chassis/fusion360/bridge/fctl.py close
    python docs/chassis/fusion360/bridge/fctl.py run \\
           docs/chassis/fusion360/bridge/motor_bracket_print.py \\
           --timeout 120

STL lands at D:\\tmp\\motor_bracket.stl — import into your slicer.

Slicer settings:
    Orient with the big 40×40 face DOWN on the build plate (already
    the case in the exported STL — Z+ is up, the flange is at z=0).
    PETG @ 240°C, bed 75°C. 4 perimeters, 40% gyroid infill. Tree
    supports if your slicer offers them (for the rail flange's
    underside). Apply thread-locker to bolts on install.

Print orientation note: print 2 bracket copies as-is for the LEFT
side of the robot; print 2 more MIRRORED (slicer's Mirror button)
for the RIGHT side. Motor flange face is asymmetric w.r.t. the dart,
so mirroring is required to match both sides.
"""

from __future__ import annotations

import os
import traceback

import adsk.core
import adsk.fusion


# --------------------------------------------------------------------------
# Dimensions (cm — multiply mm by 0.1)
# --------------------------------------------------------------------------
BRK = 0.4                         # 4mm PETG wall thickness

# Motor flange plate (40x40x4mm), lies flat in XY at z=[0, BRK]
PV_W = 4.0                        # X extent
PV_D = 4.0                        # Y extent (depth — also the distance to shaft axis)
PV_H = BRK                        # Z extent (thickness of the flat flange)

# Distance from motor-flange inner face (y=PV_D) back to rail flange.
# Derived from real motor: M_CAN + M_GBOX = 19.5+19.5 = 39mm.
ARM_LEN = 3.9

# Rail flange: vertical plate at Y=PV_D + ARM_LEN, perpendicular to Y axis.
# Sized 20mm wide × 4mm thick × 30mm tall (matches rail face height S=30mm).
RF_W = 2.0                        # X extent
RF_D = BRK                        # Y extent (thickness)
RF_H = 3.0                        # Z extent

# Rail flange is elevated in Z because the rail sits above the motor:
# vertical delta from motor-axis level up to the lower-rail-bottom edge.
# In the full chassis model AXL = 3.97, LO = 8.44, so this is about
# LO - (AXL - M_DIA/2 - BRK) = 8.44 - (3.97 - 1.25 - 0.4) = 6.12 cm.
RF_Z_BOTTOM = 6.12

# Dart gusset: triangular rib in the YZ plane, 4mm thick in X, centered
# on X=0. Connects the top of the motor flange to the bottom of the
# rail flange, forming a 45°-ish truss.
DART_THICKNESS = BRK              # 4mm thick rib

# Output path
STL_OUT = r'D:\tmp\motor_bracket.stl'


# --------------------------------------------------------------------------
# Sketch / extrude helpers (XY plane only — rev40 lesson)
# --------------------------------------------------------------------------
def _xy_plane(rc, z):
    """Construction plane parallel to XY, offset in Z by `z` cm."""
    inp = rc.constructionPlanes.createInput()
    inp.setByOffset(rc.xYConstructionPlane,
                    adsk.core.ValueInput.createByReal(z))
    return rc.constructionPlanes.add(inp)


def _box_xy(rc, name, x, y, z, sx, sy, sz):
    """Box from corner. Sketch on an XY plane at z, extrude +sz in Z.
    Uses the same pattern as B() in the main chassis script — XY-only,
    extrude in Z.
    """
    sk = rc.sketches.add(_xy_plane(rc, z))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(x, y, 0),
        adsk.core.Point3D.create(x + sx, y + sy, 0))
    ext = rc.features.extrudeFeatures.addSimple(
        sk.profiles.item(0),
        adsk.core.ValueInput.createByReal(sz),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)
    body.name = name
    return body


def _triangle_xy(rc, name, z, v1, v2, v3, thickness):
    """Triangular prism: 3 vertices in XY at plane Z=z, extruded in +Z
    by `thickness`. Used for the dart gusset's triangular XY footprint.
    """
    sk = rc.sketches.add(_xy_plane(rc, z))
    lines = sk.sketchCurves.sketchLines
    p1 = adsk.core.Point3D.create(v1[0], v1[1], 0)
    p2 = adsk.core.Point3D.create(v2[0], v2[1], 0)
    p3 = adsk.core.Point3D.create(v3[0], v3[1], 0)
    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p1)
    # Take the first (and only) closed profile
    prof = sk.profiles.item(0)
    ext = rc.features.extrudeFeatures.addSimple(
        prof,
        adsk.core.ValueInput.createByReal(thickness),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)
    body.name = name
    return body


def _combine_to_single(rc, bodies, name='MotorBracket'):
    """Boolean-union a list of bodies into one, return the result."""
    if not bodies:
        return None
    if len(bodies) == 1:
        bodies[0].name = name
        return bodies[0]
    target = bodies[0]
    tools = adsk.core.ObjectCollection.create()
    for b in bodies[1:]:
        tools.add(b)
    inp = rc.features.combineFeatures.createInput(target, tools)
    inp.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    inp.isKeepToolBodies = False
    combined = rc.features.combineFeatures.add(inp)
    result = combined.bodies.item(0)
    result.name = name
    return result


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
def run(context):
    ui = None
    app = None
    try:
        print('STEP 1: get Fusion application')
        app = adsk.core.Application.get()
        ui = app.userInterface

        print('STEP 2: open a fresh document')
        doc = app.documents.add(
            adsk.core.DocumentTypes.FusionDesignDocumentType)
        des = adsk.fusion.Design.cast(app.activeProduct)
        des.designType = adsk.fusion.DesignTypes.DirectDesignType
        rc = des.rootComponent

        print(f'STEP 3: MtrFlange 40x40x{BRK*10:.0f}mm, flat on XY at z=0')
        mtr = _box_xy(rc, 'MtrFlange',
                      -PV_W / 2, 0, 0,
                      PV_W, PV_D, PV_H)

        print(f'STEP 4: RailFlange 20x{BRK*10:.0f}x30mm '
              f'at y={PV_D + ARM_LEN:.2f}cm, z={RF_Z_BOTTOM:.2f}cm')
        rf_y0 = PV_D + ARM_LEN
        rail = _box_xy(rc, 'RailFlange',
                       -RF_W / 2, rf_y0, RF_Z_BOTTOM,
                       RF_W, RF_D, RF_H)

        # -------------------------------------------------------------------
        # Dart gusset (Design 3): triangular rib bridging MtrFlange
        # inner edge to RailFlange bottom edge diagonally. Implemented
        # as 2 stacked triangular prisms so the whole thing stays in
        # XY-plane sketches — no cross-plane API quirks.
        #
        # In YZ (side view, X-center out of page), the dart is a right
        # triangle with:
        #     A = (y=PV_D, z=0)              — motor-flange inner-bottom edge
        #     B = (y=rf_y0, z=RF_Z_BOTTOM)   — rail-flange outer-bottom edge
        #     C = (y=PV_D, z=RF_Z_BOTTOM)    — above A, at rail flange bottom height
        #
        # This is a RIGHT triangle (vertical leg from A->C, horizontal leg
        # from C->B, hypotenuse A->B at ~52° from horizontal).
        #
        # We realise this triangle as N stacked rectangular layers in XY,
        # each at a different z, forming a staircase that a slicer smooths
        # when printing at low enough layer height. 20 layers ≈ 3mm each
        # for RF_Z_BOTTOM / N = 6.12 / 20 = 0.306cm = 3.06mm per layer.
        # -------------------------------------------------------------------
        print('STEP 5: dart staircase (20 XY-plane layers)')
        N_LAYERS = 20
        dart_bodies = []
        for i in range(N_LAYERS):
            # z range for this layer
            z0 = (RF_Z_BOTTOM * i) / N_LAYERS
            z1 = (RF_Z_BOTTOM * (i + 1)) / N_LAYERS
            layer_h = z1 - z0
            # At height z0, the diagonal edge (A→B line) has interpolated Y:
            #     y(z) = PV_D + (z / RF_Z_BOTTOM) * ARM_LEN
            y_at_this_layer = PV_D + (z0 / RF_Z_BOTTOM) * ARM_LEN
            y_at_next_layer = PV_D + (z1 / RF_Z_BOTTOM) * ARM_LEN
            # Use the AVERAGE Y-extent of the layer so the staircase
            # centerline follows the hypotenuse.
            y_end = (y_at_this_layer + y_at_next_layer) / 2
            # Layer goes from Y=PV_D (flush with motor flange inner edge)
            # out to y_end (progressively further as z increases).
            y_len = y_end - PV_D
            if y_len < 1e-3:
                continue  # skip zero-area layers at the base
            name = f'DartLayer{i:02d}'
            body = _box_xy(rc, name,
                           -DART_THICKNESS / 2, PV_D, z0,
                           DART_THICKNESS, y_len, layer_h)
            dart_bodies.append(body)

        print(f'STEP 6: combine {1 + 1 + len(dart_bodies)} bodies into one')
        all_bodies = [mtr, rail] + dart_bodies
        combined = _combine_to_single(rc, all_bodies, 'MotorBracket')

        print('STEP 7: export STL')
        out_dir = os.path.dirname(STL_OUT)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)
        exp = des.exportManager
        stl_opts = exp.createSTLExportOptions(combined, STL_OUT)
        stl_opts.meshRefinement = (
            adsk.fusion.MeshRefinementSettings.MeshRefinementHigh)
        stl_opts.isBinaryFormat = True
        stl_opts.sendToPrintUtility = False
        ok = exp.execute(stl_opts)

        try:
            app.activeViewport.fit()
        except Exception:
            pass

        exists = os.path.exists(STL_OUT)
        size = os.path.getsize(STL_OUT) if exists else 0
        print(f'DONE: wrote {STL_OUT}')
        print(f'      export ok={ok} file_exists={exists} size={size} bytes')
        if ui:
            ui.messageBox(
                f'Motor bracket exported:\n{STL_OUT}\n'
                f'File size: {size} bytes\n\n'
                f'Print with the 40x40mm face DOWN on the build plate.\n'
                f'PETG 240°C, bed 75°C, 4 perimeters, 40% infill.\n'
                f'Drill M3 bolt holes (Ø3.4mm) through the motor flange '
                f'on the 17mm-apart horizontal axis; drill 2 M5 holes '
                f'(Ø5.5mm) through the rail flange for the T-slot.')

    except Exception as e:
        tb = traceback.format_exc()
        # Print to stdout so the bridge captures it in its response.
        print('FAIL')
        print(tb)
        if ui:
            ui.messageBox(
                f'motor_bracket_print FAILED:\n{tb}')
