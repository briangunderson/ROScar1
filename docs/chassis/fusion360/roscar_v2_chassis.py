import adsk.core
import adsk.fusion
import traceback
import math
import os

# =============================================================================
# ROScar1 v2 Chassis — Parametric Fusion 360 Model
# =============================================================================
# REVISION LOG:
#   rev1-3  2026-04-05  Sub-component bugs → exploded layouts
#   rev4    2026-04-05  XY-plane-only fix → correct positioning
#   rev5    2026-04-05  T-slot profiles, revolve cylinders
#   rev6    2026-04-05  Colors, corner brackets, ground plane
#   rev7    2026-04-05  Motor mounting fix (below frame on L-brackets)
#   rev8    2026-04-05  Full polish pass:
#                       - Segmented mecanum wheels (diagonal box pattern)
#                       - Two-section motor (can + gearbox)
#                       - Motor encoder disc
#                       - RPi5 port indicators (USB, HDMI, Ethernet)
#                       - Camera mount clip
#                       - Deck plate standoffs
#                       - Wire routing hints
#                       - Improved L-bracket shape
#                       - Better lidar cable detail
#   rev9    2026-04-11  STEP model imports:
#                       - Real 3030 extrusion profiles (imported + scaled)
#                       - Real 3-way corner brackets at all 8 frame joints
#                       - T-plate bracket for lidar mast
#                       - Removed procedural TZ() function
#   rev10   2026-04-11  Fixes and optimizations over rev9:
#                       - Upper-deck brackets now flip Z arm down (was up)
#                       - Explicit column-vector transform matrices (no more
#                         transformBy chaining — ambiguous composition order)
#                       - Instance reuse via addExistingComponent: 22 STEP
#                         imports reduced to 6 (~4x faster script run)
#                       - Frame imports go to ROOT (not sub-component) so that
#                         occurrence.transform actually takes effect
#                       - STEP file existence check (fail fast with clear msg)
#                       - Bounding-box sanity check for cross-section centering
#   rev11   2026-04-11  Abandon instance reuse — each rail/post/bracket is its
#                       own STEP import (22 imports). In rev10 tests, instances
#                       created via addExistingComponent were positioned at the
#                       compound transform master×new instead of just new,
#                       landing them outside the frame. ~20s slower but correct.
#   rev12   2026-04-17  Make it read as a connected machine — lots of added
#                       engineering detail showing HOW parts attach:
#                       - Mecanum wheels: real solid tire + central hub + 8
#                         proper slanted rollers at 45deg (was 6 diagonal boxes
#                         that read as "random cubes")
#                       - Palette: frame is now anodized silver so the 3030
#                         skeleton reads as structure; corner brackets gunmetal
#                       - NEW: battery retention strap (velcro band top + sides)
#                       - NEW: 4 standoffs + bolt heads under motor board
#                       - NEW: 4 standoffs + bolt heads under RPi5 Argon case
#                       - NEW: lidar mount plate between mast top and lidar
#                         base with 4 visible M2.5 screws
#                       - NEW: camera clamp-over-rail bracket (top arm + web +
#                         bottom arm + clamping bolt)
#                       - NEW: deck plate bolt heads at 4 corners per plate
#                         (8 total) showing plates bolt into rail T-nuts
#                       - NEW: motor L-bracket mounting bolts (through plate
#                         into rail T-nut + under-motor attachment)
#                       - NEW: cable ties along upper rail routing wires
#                       - Cable runs now actually connect port-to-port: power
#                         battery→board, USB board→Pi (via rear post), lidar
#                         USB→Pi (via mast+rail), camera USB→Pi
#                       - Added _mecanum_wheel() and _move_body_transform()
#                         helpers for the slanted roller geometry.
#   rev13   2026-04-17  Structural focus — Brian is cutting extrusions today,
#                       so strip procedural noise and verify the frame geometry
#                       matches the cut plan:
#                       - Non-structural components downgraded to simple shapes
#                         (battery/board/pi5/lidar/camera): dropped standoff+bolt
#                         stacks, port bumps, strap bands, wire leads, cable runs
#                       - KEPT (structural/connector): 3030 frame STEP imports,
#                         3-way corner brackets, T-plate, deck plates+bolts,
#                         motor L-brackets+bolts, lidar mount plate+screws,
#                         mecanum wheels
#                       - Wiring stripped entirely (will come back after real
#                         component STEPs are imported with real port positions)
#                       - Cut plan in design spec reconciled to match the
#                         script geometry (4x 248mm + 4x 188mm, was wrongly
#                         claiming all 8 rails at 248mm). See
#                         docs/superpowers/specs/2026-04-04-extrusion-chassis-
#                         design.md §4.6 and tasks/chassis-v2-cut-sheet.md.
# =============================================================================
VERSION = 'rev13'

# ═══════════════════════════════════════════════════════════════════════════
# Dimensions (cm) — multiply mm by 0.1
# ═══════════════════════════════════════════════════════════════════════════
FRAME = 24.8;  S = 3.0;  POST_H = 10.0;  MAST_H = 12.0

# Motor (hangs below frame on L-brackets)
M_DIA = 2.4;  M_CAN = 3.8;  M_GBOX = 2.4  # can 38mm + gearbox 24mm = 62mm
M_GBOX_DIA = 2.2  # gearbox slightly narrower
SHAFT = 1.3;  SHAFT_D = 0.4
WHL_R = 3.97;  WHL_W = 3.73
BRKT_T = 0.4  # bracket plate thickness

# Motor offset from frame face to wheel center
M_OFF = M_CAN + M_GBOX + SHAFT + WHL_W/2  # ~9.37cm

# Lower deck: frame bottom above wheel tops
GND = WHL_R * 2 + 0.5  # ~8.44cm
AXL = WHL_R  # axle height = wheel radius

# Components
BAT = (7.5, 5.5, 6.6);  BRD = (8.5, 5.6, 2.0);  RPI = (9.7, 7.3, 4.0)
LID_SZ = 5.56;  LID_H = 4.13;  PLT_T = 0.3

# Derived
INNER = FRAME - 2*S;  LO = GND;  HI = LO + S + POST_H;  MST = HI + S
HWB = 10.0  # half-wheelbase

# Wheel segments for mecanum visual
N_SEGS = 6;  SEG_GAP_RATIO = 0.12;  DIAG_SHIFT = 0.15  # cm shift per segment

# ═══════════════════════════════════════════════════════════════════════════
# Color palette
# ═══════════════════════════════════════════════════════════════════════════
PAL = {
    # rev12: palette tuned for visual legibility over engineering detail.
    # Frame is now anodized aluminum so the 3030 skeleton reads as structure;
    # brackets are subordinate gunmetal instead of loud orange; deck plates
    # are warm amber so they look like real PCB/acrylic instead of institutional blue.
    'frame':    (185, 190, 200),   # anodized aluminum silver
    'plate':    (215, 180, 120),   # warm tan acrylic / amber G10
    'battery':  (40, 95, 175),     # battery blue
    'pcb':      (35, 90, 55),      # PCB green
    'rpi':      (45, 45, 55),      # dark matte case
    'motor':    (200, 200, 205),   # motor can silver
    'gearbox':  (55, 55, 62),      # gearbox dark
    'wheel':    (28, 28, 32),      # tire rubber
    'roller':   (55, 58, 65),      # roller darker gray
    'lidar':    (30, 30, 38),      # lidar dark
    'camera':   (50, 50, 58),      # camera body
    'bracket':  (90, 95, 105),     # gunmetal (was loud orange)
    'corner':   (130, 135, 145),   # corner brackets slightly lighter than frame
    'ground':   (75, 80, 88),      # warm gray floor (was near-black)
    'hub':      (135, 140, 150),   # wheel hub
    'standoff': (185, 190, 200),   # standoffs match frame
    'lens':     (180, 190, 210),   # camera/lidar lens glass
    # Kept for backward compatibility (no longer used in rev12):
    'wire_pwr': (180, 30, 30), 'wire_usb': (40, 40, 45), 'wire_enc': (30, 130, 30),
    'port':     (35, 35, 40), 'encoder':  (25, 70, 25),
}

_cc = {};  _clog = []

# ═══════════════════════════════════════════════════════════════════════════
# STEP model paths (absolute — this script runs locally in Fusion 360)
# ═══════════════════════════════════════════════════════════════════════════
_MODELS = r'D:\localrepos\ROScar1\docs\chassis\models'
STEP_EXTRUSION = os.path.join(_MODELS, 'extrusions', '30X30 - T-slot - Aluminium Profile.step')
STEP_CORNER_BRACKET = os.path.join(_MODELS, 'brackets', '3030 - 3-way corner bracket.step')
STEP_TPLATE_M6 = os.path.join(_MODELS, 'brackets', 'T-Plate 3030 5 Holes - M6.step')

# ═══════════════════════════════════════════════════════════════════════════
# STEP import helpers
# ═══════════════════════════════════════════════════════════════════════════
# Transform convention: a placement is a 3x3 rotation (given as 3 column
# vectors) plus a translation. Each column describes where one of the body's
# local axes points in world space. Using column vectors directly avoids any
# ambiguity about rotation composition order — the matrix is constructed
# explicitly, with no chained transformBy calls.
#
# NOTE: imports go into ROOT, not a sub-component. Setting occurrence.transform
# only works when the occurrence's parent is the active edit target (root at
# script start). Nested occurrences would need moveFeatures or activateComponent
# workarounds — simpler to keep imports flat.

# Standard basis tuples — easier to read in placement tables
_X = (1, 0, 0);   _Y = (0, 1, 0);   _Z = (0, 0, 1)
_NX = (-1, 0, 0); _NY = (0, -1, 0); _NZ = (0, 0, -1)


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


def _mat(c0, c1, c2, t):
    """Build a Matrix3D from 3 column vectors + translation tuple.

    c0, c1, c2: where the body's local +X, +Y, +Z axes point in world space.
    t = (tx, ty, tz): translation applied AFTER the rotation.

    Matrix is row-major in setWithArray: rows go left-to-right, top-to-bottom.
    """
    m = adsk.core.Matrix3D.create()
    m.setWithArray([
        c0[0], c1[0], c2[0], t[0],
        c0[1], c1[1], c2[1], t[1],
        c0[2], c1[2], c2[2], t[2],
        0,     0,     0,     1,
    ])
    return m


def _place_occ(occ, c0, c1, c2, t):
    """Position an existing occurrence using column vectors + translation."""
    occ.transform = _mat(c0, c1, c2, t)


def _instance(rc, comp, c0, c1, c2, t):
    """Create a new instance of comp at the given transform.

    Uses addExistingComponent for instance reuse — all instances share the
    same underlying geometry definition, only their transforms differ.
    """
    return rc.occurrences.addExistingComponent(comp, _mat(c0, c1, c2, t))


def _scale_body_y(comp, factor):
    """Non-uniform scale all bodies in comp along Y by the given factor.

    The source extrusion STEP is 10.0 cm (100mm) along Y.
    Scale pivot is the component origin, so the Y=0 end stays fixed.
    All instances of this component see the scaled bodies.
    """
    if abs(factor - 1.0) < 1e-4:
        return
    bodies = adsk.core.ObjectCollection.create()
    for i in range(comp.bRepBodies.count):
        bodies.add(comp.bRepBodies.item(i))
    if bodies.count == 0:
        return
    inp = comp.features.scaleFeatures.createInput(
        bodies, comp.originConstructionPoint,
        adsk.core.ValueInput.createByReal(1.0))
    inp.setToNonUniform(
        adsk.core.ValueInput.createByReal(1.0),
        adsk.core.ValueInput.createByReal(factor),
        adsk.core.ValueInput.createByReal(1.0))
    comp.features.scaleFeatures.add(inp)


def _clr_occ(occ, key):
    """Apply color to all bodies in an occurrence's component."""
    comp = occ.component
    for i in range(comp.bRepBodies.count):
        _clr(comp.bRepBodies.item(i), key)


def _check_centered(occ, label):
    """Log a warning if the imported body's XZ cross-section isn't centered.

    All transform math assumes the STEP extrusion has its cross-section
    centered at origin (±S/2). This warns if that assumption is wrong.
    """
    try:
        bb = occ.boundingBox
        cx = (bb.minPoint.x + bb.maxPoint.x) / 2
        cz = (bb.minPoint.z + bb.maxPoint.z) / 2
        if abs(cx) > 0.1 or abs(cz) > 0.1:
            _clog.append(f'{label}: cross-section off-center cx={cx:.2f} cz={cz:.2f}')
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        des = adsk.fusion.Design.cast(app.activeProduct)
        des.designType = adsk.fusion.DesignTypes.ParametricDesignType
        rc = des.rootComponent

        # Frame imports go into ROOT (not a sub-component) because
        # occurrence.transform only works when the parent is the active edit
        # target. Procedural parts (plates, electronics, etc.) still use
        # sub-components for browser tree organization.
        _frame(rc)
        _plates(_comp(rc, '2 - Deck Plates'))
        _standoffs(_comp(rc, '3 - Standoffs'))

        elec = _comp(rc, '4 - Electronics')
        _components(elec)

        _drive(_comp(rc, '5 - Drive System'))
        _wiring(_comp(rc, '6 - Wiring'))
        _ground(_comp(rc, '7 - Ground Plane'))

        app.activeViewport.fit()
        trk = FRAME + 2*M_OFF
        msg = (f'ROScar1 v2 ({VERSION})\n'
               f'Frame: {FRAME*10:.0f}mm | Track: {trk*10:.0f}mm\n'
               f'WB: {HWB*20:.0f}mm | H: {(MST+MAST_H+LID_H)*10:.0f}mm\n'
               f'Cut plan: 4x248mm + 4x188mm + 4x100mm posts + 1x120mm mast')
        if _clog: msg += f'\nLog: {_clog[0]}'
        ui.messageBox(msg)
    except:
        if ui: ui.messageBox(f'FAIL ({VERSION}):\n{traceback.format_exc()}')

# ═══════════════════════════════════════════════════════════════════════════
# Primitives
# ═══════════════════════════════════════════════════════════════════════════
def _comp(parent, name):
    """Create a named sub-component. Identity transform = same coordinate space as parent."""
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component

def _zp(rc, z):
    i = rc.constructionPlanes.createInput()
    i.setByOffset(rc.xYConstructionPlane, adsk.core.ValueInput.createByReal(z))
    return rc.constructionPlanes.add(i)

def _clr(body, key):
    if key not in PAL: return
    r, g, b = PAL[key]; tag = f'R_{key}'
    try:
        app = adsk.core.Application.get()
        des = adsk.fusion.Design.cast(app.activeProduct)
        if tag in _cc: body.appearance = _cc[tag]; return
        ap = des.appearances.itemByName(tag)
        if not ap:
            base = None
            for ln in ['Fusion 360 Appearance Library','Fusion Appearance Library']:
                lib = app.materialLibraries.itemByName(ln)
                if lib:
                    for an in ['Paint - Enamel Glossy (Yellow)','Paint - Enamel Glossy (White)']:
                        base = lib.appearances.itemByName(an)
                        if base: break
                    if not base and lib.appearances.count > 0: base = lib.appearances.item(0)
                    if base: break
            if not base:
                for lib in app.materialLibraries:
                    if lib.appearances.count > 0: base = lib.appearances.item(0); break
            if not base:
                if not _clog: _clog.append('No appearance library found')
                return
            ap = des.appearances.addByCopy(base, tag)
            for pn in ['Color','color','surface_albedo']:
                p = ap.appearanceProperties.itemByName(pn)
                if p:
                    try:
                        cp = adsk.core.ColorProperty.cast(p)
                        if cp: cp.value = adsk.core.Color.create(r,g,b,255); break
                    except: continue
        _cc[tag] = ap; body.appearance = ap
    except Exception as e:
        if not _clog: _clog.append(str(e))

def B(rc, nm, x, y, z, sx, sy, sz, c=None):
    """Box from corner."""
    sk = rc.sketches.add(_zp(rc, z))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(x,y,0), adsk.core.Point3D.create(x+sx,y+sy,0))
    b = rc.features.extrudeFeatures.addSimple(sk.profiles.item(0),
        adsk.core.ValueInput.createByReal(sz),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    b.name = nm
    if c: _clr(b, c)
    return b

def CZ(rc, nm, cx, cy, z, r, h, c=None):
    """Vertical cylinder."""
    sk = rc.sketches.add(_zp(rc, z))
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(cx,cy,0), r)
    b = rc.features.extrudeFeatures.addSimple(sk.profiles.item(0),
        adsk.core.ValueInput.createByReal(h),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    b.name = nm
    if c: _clr(b, c)
    return b

def CY(rc, nm, ax, y0, az, r, l, c=None):
    """Horizontal cylinder along Y via revolve."""
    sk = rc.sketches.add(_zp(rc, az))
    a = sk.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(ax, y0-0.5, 0), adsk.core.Point3D.create(ax, y0+l+0.5, 0))
    a.isConstruction = True
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(ax+0.001, y0, 0), adsk.core.Point3D.create(ax+r, y0+l, 0))
    ri = rc.features.revolveFeatures.createInput(sk.profiles.item(0), a,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2*math.pi))
    b = rc.features.revolveFeatures.add(ri).bodies.item(0)
    b.name = nm
    if c: _clr(b, c)
    return b

# ═══════════════════════════════════════════════════════════════════════════
# Frame (imported STEP models with instance reuse)
# ═══════════════════════════════════════════════════════════════════════════
# Strategy: import each unique model variant ONCE as a template component
# (imports are slow, ~1-2s each). For each visible placement, either use the
# template's first occurrence or create a new instance via addExistingComponent.
# All instances of a template share the same geometry — changing the template
# body updates every instance automatically.
#
# Total STEP imports: 6 (long rails + short rails + posts + mast + bracket +
# T-plate). Total occurrences: 22 (8 rails + 4 posts + 1 mast + 8 brackets +
# 1 T-plate). Previously rev9 did 22 imports — rev10 is ~4x faster to run.
#
# Transform convention (see _mat helper):
#   Body +X axis points in direction c0 (in world space)
#   Body +Y axis points in direction c1
#   Body +Z axis points in direction c2
#   Body origin is translated by t = (tx, ty, tz)
# For the 3030 extrusion STEP, +Y is the length direction.

def _import_rail(rc, name, scale_factor, c0, c1, c2, t):
    """Import a 3030 extrusion, scale its body, position, and color.
    Each call does its own import — no instance reuse (see rev11 notes).
    """
    occ = _import_step(rc, STEP_EXTRUSION, name)
    _scale_body_y(occ.component, scale_factor)
    _clr_occ(occ, 'frame')
    _place_occ(occ, c0, c1, c2, t)
    return occ


def _import_bracket(rc, step_path, name, c0, c1, c2, t, color='corner'):
    """Import a bracket STEP and position it. No scaling — brackets are fixed size."""
    occ = _import_step(rc, step_path, name)
    _clr_occ(occ, color)
    _place_occ(occ, c0, c1, c2, t)
    return occ


def _frame(rc):
    """Build the frame from imported STEP models.

    Rev11: Each occurrence is a separate STEP import. Instance reuse via
    addExistingComponent (rev10) caused the 2nd/3rd/4th instances to compose
    their transform with the master occurrence's transform, landing them
    outside the frame. Individual imports avoid this — ~20s longer script
    run but 100% reliable placement.

    Imports go directly into root component — required for occurrence.transform
    to work (setter needs occurrence parent to be the active edit target).
    """
    # Fail fast if any required STEP file is missing
    for path in (STEP_EXTRUSION, STEP_CORNER_BRACKET, STEP_TPLATE_M6):
        if not os.path.exists(path):
            raise RuntimeError(f'STEP model not found: {path}')

    hs = S / 2                    # 1.5 cm: half the 3030 profile width
    inner_len = FRAME - 2 * S     # left/right rail length
    fr_scale = FRAME / 10         # front/rear rail scale factor (248/100)
    lr_scale = inner_len / 10     # left/right rail scale factor (188/100)
    post_scale = POST_H / 10      # post scale (100/100 = 1.0, but explicit)
    mast_scale = MAST_H / 10      # mast scale (120/100)

    # --------------------------------------------------------------------
    # Long rails (front/rear) — runs along world +Y, full FRAME length
    # Body +Y is the length axis, so identity rotation is correct.
    # Cross-section centered at origin, so translate +hs in X and Z.
    # --------------------------------------------------------------------
    first = _import_rail(rc, 'Rail_FR_Lo_Front', fr_scale, _X, _Y, _Z, (hs,        0, LO + hs))
    _check_centered(first, 'Long rail')
    _import_rail(rc,     'Rail_FR_Lo_Rear',  fr_scale, _X, _Y, _Z, (FRAME - hs, 0, LO + hs))
    _import_rail(rc,     'Rail_FR_Hi_Front', fr_scale, _X, _Y, _Z, (hs,        0, HI + hs))
    _import_rail(rc,     'Rail_FR_Hi_Rear',  fr_scale, _X, _Y, _Z, (FRAME - hs, 0, HI + hs))

    # --------------------------------------------------------------------
    # Short rails (left/right) — runs along world +X, inner length.
    # Body +Y (length) maps to world +X via rotation with columns (-Y, +X, +Z).
    # --------------------------------------------------------------------
    _import_rail(rc, 'Rail_LR_Lo_Left',  lr_scale, _NY, _X, _Z, (S, hs,         LO + hs))
    _import_rail(rc, 'Rail_LR_Lo_Right', lr_scale, _NY, _X, _Z, (S, FRAME - hs, LO + hs))
    _import_rail(rc, 'Rail_LR_Hi_Left',  lr_scale, _NY, _X, _Z, (S, hs,         HI + hs))
    _import_rail(rc, 'Rail_LR_Hi_Right', lr_scale, _NY, _X, _Z, (S, FRAME - hs, HI + hs))

    # --------------------------------------------------------------------
    # Vertical posts (100mm). Body +Y maps to world +Z via (+X, +Z, -Y).
    # --------------------------------------------------------------------
    _import_rail(rc, 'Post_FL', post_scale, _X, _Z, _NY, (hs,         hs,         LO + S))
    _import_rail(rc, 'Post_FR', post_scale, _X, _Z, _NY, (hs,         FRAME - hs, LO + S))
    _import_rail(rc, 'Post_RL', post_scale, _X, _Z, _NY, (FRAME - hs, hs,         LO + S))
    _import_rail(rc, 'Post_RR', post_scale, _X, _Z, _NY, (FRAME - hs, FRAME - hs, LO + S))

    # --------------------------------------------------------------------
    # Lidar mast (120mm) — single vertical post at center-rear of upper deck.
    # --------------------------------------------------------------------
    _import_rail(rc, 'Mast', mast_scale, _X, _Z, _NY, (FRAME - S, FRAME / 2, MST))

    # --------------------------------------------------------------------
    # Corner brackets (8 total) — STEP has arms along +X, +Y, +Z.
    # Lower deck: Z arm points UP. Upper deck: Z arm points DOWN.
    # Upper-deck rotations swap X/Y columns to keep det=+1 (proper rotation).
    # --------------------------------------------------------------------
    # Lower deck brackets
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Lo_FL', _X,  _Y,  _Z, (0,     0,     LO + S))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Lo_FR', _NY, _X,  _Z, (0,     FRAME, LO + S))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Lo_RL', _Y,  _NX, _Z, (FRAME, 0,     LO + S))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Lo_RR', _NX, _NY, _Z, (FRAME, FRAME, LO + S))
    # Upper deck brackets — Z flipped
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Hi_FL', _Y,  _X,  _NZ, (0,     0,     HI))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Hi_FR', _X,  _NY, _NZ, (0,     FRAME, HI))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Hi_RL', _NX, _Y,  _NZ, (FRAME, 0,     HI))
    _import_bracket(rc, STEP_CORNER_BRACKET, 'CB_Hi_RR', _NY, _NX, _NZ, (FRAME, FRAME, HI))

    # --------------------------------------------------------------------
    # T-plate bracket for lidar mast attachment to rear upper rail
    # --------------------------------------------------------------------
    _import_bracket(rc, STEP_TPLATE_M6, 'Mast_TPlate',
                    _X, _Y, _Z, (FRAME - S, FRAME / 2, MST - 0.1))

# ═══════════════════════════════════════════════════════════════════════════
# Plates & standoffs
# ═══════════════════════════════════════════════════════════════════════════
def _plates(rc):
    """Deck plates + visible fasteners securing them to the rails.

    rev12: added bolt heads at each plate corner so it's obvious the plates
    are bolted to T-nuts in the rail channels, not just resting there.
    """
    c = FRAME/2
    B(rc, 'LoPlate', c-INNER/2, c-INNER/2, LO+S, INNER, INNER, PLT_T, 'plate')
    B(rc, 'HiPlate', c-INNER/2, c-INNER/2, HI+S, INNER, INNER, PLT_T, 'plate')

    # 4 bolt heads at the inner corners of each plate (M5 socket caps into T-nuts)
    inset = 0.5  # 5mm from plate edge
    for x in [c - INNER/2 + inset, c + INNER/2 - inset]:
        for y in [c - INNER/2 + inset, c + INNER/2 - inset]:
            CZ(rc, 'LoPlateBolt', x, y, LO + S + PLT_T, 0.22, 0.15, 'bracket')
            CZ(rc, 'HiPlateBolt', x, y, HI + S + PLT_T, 0.22, 0.15, 'bracket')

def _standoffs(rc):
    """Small cylindrical standoffs at plate corners."""
    inset = 1.5  # 15mm from plate edge
    lo_z = LO + S - 0.5;  hi_z = HI + S - 0.5
    for x_off in [FRAME/2 - INNER/2 + inset, FRAME/2 + INNER/2 - inset]:
        for y_off in [FRAME/2 - INNER/2 + inset, FRAME/2 + INNER/2 - inset]:
            CZ(rc, 'SO', x_off, y_off, lo_z, 0.25, 0.5 + PLT_T, 'standoff')
            CZ(rc, 'SO', x_off, y_off, hi_z, 0.25, 0.5 + PLT_T, 'standoff')

# ═══════════════════════════════════════════════════════════════════════════
# Components
# ═══════════════════════════════════════════════════════════════════════════
def _components(rc):
    """Non-structural components as simple placeholders (rev13).

    User directive: "a simple shape representing the raspi, lidar, motor board,
    etc is fine at this stage". Focus is on getting the FRAME right for
    fabrication — detailed electronics can be added back once we have real
    STEP models from GrabCAD/manufacturers.

    Kept: bounding-box representations so the components occupy realistic
    space and clash detection still works. Dropped: all fake SMD/port/header
    detail, battery strap, cable leads — these were procedural guesses that
    look worse than real CAD.

    Kept: lidar mount plate (this IS a connector between mast and lidar —
    the user explicitly listed connectors as important).
    """
    c = FRAME/2;  cz = LO + S + PLT_T

    # Battery — simple box on lower deck
    B(rc, 'Battery', c-BAT[0]/2, c-BAT[1]/2, cz, BAT[0], BAT[1], BAT[2], 'battery')

    # Motor board — plain PCB block on lower deck
    bx = FRAME * 0.7
    B(rc, 'MotorBoard', bx-BRD[0]/2, c-BRD[1]/2, cz, BRD[0], BRD[1], BRD[2], 'pcb')

    # RPi5 in Argon NEO case — plain case block on upper deck
    rz = HI + S + PLT_T
    B(rc, 'RPi5', c-RPI[0]/2, c-RPI[1]/2, rz, RPI[0], RPI[1], RPI[2], 'rpi')

    # ── Lidar mount plate (connector between mast and lidar) ────────────
    lcx, lcy = FRAME-S, FRAME/2
    mp_t = 0.4                 # 4mm mount plate
    mp_sz = LID_SZ + 1.0       # slightly larger than lidar footprint
    mp_z = MST + MAST_H        # sits on top of mast
    B(rc, 'Lidar_MntPlate',
      lcx-mp_sz/2, lcy-mp_sz/2, mp_z, mp_sz, mp_sz, mp_t, 'corner')
    # 4 M2.5 screws holding lidar to mount plate (these ARE fasteners)
    for dx, dy in [(-LID_SZ/2*0.7, -LID_SZ/2*0.7),
                    ( LID_SZ/2*0.7, -LID_SZ/2*0.7),
                    (-LID_SZ/2*0.7,  LID_SZ/2*0.7),
                    ( LID_SZ/2*0.7,  LID_SZ/2*0.7)]:
        CZ(rc, 'Lidar_Screw', lcx+dx, lcy+dy, mp_z+mp_t, 0.18, 0.15, 'bracket')

    # Lidar — simple base + head
    lz = mp_z + mp_t
    bh = LID_H * 0.35;  hh = LID_H * 0.65
    B(rc, 'Lidar_Base', lcx-LID_SZ/2, lcy-LID_SZ/2, lz, LID_SZ, LID_SZ, bh, 'lidar')
    CZ(rc, 'Lidar_Head', lcx, lcy, lz+bh, LID_SZ/2*0.9, hh, 'lidar')

    # Camera — simple body + lens, single mounting clip
    cam_z = HI + S/2
    B(rc, 'Cam_Body', -0.5, c-3.5, cam_z, 2.5, 7.0, 2.5, 'camera')
    try: CZ(rc, 'Cam_Lens', 0.5, c, cam_z+0.8, 0.8, 0.8, 'lens')
    except Exception: pass
    # Single clamp-over-rail bracket (just the top arm — simplified)
    B(rc, 'Cam_Clip', -0.3, c-2.0, cam_z+2.5, 1.0, 4.0, 0.8, 'bracket')

# ═══════════════════════════════════════════════════════════════════════════
# Drive: motors, wheels, brackets
# ═══════════════════════════════════════════════════════════════════════════
def _drive(rc):
    xf = FRAME/2 - HWB;  xr = FRAME/2 + HWB
    for tag, mx, fy, od in [('FL',xf,0,-1),('FR',xf,FRAME,1),('RL',xr,0,-1),('RR',xr,FRAME,1)]:
        _motor_assy(_comp(rc, f'Motor {tag}'), tag, mx, fy, od)

def _move_body_transform(rc, body, mat):
    """Apply a Matrix3D transform to a single body via a MoveFeature."""
    coll = adsk.core.ObjectCollection.create()
    coll.add(body)
    try:
        mi = rc.features.moveFeatures.createInput2(coll)
        mi.defineAsFreeMove(mat)
        rc.features.moveFeatures.add(mi)
    except Exception:
        pass


def _mecanum_wheel(rc, tag, mx, wcy):
    """Proper mecanum wheel: tire + hub + 8 slanted rollers at 45deg.

    Wheel axis runs along world +Y. Rollers are short cylinders positioned
    around the outer rim, each rotated so its own axis makes a 45deg angle
    with the wheel axis (tangent to the tire surface). That's the defining
    feature of a mecanum wheel that lets the robot strafe.
    """
    # Core tire (slightly undersized so rollers stick out a touch beyond)
    tire_r = WHL_R * 0.85
    try:
        CY(rc, f'Tire_{tag}', mx, wcy - WHL_W/2, AXL, tire_r, WHL_W, 'wheel')
    except Exception:
        B(rc, f'Tire_{tag}', mx - tire_r, wcy - WHL_W/2, AXL - tire_r,
          tire_r * 2, WHL_W, tire_r * 2, 'wheel')

    # Central hub (protrudes slightly past the tire on both sides)
    try:
        CY(rc, f'Hub_{tag}', mx, wcy - WHL_W/2 - 0.25,
           AXL, WHL_R * 0.28, WHL_W + 0.5, 'hub')
    except Exception:
        pass

    # Slanted rollers around the perimeter
    N_ROLLERS = 8
    roller_r = WHL_R * 0.16
    roller_l = WHL_W * 0.92
    ring_r = WHL_R * 0.98        # where roller centers sit (on outer edge)
    half_pi_4 = math.pi / 4       # 45deg tilt
    c45 = math.cos(half_pi_4); s45 = math.sin(half_pi_4)
    try:
        for i in range(N_ROLLERS):
            theta = 2 * math.pi * i / N_ROLLERS
            ct, st = math.cos(theta), math.sin(theta)

            # 1. Create roller as a Y-axis cylinder at origin via CY.
            #    (CY draws at z=AXL by default — we'll override translation below.)
            body = CY(rc, f'Roller_{tag}_{i}', 0, -roller_l / 2, 0,
                      roller_r, roller_l, 'roller')

            # 2. Build the transform matrix that tilts the roller 45deg around
            #    the radial direction, then moves it to position theta around
            #    the wheel.
            #    - Body's local Y axis (length) should end up at 45deg between
            #      world Y and the tangent at theta (tangent = (-sin,0,cos)).
            #    - Body's local X axis (cross-section radial) should map to
            #      the wheel's radial direction at theta.
            #    - Body's local Z axis maps perpendicular to both.
            #
            #    Tangent at theta (in the XZ plane, perpendicular to Y):
            #      tang = (-sin(theta), 0, cos(theta))
            #    Radial at theta:
            #      rad  = ( cos(theta), 0, sin(theta))
            #    Roller length axis (body +Y) points 45deg between Y and tang:
            #      lax  = (-sin(theta)*s45, c45, cos(theta)*s45)
            #    Roller cross-section axes span the plane perpendicular to lax;
            #    pick the radial for body +X, and perpendicular for body +Z:
            #      body +X = rad
            #      body +Z = lax x rad  (right-handed)
            lax = (-st * s45, c45, ct * s45)
            rad = (ct, 0.0, st)
            # body +Z = rad x lax (this order gives det=+1, proper rotation).
            # Using lax x rad instead would be a reflection and render bodies
            # with inverted normals.
            zax = (rad[1] * lax[2] - rad[2] * lax[1],
                   rad[2] * lax[0] - rad[0] * lax[2],
                   rad[0] * lax[1] - rad[1] * lax[0])

            # Translation: roller center sits on the wheel perimeter at
            # angle theta, around the wheel center (mx, wcy, AXL).
            tx = mx + ring_r * ct
            ty = wcy
            tz = AXL + ring_r * st

            mat = _mat(rad, lax, zax, (tx, ty, tz))
            _move_body_transform(rc, body, mat)
    except Exception:
        pass  # rollers are decorative; tire+hub is enough if this fails


def _motor_assy(rc, tag, mx, fy, od):
    """Motor + mecanum wheel assembly hanging off the outer face of the frame.

    Rev12: simplified — removed the encoder disc, mounting bolts, and the
    `BrkV/BrkH` orange L-bracket construction (replaced with a cleaner single
    angle plate). Wheel now has real slanted rollers instead of 6 boxes.
    """
    wcy = fy + od * M_OFF
    _mecanum_wheel(rc, tag, mx, wcy)

    # Motor body sections hanging below the frame
    if od < 0:
        can_start = fy - M_CAN
        gbox_start = can_start - M_GBOX
    else:
        can_start = fy
        gbox_start = fy + M_CAN

    # Motor can — clean silver cylinder
    try: CY(rc, f'MCan_{tag}', mx, can_start, AXL, M_DIA/2, M_CAN, 'motor')
    except Exception:
        B(rc, f'MCan_{tag}', mx-M_DIA/2, can_start, AXL-M_DIA/2,
          M_DIA, M_CAN, M_DIA, 'motor')

    # Gearbox — narrower dark cylinder
    try: CY(rc, f'MGbx_{tag}', mx, gbox_start, AXL, M_GBOX_DIA/2, M_GBOX, 'gearbox')
    except Exception:
        B(rc, f'MGbx_{tag}', mx-M_GBOX_DIA/2, gbox_start, AXL-M_GBOX_DIA/2,
          M_GBOX_DIA, M_GBOX, M_GBOX_DIA, 'gearbox')

    # Shaft — thin silver cylinder reaching the wheel hub
    shaft_start = (gbox_start + M_GBOX) if od > 0 else (gbox_start - SHAFT)
    try: CY(rc, f'Shaft_{tag}', mx, shaft_start, AXL, SHAFT_D/2, SHAFT, 'hub')
    except Exception: pass

    # L-bracket clamping motor to the outer face of the lower-deck rail.
    # This IS a "connector" per the user's structural-focus guidance, so keep
    # both plates plus the visible mounting bolts. (Encoder PCB removed — it's
    # a non-structural detail that will come back when we have a real motor
    # STEP model with the encoder already integrated.)
    bk_top = LO
    bk_bot = AXL - M_DIA/2 - 0.1
    bk_h = bk_top - bk_bot
    if bk_h > 0:
        bk_y = fy - BRKT_T if od < 0 else fy
        # Vertical plate — mounts to the rail's outer T-slot face
        B(rc, f'BrkV_{tag}', mx-S/3, bk_y, bk_bot, S*2/3, BRKT_T, bk_h, 'bracket')
        # Horizontal shelf under the motor
        shelf_y = fy - M_CAN*0.6 if od < 0 else fy
        B(rc, f'BrkH_{tag}', mx-S/3, shelf_y, bk_bot-BRKT_T,
          S*2/3, M_CAN*0.6, BRKT_T, 'bracket')
        # 2 bolts through the vertical plate into the rail's T-nut
        bolt_side = bk_y - 0.15 if od < 0 else bk_y + BRKT_T + 0.15
        for bz in [bk_bot + bk_h*0.3, bk_bot + bk_h*0.7]:
            B(rc, f'Bolt_V_{tag}_{bz:.1f}',
              mx-0.2, bolt_side-0.05 if od<0 else bolt_side,
              bz-0.2, 0.4, 0.2, 0.4, 'bracket')
        # 1 bolt up through the shelf into the motor housing
        CZ(rc, f'Bolt_H_{tag}', mx, shelf_y+M_CAN*0.3 if od>0 else shelf_y-M_CAN*0.3,
           bk_bot-BRKT_T-0.1, 0.18, 0.2, 'bracket')

# ═══════════════════════════════════════════════════════════════════════════
# Wiring
# ═══════════════════════════════════════════════════════════════════════════
def _wiring(rc):
    """Wiring removed in rev13 — we'll add realistic cable runs once the
    electronics have real STEP models (from GrabCAD / manufacturers) with
    actual port positions to route to. The '6 - Wiring' sub-component is
    left as a no-op placeholder.
    """
    pass

# ═══════════════════════════════════════════════════════════════════════════
# Ground
# ═══════════════════════════════════════════════════════════════════════════
def _ground(rc):
    p = 3.0
    B(rc, 'Ground', -p, -M_OFF-p, -0.05, FRAME+2*p, FRAME+2*M_OFF+2*p, 0.05, 'ground')
