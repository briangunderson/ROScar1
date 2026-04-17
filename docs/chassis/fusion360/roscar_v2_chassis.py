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
# =============================================================================
VERSION = 'rev11'

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
    'frame':    (70, 70, 75),
    'plate':    (160, 200, 240),
    'battery':  (30, 90, 200),
    'pcb':      (20, 80, 30),
    'rpi':      (45, 45, 50),
    'motor':    (195, 195, 200),
    'gearbox':  (60, 60, 65),
    'wheel':    (30, 30, 30),
    'roller':   (50, 50, 55),
    'lidar':    (25, 25, 30),
    'camera':   (55, 55, 60),
    'bracket':  (230, 120, 30),
    'corner':   (100, 100, 105),
    'ground':   (35, 37, 42),
    'hub':      (85, 85, 90),
    'wire_pwr': (180, 30, 30),
    'wire_usb': (40, 40, 45),
    'wire_enc': (30, 130, 30),
    'port':     (35, 35, 40),
    'standoff': (200, 200, 205),
    'encoder':  (25, 70, 25),
    'lens':     (150, 160, 180),
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
               f'Frame: 22 individual STEP imports')
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
    c = FRAME/2
    B(rc, 'LoPlate', c-INNER/2, c-INNER/2, LO+S, INNER, INNER, PLT_T, 'plate')
    B(rc, 'HiPlate', c-INNER/2, c-INNER/2, HI+S, INNER, INNER, PLT_T, 'plate')

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
    c = FRAME/2;  cz = LO + S + PLT_T

    # Battery
    B(rc, 'Battery', c-BAT[0]/2, c-BAT[1]/2, cz, BAT[0], BAT[1], BAT[2], 'battery')
    # Battery wire leads
    B(rc, 'BatWire+', c+BAT[0]/2, c-0.15, cz+BAT[2]*0.7, 1.5, 0.15, 0.15, 'wire_pwr')
    B(rc, 'BatWire-', c+BAT[0]/2, c+0.15, cz+BAT[2]*0.7, 1.5, 0.15, 0.15, 'wire_usb')

    # Motor board
    bx = FRAME * 0.7
    B(rc, 'MotorBoard', bx-BRD[0]/2, c-BRD[1]/2, cz, BRD[0], BRD[1], BRD[2], 'pcb')
    # USB port on board
    B(rc, 'Board_USB', bx-BRD[0]/2-0.2, c-0.4, cz+BRD[2], 0.9, 0.8, 0.35, 'port')
    # Connector headers
    for i in range(4):
        B(rc, f'Hdr_{i}', bx+BRD[0]/2-0.3, c-BRD[1]/2+0.8+i*1.2, cz+BRD[2], 0.3, 0.8, 0.5, 'wire_enc')

    # RPi5 in Argon NEO 5
    rz = HI + S + PLT_T
    B(rc, 'RPi5', c-RPI[0]/2, c-RPI[1]/2, rz, RPI[0], RPI[1], RPI[2], 'rpi')
    CZ(rc, 'RPi_Fan', c, c, rz+RPI[2], 1.5, 0.15, 'hub')
    # Ports on rear side (high-X side)
    px = c + RPI[0]/2
    B(rc, 'USB_A1', px, c-RPI[1]/2+0.5, rz+0.8, 0.3, 1.4, 0.7, 'port')
    B(rc, 'USB_A2', px, c-RPI[1]/2+2.2, rz+0.8, 0.3, 1.4, 0.7, 'port')
    B(rc, 'Ethernet', px, c-RPI[1]/2+4.0, rz+0.3, 0.3, 1.6, 1.3, 'port')
    B(rc, 'HDMI1', px, c+RPI[1]/2-2.5, rz+1.5, 0.3, 0.8, 0.35, 'port')
    B(rc, 'HDMI2', px, c+RPI[1]/2-1.2, rz+1.5, 0.3, 0.8, 0.35, 'port')

    # RPLIDAR C1
    lcx, lcy = FRAME-S, FRAME/2;  lz = MST + MAST_H
    bh = LID_H * 0.35;  hh = LID_H * 0.65
    B(rc, 'Lidar_Base', lcx-LID_SZ/2, lcy-LID_SZ/2, lz, LID_SZ, LID_SZ, bh, 'lidar')
    CZ(rc, 'Lidar_Head', lcx, lcy, lz+bh, LID_SZ/2*0.9, hh, 'lidar')
    CZ(rc, 'Lidar_Lens', lcx+LID_SZ/2*0.7, lcy, lz+bh+hh*0.3, 0.3, 0.1, 'lens')
    # Cable exit
    B(rc, 'Lidar_Cable', lcx-LID_SZ/2-0.8, lcy-0.15, lz+bh*0.3, 0.8, 0.3, 0.3, 'wire_usb')

    # Camera
    cam_z = HI + S/2
    B(rc, 'Cam_Body', -0.5, c-3.5, cam_z, 2.5, 7.0, 2.5, 'camera')
    CZ(rc, 'Cam_Lens', 0.5, c, cam_z+0.8, 0.8, 0.8, 'lidar')
    CZ(rc, 'Cam_Ring', 0.5, c, cam_z+1.5, 0.9, 0.15, 'hub')
    # Mount clip (wraps over top of rail)
    B(rc, 'Cam_Clip', -0.3, c-2.0, cam_z+2.5, 1.0, 4.0, 0.8, 'camera')
    B(rc, 'Cam_ClipR', -0.3, c-2.0, cam_z-0.5, 1.0, 4.0, 0.5, 'camera')

# ═══════════════════════════════════════════════════════════════════════════
# Drive: motors, wheels, brackets
# ═══════════════════════════════════════════════════════════════════════════
def _drive(rc):
    xf = FRAME/2 - HWB;  xr = FRAME/2 + HWB
    for tag, mx, fy, od in [('FL',xf,0,-1),('FR',xf,FRAME,1),('RL',xr,0,-1),('RR',xr,FRAME,1)]:
        _motor_assy(_comp(rc, f'Motor {tag}'), tag, mx, fy, od)

def _motor_assy(rc, tag, mx, fy, od):
    """Full motor+wheel assembly with mecanum wheel segments."""
    # ── Segmented mecanum wheel ──
    wcy = fy + od * M_OFF
    seg_total = WHL_W
    seg_w = seg_total / (N_SEGS + (N_SEGS-1)*SEG_GAP_RATIO)
    gap = seg_w * SEG_GAP_RATIO
    mid = (N_SEGS - 1) / 2.0

    for i in range(N_SEGS):
        y_off = i * (seg_w + gap)
        x_shift = (i - mid) * DIAG_SHIFT  # diagonal offset for mecanum look
        seg_y = wcy - WHL_W/2 + y_off
        # Each roller segment is slightly barrel-shaped (narrower than full diameter)
        barrel_shrink = 0.15  # rollers are slightly smaller than wheel OD
        B(rc, f'Roller_{tag}_{i}',
          mx - WHL_R + barrel_shrink + x_shift, seg_y, 0,
          (WHL_R - barrel_shrink)*2, seg_w, WHL_R*2, 'roller')

    # Hub cylinder (visible through gaps between rollers)
    try:
        CY(rc, f'Hub_{tag}', mx, wcy-WHL_W/2-0.05, AXL, WHL_R*0.35, WHL_W+0.1, 'hub')
    except:
        pass

    # ── Motor body: two sections (can + gearbox) hanging below frame ──
    if od < 0:
        can_y = fy - M_CAN          # motor can starts at frame face, goes outward
        gbox_y = can_y - M_GBOX     # gearbox continues outward
    else:
        can_y = fy                   # motor can starts at frame face
        gbox_y = can_y + M_CAN      # gearbox after can

    # Motor can (larger silver cylinder)
    try: CY(rc, f'MCan_{tag}', mx, can_y if od>0 else fy-M_CAN, AXL, M_DIA/2, M_CAN, 'motor')
    except: B(rc, f'MCan_{tag}', mx-M_DIA/2, can_y if od>0 else fy-M_CAN, AXL-M_DIA/2, M_DIA, M_CAN, M_DIA, 'motor')

    # Gearbox (slightly narrower, darker)
    try: CY(rc, f'MGbx_{tag}', mx, gbox_y if od>0 else gbox_y, AXL, M_GBOX_DIA/2, M_GBOX, 'gearbox')
    except: B(rc, f'MGbx_{tag}', mx-M_GBOX_DIA/2, gbox_y, AXL-M_GBOX_DIA/2, M_GBOX_DIA, M_GBOX, M_GBOX_DIA, 'gearbox')

    # Encoder disc (small green PCB at inboard end)
    enc_y = fy + (0.1 if od > 0 else -0.1)
    try: CY(rc, f'Enc_{tag}', mx, enc_y if od>0 else fy-M_CAN-0.3, AXL, M_DIA/2*0.8, 0.2, 'encoder')
    except: pass

    # ── Shaft ──
    shaft_y = (gbox_y + M_GBOX) if od > 0 else (gbox_y - SHAFT)
    try: CY(rc, f'Shaft_{tag}', mx, shaft_y if od>0 else gbox_y-SHAFT, AXL, SHAFT_D/2, SHAFT, 'hub')
    except: pass

    # ── L-bracket: drops from frame to motor ──
    bk_top = LO                    # bottom of lower rail
    bk_bot = AXL - M_DIA/2 - 0.1  # just above motor top
    bk_h = bk_top - bk_bot
    if bk_h > 0:
        # Vertical plate
        bk_y = fy - BRKT_T if od < 0 else fy
        B(rc, f'BrkV_{tag}', mx-S/3, bk_y, bk_bot, S*2/3, BRKT_T, bk_h, 'bracket')
        # Horizontal shelf under motor
        shelf_y = fy - M_CAN*0.6 if od < 0 else fy
        B(rc, f'BrkH_{tag}', mx-S/3, shelf_y, bk_bot-BRKT_T, S*2/3, M_CAN*0.6, BRKT_T, 'bracket')
        # Mounting bolts (two small cylinders)
        for bz in [bk_bot + bk_h*0.3, bk_bot + bk_h*0.7]:
            CZ(rc, f'Bolt_{tag}', mx, fy + od*(-0.1), bz-0.1, 0.15, BRKT_T+0.2, 'standoff')

# ═══════════════════════════════════════════════════════════════════════════
# Wiring
# ═══════════════════════════════════════════════════════════════════════════
def _wiring(rc):
    """Representative wire runs along frame rails."""
    c = FRAME/2
    # USB cable: motor board → up rear post → RPi
    B(rc, 'USB_vert', FRAME-S-0.5, c-0.1, LO+S, 0.25, 0.2, POST_H, 'wire_usb')
    # Power cable: battery → motor board
    B(rc, 'Pwr_run', c+BAT[0]/2, c, LO+S+PLT_T+BAT[2]*0.7, FRAME*0.2-BAT[0]/2, 0.2, 0.2, 'wire_pwr')
    # Lidar USB: down mast → along upper rail → down post → RPi
    B(rc, 'LidarUSB_mast', FRAME-S-0.3, c+0.3, HI+S, 0.2, 0.2, MST-HI-S+MAST_H*0.3, 'wire_usb')
    B(rc, 'LidarUSB_rail', S, c+0.3, HI+S-0.3, FRAME-2*S, 0.2, 0.2, 'wire_usb')

# ═══════════════════════════════════════════════════════════════════════════
# Ground
# ═══════════════════════════════════════════════════════════════════════════
def _ground(rc):
    p = 3.0
    B(rc, 'Ground', -p, -M_OFF-p, -0.05, FRAME+2*p, FRAME+2*M_OFF+2*p, 0.05, 'ground')
