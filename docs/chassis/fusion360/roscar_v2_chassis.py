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
# =============================================================================
VERSION = 'rev9'

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


def _place(occ, tx=0, ty=0, tz=0, rot_axis=None, rot_angle=0):
    """Position an occurrence via rotation (about world origin) then translation."""
    m = adsk.core.Matrix3D.create()
    if rot_axis and rot_angle != 0:
        m.setToRotation(rot_angle, rot_axis, adsk.core.Point3D.create(0, 0, 0))
    t = adsk.core.Matrix3D.create()
    t.translation = adsk.core.Vector3D.create(tx, ty, tz)
    m.transformBy(t)
    occ.transform = m


def _scale_y(occ, target_len_cm):
    """Non-uniform scale an occurrence's bodies along Y to target length.

    The source extrusion STEP is 10.0 cm (100mm) along Y.
    Scale factor = target_len_cm / 10.0.
    Scale from component origin — position AFTER scaling.
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


def _clr_occ(occ, key):
    """Apply color to all bodies in an occurrence's component."""
    comp = occ.component
    for i in range(comp.bRepBodies.count):
        _clr(comp.bRepBodies.item(i), key)


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

        # Organized into named sub-components for clean browser tree.
        # All geometry uses XY plane only — safe with sub-components
        # (the XZ/YZ axis mapping bug only affects non-XY planes).
        _frame(_comp(rc, '1 - Frame'))
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
               f'WB: {HWB*20:.0f}mm | H: {(MST+MAST_H+LID_H)*10:.0f}mm')
        if _clog: msg += f'\nColor: {_clog[0]}'
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
# Frame (imported STEP models)
# ═══════════════════════════════════════════════════════════════════════════
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

    STEP extrusion: cross-section in XZ (centered at origin, ±1.5cm),
    length along +Y. Front/rear rails run along Y — no rotation.
    Left/right rails run along X — rotate -90deg around Z.
    """
    hs = S / 2  # 1.5 cm — half of 3030 profile
    z_axis = adsk.core.Vector3D.create(0, 0, 1)
    inner_len = FRAME - 2 * S  # left/right rails fit between front and rear

    for deck, z in [('Lo', LO), ('Hi', HI)]:
        # Front/Rear rails — along Y, full FRAME length
        _import_rail(rc, f'{deck}_Front', FRAME, hs, 0, z + hs)
        _import_rail(rc, f'{deck}_Rear', FRAME, FRAME - hs, 0, z + hs)

        # Left/Right rails — rotate -90 around Z so +Y becomes -X
        # Rail origin at (FRAME-S, y_center, z_center) extends in -X to x=S
        _import_rail(rc, f'{deck}_Left', inner_len, FRAME - S, hs, z + hs,
                     z_axis, -math.pi / 2)
        _import_rail(rc, f'{deck}_Right', inner_len, FRAME - S, FRAME - hs, z + hs,
                     z_axis, -math.pi / 2)


def _frame_posts(rc):
    """4 vertical corner posts.

    Rotate +90 around X so +Y becomes +Z (upward).
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
    _import_rail(rc, 'Mast', MAST_H, FRAME - S, FRAME / 2, MST,
                 x_axis, math.pi / 2)


def _frame_corners(rc):
    """8 corner brackets (imported STEP) at frame-post junctions.

    The STEP bracket has arms along +X, +Y, +Z. Rotate around Z to align
    arms with the rails/post meeting at each corner. Rotations may need
    adjustment after visual inspection — see risk mitigation in plan.
    """
    z_axis = adsk.core.Vector3D.create(0, 0, 1)

    # (corner_name, x, y, z_rotation_angle)
    corners = [
        ('FL', 0, 0, 0),
        ('FR', 0, FRAME, -math.pi / 2),
        ('RL', FRAME, 0, math.pi / 2),
        ('RR', FRAME, FRAME, math.pi),
    ]

    lo_z = LO + S   # lower junction (top of lower rail)
    hi_z = HI        # upper junction (bottom of upper rail)

    for name, cx, cy, angle in corners:
        for deck, z in [('lo', lo_z), ('hi', hi_z)]:
            occ = _import_step(rc, STEP_CORNER_BRACKET, f'CB_{deck}_{name}')
            _place(occ, cx, cy, z, z_axis if angle != 0 else None, angle)
            _clr_occ(occ, 'corner')


def _frame_tplate(rc):
    """T-plate bracket for lidar mast attachment to rear upper rail."""
    occ = _import_step(rc, STEP_TPLATE_M6, 'Mast_TPlate')
    _place(occ, FRAME - S, FRAME / 2, MST - 0.1)
    _clr_occ(occ, 'corner')

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
