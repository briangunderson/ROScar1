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
#                       - Palette refinement: frame color changed to black
#                         anodized (35,37,42) to match physical stock; motor
#                         L-brackets back to orange (pop against black frame).
#                       - Fixed _check_centered() false-positive by reading
#                         component-local body bbox instead of post-transform
#                         occurrence bbox.
#   rev14   2026-04-17  Real component STEP imports begin:
#                       - Raspberry Pi 5 STEP from raspberrypi.com replaces the
#                         procedural RPi5 block when the file is present
#                       - RPLIDAR C1 STEP from slamtec.com replaces the
#                         procedural Lidar_Base + Lidar_Head blocks
#                       - Both imports are conditional: script falls back to
#                         the procedural placeholder if the STEP file is
#                         missing (e.g. fresh checkout without running the
#                         download step).
#                       - Placement is an initial guess — may need small
#                         translation/rotation offsets after visual check
#                         depending on each STEP's native origin reference.
#   rev15   2026-04-17  Fix 'floating objects' bug from rev14:
#                       - Pi5 and RPLIDAR STEPs were being imported into the
#                         '4 - Electronics' sub-component, where
#                         occurrence.transform silently fails (the parent
#                         must be the active edit target — root at script
#                         start). Both STEPs ended up stuck at origin (0,0,0)
#                         while the frame and rest of the model were up in
#                         the chassis — looked like they were 'floating'.
#                       - _components() now takes a root arg; STEP imports
#                         go to root, procedural placeholders stay in the
#                         Electronics sub-component.
#                       - Frame color (95,100,110) dark graphite — the pure
#                         black (35,37,42) from rev13 blended with Fusion's
#                         dark blue background; upper rails + posts + mast
#                         were invisible.
#   rev16   2026-04-17  Give up matching physical black stock for CAD view.
#                       Dark frame colors (black, graphite) consistently
#                       blend with Fusion's viewport and make structural
#                       elements look absent. Switched to bright anodized
#                       aluminum silver (175,180,190) for the frame so rails,
#                       posts, and mast read clearly from any angle. Darker
#                       silver (130,135,148) for corner brackets so they
#                       remain distinguishable from the aluminum rails.
#                       Physical-color matching can be done via Fusion
#                       appearance overrides for presentation renders.
#   rev17   2026-04-17  Hide decorative stub extrusions in corner bracket
#                       STEP imports. The GrabCAD bracket assembly includes
#                       2 short pieces of 3030 profile per bracket that were
#                       physically overlapping with the real posts and rails
#                       at each corner — making the structural extrusions
#                       look 'absent' in visual inspection. New helper
#                       _hide_bracket_stubs() walks the imported component
#                       tree and toggles isLightBulbOn=False on any sub-
#                       occurrence whose name contains 'profile', 'extrusion',
#                       or 't-slot'. Grub screws and the bracket body stay
#                       visible. (Partial success — logic still needs work
#                       to reliably find stub bodies in nested assemblies.)
#   rev18   2026-04-17  Corner brackets to brass/gold (205,160,75) — they
#                       were blending with aluminum rails in silver and
#                       making it hard to tell bracket from rail. Brass
#                       is maximally visually distinct and reads as a
#                       commodity hardware color.
#   rev19   2026-04-17  Rewrote _hide_bracket_stubs() to use body SIZE as
#                       the primary filter instead of component names (which
#                       weren't reliably matching due to STEP importer
#                       flattening). Now hides any body > 5cm in any
#                       bounding-box dimension — that captures stub
#                       extrusions (100mm) while keeping the bracket body
#                       (30mm) and grub screws (10mm) visible. Also still
#                       does name matching as a backup for nested
#                       sub-occurrences.
#   rev22   2026-04-18  THE import bug: importToTarget2's returned
#                       ObjectCollection was giving back the FIRST-ever
#                       imported occurrence on each call (not the newly
#                       created one), which meant every _place_occ() call
#                       after the very first was re-positioning the same
#                       occurrence. The 21 other frame pieces stayed at
#                       identity (origin), stacked on top of each other —
#                       the user reported 'almost all of the aluminum frame
#                       pieces are in the exact same position'. Fix:
#                       track target_comp.occurrences.count before/after
#                       the import and use occurrences.item(count_after-1)
#                       to grab the newly appended occurrence.
#   rev23   2026-04-18  Motor L-bracket engineering polish + observability:
#                       - Added a gusset reinforcement block to each of
#                         the 4 motor L-brackets: a 20x20x3mm rib in the
#                         inside corner of the L, bracing the vertical
#                         plate against the horizontal shelf. Makes each
#                         bracket look like an engineered (stiff) part
#                         rather than two floppy flat plates.
#                       - Position verification: _frame() now appends
#                         an entry to _clog for each rail / post / mast
#                         with its world-space translation. Shown in the
#                         final message box so the user can confirm rev22
#                         really placed all 22 pieces at distinct
#                         positions. (Catches future regressions early.)
#                       - Cleaned up dead code in _import_step: the old
#                         else-branch was a copy-paste duplicate of the
#                         if-branch. Now raises RuntimeError if Fusion's
#                         import doesn't actually add an occurrence (was
#                         silently falling through before).
#   rev24   2026-04-18  THE REAL fix: user tested rev23 and reported 'all
#                       aluminum extrusion pieces occupying the same
#                       space' — even though the dialog said '22 unique
#                       positions OK'. The dialog was LYING: it was
#                       reporting the translation tuples we *intended*
#                       to pass, not the actual post-set transforms.
#                       Root cause: `importManager.importToTarget2()`
#                       silently changes the active edit target to the
#                       newly imported sub-component. Immediately after
#                       the import, root is no longer the active target
#                       — and Fusion's `occurrence.transform` setter
#                       silently fails when the occurrence's parent
#                       isn't the active edit target. Rev22 correctly
#                       identified the right occurrence but then asked
#                       a no-op setter to move it.
#                       Two-part fix:
#                       1) `_place_occ` now calls `rootComponent.activate()`
#                          (or parent-occurrence activate for nested ones)
#                          before the transform assignment.
#                       2) After the assignment we READ BACK the actual
#                          `occ.transform.translation` and compare to the
#                          intended one. If they disagree, we fall back
#                          to `_place_via_move` — a MoveFeature-based
#                          relocation that isn't subject to the
#                          active-target restriction.
#                       Verification in the final dialog now reports the
#                       actual post-set positions (not the intended
#                       tuples), so we can never again be misled by a
#                       silent failure.
# =============================================================================
#   rev25   2026-04-18  Diagnostic instrumentation. rev24 still left
#                       all 22 imports at (0,0,0) even though _clog was
#                       empty (no exceptions from either setter or
#                       MoveFeature fallback). Adds per-call print()
#                       in _place_occ reporting: pre/post transform
#                       translations, entityToken, which component was
#                       activated, whether the setter raised, whether
#                       the immediate post-set read matched intent,
#                       whether MoveFeature was used and its result.
#                       The bridge captures stdout, so we can see
#                       every call's outcome and isolate which step
#                       is silently failing.
# =============================================================================
VERSION = 'rev25'

# Setting this to False before calling run() suppresses the final
# summary ui.messageBox (a modal dialog that blocks on the main thread).
# The bridge add-in flips this for automated runs so it can return
# results without a human having to click OK. When the script is run
# manually via Fusion's Scripts and Add-Ins dialog, leave it True so
# the user still sees the summary.
SHOW_MSGBOX = True

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
    # rev16: frame color abandoned the "matches physical black stock" goal
    # and went full aluminum silver. In the viewport, dark colors blended
    # into shadows and users reported parts looking 'missing' or 'floating'.
    # The CAD model is a DESIGN tool; matching physical color can be done
    # manually via appearance overrides for presentation renders. For working
    # in Fusion, bright anodized aluminum gives the clearest structural read.
    'frame':    (175, 180, 190),   # bright anodized aluminum silver
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
    'bracket':  (230, 120, 30),    # orange (motor L-brackets) — pops against frame
    'corner':   (218, 135, 25),    # bold brass/copper — needs to pop vs tan plates AND silver rails
    'ground':   (85, 90, 98),      # warm gray floor
    'hub':      (135, 140, 150),   # wheel hub
    'standoff': (185, 190, 200),   # standoffs silver-ish
    'lens':     (180, 190, 210),   # camera/lidar lens glass
    # Kept for backward compatibility:
    'wire_pwr': (180, 30, 30), 'wire_usb': (40, 40, 45), 'wire_enc': (30, 130, 30),
    'port':     (35, 35, 40), 'encoder':  (25, 70, 25),
}

_cc = {};  _clog = [];  _frame_positions = []

# ═══════════════════════════════════════════════════════════════════════════
# STEP model paths (absolute — this script runs locally in Fusion 360)
# ═══════════════════════════════════════════════════════════════════════════
_MODELS = r'D:\localrepos\ROScar1\docs\chassis\models'
STEP_EXTRUSION = os.path.join(_MODELS, 'extrusions', '30X30 - T-slot - Aluminium Profile.step')
STEP_CORNER_BRACKET = os.path.join(_MODELS, 'brackets', '3030 - 3-way corner bracket.step')
STEP_TPLATE_M6 = os.path.join(_MODELS, 'brackets', 'T-Plate 3030 5 Holes - M6.step')
# Non-structural (optional — script falls back to placeholder blocks if absent)
STEP_RPI5 = os.path.join(_MODELS, 'electronics', 'RaspberryPi5.step')
STEP_RPLIDAR_C1 = os.path.join(_MODELS, 'electronics', 'rplidar-c1.stp')

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
    """Import a STEP file into target_comp. Returns the new Occurrence.

    CRITICAL: uses the target_comp.occurrences count before/after the
    import to identify the new occurrence. `importToTarget2`'s returned
    ObjectCollection has been observed to return the FIRST-ever imported
    occurrence (not the newest) when called multiple times — which meant
    every _place_occ() call after the first one was setting the transform
    on the SAME occurrence, leaving all subsequent imports at identity
    (origin). All the aluminum frame pieces ended up stacked at (0,0,0)
    while only one got positioned correctly. Tracking count fixes this.
    """
    app = adsk.core.Application.get()
    mgr = app.importManager
    opts = mgr.createSTEPImportOptions(step_path)
    opts.isViewFit = False

    count_before = target_comp.occurrences.count
    mgr.importToTarget2(opts, target_comp)
    count_after = target_comp.occurrences.count

    if count_after <= count_before:
        # Fusion's STEP import didn't add any occurrence — this should be
        # impossible for a valid STEP file. Raise instead of silently using
        # a stale occurrence (which was the rev14-21 bug).
        raise RuntimeError(
            f'STEP import added no occurrences: {step_path}')

    # New occurrences are appended at the end of the collection.
    occ = target_comp.occurrences.item(count_after - 1)
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
    """Position an existing occurrence using column vectors + translation.

    CRITICAL (rev24): Fusion's `occurrence.transform` setter silently fails
    if the occurrence's parent component isn't the currently active edit
    target. `importManager.importToTarget2()` has been observed to change
    the active target to the newly imported sub-component — so by the time
    we call this for a freshly imported occurrence, root (the occurrence's
    parent) is no longer active. The assignment gets silently rejected and
    the occurrence stays at identity (origin). That's what caused the user
    to report 'all aluminum extrusion pieces occupying the same space'
    even AFTER the rev22 occurrence-identification fix — the right
    occurrence was being targeted, but the setter was a no-op.

    Fix: explicitly activate the occurrence's parent component before
    assigning transform. For occurrences directly under root (which is
    all frame imports), that means re-activating the root component.

    We also read back the actual post-assignment transform to detect
    silent failures. If the identity transform is still in effect after
    our call, we fall back to creating a MoveFeature, which isn't subject
    to the active-target restriction.
    """
    mat = _mat(c0, c1, c2, t)
    name = occ.component.name if occ.component else '?'

    # --- DIAGNOSTIC (rev25): capture pre-state ---
    try:
        pre_tr = occ.transform.translation
        pre = (pre_tr.x, pre_tr.y, pre_tr.z)
    except Exception as e:
        pre = f'PRE_READ_FAILED: {e}'
    try:
        token = occ.entityToken[:16] + '...'
    except Exception:
        token = '?'

    try:
        app = adsk.core.Application.get()
        des = adsk.fusion.Design.cast(app.activeProduct)
        ac = occ.assemblyContext  # None if occ is directly under root
        if ac is None:
            des.rootComponent.activate()
            active_kind = 'root'
        else:
            try:
                ac.activate(); active_kind = 'parent'
            except Exception:
                des.rootComponent.activate(); active_kind = 'root(fallback)'
    except Exception as e:
        active_kind = f'activate_failed:{e}'

    setter_raised = None
    try:
        occ.transform = mat
    except Exception as e:
        setter_raised = str(e)
        _clog.append(f'_place_occ[{name}] transform set raised: {e}')

    # --- DIAGNOSTIC: post-setter read ---
    try:
        post_tr = occ.transform.translation
        post = (post_tr.x, post_tr.y, post_tr.z)
    except Exception as e:
        post = f'POST_READ_FAILED: {e}'

    setter_ok = (
        isinstance(post, tuple)
        and abs(post[0] - t[0]) <= 1e-3
        and abs(post[1] - t[1]) <= 1e-3
        and abs(post[2] - t[2]) <= 1e-3
    )
    move_raised = None
    move_post = None
    if not setter_ok and setter_raised is None:
        # Silent failure — fall back to MoveFeature
        move_raised = _place_via_move(occ, mat)
        try:
            mp = occ.transform.translation
            move_post = (mp.x, mp.y, mp.z)
        except Exception as e:
            move_post = f'MOVE_POST_READ_FAILED: {e}'

    # Emit a single line of diagnostics per call. Goes to stdout which
    # the bridge captures.
    print(
        f'_place_occ name={name} token={token} want={tuple(round(v,3) for v in t)} '
        f'pre={pre} active={active_kind} setter_raised={setter_raised} '
        f'post={post} setter_ok={setter_ok} '
        f'move_raised={move_raised} move_post={move_post}')


def _place_via_move(occ, mat):
    """Move an occurrence using a MoveFeature instead of the transform
    setter. Works even when the parent component isn't the active edit
    target. Uses the occurrence's current transform as the 'from' state
    and computes the delta matrix to reach `mat`.

    Returns None on success, an error string on failure — for diagnostics.
    """
    try:
        app = adsk.core.Application.get()
        des = adsk.fusion.Design.cast(app.activeProduct)
        rc = des.rootComponent
        cur = occ.transform
        cur_inv = cur.copy()
        if not cur_inv.invert():
            return 'current transform not invertible'
        delta = mat.copy()
        delta.transformBy(cur_inv)

        coll = adsk.core.ObjectCollection.create()
        coll.add(occ)
        mi = rc.features.moveFeatures.createInput2(coll)
        mi.defineAsFreeMove(delta)
        rc.features.moveFeatures.add(mi)
        return None
    except Exception as e:
        return str(e)


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

    IMPORTANT: check the BODY's bounding box in component-local space, NOT
    the occurrence's bounding box in world space. occ.boundingBox would be
    post-transform (after we've already translated the rail to its world
    position) and would always falsely flag non-origin positions.
    """
    try:
        comp = occ.component
        if comp.bRepBodies.count == 0:
            return
        bb = comp.bRepBodies.item(0).boundingBox
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

        # Reset per-run globals (module stays loaded across runs in Fusion,
        # so lists would accumulate stale entries from previous invocations).
        _clog.clear()
        _frame_positions.clear()

        # Frame imports go into ROOT (not a sub-component) because
        # occurrence.transform only works when the parent is the active edit
        # target. Procedural parts (plates, electronics, etc.) still use
        # sub-components for browser tree organization.
        _frame(rc)
        _plates(_comp(rc, '2 - Deck Plates'))
        _standoffs(_comp(rc, '3 - Standoffs'))

        # Electronics: pass BOTH the sub-component (for procedural placeholder
        # shapes) and root (for STEP imports — occurrence.transform requires
        # the occurrence's parent to be the active edit target at script
        # start, which is always root).
        elec = _comp(rc, '4 - Electronics')
        _components(elec, rc)

        _drive(_comp(rc, '5 - Drive System'))
        _wiring(_comp(rc, '6 - Wiring'))
        _ground(_comp(rc, '7 - Ground Plane'))

        app.activeViewport.fit()
        trk = FRAME + 2*M_OFF

        # rev24: position verification — read back the ACTUAL post-transform
        # position of every imported occurrence, NOT the intended tuple. In
        # rev23 we trusted the tuples we passed in, but the setter was
        # silently failing and the pieces stayed at identity. Reading back
        # `occ.transform.translation` tells us what truly landed.
        nfp = len(_frame_positions)
        actual = []
        for nm, intended, occ in _frame_positions:
            try:
                tr = occ.transform.translation
                actual.append((nm, (tr.x, tr.y, tr.z), intended))
            except Exception:
                actual.append((nm, (None, None, None), intended))
        unique_actual = {pos for _, pos, _ in actual if None not in pos}
        mismatches = [
            (nm, got, want)
            for nm, got, want in actual
            if None in got or any(abs(g - w) > 1e-3 for g, w in zip(got, want))
        ]
        placement_ok = (len(unique_actual) == nfp) and not mismatches

        if actual:
            xs = [p[0] for _, p, _ in actual if p[0] is not None]
            ys = [p[1] for _, p, _ in actual if p[1] is not None]
            zs = [p[2] for _, p, _ in actual if p[2] is not None]
            if xs:
                bbox = (f'X:[{min(xs):.1f},{max(xs):.1f}]  '
                        f'Y:[{min(ys):.1f},{max(ys):.1f}]  '
                        f'Z:[{min(zs):.1f},{max(zs):.1f}] cm')
            else:
                bbox = '(no readable positions)'
        else:
            bbox = '(no frame imports)'

        status = 'OK' if placement_ok else f'FAIL ({len(mismatches)} mismatches)'
        msg = (f'ROScar1 v2 ({VERSION})\n'
               f'Frame: {FRAME*10:.0f}mm | Track: {trk*10:.0f}mm\n'
               f'WB: {HWB*20:.0f}mm | H: {(MST+MAST_H+LID_H)*10:.0f}mm\n'
               f'Cut plan: 4x248mm + 4x188mm + 4x100mm posts + 1x120mm mast\n'
               f'Real STEPs: RPi5 + RPLIDAR C1\n\n'
               f'Frame placement check (ACTUAL post-set positions):\n'
               f'  {nfp} STEP imports, {len(unique_actual)} unique {status}\n'
               f'  bbox {bbox}')
        if mismatches:
            # Show the first 3 mismatches inline so the user sees what went wrong.
            msg += '\nFirst mismatches (want vs got):'
            for nm, got, want in mismatches[:3]:
                msg += f'\n  {nm}: want {want}, got {got}'
        if _clog: msg += f'\nLog: {_clog[0]}'
        # Always print so the bridge can capture it; only popup a modal
        # dialog when a human is running the script interactively.
        print(msg)
        if SHOW_MSGBOX:
            ui.messageBox(msg)
    except:
        tb = traceback.format_exc()
        print(f'FAIL ({VERSION}):\n{tb}')
        if ui and SHOW_MSGBOX:
            ui.messageBox(f'FAIL ({VERSION}):\n{tb}')

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
    _frame_positions.append((name, t, occ))
    return occ


def _import_bracket(rc, step_path, name, c0, c1, c2, t, color='corner',
                    hide_stubs=True):
    """Import a bracket STEP and position it. No scaling — brackets are fixed size.

    The 3-way corner bracket STEP assembly from GrabCAD includes decorative
    'stub extrusions' (100mm pieces of 3030 profile showing how the bracket
    connects to rails). Those stubs physically overlap with our actual posts
    and rails in the assembled model, making the real frame hard to see.
    hide_stubs=True toggles off the visibility of any sub-occurrence whose
    component name mentions 'profile' or 'extrusion' (case-insensitive).
    Grub screws stay visible so the bracket still looks like a real bracket.
    """
    occ = _import_step(rc, step_path, name)
    _clr_occ(occ, color)
    _place_occ(occ, c0, c1, c2, t)
    _frame_positions.append((name, t, occ))
    if hide_stubs:
        _hide_bracket_stubs(occ.component)
    return occ


def _hide_bracket_stubs(comp, max_body_len_cm=8.0):
    """Turn off visibility of any body that's too long to be a bracket body
    or grub screw — those are the decorative stub extrusions that overlap
    the real structural rails/posts.

    Real 3-way corner bracket body is <= 30x30x30 mm (3cm max).
    M6 grub screws are <= 10mm long.
    Stub extrusions are ~100mm (10cm) long.
    T-plates are up to 85mm in their long dimension.
    Threshold default of 8cm safely hides only stubs (10cm) while keeping
    bracket bodies, grub screws, AND T-plates visible.

    Walks the imported component tree recursively — also checks bodies in
    nested sub-occurrences (Fusion's STEP importer puts assembly parts in
    sub-components).
    """
    try:
        # Hide direct bodies over the length threshold
        for i in range(comp.bRepBodies.count):
            body = comp.bRepBodies.item(i)
            try:
                bb = body.boundingBox
                dx = bb.maxPoint.x - bb.minPoint.x
                dy = bb.maxPoint.y - bb.minPoint.y
                dz = bb.maxPoint.z - bb.minPoint.z
                if max(dx, dy, dz) > max_body_len_cm:
                    body.isLightBulbOn = False
            except Exception:
                pass
        # Recurse into sub-components
        for i in range(comp.occurrences.count):
            child = comp.occurrences.item(i)
            # Also name-match for good measure (some importers flatten bodies
            # into sub-occurrences)
            name = (child.component.name or '').lower()
            if 'profile' in name or 'extrusion' in name or 't-slot' in name:
                child.isLightBulbOn = False
            else:
                _hide_bracket_stubs(child.component, max_body_len_cm)
    except Exception:
        pass


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
def _components(rc, root=None):
    """Non-structural components as simple placeholders (rev13+).

    Procedural placeholder geometry (battery/board/camera) is created IN the
    Electronics sub-component (rc). Real STEP imports, however, must go into
    ROOT (not the sub-component) because `occurrence.transform` only works
    when the occurrence's parent is the active edit target at script start
    — which is always root. STEPs imported into a sub-component silently
    land at origin instead of their placement transform.

    That was the rev14 bug the user saw as "objects floating in the air" —
    the Pi5 and RPLIDAR STEPs ended up stuck at (0,0,0) while everything
    else was up in the chassis.

    Pass root=None to place STEP imports in rc too (legacy behaviour).
    """
    if root is None:
        root = rc
    c = FRAME/2;  cz = LO + S + PLT_T

    # Battery — simple box on lower deck
    B(rc, 'Battery', c-BAT[0]/2, c-BAT[1]/2, cz, BAT[0], BAT[1], BAT[2], 'battery')

    # Motor board — plain PCB block on lower deck
    bx = FRAME * 0.7
    B(rc, 'MotorBoard', bx-BRD[0]/2, c-BRD[1]/2, cz, BRD[0], BRD[1], BRD[2], 'pcb')

    # ── RPi5: import real STEP if available, else fall back to placeholder ──
    rz = HI + S + PLT_T
    if os.path.exists(STEP_RPI5):
        # Real Raspberry Pi 5 CAD from raspberrypi.com. PCB is 85 x 56 mm.
        # Imported into ROOT so occurrence.transform actually places it
        # where we ask. Initial guess: PCB origin at upper-deck center.
        try:
            rpi_occ = _import_step(root, STEP_RPI5, 'RPi5')
            _clr_occ(rpi_occ, 'rpi')
            _place_occ(rpi_occ, _X, _Y, _Z, (c - 4.25, c - 2.8, rz))
        except Exception as e:
            _clog.append(f'RPi5 STEP import failed: {e}')
            B(rc, 'RPi5', c-RPI[0]/2, c-RPI[1]/2, rz, RPI[0], RPI[1], RPI[2], 'rpi')
    else:
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

    # ── RPLIDAR C1: real STEP or placeholder ──
    lz = mp_z + mp_t
    if os.path.exists(STEP_RPLIDAR_C1):
        # Real RPLIDAR C1 CAD from slamtec.com. Body is 55.6x55.6x41.3mm.
        # Imported into ROOT so the placement transform actually applies.
        try:
            lid_occ = _import_step(root, STEP_RPLIDAR_C1, 'RPLIDAR_C1')
            _clr_occ(lid_occ, 'lidar')
            _place_occ(lid_occ, _X, _Y, _Z, (lcx - LID_SZ/2, lcy - LID_SZ/2, lz))
        except Exception as e:
            _clog.append(f'RPLIDAR STEP import failed: {e}')
            bh = LID_H * 0.35; hh = LID_H * 0.65
            B(rc, 'Lidar_Base', lcx-LID_SZ/2, lcy-LID_SZ/2, lz, LID_SZ, LID_SZ, bh, 'lidar')
            CZ(rc, 'Lidar_Head', lcx, lcy, lz+bh, LID_SZ/2*0.9, hh, 'lidar')
    else:
        bh = LID_H * 0.35; hh = LID_H * 0.65
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

        # --------------------------------------------------------------
        # Reinforcement gusset (rev23). A rib that braces the vertical
        # plate against the horizontal shelf, dramatically stiffening
        # the bracket against torque from the motor cantilevered below.
        # Modeled as a rectangular block in the inside corner of the L
        # — structurally equivalent stiffness to the triangular gussets
        # a 3D-printed bracket would have, but simpler to model without
        # non-XY plane sketches. For a real printed part, this block
        # would become a triangular fillet in the slicer.
        # --------------------------------------------------------------
        gusset_w = min(M_CAN * 0.5, 2.0)      # ~20mm Y
        gusset_h = min(bk_h * 0.35, 2.5)      # ~20-25mm Z
        gusset_t = 0.3                          # 3mm X (thin rib)
        # Place the rib centered on the bracket plate width (X=mx)
        gx = mx - gusset_t / 2
        # Rib sits in the *inside* of the L, against whichever side of the
        # vertical plate faces away from the rail.
        if od < 0:
            g_y0 = bk_y - gusset_w              # extend -Y from plate
        else:
            g_y0 = bk_y + BRKT_T                # extend +Y from plate
        B(rc, f'BrkG_{tag}', gx, g_y0, bk_bot,
          gusset_t, gusset_w, gusset_h, 'bracket')

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
    """Small ground plane — just big enough to catch the wheel contact
    patches. Previously extended way beyond the chassis which made the
    robot look small in the viewport."""
    # Extends 1cm beyond the wheels' outer edge on each side.
    p = 1.0
    B(rc, 'Ground',
      -p, -M_OFF - WHL_W/2 - p, -0.05,
      FRAME + 2*p, FRAME + 2*M_OFF + WHL_W + 2*p, 0.05,
      'ground')
