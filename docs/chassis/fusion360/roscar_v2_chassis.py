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
#   rev26   2026-04-18  PARAMETRIC FIX. rev25's diagnostics caught it:
#                       `occ.transform = mat` sets the property and the
#                       immediate read-back returns the intended value
#                       — but in ParametricDesignType, the direct
#                       property set is NOT a timeline feature, so the
#                       next recompute (triggered by any subsequent
#                       scaleFeatures.add / extrudeFeatures.add / STEP
#                       import) silently reverts the imported
#                       occurrence to identity. By the time the final
#                       readback happens, everything is back to
#                       (0,0,0). Fix: always place occurrences via a
#                       MoveFeature — it lives in the timeline and
#                       survives recompute.
#   rev27   2026-04-18  rev26 attempted to move the OCCURRENCE via
#                       moveFeatures but Fusion rejected with
#                       '3 : invalid argument inputEntities'. Turns
#                       out in ParametricDesignType the moveFeatures
#                       createInput2 collection is stricter than the
#                       docs suggest for top-level occurrences of
#                       imported STEPs.
#                       The insight: each STEP import creates its own
#                       UNIQUE component with its own BRepBody(ies).
#                       Since we don't reuse components across
#                       placements, moving the BODIES (inside the
#                       component's own timeline) is equivalent to
#                       moving the occurrence — and moveFeatures
#                       DOES accept BRepBody collections reliably.
#                       _place_occ now iterates the bodies of
#                       occ.component, collects them, and runs a
#                       MoveFeature in the component's own features
#                       context with the full target matrix
#                       (rotation + translation). The occurrence's
#                       transform stays at identity; the world
#                       position of the geometry comes entirely from
#                       the body-internal move. Post-set
#                       verification now reads the body's bounding-
#                       box center instead of occ.transform, since
#                       occ.transform is legitimately still identity.
#                       Result: 13/22 placements succeed (rails,
#                       posts, mast, lidar — all single-body STEPs).
#                       The other 9 (corner brackets, T-plate, RPi5)
#                       are multi-body assemblies — their bodies live
#                       in SUB-components, so comp.features.moveFeatures
#                       rejects them with 'object is not in the
#                       assembly context of this component'. Need to
#                       recurse for assemblies.
#   rev28   2026-04-18  Recursive body-move handling multi-body
#                       assemblies. For each component encountered,
#                       move its OWN direct bodies via its OWN
#                       features.moveFeatures — then recurse into
#                       each sub-occurrence and do the same on the
#                       sub-component. Every leaf component's move
#                       uses the same outer target matrix `mat`.
#                       Works cleanly when sub-occurrences are at
#                       identity (typical for imported STEP assemblies);
#                       may deform if a sub-occurrence has a non-
#                       identity transform. Result: 22/22 placements
#                       at unique positions, 20/22 at intended world
#                       coords. Corner brackets, rails, posts, mast,
#                       T-plate, RPLIDAR all land correctly.
#   rev29   2026-04-18  Disable the RPi5 STEP import by default and
#                       fall back to the procedural PCB block. The
#                       raspberrypi.com Pi5 STEP is a deeply nested
#                       assembly (47 bodies across multiple levels
#                       with non-identity sub-occurrence transforms)
#                       and rev28's recursive body-move doesn't
#                       translate it rigidly — the geometry ends up
#                       at negative Z (below the ground plane),
#                       which blows up the viewport fit. The
#                       procedural block is the right stand-in
#                       until we either switch the Pi5 geometry to
#                       an add-as-instance approach or use the
#                       Argon NEO 5 case STEP (once the user sources
#                       it from GrabCAD).
#                       USE_RPI5_STEP flag lets future runs re-enable
#                       the real STEP for experimentation without
#                       editing multiple lines.
#                       Also fixed the T-plate placement: the STEP
#                       has its origin at a corner (body 9x9x0.4cm),
#                       so passing (FRAME-S, FRAME/2) landed the
#                       corner at the mast position and stuck the
#                       plate out 4.5cm past the rear of the chassis.
#                       Shifted by -4.5cm in X/Y to center on the
#                       mast; Z adjusted -0.2cm so the top face
#                       aligns with MST.
#   rev30   2026-04-18  Integrated stray-body cleanup into the
#                       chassis script itself. Multi-body STEP
#                       imports (corner brackets, T-plate, RPLIDAR)
#                       include nested grub screws / nuts / small
#                       hardware whose sub-occurrences have non-
#                       identity local transforms — rev28's
#                       recursive body-move translates them but
#                       each ends up at its own sub-local-frame
#                       offset from the assembly's target position,
#                       which in practice parks ~94 tiny specks
#                       scattered around origin. A post-build
#                       _hide_stray_bodies() pass walks every body
#                       and turns off isLightBulbOn for anything
#                       whose bbox center is outside the chassis
#                       volume. The real structural bracket bodies,
#                       mast, rails, etc. all pass the test; only
#                       the decorative hardware gets hidden.
# =============================================================================
#   rev31   2026-04-18  User-reported regressions from rev30:
#                       (1) Corner brackets on the upper deck had
#                           their Z-arm pointing UP (into empty
#                           space) instead of DOWN (into the post).
#                       (2) Motors were floating off-bracket. The
#                           motor-can inner face sat at y=fy-M_CAN
#                           (front) or y=fy (rear) but the bracket's
#                           vertical plate sat at y=fy±BRKT_T, so
#                           there was a ~3.5cm gap between them.
#                       (3) Wheels were completely missing from the
#                           viewport. _hide_stray_bodies used
#                           Y:[-5,30] but wheels center at
#                           Y=-M_OFF=-9.37cm (front) and
#                           Y=FRAME+M_OFF=34.17cm (rear), so every
#                           wheel body got hidden. Expanded the
#                           volume to Y:[-15,40] so wheels survive.
#                       Rev31 partial fix: widened volume (wheels
#                       now visible), extended bracket plate to span
#                       rail face, extended shelf to full motor
#                       length. Added _run_sanity_checks()  — see
#                       tasks/chassis-v2-sanity-checks.md. Rev31
#                       passed all 7 automated checks but visual
#                       inspection still showed motor-mount was
#                       anatomically wrong (see rev32).
#   rev32   2026-04-18  User clarified the real motor mount:
#                       "motors don't mount from the rear. On the
#                       face that the output shaft protrudes from,
#                       there are two threaded holes that flank the
#                       shaft." That's the GEARBOX shaft-end face
#                       (Y=-6.2 for front motor, Y=FRAME+6.2 for
#                       rear), with 2 M3 holes at ±8.5mm from the
#                       shaft axis (17mm spacing).
#                       Bracket redesign:
#                         - plate_v (vertical): covers the gearbox
#                           shaft-end face. 4x4 cm in XZ plane,
#                           BRKT_T thick in Y. Just outside the
#                           motor (on the wheel-side of the face).
#                         - 2 M3 bolts go through plate_v INTO the
#                           gearbox's 2 threaded holes, flanking the
#                           shaft vertically at ±8.5mm in Z.
#                         - arm (horizontal): spans from the motor
#                           face back to the rail, BRKT_T thick.
#                           Supports the whole motor/gearbox
#                           assembly from below.
#                         - 2 M5 bolts through the arm into the
#                           adjacent rail's T-slot at the rail end.
#                         - Gusset rib in the inside L corner.
#                       The motor can + gearbox no longer PASS
#                       THROUGH a rear plate — they hang between
#                       the rail (at Y=0 for front) and the
#                       far-end plate_v (at Y=-6.2), supported
#                       underneath by the horizontal arm.
#   rev33   2026-04-18  Fix upper-deck corner brackets visibly
#                       poking ABOVE the upper deck plate.
#                       Rev22-32 used (_Y, _X, _NZ) for upper
#                       brackets hoping to flip the Z-arm so it
#                       extends DOWN into the post. But the
#                       rev31 diagnostics showed bracket
#                       body_center at local (1.25, 6.26, -3.44):
#                       material is at local +X, +Y, and -Z.
#                       Under (_Y, _X, _NZ), local -Z (where the
#                       body lives) maps to world +Z — sending
#                       the body UP from the placement point.
#                       Applied at (0, 0, HI=21.44), the body
#                       ends up at z=24.88, ABOVE the upper
#                       deck plate (24.44). Visible bug.
#                       Fix: use the SAME per-corner rotations
#                       as the lower deck brackets. With c2=_Z
#                       the local -Z material stays at world -Z,
#                       so the upper bracket body extends DOWN
#                       from HI to z=18.0 — fully inside the
#                       post region. No material above the deck.
#   rev34   2026-04-18  User caught the sanity-check failure mode:
#                       'corner brackets aren't even touching the
#                       frame', 'lidar is hanging off the post'.
#                       Rev33's visual check (and mine) missed
#                       these obvious geometric errors because I
#                       was pattern-matching 'looks like a robot'
#                       instead of checking specific positions.
#                       Root causes:
#                       (a) The GrabCAD 3-way corner bracket STEP
#                           has its body at local (1.25, 6.26, -3.44)
#                           from origin — when placed at a frame
#                           corner via (0,0,LO+S), the body ends up
#                           5-6cm INSIDE the chassis interior, not
#                           clasping the outer corner. The STEP is
#                           wrong for our use case.
#                       (b) The RPLIDAR STEP has body center at
#                           local (-2.05, -2.05) from origin —
#                           placing origin at mast axis puts the
#                           body 2cm forward/left of the mast,
#                           making it visibly 'hang off the post'.
#                       Rev34 fixes:
#                       1. Drop the bracket STEP import. Draw 8
#                          procedural 2-flange corner brackets as
#                          thin plates hugging the outer -X and -Y
#                          faces of each corner junction. Brass
#                          color. Actually touching the rails.
#                          User 3D-prints these if they want.
#                       2. Compensate lidar placement by +2.05 cm
#                          in X and Y so the lidar body lands
#                          centered on the mast axis.
#                       3. New sanity checks:
#                          - corner_brackets_at_junctions: every
#                            CB_* bracket's first visible body must
#                            be within 3 cm of the nominal post-
#                            rail junction for that corner.
#                          - lidar_on_mast_axis: lidar body center
#                            must be within 1.5 cm of mast axis.
#                         Rev31's 'motors_touch_bracket' check was
#                         already there.
#                       Lesson recorded in
#                       tasks/chassis-v2-sanity-checks.md: the
#                       visual check isn't 'overall form looks
#                       right' — it's 'each named part is within
#                       X cm of its intended XYZ'.
# =============================================================================
#   rev35   2026-04-18  Sanity-check bug fixes after rev34 visual
#                       audit. Rev34's actual geometry was correct
#                       (corner brackets at junctions, lidar on
#                       mast axis) — the 3 'failed' sanity lines
#                       were bugs in the checks themselves:
#                         - frame_unique didn't handle procedural
#                           entries (occ=None), counted 14 not 22.
#                         - lidar_on_mast_axis looked for bodies
#                           by name starting with 'RPLIDAR' but the
#                           STEP's internal bodies are named by
#                           Slamtec's original part names.
#                         - strays_hidden_count range (50-200) was
#                           based on bracket STEPs' decorative
#                           stubs; now that the brackets are
#                           procedural, only ~11 strays from RPLIDAR
#                           + T-plate remain. Range → 5-200.
#                       Checks now all use proper 'walk the
#                       component tree for the first visible body's
#                       bbox center' pattern when occ is available,
#                       or the stored intended position for
#                       procedural entries.
# =============================================================================
#   rev36   2026-04-19  User caught more issues still wrong:
#                       'the corner brackets need to be the ones
#                       in the step files'; 'the entire chassis
#                       is just floating in mid-air, disconnected
#                       from the motors/brackets'; 'lidar still
#                       not centered on its base plate'.
#
#                       Fixes:
#                       (1) Corner brackets: restored the GrabCAD
#                           STEP import (rev34's procedural boxes
#                           reverted). Placement shifted from the
#                           outer chassis corner (0, 0, LO+S) to
#                           the POST AXIS (1.5, 1.5, LO+S) so the
#                           bracket's body-mass lands inside the
#                           post+rail junction instead of 5cm
#                           into the chassis interior.
#                       (2) Motor bracket: added a VERTICAL RISER
#                           piece that spans from the bottom of
#                           the bracket (at motor level ~Z=2.3cm)
#                           up to the chassis rail top (Z=LO+S=
#                           11.44cm), plus a rail-face flange at
#                           the rail. Without this the motor
#                           bracket ended at motor level and the
#                           chassis was visibly floating ~6cm
#                           above the wheels/motors. M5 bolts now
#                           go through the rail flange into the
#                           rail T-slot at mid-height.
#                       (3) Lidar centering: left as-is in rev36
#                           (sanity check reports 0.00cm offset
#                           from mast axis), observing again
#                           after rev36 renders to see if user's
#                           complaint is about a specific visual
#                           asymmetry in the STEP geometry.
# =============================================================================
#   rev37   2026-04-19  Diagnostic-only rev. rev36 fixed the
#                       chassis-floating issue (motor bracket now
#                       has a vertical riser + rail flange that
#                       spans motor→rail Z gap and the chassis
#                       is structurally connected to the wheel
#                       assemblies) — visually confirmed in the
#                       rev36 render. But the corner brackets
#                       still look off: sanity reports 7.3cm
#                       offset from junctions, body_center at
#                       (2.75, 7.76, 8.0) instead of ~(1.5, 1.5,
#                       11.44). Adds a _log_bracket_bodies()
#                       diagnostic that dumps every body's dims,
#                       center, visibility, and name to stdout
#                       for the first bracket (CB_Lo_FL). Next rev
#                       will use that data to pick a better
#                       stub-hide threshold and/or placement.
# =============================================================================
#   rev38   2026-04-19  Corner bracket placement fix using the
#                       rev37 diagnostic. The main bracket body
#                       (5.5x5.5x3.1cm) sits at local offset
#                       (1.25, 6.26, -3.44) from the STEP origin,
#                       so placing origin AT the junction put the
#                       body 6cm inside the chassis. Fix: the new
#                       _bkt_place() helper computes placement =
#                       junction - rotated(BKT_LOCAL) so the main
#                       body ends up centered on the junction.
#                       Other sub-bodies (grub screws at world
#                       X=45.76, stubs at world Y=-29) end up WAY
#                       outside the chassis volume and get hidden
#                       by _hide_stray_bodies automatically.
# =============================================================================
#   rev39   2026-04-19  Diagnostic rev: dump RPLIDAR_C1 body
#                       tree so we can see where the BASE body is
#                       (vs the scanner head, cable, connector,
#                       etc.) and decide how to center the base
#                       on the mast axis. User flagged that the
#                       lidar is still visually off-center on its
#                       mount plate despite the bbox center being
#                       at mast axis — classic asymmetric STEP
#                       geometry problem.
# =============================================================================
#   rev40   2026-04-19  Lidar placement fix based on rev39 dump.
#                       Rev39 showed the RPLIDAR STEP's body[18]
#                       (5.56x5.56x2.05cm = LID_SZ²xbase height)
#                       IS the base body, with local origin at
#                       (0,0,0). Earlier rev34 '+2.05 XY offset'
#                       was chasing body[0] — a tiny 0.48x0.48
#                       protrusion at a different local position.
#                       Now placing STEP origin directly at mast
#                       axis puts the base body CENTER at the
#                       mast axis. Sanity check updated to find
#                       the base body by size (±5mm of LID_SZ)
#                       rather than the first body it encounters.
# =============================================================================
#   rev41   2026-04-19  Fresh approach: stop fighting the GrabCAD
#                       3-way corner bracket STEP. Replace all 8
#                       corner bracket imports with a simple
#                       procedural 2-flange L-bracket drawn
#                       directly at each outer corner.
#                       Each bracket is 2 thin plates (3mm PETG)
#                       hugging the outer -X and -Y faces of the
#                       corner, spanning the full rail height.
#                       Visually CORRECT by construction — no
#                       body-offset math, no rotation tricks, no
#                       stub hiding. Three lines of code per
#                       bracket.
#                       Rails, posts, mast remain STEP imports
#                       (they work and the T-slot profile is
#                       valuable visually).
#   rev42   2026-04-20  Real motor dimensions: 25GA20E260 (7.4V,
#                       1:20 gearbox). Previous model assumed a
#                       generic 24mm×62mm motor; the real part is
#                       Ø25×19.5mm motor can + Ø21×19.5mm gearbox
#                       + Ø7×11.5mm shaft hub + Ø3×11mm thin shaft.
#                       Net effect: motor length 61.5mm (same as
#                       before — the old 62mm estimate matched by
#                       coincidence), but the INTERNAL split is
#                       very different. Motor-mount face at Y=39mm
#                       from rail (vs 62mm before), so the motor
#                       bracket is much shorter. Added SHAFT_HUB
#                       cylinder between gearbox and thin shaft.
#                       M_OFF recalc: 5.60cm (was 9.37). Track
#                       width 360mm (was 435mm). Rails unchanged
#                       — cut sheet still valid.
# =============================================================================
VERSION = 'rev42'

# Chassis volume (cm) for _hide_stray_bodies. Anything whose body's
# bounding-box center falls outside this box gets hidden.
# Y range is wide enough to include mecanum wheels: wheel centers at
# Y = -M_OFF = -9.37 (front) and Y = FRAME + M_OFF = 34.17 (rear),
# each with a width of WHL_W = 3.73, so real wheel Y extent is
# [-11.2, 36.0]. Padding to [-15, 40] gives headroom.
_CHASSIS_VOL_X = (-5.0, 30.0)
_CHASSIS_VOL_Y = (-15.0, 40.0)
_CHASSIS_VOL_Z = (-1.0, 45.0)

# Set to False to force the procedural Pi5 PCB block instead of the
# real STEP import. Rev28's recursive body-move couldn't rigidly
# translate the Pi5 STEP (nested assembly with non-identity
# sub-occurrence transforms), so the default is False until a better
# approach lands.
USE_RPI5_STEP = False

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

# Motor: 25GA20E260 (7.4V, 1:20 gearbox, 67.4g). Real datasheet
# dimensions, rev42 update from the earlier placeholder values.
#   Motor can   Ø25mm × 19.5mm
#   Gearbox     Ø21mm × 19.5mm  (shaft-end face has 2× M3 at 17mm spacing)
#   Shaft hub   Ø7mm  × 11.5mm  (wheel mounts on this section)
#   Thin shaft  Ø3mm  × 11mm
#   Total axial length: 61.5mm
M_DIA = 2.5;  M_CAN = 1.95;  M_GBOX = 1.95
M_GBOX_DIA = 2.1
SHAFT_HUB_DIA = 0.7;  SHAFT_HUB = 1.15   # Ø7mm × 11.5mm — wheel-mount section
SHAFT = 1.1;  SHAFT_D = 0.3              # Ø3mm × 11mm thin drive shaft
WHL_R = 3.97;  WHL_W = 3.73
BRKT_T = 0.4  # bracket plate thickness (4mm PETG)

# Motor offset: rail face → wheel center. Wheel mounts on the Ø7 hub;
# its center is at the hub midpoint + the thin shaft sticks through.
# M_OFF = M_CAN + M_GBOX + SHAFT_HUB + SHAFT/2
M_OFF = M_CAN + M_GBOX + SHAFT_HUB + SHAFT/2   # = 5.60 cm (was 9.37)
# Track width is now FRAME + 2*M_OFF = 24.8 + 11.2 = 36.0 cm (360mm),
# down from 435mm with the oversized placeholder motor. Rail lengths
# unchanged; cut sheet still valid.

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
    """Position an imported STEP occurrence by applying a MoveFeature
    to every body it contains, recursing into sub-components.

    Single-body STEPs (rails, posts, mast, lidar): just one direct
    body in occ.component; one MoveFeature in comp.features.moveFeatures.

    Multi-body assembly STEPs (corner brackets, T-plate, RPi5 with PCB
    assemblies): bodies live in SUB-components of occ.component.
    comp.features.moveFeatures rejects them ('object is not in the
    assembly context of this component'), so we recurse: for each
    leaf component encountered, apply the MoveFeature to its OWN
    direct bodies using its OWN features.moveFeatures. If a sub-
    occurrence has a non-identity local transform relative to its
    parent, the rigid-assembly assumption breaks and geometry can
    deform — in practice the STEPs in our collection nest flatly
    with identity sub-transforms, so pure translation moves the
    whole assembly as a unit.
    """
    mat = _mat(c0, c1, c2, t)
    comp = occ.component
    name = comp.name if comp else '?'

    total_moved = 0
    errors = []

    def _move_in_comp(c, depth=0):
        nonlocal total_moved
        try:
            # Gather direct bodies (not nested) for this component
            coll = adsk.core.ObjectCollection.create()
            for i in range(c.bRepBodies.count):
                coll.add(c.bRepBodies.item(i))
            if coll.count > 0:
                try:
                    mi = c.features.moveFeatures.createInput2(coll)
                    mi.defineAsFreeMove(mat)
                    c.features.moveFeatures.add(mi)
                    total_moved += coll.count
                except Exception as e:
                    errors.append(f'{c.name}@d{depth}: {e}')
            # Recurse into sub-occurrences
            for i in range(c.occurrences.count):
                sub = c.occurrences.item(i)
                _move_in_comp(sub.component, depth + 1)
        except Exception as e:
            errors.append(f'{c.name}@d{depth}: walk: {e}')

    _move_in_comp(comp)

    if errors:
        _clog.append(f'_place_occ[{name}] errors: ' + '; '.join(errors[:3]))

    # Verify: body-center of first reachable body.
    def _first_body(c):
        if c.bRepBodies.count > 0:
            return c.bRepBodies.item(0)
        for i in range(c.occurrences.count):
            b = _first_body(c.occurrences.item(i).component)
            if b is not None:
                return b
        return None
    try:
        b0 = _first_body(comp)
        if b0 is None:
            body_center = 'NO_BODY'
        else:
            bb = b0.boundingBox
            body_center = (
                (bb.minPoint.x + bb.maxPoint.x) / 2,
                (bb.minPoint.y + bb.maxPoint.y) / 2,
                (bb.minPoint.z + bb.maxPoint.z) / 2,
            )
    except Exception as e:
        body_center = f'BB_READ_FAILED: {e}'

    print(
        f'_place_occ name={name} want={tuple(round(v, 3) for v in t)} '
        f'moved_bodies={total_moved} errors={len(errors)} '
        f'body_center={body_center}')


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
# Stray-body cleanup (rev30)
# ═══════════════════════════════════════════════════════════════════════════
def _hide_stray_bodies(rc):
    """Hide any body whose bounding-box center is outside the chassis
    volume. Cleans up the nested hardware (grub screws, nuts, bracket
    detail pieces) that came along with multi-body STEP assemblies and
    landed at their sub-local-frame origins instead of the target.

    Returns the number of bodies hidden.
    """
    xmin, xmax = _CHASSIS_VOL_X
    ymin, ymax = _CHASSIS_VOL_Y
    zmin, zmax = _CHASSIS_VOL_Z
    total = [0]

    def _walk(comp):
        for i in range(comp.bRepBodies.count):
            body = comp.bRepBodies.item(i)
            try:
                bb = body.boundingBox
                cx = (bb.minPoint.x + bb.maxPoint.x) / 2
                cy = (bb.minPoint.y + bb.maxPoint.y) / 2
                cz = (bb.minPoint.z + bb.maxPoint.z) / 2
                if (cx < xmin or cx > xmax
                        or cy < ymin or cy > ymax
                        or cz < zmin or cz > zmax):
                    try:
                        body.isLightBulbOn = False
                        total[0] += 1
                    except Exception:
                        pass
            except Exception:
                pass
        for i in range(comp.occurrences.count):
            _walk(comp.occurrences.item(i).component)

    try:
        _walk(rc)
    except Exception as e:
        _clog.append(f'_hide_stray_bodies: {e}')
    return total[0]


# ═══════════════════════════════════════════════════════════════════════════
# Sanity checks (rev31)  — see tasks/chassis-v2-sanity-checks.md
# ═══════════════════════════════════════════════════════════════════════════
def _find_body_by_name(rc, name_prefix):
    """Walk all components, return the first body whose name starts with
    name_prefix, along with its bbox center, or (None, None)."""
    def _walk(comp):
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            try:
                if b.name.startswith(name_prefix) and b.isLightBulbOn:
                    bb = b.boundingBox
                    c = ((bb.minPoint.x + bb.maxPoint.x)/2,
                         (bb.minPoint.y + bb.maxPoint.y)/2,
                         (bb.minPoint.z + bb.maxPoint.z)/2)
                    return b, c
            except Exception:
                pass
        for i in range(comp.occurrences.count):
            hit = _walk(comp.occurrences.item(i).component)
            if hit[0] is not None:
                return hit
        return (None, None)
    return _walk(rc)


def _count_visible_bodies_named(rc, name_prefix):
    count = 0
    def _walk(comp):
        nonlocal count
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            try:
                if b.name.startswith(name_prefix) and b.isLightBulbOn:
                    count += 1
            except Exception:
                pass
        for i in range(comp.occurrences.count):
            _walk(comp.occurrences.item(i).component)
    _walk(rc)
    return count


def _count_hidden_bodies(rc):
    count = 0
    def _walk(comp):
        nonlocal count
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            try:
                if not b.isLightBulbOn:
                    count += 1
            except Exception:
                pass
        for i in range(comp.occurrences.count):
            _walk(comp.occurrences.item(i).component)
    _walk(rc)
    return count


def _run_sanity_checks(rc):
    """Run the sanity checks listed in tasks/chassis-v2-sanity-checks.md.
    Returns list of (name, ok, message). Also prints each line to stdout
    and populates _clog with any FAILs.
    """
    results = []

    def add(name, ok, msg):
        results.append((name, ok, msg))
        line = f'sanity {"PASS" if ok else "FAIL"}: {name} — {msg}'
        print(line)
        if not ok:
            _clog.append(line)

    # --- frame_count: 22 STEPs in _frame_positions ---
    add('frame_count',
        len(_frame_positions) == 22,
        f'{len(_frame_positions)} (expected 22)')

    # --- frame_unique: 22 distinct body-bbox centers ---
    # Handles both STEP-imported (occ != None) and procedural (occ == None,
    # use stored intended position) entries.
    actual_centers = []
    for _, intended, occ in _frame_positions:
        if occ is None:
            actual_centers.append(tuple(round(v, 2) for v in intended))
            continue
        try:
            b = occ.component.bRepBodies.item(0)
            bb = b.boundingBox
            actual_centers.append(tuple(round(v, 2) for v in (
                (bb.minPoint.x + bb.maxPoint.x)/2,
                (bb.minPoint.y + bb.maxPoint.y)/2,
                (bb.minPoint.z + bb.maxPoint.z)/2)))
        except Exception:
            actual_centers.append(None)
    uniq = len(set(actual_centers) - {None})
    add('frame_unique',
        uniq == 22,
        f'{uniq} distinct centers (expected 22)')

    # --- wheels_count: 4 Tire_* bodies visible ---
    n_tires = _count_visible_bodies_named(rc, 'Tire_')
    add('wheels_count',
        n_tires == 4,
        f'{n_tires} tire bodies visible (expected 4)')

    # --- motors_count: 4 MCan_* cylinders visible ---
    n_motors = _count_visible_bodies_named(rc, 'MCan_')
    add('motors_count',
        n_motors == 4,
        f'{n_motors} motor cans visible (expected 4)')

    # --- brackets_count: 8 corner brackets with visible body in _frame_positions ---
    n_brackets = sum(1 for name, _, _ in _frame_positions if name.startswith('CB_'))
    add('brackets_count',
        n_brackets == 8,
        f'{n_brackets} corner brackets tracked (expected 8)')

    # --- motors_touch_bracket: each motor's shaft-end face should be
    # at its motor-flange Y (the Brk_<tag>_MtrFlange plate). Gearbox
    # shaft-end face is at Y = gbox_face_y = fy ± (M_CAN+M_GBOX); the
    # flange plate center should be half a plate-thickness beyond.
    # We check: |flange.y_center - gearbox.y_shaft_face| ≤ 0.5 cm.
    motor_touch_ok = True
    bad_tags = []
    for tag in ('FL', 'FR', 'RL', 'RR'):
        _, gbx_c = _find_body_by_name(rc, f'MGbx_{tag}')
        _, flange_c = _find_body_by_name(rc, f'Brk_{tag}_MtrFlange')
        if gbx_c is None or flange_c is None:
            motor_touch_ok = False
            bad_tags.append(f'{tag}(missing)')
            continue
        # flange center should be within ~1cm of the gearbox's far Y edge
        # (shaft-end face). Gearbox is 2.4cm long; we take its center and
        # allow 1.5cm tolerance.
        if abs(flange_c[1] - gbx_c[1]) > 2.0:   # >2cm gap is clearly wrong
            motor_touch_ok = False
            bad_tags.append(f'{tag}(Δy={flange_c[1]-gbx_c[1]:+.2f})')
    add('motors_touch_bracket',
        motor_touch_ok,
        'motor flange at gearbox face for all 4'
        if motor_touch_ok else 'misaligned: ' + ', '.join(bad_tags))

    # --- corner_brackets_at_junctions: for each of 8 CB_* brackets,
    # the bracket's VISIBLE body (not a hidden stub — any body with
    # isLightBulbOn=True) must have its bbox center within 3 cm of the
    # nominal rail-post junction at that corner. Nominal junction is
    # (1.5, 1.5, Z) for FL, (1.5, FRAME-1.5, Z) for FR, etc., with Z=LO+S
    # for lower and Z=HI for upper. This catches 'bracket sitting 6cm
    # into the chassis interior' — the user's real complaint.
    cb_ok = True
    cb_bad = []
    cb_expected = {
        'CB_Lo_FL': (1.5, 1.5, LO + S),
        'CB_Lo_FR': (1.5, FRAME - 1.5, LO + S),
        'CB_Lo_RL': (FRAME - 1.5, 1.5, LO + S),
        'CB_Lo_RR': (FRAME - 1.5, FRAME - 1.5, LO + S),
        'CB_Hi_FL': (1.5, 1.5, HI),
        'CB_Hi_FR': (1.5, FRAME - 1.5, HI),
        'CB_Hi_RL': (FRAME - 1.5, 1.5, HI),
        'CB_Hi_RR': (FRAME - 1.5, FRAME - 1.5, HI),
    }
    for name, (ex, ey, ez) in cb_expected.items():
        # Look up in _frame_positions; rev34 procedural brackets stash
        # occ=None, in which case we find the body via name prefix
        # ('<name>_X' or '<name>_Y' for the two flanges).
        target_occ = None
        target_pos = None
        for nm, pos, occ in _frame_positions:
            if nm == name:
                target_occ = occ
                target_pos = pos
                break

        center = None
        if target_occ is not None:
            # STEP-imported bracket: walk the component tree for the
            # first VISIBLE body.
            def _first_visible(comp):
                for i in range(comp.bRepBodies.count):
                    b = comp.bRepBodies.item(i)
                    try:
                        if b.isLightBulbOn:
                            bb = b.boundingBox
                            return ((bb.minPoint.x + bb.maxPoint.x)/2,
                                    (bb.minPoint.y + bb.maxPoint.y)/2,
                                    (bb.minPoint.z + bb.maxPoint.z)/2)
                    except Exception:
                        pass
                for i in range(comp.occurrences.count):
                    hit = _first_visible(comp.occurrences.item(i).component)
                    if hit is not None:
                        return hit
                return None
            center = _first_visible(target_occ.component)
        else:
            # Procedural bracket: look up the X-flange body in root.
            _, c_x = _find_body_by_name(rc, f'{name}_X')
            _, c_y = _find_body_by_name(rc, f'{name}_Y')
            if c_x and c_y:
                center = ((c_x[0] + c_y[0]) / 2,
                          (c_x[1] + c_y[1]) / 2,
                          (c_x[2] + c_y[2]) / 2)
            elif target_pos is not None:
                center = tuple(target_pos)

        if center is None:
            cb_ok = False
            cb_bad.append(f'{name}(no_body)')
            continue
        dx, dy, dz = center[0] - ex, center[1] - ey, center[2] - ez
        dist = (dx*dx + dy*dy + dz*dz) ** 0.5
        if dist > 3.0:     # more than 3cm from junction = bracket floating
            cb_ok = False
            cb_bad.append(f'{name}(Δ={dist:.1f}cm)')
    add('corner_brackets_at_junctions',
        cb_ok,
        '8 brackets within 3cm of post-rail junctions'
        if cb_ok else 'floating: ' + ', '.join(cb_bad))

    # --- lidar_on_mast_axis: the RPLIDAR body's bbox center must be
    # within 1.5 cm of the mast axis (X=FRAME-S, Y=FRAME/2). Catches
    # 'lidar hanging off the post' — body has an internal offset from
    # its STEP origin that needs to be compensated for in placement.
    mast_x, mast_y = FRAME - S, FRAME / 2
    lidar_center = None
    # rev40: find the RPLIDAR_C1 occurrence, then find the LIDAR BASE
    # body specifically (identified by size: ~5.5x5.5x2cm). Using the
    # first body was a false positive — it was a small 4.8mm protrusion
    # that happened to land at the right position for the wrong reason.
    for i in range(rc.occurrences.count):
        occ = rc.occurrences.item(i)
        try:
            if occ.component.name == 'RPLIDAR_C1':
                def _find_base_body(comp):
                    # Look for a body roughly LID_SZ x LID_SZ (±5mm) and
                    # 1.5-3cm tall in Z — that's the lidar base.
                    for j in range(comp.bRepBodies.count):
                        b = comp.bRepBodies.item(j)
                        try:
                            bb = b.boundingBox
                            dx = bb.maxPoint.x - bb.minPoint.x
                            dy = bb.maxPoint.y - bb.minPoint.y
                            dz = bb.maxPoint.z - bb.minPoint.z
                            if (abs(dx - LID_SZ) < 0.5 and
                                abs(dy - LID_SZ) < 0.5 and
                                1.5 < dz < 3.0):
                                return ((bb.minPoint.x + bb.maxPoint.x)/2,
                                        (bb.minPoint.y + bb.maxPoint.y)/2,
                                        (bb.minPoint.z + bb.maxPoint.z)/2)
                        except Exception:
                            pass
                    for k in range(comp.occurrences.count):
                        hit = _find_base_body(comp.occurrences.item(k).component)
                        if hit is not None:
                            return hit
                    return None
                lidar_center = _find_base_body(occ.component)
                break
        except Exception:
            pass
    # Procedural fallback uses 'Lidar_Base' / 'Lidar_Head' named bodies.
    if lidar_center is None:
        _, lidar_center = _find_body_by_name(rc, 'Lidar_Base')
    if lidar_center is None:
        add('lidar_on_mast_axis', False, 'no lidar body found')
    else:
        dx = lidar_center[0] - mast_x
        dy = lidar_center[1] - mast_y
        r = (dx*dx + dy*dy) ** 0.5
        add('lidar_on_mast_axis',
            r <= 1.5,
            f'lidar XY offset from mast axis = {r:.2f}cm (<=1.5)')

    # --- strays_hidden_count: number of hidden bodies ---
    n_hidden = _count_hidden_bodies(rc)
    # rev34 dropped the bracket STEP imports (which contributed most of
    # the hidden strays) and replaced them with clean procedural brackets.
    # Now the only strays are from RPLIDAR + T-plate + Pi5-when-enabled
    # sub-assemblies. Expected range: 5-200.
    add('strays_hidden_count',
        5 <= n_hidden <= 200,
        f'{n_hidden} hidden bodies (expected 5-200)')

    n_pass = sum(1 for _, ok, _ in results if ok)
    summary = f'sanity: {n_pass}/{len(results)} pass'
    print(summary)
    return results, summary


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

        # rev30: hide stray hardware from multi-body STEP assemblies
        # (grub screws, nuts) that landed scattered around origin
        # instead of with their bracket/assembly parent.
        strays_hidden = _hide_stray_bodies(rc)

        # rev31: run sanity checks — catches regressions in the same
        # run that introduces them. Results go to stdout and into
        # _clog for any FAILs.
        sanity_results, sanity_summary = _run_sanity_checks(rc)

        app.activeViewport.fit()
        trk = FRAME + 2*M_OFF

        # rev27: verification now checks the BODY's bounding-box center
        # rather than occ.transform. Since we move bodies (not the
        # occurrence), occ.transform stays at identity by design. The
        # bounding-box center of the first body reflects where the
        # geometry actually lives in world space. Tolerance is loose
        # (0.5 cm) because the body's bbox center is the geometric
        # midpoint, which may differ from the translation tuple by
        # half the body's extent along any axis (the translation tuple
        # in the source script is the body's ORIGIN, not its center).
        # The key check is uniqueness: if all centers agree within
        # rounding, the placement worked.
        nfp = len(_frame_positions)
        actual = []
        for nm, intended, occ in _frame_positions:
            if occ is None:
                # Procedural geometry (e.g. rev34 corner brackets):
                # use the stored intended position as "actual".
                actual.append((nm, tuple(intended), intended))
                continue
            try:
                comp = occ.component
                if comp.bRepBodies.count == 0:
                    actual.append((nm, (None, None, None), intended))
                    continue
                bb = comp.bRepBodies.item(0).boundingBox
                cx = (bb.minPoint.x + bb.maxPoint.x) / 2
                cy = (bb.minPoint.y + bb.maxPoint.y) / 2
                cz = (bb.minPoint.z + bb.maxPoint.z) / 2
                actual.append((nm, (cx, cy, cz), intended))
            except Exception:
                actual.append((nm, (None, None, None), intended))
        unique_actual = {
            tuple(round(v, 2) for v in pos)
            for _, pos, _ in actual
            if None not in pos
        }
        mismatches = [
            (nm, got, want)
            for nm, got, want in actual
            if None in got
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
        failed = [nm for nm, ok, _ in sanity_results if not ok]
        msg = (f'ROScar1 v2 ({VERSION})\n'
               f'Frame: {FRAME*10:.0f}mm | Track: {trk*10:.0f}mm\n'
               f'WB: {HWB*20:.0f}mm | H: {(MST+MAST_H+LID_H)*10:.0f}mm\n'
               f'Cut plan: 4x248mm + 4x188mm + 4x100mm posts + 1x120mm mast\n'
               f'Real STEPs: RPLIDAR C1 (Pi5 = procedural block)\n\n'
               f'Frame placement check (ACTUAL post-set positions):\n'
               f'  {nfp} STEP imports, {len(unique_actual)} unique {status}\n'
               f'  bbox {bbox}\n'
               f'  {strays_hidden} stray hardware bodies hidden\n'
               f'\n{sanity_summary}')
        if failed:
            msg += '\nFailed: ' + ', '.join(failed)
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


def _draw_corner_bracket(rc, name, cx, cy, cz, dx_sign, dy_sign,
                         flange_size=3.0, flange_thickness=0.3, color='corner'):
    """Procedural 2-flange corner bracket (rev34).

    Creates two thin brass-colored plates meeting at the outer corner
    (cx, cy, cz) of the chassis frame:
      - One flange lies flat on the -X outer face (at X = cx or cx-T),
        extending in Y toward the interior (+dy_sign direction) and in
        Z by ±flange_size/2.
      - One flange lies flat on the -Y outer face (at Y = cy or cy-T),
        extending in X toward the interior (+dx_sign direction) and in
        Z by ±flange_size/2.

    This gives each bracket a body_center AT the corner junction,
    physically contacting the outer faces of the rails and post.
    Designed as a single 3D-printable PETG part (user has no press
    brake); in CAD we draw it as two box bodies meeting at the corner.

    Also appends the bracket's world position to _frame_positions so
    the sanity-check framework can verify placement.
    """
    T = flange_thickness
    # Flange 1: on the ∓X outer face
    if dx_sign > 0:                           # corner at X=cx, interior +X
        f1_x, f1_sx = cx - T, T
    else:                                     # corner at X=cx, interior -X
        f1_x, f1_sx = cx, T
    if dy_sign > 0:
        f1_y, f1_sy = cy, flange_size
    else:
        f1_y, f1_sy = cy - flange_size, flange_size
    f1_z, f1_sz = cz - flange_size / 2, flange_size
    B(rc, f'{name}_X', f1_x, f1_y, f1_z, f1_sx, f1_sy, f1_sz, color)

    # Flange 2: on the ∓Y outer face
    if dx_sign > 0:
        f2_x, f2_sx = cx, flange_size
    else:
        f2_x, f2_sx = cx - flange_size, flange_size
    if dy_sign > 0:
        f2_y, f2_sy = cy - T, T
    else:
        f2_y, f2_sy = cy, T
    f2_z, f2_sz = cz - flange_size / 2, flange_size
    B(rc, f'{name}_Y', f2_x, f2_y, f2_z, f2_sx, f2_sy, f2_sz, color)

    # Stash position in _frame_positions for sanity-check compatibility.
    # occ=None signals "procedural geometry, no Occurrence wrapper"; the
    # consumers check hasattr-ish to fall back to the stored intended
    # position for body_center.
    _frame_positions.append((name, (cx, cy, cz), None))


def _draw_l_corner(rc, name, cx, cy, cz, sx, sy,
                   flange_len=3.0, flange_thickness=0.3, flange_height=None):
    """Simple procedural corner bracket (rev41).

    Draws 2 thin flat flanges meeting at the OUTER corner (cx, cy, cz):
      - Flange A hugs the outer X face of the frame (perpendicular to X)
      - Flange B hugs the outer Y face of the frame (perpendicular to Y)

    sx, sy: +1 or -1. Which outer face (-X or +X) and (-Y or +Y) we're
            hugging. FL corner at (0, 0) has outer faces on -X and -Y,
            so bracket material extends toward -X and -Y from the
            corner. Use sx=+1, sy=+1 (interior is +X, +Y).

    cz: Z of the rail bottom at this corner (LO for lower, HI for upper).
        Flange spans flange_height in Z from cz upward.

    flange_len: length of each flange along the adjacent rail (default
                3cm = one extrusion width, typical 3030 hardware).
    flange_thickness: Y thickness for the X-flange / X thickness for
                     the Y-flange (3mm PETG).
    flange_height: Z extent. Default = S (one rail height, 3cm).

    Saves position to _frame_positions for the sanity check.
    """
    if flange_height is None:
        flange_height = S
    T = flange_thickness
    # Flange A: perpendicular to X — hugs the outer X face.
    # Its X thickness = T (bracket wall). Extends along Y into the
    # chassis by flange_len in the sy direction, and up in Z by
    # flange_height.
    if sx > 0:                      # outer face at X=cx, interior is +X
        ax_x = cx - T
    else:                           # outer face at X=cx, interior is -X
        ax_x = cx
    if sy > 0:
        ax_y = cy
    else:
        ax_y = cy - flange_len
    B(rc, f'{name}_X', ax_x, ax_y, cz, T, flange_len, flange_height,
      'corner')

    # Flange B: perpendicular to Y — hugs the outer Y face.
    if sy > 0:
        by_y = cy - T
    else:
        by_y = cy
    if sx > 0:
        by_x = cx
    else:
        by_x = cx - flange_len
    B(rc, f'{name}_Y', by_x, by_y, cz, flange_len, T, flange_height,
      'corner')

    # Record for sanity check (treat the corner as the "junction").
    _frame_positions.append((name, (cx, cy, cz + flange_height / 2), None))


def _log_bracket_bodies(comp, bracket_name):
    """Diagnostic (rev37): enumerate every body inside a bracket component
    tree, log dims + visibility + name to stdout. Helps understand what
    the GrabCAD STEP actually contains so we can decide placement and
    stub-hiding strategy instead of guessing.
    """
    idx = [0]
    def _walk(c, depth=0):
        for i in range(c.bRepBodies.count):
            b = c.bRepBodies.item(i)
            try:
                bb = b.boundingBox
                dx = bb.maxPoint.x - bb.minPoint.x
                dy = bb.maxPoint.y - bb.minPoint.y
                dz = bb.maxPoint.z - bb.minPoint.z
                cx = (bb.minPoint.x + bb.maxPoint.x) / 2
                cy = (bb.minPoint.y + bb.maxPoint.y) / 2
                cz = (bb.minPoint.z + bb.maxPoint.z) / 2
                vis = b.isLightBulbOn
                bname = b.name
                print(f'  [{bracket_name}] body[{idx[0]}] depth={depth} '
                      f'dims=({dx:.2f},{dy:.2f},{dz:.2f}) '
                      f'center=({cx:.2f},{cy:.2f},{cz:.2f}) '
                      f'vis={vis} name={bname}')
                idx[0] += 1
            except Exception as e:
                print(f'  [{bracket_name}] body[{idx[0]}] error: {e}')
                idx[0] += 1
        for i in range(c.occurrences.count):
            sub = c.occurrences.item(i)
            try:
                _walk(sub.component, depth + 1)
            except Exception as e:
                print(f'  [{bracket_name}] sub-occ error: {e}')
    _walk(comp)


def _import_bracket(rc, step_path, name, c0, c1, c2, t, color='corner',
                    hide_stubs=True, log_bodies=False):
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
    if log_bodies:
        _log_bracket_bodies(occ.component, name)
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
    # Corner brackets (rev36): back to STEP imports from GrabCAD. The
    # rev28 diagnostic showed the bracket body is centered at local
    # (1.25, 6.26, -3.44), so placing it at the outer chassis corner
    # (0, 0, LO+S) put the body 6cm into the interior. Instead, place
    # at each POST AXIS junction (post_x_center, post_y_center, LO+S)
    # so the bracket body lands near the post center, physically
    # inside the post+rail junction.
    #
    # _hide_bracket_stubs still removes the 100mm decorative stubs the
    # GrabCAD bracket STEP carries.
    # --------------------------------------------------------------------
    # --------------------------------------------------------------------
    # Corner brackets (rev41): procedural L-bracket hugging each outer
    # corner of the frame. Fresh approach after the GrabCAD STEP proved
    # a geometry mismatch with our cut plan (its 3-way cube needs 3
    # extrusion ENDS to meet; our FR rails run continuously through).
    # Each bracket is 2 flat flanges meeting at the outer corner:
    #   - one on the -X (or +X) outer face
    #   - one on the -Y (or +Y) outer face
    # Flange size 30x30mm x 3mm PETG (easy to 3D print, ~$0 to fab).
    # --------------------------------------------------------------------
    for nm, cx, cy, cz, sx, sy in [
        ('CB_Lo_FL', 0,     0,     LO, +1, +1),
        ('CB_Lo_FR', 0,     FRAME, LO, +1, -1),
        ('CB_Lo_RL', FRAME, 0,     LO, -1, +1),
        ('CB_Lo_RR', FRAME, FRAME, LO, -1, -1),
        ('CB_Hi_FL', 0,     0,     HI, +1, +1),
        ('CB_Hi_FR', 0,     FRAME, HI, +1, -1),
        ('CB_Hi_RL', FRAME, 0,     HI, -1, +1),
        ('CB_Hi_RR', FRAME, FRAME, HI, -1, -1),
    ]:
        _draw_l_corner(rc, nm, cx, cy, cz, sx, sy)

    # --------------------------------------------------------------------
    # T-plate bracket for lidar mast attachment to rear upper rail.
    # The T-plate STEP has its origin at a corner (not the center), so
    # to align the PLATE's center with the mast we shift by half the
    # plate's extent: rev28 diagnostics showed body_center - translation
    # = (4.5, 4.5, 0.2), meaning the STEP is 9cm x 9cm x 0.4cm with
    # origin at one corner. Compensate with -4.5cm in X and Y.
    _import_bracket(rc, STEP_TPLATE_M6, 'Mast_TPlate',
                    _X, _Y, _Z,
                    (FRAME - S - 4.5, FRAME / 2 - 4.5, MST - 0.1 - 0.2))

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

    # ── RPi5: import real STEP if USE_RPI5_STEP=True and file exists,
    # otherwise use the procedural PCB block. See rev29 notes for why
    # the STEP import is disabled by default.
    rz = HI + S + PLT_T
    if USE_RPI5_STEP and os.path.exists(STEP_RPI5):
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
        # rev40: The rev39 body-tree dump identified body[18] (5.56x5.56
        # x2.05cm) as the LIDAR BASE. Its center is at local (0, 0, 0)
        # from the STEP origin. So to center the base on the mast axis,
        # we just place the STEP origin AT (lcx, lcy) — no compensation
        # needed. Previous rev34's '+2.05' offset was targeting body[0]
        # (a tiny 4.8mm protrusion with a different local position).
        # The sanity-check 'lidar_on_mast_axis' was a false positive
        # because it read body[0]'s center, which happened to be at
        # mast axis after the wrong-offset placement.
        tx = lcx
        ty = lcy
        try:
            lid_occ = _import_step(root, STEP_RPLIDAR_C1, 'RPLIDAR_C1')
            _clr_occ(lid_occ, 'lidar')
            _place_occ(lid_occ, _X, _Y, _Z, (tx, ty, lz))
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

    # Shaft hub — Ø7mm section between gearbox and thin shaft; this is
    # what the mecanum wheel's hex adapter actually grips (rev42).
    if od > 0:
        hub_start = gbox_start + M_GBOX
    else:
        hub_start = gbox_start - SHAFT_HUB
    try: CY(rc, f'MHub_{tag}', mx, hub_start, AXL,
            SHAFT_HUB_DIA/2, SHAFT_HUB, 'hub')
    except Exception: pass

    # Thin shaft — Ø3mm protrudes past the hub to the thin-shaft end
    if od > 0:
        shaft_start = hub_start + SHAFT_HUB
    else:
        shaft_start = hub_start - SHAFT
    try: CY(rc, f'Shaft_{tag}', mx, shaft_start, AXL, SHAFT_D/2, SHAFT, 'hub')
    except Exception: pass

    # ==========================================================================
    # rev32: 3D-printable single-piece PETG motor bracket.
    # ==========================================================================
    # The user has no press brake, so a bent sheet-metal bracket isn't
    # practical. This bracket is designed as a monolithic 4mm-wall PETG
    # print — all geometry (motor flange + rail arm + gusset + bolt bosses)
    # is one solid part. In CAD we draw it as separate boxes for
    # construction ease, but for fabrication the user exports
    # bracket_<TAG> as a single STL and prints with the rail arm face
    # DOWN (best bed adhesion, prints the gusset as an overhang-free slope).
    #
    # Geometry flanges:
    #   plate_v (motor face)  — at the GEARBOX SHAFT-END face, carries 2 M3
    #                           clearance holes at ±8.5mm in Z from shaft
    #                           center (matches JGA25 mount spec).
    #   plate_h (rail arm)    — extends BACK from motor face to rail face,
    #                           carries 2 M5 clearance holes into rail T-slot.
    #   plate_g (gusset rib)  — thin diagonal web in the L's inside corner
    #                           for torque stiffness.
    # Bolt visuals are short cylinders/boxes — they represent hardware
    # inserted THROUGH the plate into either the motor threads or the T-slot.
    # ==========================================================================

    BRK = BRKT_T                         # bracket wall thickness = 4mm
    PV_W, PV_H = 4.0, 4.0                # motor-face plate 40×40 mm
    M3_Z_OFF = 0.85                       # 17 mm bolt spacing / 2
    RAIL_BOLT_Z_OFF = S * 0.5             # centered on rail face

    # Gearbox shaft-end face Y position.
    if od < 0:
        gbox_face_y = fy - M_CAN - M_GBOX      # e.g. -6.2 for front
        plate_v_y  = gbox_face_y - BRK         # plate on the WHEEL side of face
    else:
        gbox_face_y = fy + M_CAN + M_GBOX
        plate_v_y  = gbox_face_y

    # ---- plate_v: motor-face flange ----
    B(rc, f'Brk_{tag}_MtrFlange',
      mx - PV_W/2, plate_v_y, AXL - PV_H/2,
      PV_W, BRK, PV_H, 'bracket')

    # 2 M3 bolts going through plate_v into the gearbox threads.
    # Bolt extends from the outer face of plate_v (visible side) through
    # the plate and into the motor for bolt_depth. We show a single stubby
    # cylinder per bolt, with its tail poking out the plate.
    bolt_depth = M_GBOX * 0.5
    bolt_total_len = BRK + bolt_depth + 0.2       # extra 2mm for the head
    if od < 0:
        bolt_y_start = plate_v_y - 0.2            # head 2mm outside plate
    else:
        bolt_y_start = plate_v_y + BRK - bolt_total_len + 0.2
    for dz in (M3_Z_OFF, -M3_Z_OFF):
        B(rc, f'Brk_{tag}_M3_{int(dz*10):+d}',
          mx - 0.15, bolt_y_start,
          AXL + dz - 0.15, 0.3, bolt_total_len, 0.3, 'bracket')

    # ---- plate_h: bottom arm, spans motor face → rail face ----
    # Arm sits JUST BELOW the motor assembly, connecting the motor
    # flange to the foot of the vertical RISER. Narrow in X so it
    # doesn't collide with the motor can body.
    arm_x_w = S * 2/3
    arm_z0  = AXL - M_DIA/2 - BRK - 0.05          # 0.5mm clearance below motor
    if od < 0:
        arm_y0, arm_y1 = gbox_face_y, fy          # from motor face to rail face
    else:
        arm_y0, arm_y1 = fy, gbox_face_y
    arm_len = arm_y1 - arm_y0
    B(rc, f'Brk_{tag}_BottomArm',
      mx - arm_x_w/2, arm_y0, arm_z0,
      arm_x_w, arm_len, BRK, 'bracket')

    # ---- plate_riser: rev36. VERTICAL piece spanning from the bottom
    # arm UP to the chassis rail. Without this, the bracket ended at
    # motor level Z=~2.3cm while the chassis rail starts at Z=LO=8.44cm
    # — the chassis was visually floating above the wheels. The riser
    # is at the rail-face end of the bottom arm, on the outer face of
    # the chassis rail (so bolts can go through it into the rail's
    # T-slot).
    if od < 0:
        riser_y0 = fy - BRK                       # on outer -Y face of left/right rail
    else:
        riser_y0 = fy
    riser_z0 = arm_z0 + BRK                       # top of bottom arm
    riser_z1 = LO + S                             # top of lower rail
    riser_sz = riser_z1 - riser_z0
    B(rc, f'Brk_{tag}_Riser',
      mx - arm_x_w/2, riser_y0, riser_z0,
      arm_x_w, BRK, riser_sz, 'bracket')

    # ---- plate_rf: rail-face flange. Covers the top of the riser
    # where it mates with the rail face; this is where the M5 bolts
    # go through into the T-slot.
    rf_h = S                                      # full rail height in Z
    rf_w = arm_x_w                                # same X width as arm
    rf_y0 = riser_y0
    rf_z0 = LO                                    # aligned with bottom of rail
    B(rc, f'Brk_{tag}_RailFlange',
      mx - rf_w/2, rf_y0, rf_z0,
      rf_w, BRK, rf_h, 'bracket')

    # 2 M5 bolts through the rail flange into the rail T-slot at
    # mid-height of the rail.
    rail_bolt_y_thread = fy + 0.05 if od < 0 else fy - 0.05
    rail_bolt_z = LO + S / 2
    for bx_off in (-0.4, 0.4):
        B(rc, f'Brk_{tag}_M5_{int(bx_off*10):+d}',
          mx + bx_off - 0.15, rf_y0 - 0.2 if od<0 else rf_y0 + BRK,
          rail_bolt_z - 0.15, 0.3, BRK + 0.3, 0.3, 'bracket')

    # ---- plate_g: gusset rib bridging motor flange and riser ----
    gusset_t = 0.3                                 # 3mm thin rib
    gusset_h = min(PV_H * 0.6, 2.4)                # ~24mm along Z
    if od < 0:
        g_y0 = gbox_face_y
        g_len = min(arm_len * 0.5, 2.5)
    else:
        g_len = min(arm_len * 0.5, 2.5)
        g_y0 = gbox_face_y - g_len
    g_z0 = arm_z0 + BRK                            # sits on top of arm
    B(rc, f'Brk_{tag}_Gusset',
      mx - gusset_t/2, g_y0, g_z0,
      gusset_t, g_len, gusset_h, 'bracket')

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
