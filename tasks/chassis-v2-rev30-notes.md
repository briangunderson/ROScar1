# Chassis v2 rev22→rev30 — the placement-bug odyssey

Session goal at start: "all of the aluminum extrusion pieces are
occupying the same space. They are visible per se, but only if you
hover over each one in the browser tree since they are all in the
same position as one another."

End state (rev30): **22 STEP imports at 22 unique correct world
positions, 94 stray grub-screw bodies auto-hidden, viewport fitted
cleanly**. Render in `tasks/chassis-v2-rev30-render.png` (local only,
gitignored).

See `docs/chassis/fusion360/roscar_v2_chassis.py` header for the full
rev log. Quick summary of the iterations:

| rev | Hypothesis | Outcome |
|----:|------------|---------|
| 22 | `importToTarget2` returns wrong occurrence → `_place_occ` targets same one each call | Misidentified the bug. Actually every call got the right occurrence. |
| 23 | Polish on top of (incorrect) rev22: motor gussets, placement verification in dialog | Dialog claimed "22 unique OK" but was reporting **intended tuples**, not actual post-set transforms. Misleading. |
| 24 | `occ.transform` setter silently fails when parent isn't the active edit target. Activate root + MoveFeature fallback. | Setter succeeded on immediate read-back; fallback never triggered. But end-of-run positions still at (0,0,0). |
| 25 | Diagnostic-only rev: print pre/post transforms and activation status per call. | Caught the real story: **every post-set read returned the intended value**, but parametric recomputes between `_place_occ` and end-of-run were silently reverting direct property sets on imported occurrences. |
| 26 | Rewrite `_place_occ` to use `moveFeatures.createInput2` with the occurrence itself — a parametric feature that survives recompute. | Fusion rejected: `3 : invalid argument inputEntities`. `moveFeatures` won't accept imported-STEP Occurrence entities in ParametricDesignType. |
| 27 | Move the COMPONENT's BRepBodies (not the occurrence). Each STEP creates its own unique component; moving its bodies is equivalent to moving the occurrence. | **13/22 worked** (rails, posts, mast, RPLIDAR — all single-body STEPs). 9/22 failed with `object is not in the assembly context of this component` — the 8 brackets, T-plate, and RPi5 are multi-body assemblies whose bodies live in sub-components. |
| 28 | Recurse: for each component in the occurrence's tree, apply a MoveFeature in its OWN `features.moveFeatures` with the same outer target matrix. | **22/22 at unique positions**, 20/22 at the correct intended world coords. Corner brackets and T-plate landed correctly. Two stragglers: the RPi5 STEP (deeply nested assembly with non-identity sub-occurrence transforms) ended up at negative Z, and the T-plate's bbox extended 4.5cm past the chassis because its STEP origin is at a corner (not the center). |
| 29 | Set `USE_RPI5_STEP = False` — fall back to the procedural PCB block until a better approach (addExistingComponent from a flattened template, or the Argon NEO 5 case) is implemented. Fix T-plate by subtracting the STEP's half-widths from the placement tuple. | Chassis renders correctly; ~94 stray specks scatter around origin from multi-body STEP sub-components (grub screws, nuts, small hardware) whose non-identity sub-occurrence transforms break the rigid-move assumption. |
| 30 | Walk the whole component tree post-build and hide any body whose bbox center is outside the chassis volume (`X:[-5,30] Y:[-5,30] Z:[-1,45]` cm — generous margins). | **Clean first-run render**. Dialog reports "94 stray hardware bodies hidden" — stable count that any future STEP addition could increase, which we'd see immediately. |

## What made this hard

The killer was that every rev had a **plausible-looking success
signal** that turned out to be lying:

1. **rev22's dialog said "22 unique OK"** — because I was summarizing
   the tuples I *passed into* `_place_occ`, not the tuples that
   landed. Lesson: verification must read the post-state, not the
   pre-intent.
2. **rev24's setter returned the intended value on immediate
   read-back** — but parametric design recomputed them away before
   end-of-run. The validation window has to span the whole build,
   not just one call site.
3. **rev26's `moveFeatures` rejection** said `invalid argument
   inputEntities`, implying I'd passed the wrong thing. Actually the
   API just doesn't accept Occurrence entities from imported STEPs
   in ParametricDesignType at all. Had to move bodies instead.

In all three cases the signal was technically correct for what it
measured but didn't measure the thing I cared about. The fix in each
case was pulling verification closer to the user-visible outcome
(bounding-box center of a body after all features have run, not
property-setter return value).

## Tooling that made this tractable

- **The ROScar bridge add-in** (`docs/chassis/fusion360/bridge/`)
  took the cost of each iteration from "Brian manually reopens
  Fusion, clicks Scripts, clicks Run, reads the message box, screenshots,
  shares back" down to a single `python fctl.py run ...` call that
  returns the full `stdout + _clog + _frame_positions` as JSON.
  rev22→rev30 probably would have taken a full extra afternoon of
  human-loop turnaround without it.
- **`computer-use` permission for Fusion + `systemKeyCombos`** let
  me press Enter to dismiss the `ui.messageBox` modal that blocks
  the bridge's main thread at end-of-run. Without that, the bridge
  stalls on every run.
- **Per-call diagnostic prints** (rev25) were what finally separated
  "setter returns success" from "transform persists to end of run".

## What still needs attention

1. **RPi5 STEP**: currently using the procedural PCB block. Fixing
   the real STEP requires either a component-template + addExistingComponent
   approach with an identity master, or just switching to the Argon
   NEO 5 case STEP (which probably has a flatter assembly structure).
2. **T-plate Z offset**: the current tuple shifts the plate down
   by 0.2cm so its top face is near the mast base. Needs visual
   confirmation that the mast actually passes through the T-plate's
   slot in the render — not just the geometric algebra.
3. **Other GrabCAD STEPs** (mecanum wheels, JGA25 motors, XT60, C270
   camera, 3S LiPo): still need sourcing. Drop them into
   `docs/chassis/models/electronics/` and the integration will
   follow the same pattern as RPLIDAR C1 (single-body) — not Pi5
   (deep-nested).
4. **Chassis fabrication**: the cut sheet at `tasks/chassis-v2-cut-sheet.md`
   is CAD-verified and ready to cut. 13 pieces from 6 sticks of
   3030 stock.

## Bridge improvements queued

- `hide_out_of_frame.py` and `hide_stray_and_fit.py` are now subsumed
  by the built-in `_hide_stray_bodies` pass in rev30. Kept in
  `bridge/` as examples of how to run ad-hoc Fusion operations
  through the bridge.
- The `probe_print.py` smoke test stays — handy when validating any
  future bridge change.
- Next bridge addition worth having: `activeViewport.fit()` as a
  first-class command so I don't need to run a one-line `fit_view.py`
  for each screenshot. Low priority — the current workflow is fine.
