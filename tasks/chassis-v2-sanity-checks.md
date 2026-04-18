# Chassis v2 sanity checks — run after every script iteration

These are the minimum things that have to be true in the built model
before a rev can be called "working." The chassis script runs each
check at end of `run()` and prints `PASS` / `FAIL` per item into
stdout. The bridge captures stdout, so the full check result is
visible in the run-script response.

## Philosophy

Rev22→rev30 taught us that individual "did it work?" signals lie in
ways that look plausible. The rule from here on:

> If a check didn't explicitly re-verify the user-visible outcome,
> it does not count. No trusting intermediate API return values; no
> trusting the translation tuples we tried to set. Read the state
> you care about, compare to what you expected.

Every check below reads a real geometric property (bbox center,
visibility flag, body count) and compares to an expected range.

## The checks

Run in order. A single `FAIL` is a regression — stop and fix before
adding new features.

### Frame placements
- **`frame_count`** — 22 STEP-imported occurrences in `_frame_positions` (8 rails + 4 posts + 1 mast + 8 brackets + 1 T-plate)
- **`frame_unique`** — all 22 have distinct body-bbox centers (rounded to 2 dp)
- **`frame_in_volume`** — every frame body center is within the expected chassis volume
- **`frame_bbox_tight`** — combined frame bbox X∈[0,25], Y∈[0,25], Z∈[5,35]

### Wheels
- **`wheels_count`** — 4 tire bodies exist with `isLightBulbOn == True`
- **`wheels_at_corners`** — each wheel center has correct X and Y sign for its corner (FL/FR/RL/RR)
- **`wheels_above_ground`** — all wheel Z centers are at `AXL ± 0.5cm`

### Motors
- **`motors_count`** — 4 motor can cylinders visible
- **`motors_touch_bracket`** — no gap between motor can's inner face and bracket's vertical plate (|Δy| ≤ 0.5 cm)
- **`motors_under_frame`** — all motor Z centers are below `LO` (hanging below the frame)

### Corner brackets
- **`brackets_count`** — 8 corner brackets visible
- **`brackets_at_corners`** — each bracket body-bbox center is within 4cm of the nominal corner
- **`brackets_z_arm_direction`** — upper-deck brackets have their Z-arm extending DOWN (body below HI); lower-deck brackets have theirs extending UP (body above LO+S). Distinguishable by body Z-range sign.

### Mast + lidar
- **`mast_tall`** — mast body Z range spans at least 10 cm above the upper deck
- **`tplate_centered`** — Mast_TPlate body center is within 1 cm of mast axis (x=FRAME-S, y=FRAME/2)
- **`lidar_on_tplate`** — RPLIDAR body Z bottom is within 1 cm of T-plate Z top

### Visibility & cleanliness
- **`strays_hidden_count`** — number of `isLightBulbOn=False` bodies matches expected (≥ 80); a sudden jump means a new STEP is dirty
- **`total_visible_bodies`** — total visible body count is in expected range (tolerate ±10% between runs)

## How it runs

The script builds everything, hides strays, then calls
`_run_sanity_checks(rc)` which returns a list of `(name, ok,
message)` tuples. Both the individual lines and a one-line summary
(`sanity: N/M pass`) go into stdout and into the final messageBox.
The bridge JSON response has the full structured result in
`data.stdout`.

When a check fails, a human reading the dialog sees immediately
which check failed and what the expected/actual values were. The
bridge client can parse the stdout lines for programmatic pass/fail
gating of longer automation loops.

## Adding a new check

Keep each check to: name, short description, callable that returns
`(bool_ok, str_message)`. Favor reading one concrete geometric value
and comparing to a range. Don't write a check that trusts a
return-value-return-value-return-value chain — that's the same class
of mistake as the rev23 dialog.

Checks live in the `_SANITY_CHECKS` list at the bottom of
`roscar_v2_chassis.py` so they move with the script.

## Visual sanity check (EVERY rev)

Automated checks can't catch "it looks wrong." Claude must ALSO
take a fresh viewport screenshot after every iteration and walk
this list by eye:

### Overall form
- Chassis reads as a robot (deck plates, frame, wheels, motors all present)
- No stray specks floating outside the chassis volume
- Colors correct: silver rails, brass brackets, tan plates, orange motor brackets

### Frame
- 8 silver 3030 rails visible, forming 2 horizontal rectangles (upper + lower)
- 4 vertical silver posts at the corners connecting the decks
- Mast rises from the rear with the T-plate + RPLIDAR on top

### Corner brackets
- 8 brass brackets, one at each rail-rail-post junction
- Lower-deck brackets: Z-arm extends UP into the post (above LO+S)
- Upper-deck brackets: Z-arm extends DOWN into the post (below HI)
- NO bracket material poking above the upper deck plate
- X and Y arms extend horizontally along the adjacent rails

### Wheels
- 4 wheels visible at the corners
- Each wheel has visible mecanum rollers (not a solid black disk)
- Wheels sit below the chassis with their outer edge flush with or just beyond the rail face

### Motor assemblies
- 4 motor cans visible, one per corner
- Each motor rests on the horizontal shelf of its L-bracket (no gap)
- Vertical L-bracket plate is bolted to the outer face of the adjacent rail (plate covers the rail face)
- Orange gusset rib visible in the inside corner of the L

### Deck plates + electronics
- 2 tan plates, one at LO+S and one at HI+S
- Each plate has 4 bolt heads at its corners
- Blue battery + green motor board visible on the lower deck
- Black Pi5 block (procedural) visible on the upper deck

### Reporting
Write the visual check result as a short bullet list per rev (PASS
items and FAIL items). Attach the screenshot. Fail items become
rev+1 targets.
