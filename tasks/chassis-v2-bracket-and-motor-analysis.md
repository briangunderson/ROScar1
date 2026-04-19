# Chassis v2 — corner bracket geometry mismatch + motor bracket feasibility

Analysis written 2026-04-19 in response to three user questions:
1. Are the corner brackets placed properly?
2. Why is the lidar still off-center?
3. Are the motor brackets structurally feasible?

## 1. Corner bracket geometry mismatch

### How a 3030 3-way corner bracket actually works

Per Zyltech, Handson Tech, and the rest of the search results:

> "The bracket is inserted into the pit of the butt joint of two
> profiles, and then tightened with set screws. It is suitable for
> connecting European standard 2020/3030/4040 series aluminum frame
> profiles at a 90-degree angle. The 3-way design allows connections
> to be made at multiple 90-degree angles in three dimensions."

So this bracket is a **cube-style internal connector**:
- Small cube (~30mm) with 3 threaded holes (M6), one per face
- Cube sits INSIDE the corner where 3 extrusions meet
- Each extrusion's END CAP faces a threaded hole in the cube
- M6 bolts go from each extrusion end INTO the cube's threads
- Set screws add secondary clamping

**The bracket REQUIRES all three extrusions to END at the corner.**

### Our chassis geometry doesn't match

Current chassis layout at the FL lower corner:
- **Post FL** (vertical): X=[0,3], Y=[0,3], Z=[LO+S, HI]. Its lower end is at Z=LO+S. ✓ ends at the corner
- **Left rail** Rail_LR_Lo_Left (X-running): X=[3, 21.8], Y=[0,3], Z=[LO, LO+S]. Its -X end is at X=3. ✓ ends at the corner
- **Front rail** Rail_FR_Lo_Front (Y-running): X=[0,3], Y=[**0, FRAME=24.8**], Z=[LO, LO+S]. Its ends are at Y=0 and Y=FRAME. ✗ **passes straight through the corner** — doesn't end there.

So only 2 of 3 extrusions have their ends meeting at each corner. The third (the continuous FR rail) just has a side face there. The 3-way bracket's third M6 hole has nothing to bolt into.

This is a direct consequence of the cut plan (`tasks/chassis-v2-cut-sheet.md`):
- 4× 248mm FR rails = full-length, continuous through corners
- 4× 188mm LR rails = cut short, butt-join between the posts

### Three ways forward

**A. Keep current cut plan; change bracket type**. Use 2-way 90° L-brackets instead of the 3-way cube connector. Two L-brackets per corner (one bolting left rail to post, one bolting front rail's T-slot to post). Total: 16 L-brackets instead of 8 3-way brackets. These are widely available and cheap. The CAD model would draw them procedurally or import simpler STEPs.

**B. Change cut plan to match the 3-way bracket**. Every rail gets cut shorter so all 4 corners have 3 extrusion ends meeting. Cut plan becomes:
- 8× 218mm FR rails (248mm − 30mm for the cube at each end, then halved into 2 pieces)
- 8× 188mm LR rails (unchanged)
- etc.
This doubles the rail count from 8 to 16 and adds 8 cuts. Physically stronger joint at the corners (all 3 extrusions clamped) but more fabrication work.

**C. Accept the current STEP as decorative**. The 3-way bracket model sits at the junction and looks right; only 2 of its 3 bolts are load-bearing in reality. This is fine for non-structural demo robots. The rev38 placement already does this — brackets are AT the junctions, they just wouldn't be 100% utilized.

**Recommendation**: **A** is the cheapest and most common approach in hobbyist 3030 builds. Swap to 2-way L-brackets + continue with the current cut plan. Low effort, physically sound, widely understood.

Brian's decision needed before rev41 can pick a direction.

## 2. Lidar off-center (rev40 fix)

Rev39's body-tree dump of the RPLIDAR C1 STEP revealed the problem.
The STEP contains 22 bodies:
- Body 0: 0.48×0.48×0.99cm — tiny protrusion (connector or button)
- **Body 18: 5.56×5.56×2.05cm — this is the LIDAR BASE**, local origin at (0,0,0)
- Body 20: 5.46×5.46×2.28cm — the rotating scanner head
- 20 other small bodies (screws, tabs, cable bits)

The earlier rev34 `+2.05cm XY compensation` was derived from body 0's
offset — a small decorative piece, not the base. That fix put body 0
at the mast axis but left the 5.56cm base offset by 2cm, which is
exactly what Brian saw visually.

Rev40 removes the compensation (STEP origin now placed directly at
mast axis) because body 18's local origin IS at (0,0,0). The lidar
base will now land on the mast axis. Sanity check also rewritten to
find the base body by matching its known dimensions (`dx, dy within
5mm of LID_SZ, dz in [1.5, 3]cm`), not by iterating from body 0.

## 3. Motor bracket structural feasibility

### Loads

Assume JGA25-370 (24mm × 62mm geared motor) with an 80mm mecanum wheel:
- Motor + gearbox mass: ~150 g → 1.5 N weight
- Stall torque: ~5 kg·cm = 0.49 N·m
- Impact from wheel hitting a bump at 0.5 m/s: peak force ~20 N (2× motor weight)
- Motor holding torque on bracket: ~0.5 N·m

### Current design (rev36/rev38)

Bracket is a monolithic PETG 3D print with four regions:
- `MtrFlange`: 40 × 40 × 4 mm plate, bolted to gearbox shaft-end face via 2× M3
- `BottomArm`: 20 × 62 × 4 mm horizontal plate, from motor face back to rail
- `Riser`: 20 × 4 × ~60 mm vertical piece, from arm top up to rail top
- `RailFlange`: 20 × 4 × 30 mm plate, bolted to rail T-slot via 2× M5
- `Gusset`: 3 mm rib reinforcing the L-bend inside corner

PETG material properties:
- Tensile strength: ~45 MPa
- Flexural strength: ~70 MPa
- Flexural modulus: ~1.5 GPa

### Stress check — the most loaded cross-section

The BottomArm is a cantilever beam 62mm long carrying ~1.5N motor weight
+ ~20N impact peak = ~22N max lateral force at its free end.

Cross-section (beam W × H): 20 × 4 mm
- Second moment of area about the horizontal axis: I = W × H³ / 12 = 20 × 64 / 12 = **107 mm⁴**
- Distance to outer fiber: c = H / 2 = 2 mm

Bending moment at the fixed (rail) end: M = F × L = 22 × 0.062 = **1.36 N·m = 1360 N·mm**

Peak bending stress: σ = M × c / I = 1360 × 2 / 107 = **25.4 MPa**

PETG flexural strength is ~70 MPa → factor of safety ≈ **2.75**

### The motor-flange M3 bolts

Each M3 bolt carries about half the motor's lateral force plus a moment
share. Pullout strength of an M3 bolt in 4mm PETG with a tapped insert
(or threaded directly into the gearbox) is >500 N per bolt. Well over
what we need.

### The rail-flange M5 bolts into the T-slot

Standard 3030 T-nut pull-out ratings: 500-1000 N per bolt. Two M5s
into the rail's T-slot can easily resist the moment at the top of the
Riser plus rotational forces.

### Verdict

**Structurally feasible for a hobby-weight robot.** Factor of safety
~2.75 against PETG's quasi-static flex strength at the worst
expected load. Adequate for normal use.

Caveats:
- Print orientation matters. Print with the BottomArm face on the
  build plate so layer lines run along the load path, not across it.
- PETG fatigue: after 10⁵-10⁶ load cycles (wheel hitting bumps), a
  small crack could initiate at the inside corner of the L. Add a
  fillet there (or a triangular gusset instead of the rectangular
  one currently modeled) for a >10× fatigue life improvement.
- Heat: PETG glass transition ≈ 80 °C. JGA25 motors stay under 60 °C
  in normal use; near the motor can body they heat the bracket by
  ~10-15 °C. Ambient + motor heat rise = 40-55 °C. Comfortable margin.

### Suggested print settings for robust bracket

- Nozzle 0.4mm, layer height 0.2mm
- 4 perimeters minimum (the bracket has thin walls at 4mm — 4
  perimeters fills them with solid material end-to-end)
- 40% gyroid infill (infill only contributes at cut sections >10mm;
  most of our bracket is 4-6mm thick walls where perimeters dominate)
- Print with MtrFlange face DOWN (so that flange builds up vs layer
  lines parallel to its 40×40mm plate; both BottomArm and Riser
  then come off at 90° to the plate and layer lines run along
  their length)
- Apply thread-locker to all M3 and M5 bolts

If Brian wants higher safety factors:
- Wall thickness 5mm instead of 4mm → 2× increase in I → stress halves
- Switch to PETG-CF (carbon fiber) or PC-ABS → ~3× strength
- Or just use 1.5mm aluminum L-brackets (free from Misumi/Aliexpress,
  ~$3-5 each, no printing at all) — zero-effort alternative

## Summary for rev41 onward

1. **Corner brackets**: decision needed. Recommend option A (switch to 2-way L-brackets). I'll wire in whatever you pick.
2. **Lidar**: rev40 should land the base on the mast axis. Verifying next.
3. **Motor bracket**: passes structural check with factor ≈ 2.75; can stay as-is for build. No action required unless you want higher margin.
