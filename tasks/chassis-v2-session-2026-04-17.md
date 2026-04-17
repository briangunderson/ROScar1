# Chassis v2 session — 2026-04-17 autonomous iteration

Session goal: "keep iterating until I get back." Focused on:
1. Correctness of the frame (you're cutting today)
2. Real component STEPs replacing procedural placeholders
3. Palette/visual cleanup so the model reads as an engineered chassis

## TL;DR — you are good to cut aluminum today

Cut sheet: **`tasks/chassis-v2-cut-sheet.md`** (print this). Summary:

- **4× 248mm** — front/rear rails, `F-Lo`, `R-Lo`, `F-Hi`, `R-Hi`
- **4× 188mm** — left/right rails (fit between F/R), `L-Lo`, `Ri-Lo`, `L-Hi`, `Ri-Hi`
- **4× 100mm** — vertical corner posts, `P-FL`/`P-FR`/`P-RL`/`P-RR`
- **1× 120mm** — lidar mast, `M`
- Stock: 2× 500mm + 4× 400mm black 3030 (6 sticks total)
- Frame outer: 248 × 248 mm
- Chassis height: 280mm bottom rail to top of mast, 406mm to top of lidar body

The cut plan has been **CAD-verified** — rail lengths in the Fusion 360 model match the cut sheet exactly.

## Commits made this session

```
279cbd0 feat(chassis): rev14 — integrate real RPi5 and RPLIDAR C1 STEPs
11db42d feat(chassis): add Raspberry Pi 5 and RPLIDAR C1 STEP models
23e1339 fix(chassis): rev13 palette — black anodized frame + orange motor brackets
1b594de fix(chassis): correct track width + total height from CAD audit
0d2255a feat(chassis): rev13 — structural focus for fabrication
```

## Key corrections to the design spec

| Parameter | Was (spec) | Correct (CAD) | Notes |
|-----------|-----------|---------------|-------|
| Rail lengths | 8× 248mm | 4× 248mm + 4× 188mm | box-joint layout; spec's cut plan was wrong |
| Motor offset M_OFF | 35mm | 93.6mm | spec omitted 62mm motor body length |
| Track width | 320mm | 435mm | follows from M_OFF fix |
| Wheelbase | 200mm | 200mm | unchanged |
| Stability polygon | 64,000 mm² | 87,000 mm² | 4.4× original, not 3.2× |
| Total height to mast top | 267mm | 280mm | cut sheet arithmetic was off |

Real chassis width (outer-to-outer wheel): ~472mm — fits any standard doorway.

## Rev14 script state

- **Frame:** all 22 STEP imports working (8 rails, 4 posts, 1 mast, 8 corner brackets, 1 T-plate)
- **Electronics (non-structural):** real STEPs for Pi5 and RPLIDAR C1; simple placeholder blocks for battery, motor board, camera
- **Drive system:** procedural mecanum wheels with proper slanted rollers at 45°, motor cylinders, orange L-brackets with mounting bolts
- **Wiring:** stripped (will come back once all real STEPs have real ports to route to)

## Palette (rev13+)

- **frame** = (35, 37, 42) — black anodized 3030, matching your physical stock
- **plate** = (215, 180, 120) — warm tan acrylic/G10
- **bracket** = (230, 120, 30) — orange motor L-brackets (engineering pop)
- **corner** = (175, 180, 190) — silver/raw-aluminum 3-way brackets
- **motor** = (200, 200, 205) — motor can silver
- **rpi/lidar** — dark, recognizable blocks

The full chassis now reads as "engineered machine" rather than "random parts."

## STEPs downloaded (both no-login from the manufacturer)

| Part | File | Size | Source |
|------|------|------|--------|
| Raspberry Pi 5 | `docs/chassis/models/electronics/RaspberryPi5.step` | 5.5 MB | raspberrypi.com |
| RPLIDAR C1 | `docs/chassis/models/electronics/rplidar-c1.stp` | 7.7 MB | slamtec.com |

## STEPs still needed (GrabCAD login required)

Drop them in `docs/chassis/models/electronics/` when you get them and I'll integrate into `_components()`:

- Mecanum wheels (80mm, hex bore, ABBA pattern) — replaces procedural mecanum rollers
- JGA25-370 / JGB37 encoder motor — replaces procedural motor cylinders
- XT60 female connector — for motor board
- Logitech C270 webcam — replaces placeholder camera block
- Argon NEO 5 case — replaces placeholder RPi5 case (may need to manually overlay on the imported Pi5 PCB STEP)
- 3S LiPo battery — replaces placeholder battery block

See `docs/chassis/models/electronics/README.md` for source URLs.

## Known issues / to-revisit when back

1. **RPi5 and RPLIDAR placement may need small offsets.** Their STEPs have manufacturer-native origin points that I guessed at. If either is a few mm off, edit the translation tuple in `_components()` — `_place_occ(rpi_occ, _X, _Y, _Z, (c - 4.25, c - 2.8, rz))` for Pi5 and similar for lidar.
2. **Electronics imports go into a sub-component (`4 - Electronics`).** Fusion's `occurrence.transform` requires the parent to be the active edit target; root imports like the frame are fine but sub-component imports can silently fail to apply the transform. If the Pi5 or lidar end up at origin instead of the intended position, the fix is to import into root and rename.
3. **3D-printable motor mount brackets** — the design spec calls for parametric PETG brackets (§4.4, §7 Phase 2). These are still to design in Fusion based on the 24mm motor diameter + 17mm screw spacing.

## What I did NOT touch

- ROS2 codebase (`roscar_ws/`) — untouched
- URDF — untouched (still reflects v1 chassis, will update in Phase 6 per spec)
- Launch files — untouched

## Good stopping point

The frame is cut-ready. The CAD model reflects what you'll build accurately. Real Pi5 and lidar CAD in, more to come when you source the rest from GrabCAD.
