# ROScar1 v2 — Aluminum Extrusion Cut Sheet

**Print this. Take it to the saw.**

Matches Fusion 360 CAD model `docs/chassis/fusion360/roscar_v2_chassis.py` rev11+.

---

## Cuts needed

**13 pieces total**, all straight 90° cuts from 3030 extrusion (30×30mm).

Label each piece with a Sharpie on an end face AS YOU CUT IT so you don't mix them up later.

### Frame rails — front/rear (long)

| Label | Length | Qty |
|-------|--------|-----|
| `F-Lo` | **248mm** | 1 — lower deck, front |
| `R-Lo` | **248mm** | 1 — lower deck, rear |
| `F-Hi` | **248mm** | 1 — upper deck, front |
| `R-Hi` | **248mm** | 1 — upper deck, rear |

### Frame rails — left/right (short)

| Label | Length | Qty |
|-------|--------|-----|
| `L-Lo` | **188mm** | 1 — lower deck, left |
| `Ri-Lo` | **188mm** | 1 — lower deck, right |
| `L-Hi` | **188mm** | 1 — upper deck, left |
| `Ri-Hi` | **188mm** | 1 — upper deck, right |

### Vertical posts

| Label | Length | Qty |
|-------|--------|-----|
| `P-FL` | **100mm** | 1 — front-left |
| `P-FR` | **100mm** | 1 — front-right |
| `P-RL` | **100mm** | 1 — rear-left |
| `P-RR` | **100mm** | 1 — rear-right |

### Lidar mast

| Label | Length | Qty |
|-------|--------|-----|
| `M` | **120mm** | 1 — center-rear of upper deck |

---

## Source stock → cut mapping

All black 3030 stock. Kerf loss ~3mm per cut.

| Stock (mm) | Qty | Cut into | Yields | Remainder |
|-----------:|----:|----------|--------|-----------|
| 500 | 2 | 2× 248mm per stick | 4× `F-Lo/F-Hi/R-Lo/R-Hi` | ~0mm waste |
| 400 | 2 | 2× 188mm per stick | 4× `L-Lo/L-Hi/Ri-Lo/Ri-Hi` | ~20mm waste |
| 400 | 1 | 3× 100mm | 3× posts | ~91mm spare |
| 400 | 1 | 1× 100mm + 1× 120mm | 1× post + 1× mast | ~174mm spare |

**Total stock consumed: 2× 500mm + 4× 400mm = 6 sticks.**

Stock preserved (don't cut these today):
- 2× 500mm black
- 4× 600mm black
- 4× 1000mm silver

---

## Cutting tips

1. **Deburr every cut end** with a file or sandpaper — the 3030 channels have internal ribs that leave sharp flashing when sawn.
2. **Tolerance: ±1mm.** Measure each piece after cutting. If one rail is 249mm and another is 247mm, that's fine — pair them by matching opposing sides.
3. **Cut the 188mm pieces first** from the 400mm sticks where the kerf matters more (less margin). Save the 500mm sticks for the 248mm pieces where you have plenty of slack.
4. **Pair and match.** When done, lay out:
   - 4× 248mm in one pile → pair by measured length, closest match → `F-Lo`+`R-Lo` (lower deck) and `F-Hi`+`R-Hi` (upper deck).
   - 4× 188mm in another pile → same pairing.
5. **Frame squareness is set during assembly, not by cut precision** — measure diagonals during assembly and adjust brackets before final tightening.

---

## Frame geometry sanity check

After cutting, the assembled frame should be:
- **248mm × 248mm** outer footprint (per deck)
- **100mm** deck-to-deck spacing (post length)
- **120mm** mast above the upper deck
- **Total chassis height** from lower rail bottom to lidar mast top: ~267mm (plus ~40mm for the lidar body above that)

If the post length is right but the frame doesn't sit square, check that the 248mm rails are exactly equal and the 188mm rails are exactly equal. Any mismatch within a pair will force the frame to be rhomboid.

---

## After cutting

See `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md` §7 (Fabrication Plan) for the assembly sequence. Key order:

1. Assemble lower deck frame (4 rails + 4× 3-way corner brackets, hand-tight)
2. Measure both diagonals, adjust to equal, then tighten brackets
3. Insert 4 posts, with 3-way brackets clamping post bottoms to the deck corners
4. Assemble upper deck on top of posts, same squaring process
5. Attach mast at rear-center of upper deck (using 3030 T-plate bracket)
6. Install deck plates, 3D-printed motor mount brackets, and finally components
