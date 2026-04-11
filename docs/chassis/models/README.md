# Chassis v2 — 3D Models

CAD models for the ROScar v2 aluminum extrusion chassis.
All hardware uses **3030 (30×30mm) T-slot aluminum extrusion**.

## Directory Structure

```
models/
├── extrusions/          # Profile bodies
│   └── 30X30 - T-slot - Aluminium Profile (.f3d, .iges, .step)
├── brackets/            # Frame connectors and plates
│   ├── 3030 - 3-way corner bracket (.iges, .step)   ← 8× in BOM
│   ├── T-Plate 3030 5 Holes - M5 (.iges, .step)     ← lidar mast mount
│   ├── T-Plate 3030 5 Holes - M6 (.iges, .step)     ← lidar mast mount (alt)
│   └── L-Plate 3030 5 Holes - M6 (.iges, .step)     ← future cross-bracing
└── fasteners/           # Slider T-Nuts for mounting components to T-slot channels
    ├── Slider T-Nut for 3030 - M3 (.f3d, .iges, .step)
    ├── Slider T-Nut for 3030 - M4 (.f3d, .iges, .step)
    ├── Slider T-Nut for 3030 - M5 (.f3d, .iges, .step)
    └── Slider T-Nut for 3030 - M6 (.f3d, .iges, .step)
```

## File Formats

| Extension | Use |
|-----------|-----|
| `.step`   | Universal — import into Fusion 360, FreeCAD, any CAD tool |
| `.iges`   | Universal — older format, also widely supported |
| `.f3d`    | Fusion 360 native — opens directly, preserves parametric features |

Prefer `.step` for importing into assemblies. Use `.f3d` when you want to edit the model parametrically in Fusion 360.

## How These Relate to the Design

See the full design spec: `docs/superpowers/specs/2026-04-04-extrusion-chassis-design.md`

| Model | Where It's Used | Qty |
|-------|----------------|-----|
| 30X30 T-slot profile | All frame rails, vertical posts, lidar mast | 13 pieces (cut from stock) |
| 3-way corner bracket | All 8 frame corners (lower + upper deck) | 8× |
| T-Plate (M5 or M6) | Lidar mast attachment to rear upper rail | 1× |
| L-Plate M6 | Spare — available for future cross-bracing | 0 (initial build) |
| Slider T-Nut | Attaching deck plates, RPi5, motor board, camera, lidar mount | ~30× (various sizes) |

## Fusion 360 Workflow

The Fusion 360 Python script (`docs/chassis/fusion360/roscar_v2_chassis.py`) generates
a procedural visualization of the chassis. These STEP/F3D files are for:

1. **Reference** — import into the script's design as placed bodies to replace box approximations
2. **Assembly model** — create a proper F360 assembly (separate from the script) for fit-checking
3. **Fabrication reference** — verify slot dimensions, hole patterns, clearances before cutting

To import a STEP file in Fusion 360: **Insert → Insert Mesh** or **Open** → navigate to the `.step` file.
