# ROScar1 v2 Chassis — Fusion 360 Mockup

## How to Run

1. Open Fusion 360
2. Go to **UTILITIES** > **Scripts and Add-Ins** (or press Shift+S)
3. Click the **+** button next to "My Scripts"
4. Navigate to this directory and select `roscar_v2_chassis.py`
5. Click **Run**

The script creates a new document with the complete chassis assembly.

## What It Creates

- **Lower Deck Frame** — 4x 3030 rails forming a 248mm square
- **Upper Deck Frame** — matching 4x 3030 rails
- **Vertical Posts** — 4x 100mm corner posts connecting the decks
- **Lidar Mast** — 120mm post at center-rear of upper deck
- **Deck Plates** — translucent blue acrylic plates on each deck
- **Battery** — blue box on lower deck (75x55x66mm, 574g)
- **Motor Board** — black box behind battery (56x85mm)
- **RPi5** — dark gray box on upper deck (Argon NEO 5 case)
- **RPLIDAR C1** — black box on top of mast
- **Camera** — gray box on front upper rail
- **4x Motor + Wheel assemblies** — at frame corners with orange brackets

## Modifying Dimensions

All dimensions are defined as Python constants at the top of the script. Edit the value, save, and re-run the script (it creates a new document each time).
Key constants:
- `FRAME_SIDE` — rail length (controls overall frame size)
- `POST_HEIGHT` — vertical spacing between decks
- `MAST_HEIGHT` — lidar mast height
- `MOTOR_OFFSET` — how far motors sit from the frame

## After Running

- Use Fusion 360's **Inspect > Measure** tool to verify dimensions
- Toggle component visibility in the browser tree to see individual layers
- Use **Section Analysis** to check internal clearances
- Export STL meshes from individual components if needed for 3D printing
