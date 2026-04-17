# Electronics STEP models

Real CAD models from the manufacturers for use in the Fusion 360 chassis assembly.

## Files

| File | Part | Source | Downloaded |
|------|------|--------|-----------|
| `RaspberryPi5.step` | Raspberry Pi 5 SBC | raspberrypi.com (official) — [pip.raspberrypi.com/documents/RP-008369-DS](https://pip.raspberrypi.com/documents/RP-008369-DS) | 2026-04-17 |
| `rplidar-c1.stp` | RPLIDAR C1 360° lidar | slamtec.com (official) — [RPLIDAR C1 3D model](https://download-en.slamtec.com/api/download/rplidar-c1-model-3d-stp/1?lang=netural) | 2026-04-17 |

Both are official, no-login downloads from the manufacturer. They supersede
the procedural placeholder blocks in `roscar_v2_chassis.py` once integrated
as STEP imports (rev14+).

## Still needed (require GrabCAD login or purchase)

- **Argon NEO 5 case** — Argon40 doesn't publish CAD; community model on GrabCAD
- **Mecanum wheels 80mm, ABBA pattern** — GrabCAD (Yahboom/Nexus style with hex bore)
- **JGA25-370 / JGB37 encoder motor** — GrabCAD; verify 24mm dia, 62mm length, 17mm screw spacing
- **XT60 female panel mount connector** — GrabCAD or TraceParts
- **Logitech C270 webcam** — GrabCAD community model
- **3S 2200mAh LiPo battery** — GrabCAD (generic brick, ~75×55×66mm)

Drop these into this folder (or let me know the filename) and I'll integrate
them into the Fusion 360 script.
