#!/usr/bin/env bash
# Flash roscar_fw.elf to the YB-ERF01-V3.0 via ST-Link V2 over SWD.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELF="${1:-$ROOT/build/roscar_fw.elf}"
[ -f "$ELF" ] || { echo "ELF not found: $ELF — run 'make' first." >&2; exit 1; }
exec openocd \
    -f interface/stlink.cfg \
    -f target/stm32f1x.cfg \
    -c "program $ELF verify reset exit"
