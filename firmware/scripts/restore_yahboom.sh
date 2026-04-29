#!/usr/bin/env bash
# Restore the original Yahboom firmware from a previously-saved backup.
# Save your stock firmware first with `scripts/backup_yahboom.sh` — this is
# only useful if you have that backup. If you skipped the backup step, you
# can re-flash from Yahboom's official binary (download from the product page).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${1:-$ROOT/scripts/yahboom_stock.bin}"
[ -f "$BIN" ] || { echo "Backup not found: $BIN" >&2; exit 1; }
exec openocd \
    -f interface/stlink.cfg \
    -f target/stm32f1x.cfg \
    -c "program $BIN 0x08000000 verify reset exit"
