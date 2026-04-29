#!/usr/bin/env bash
# CRITICAL: run this BEFORE flashing your own firmware.
# Dumps the current STM32 flash to a binary file so you can restore the stock
# Yahboom firmware later if needed.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$ROOT/scripts/yahboom_stock.bin}"
mkdir -p "$(dirname "$OUT")"
openocd \
    -f interface/stlink.cfg \
    -f target/stm32f1x.cfg \
    -c "init; reset halt; dump_image $OUT 0x08000000 0x40000; exit"
echo "Stock firmware backed up to $OUT"
