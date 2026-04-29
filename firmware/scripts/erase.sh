#!/usr/bin/env bash
# Mass-erase the STM32 flash (recovers from a bricked-by-write attempt).
set -euo pipefail
exec openocd \
    -f interface/stlink.cfg \
    -f target/stm32f1x.cfg \
    -c "init; reset halt; stm32f1x mass_erase 0; exit"
