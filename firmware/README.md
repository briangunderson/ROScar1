# ROScar1 Firmware (STM32F103RCT6 / Yahboom YB-ERF01-V3.0)

Custom replacement for the Yahboom stock firmware. Speaks the same USART1
wire protocol as Yahboom's `Rosmaster_Lib`, so the existing
`roscar_driver` ROS2 node works unchanged.

**Why we wrote our own:** the stock firmware embeds wheel diameter, track,
and wheelbase in compiled code we don't have source for. Chassis v2 will
change those dimensions, which would otherwise leave odometry permanently
miscalibrated. Owning the firmware also unlocks PID retuning, gyro
bias correction in firmware, and any future protocol changes.

See:
- [`docs/firmware/architecture.md`](../docs/firmware/architecture.md) — module layout and runtime model
- [`docs/firmware/pinmap.md`](../docs/firmware/pinmap.md) — STM32F103 pin assignments
- [`docs/firmware/wire-protocol.md`](../docs/firmware/wire-protocol.md) — byte-level protocol spec

## Prerequisites

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi \
                 libnewlib-arm-none-eabi openocd
```

ST-Link V2 (clone is fine, ~$3 on Amazon) — connects to the SWD header
labelled item ㉑ on the V3.0 silkscreen.

## Build

```bash
cd firmware
make hal     # one-time: clones STM32CubeF1 HAL into vendor/ (~150 MB)
make         # produces build/roscar_fw.{elf,hex,bin}
```

On a clean tree the first build takes ~30 s. Memory budget: ~19 KB flash,
~5 KB RAM out of 256 KB / 48 KB available.

## Flash

**Always back up the stock firmware first** so you can roll back:

```bash
scripts/backup_yahboom.sh    # saves to scripts/yahboom_stock.bin
```

Then flash:

```bash
make flash                   # uses openocd
# or
scripts/flash.sh
```

Roll back to stock if anything goes wrong:

```bash
scripts/restore_yahboom.sh
```

## Verification Checklist

Once flashed, with the board powered (USB-only is fine for everything except
motor commands — those need 12 V on the T-jack):

1. **Heartbeat LED** blinks at 1 Hz → main loop alive.
2. From the Pi:
   ```bash
   picocom -b 115200 /dev/roscar_board
   ```
   You should see binary noise once `set_auto_report_state(True)` is sent.
3. Run the existing ROS2 driver:
   ```bash
   ros2 launch roscar_bringup teleop.launch.py
   ```
   - `ros2 topic echo /battery_voltage` → small positive number (depends on ADC pin verification).
   - `ros2 topic echo /imu/data_raw` → az ≈ +9.81 with board flat.
   - `ros2 topic hz /odom_raw` → ≈ 25 Hz.
4. Drive forward with teleop. Encoder counts should accumulate; `/odom_raw`
   pose should integrate forward.
5. Kill the teleop publisher → motors stop within 500 ms (cmd_vel watchdog).

## Known TODOs Before Stable Use

| Topic | Status |
|-------|--------|
| Verify TIM5 encoder pin remap on actual hardware | TBD |
| Confirm battery ADC pin (currently PA4 placeholder) and divider ratio | TBD |
| Tune PID gains under load | TBD |
| Verify accel scaling (×10000 vs raw LSB) end-to-end | TBD |
| Locate buzzer pin in schematic and add `bsp_buzzer.c` | optional |
| Add WS2812 RGB LED bar driver | optional |
| Persist PID + car_type to flash on `forever=0x5F` | nice-to-have |

## Debugging

GDB + OpenOCD:

```bash
# terminal 1
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg
# terminal 2
arm-none-eabi-gdb build/roscar_fw.elf -ex "target remote :3333"
```

Useful breakpoints: `main`, `control_loop_tick`, `dispatch_frame`.

## Layout

```
firmware/
├── Makefile           # ARM GCC build, slim HAL subset
├── inc/               # Project headers
├── src/               # main.c + bsp + app
├── startup/           # ARM startup .s + system_stm32f1xx.c
├── linker/            # stm32f103rc.ld
├── scripts/           # flash / erase / backup / restore
└── vendor/            # STM32CubeF1 (gitignored, fetched by `make hal`)
```
