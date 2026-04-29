# YB-ERF01-V3.0 Pin Map

Extracted from Yahboom reference PDFs (V3.0 board introduction + per-peripheral driver tutorials in their official GitHub repo). Verified against schematic descriptions in [`1. Expansion Board Introduction_V3.0.pdf`].

## MCU
- **STM32F103RCT6** (LQFP64, Cortex-M3, 72 MHz, 256 KB flash, 48 KB RAM)
- HSE: 8 MHz crystal → PLL ×9 → 72 MHz SYSCLK
- Boot pins: BOOT0 button on board (UART/USB DFU bootloader), RESET button

## Programming / Debug
- **SWD header (item ㉑)** — exposed on V3.0 board, intended for ST-Link / J-Link
  - Pins: VCC (3.3V), SWDIO (PA13), SWCLK (PA14), GND
  - Use `arm-none-eabi-gdb` + `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg`

## Motor Drive (4× AM2861 H-bridge)
Each motor uses 2 PWM signals (IN1/IN2) for direction control. 4 motors × 2 = 8 PWM outputs across **TIM1** + **TIM8**.

| Motor | Position | TIM1 PWM | TIM8 PWM |
|-------|----------|----------|----------|
| M1    | Front-Left  | PA8  (CH1)  | PC6 (CH1) |
| M2    | Rear-Left   | PB0  (CH2N) | PC7 (CH2) |
| M3    | Front-Right | PB1  (CH3N) | PC8 (CH3) |
| M4    | Rear-Right  | PA11 (CH4)  | PC9 (CH4) |

PWM resolution: 3600 counts (per Yahboom reference). 0 = stop, ±(3600 − dead-zone).
Dead-zone (`MOTOR_IGNORE_PULSE`): ~30 counts default — below this the motor stalls.

## Encoders (quadrature, 4× wheels)

| Motor | Timer | Mode | Pins (typical, verify on scope) |
|-------|-------|------|-------------------------------|
| M1 (FL) | TIM2 | Encoder TI1+TI2, ×4 | PA0 (CH1), PA1 (CH2) |
| M2 (RL) | TIM4 | Encoder TI1+TI2, ×4 | PB6 (CH1), PB7 (CH2) |
| M3 (FR) | TIM5 | Encoder TI1+TI2, ×4 | PA0 (CH1), PA1 (CH2)? remap likely |
| M4 (RR) | TIM3 | Encoder TI1+TI2, ×4 | PA6 (CH1), PA7 (CH2) |

> **TBD**: TIM5 default pins overlap TIM2. The schematic remap must be verified — test on hardware by spinning each wheel and watching counter direction.

Encoder spec (from CLAUDE.md, verified):
- **1320 counts/rev** = 30:1 gear × 11 PPR × 4× quadrature
- 16-bit hardware counter, sample at 100 Hz, accumulate in 32-bit software counter

## IMU — ICM20948 (9-axis)
**SPI2** (NOT bit-bang I2C — this corrects an error in CLAUDE.md that described stock-firmware behavior).

| Signal | Pin | Function |
|--------|-----|----------|
| SCLK   | PB13 | SPI2_SCK |
| MISO   | PB14 | SPI2_MISO (= ICM20948 SDO/AD0) |
| MOSI   | PB15 | SPI2_MOSI (= ICM20948 SDA/SDI) |
| CS     | PB12 | GPIO output, software-controlled NSS |

Magnetometer: AK09916 (embedded inside ICM20948), accessed via I2C-master mode of the ICM20948 itself (no separate bus).

Defaults from Yahboom reference:
- Gyro full scale: ±2000 dps
- Accel full scale: ±16 g
- Sample rate divider: 0 (max rate)
- Low-pass filters: bypassed (cfg=0)

## UART (Host link)
**USART1** to onboard CH340 USB-serial bridge → exposes as `/dev/ttyUSB0` (or `/dev/roscar_board` via udev rule).

| Signal | Pin |
|--------|-----|
| TX     | PA9  |
| RX     | PA10 |

- Baud: 115200, 8N1
- TX uses DMA1_Channel4 (USART1_TX)
- RX uses interrupt + ring buffer (per Yahboom example)

## Buzzer
- Pin: TBD (likely PB5 or similar — verify with active-buzzer beep test)
- Active-high GPIO output, drives the AM2875 buzzer driver on board

## RGB LED bar (WS2812)
- Pin: TBD — typically uses TIM PWM + DMA for WS2812 timing
- Out of scope for v1 firmware (drive zero brightness on boot to keep dark)

## Battery voltage (ADC)
- Pin: TBD — analog input through resistor divider
- ADC1, single-channel, periodic sample at ~10 Hz

## Servo outputs (PWM, 4× channels)
- Out of scope for v1 firmware (we don't use servos on roscar)
- Documented in PDFs: TIM3 channels via remap, 50 Hz PWM, voltage-switchable 5V/6.8V rail

## CAN bus, SBUS
- Out of scope for v1 firmware

## On-board I2C header (item ⑦)
- External user I2C (e.g. for OLED) — out of scope for v1

## Notes & Gotchas
- **Board mounted 180° rotated** on chassis (see CLAUDE.md). Motor wiring is physically corrected at the connectors so M1=FL etc. matches board labels with correct polarity. IMU axes still need software correction (negate ax/ay/az and gx/gy/gz) — this lives in firmware now, so the ROS driver no longer needs to do it.
- **Power**: motor drivers need 12 V on the T-jack for motor current; logic comes from on-board regulator. **Do NOT** try to drive motors on USB-only 5 V.
- **AM2861 H-bridge**: complementary PWM with TIM1 CH2N/CH3N is intentional — those AM2861 inputs are wired to the inverted timer outputs to drive the H-bridge correctly. The ITR/MOE bits and dead-time generator must be configured properly for advanced-control timers.
