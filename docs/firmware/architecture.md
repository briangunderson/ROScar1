# ROScar1 Firmware Architecture

## Goals

1. Drop-in replacement for the Yahboom YB-ERF01-V3.0 stock firmware. Speak the
   same USART1 wire protocol as the Yahboom binary so `roscar_driver` stays
   unchanged.
2. **Source-code ownership** of mecanum kinematics, PID, encoder calibration,
   IMU corrections — eliminates the "wheel dimensions live in firmware
   functions we can't see" pain point that blocks chassis v2.
3. Minimal, hand-rolled build (Makefile + ARM GCC + STM32 HAL) — no Eclipse.
4. Safe by default: cmd-vel watchdog, brown-out reset, conservative
   acceleration limits.

## Toolchain

| Tool | Why |
|------|-----|
| `gcc-arm-none-eabi` 13.x | Compile / link |
| `STM32CubeF1` HAL v1.8.5 | Peripheral library, vendor-blessed |
| `openocd` 0.12+ | Flash via ST-Link clone |
| `arm-none-eabi-gdb` | On-chip debug |
| `picocom` / `minicom` | Serial inspection |

ST-Link V2 clone (~$3) talks to SWD header on board (item ㉑ on V3.0 silkscreen).

## Directory Layout

```
firmware/
├── Makefile                 # Top-level build
├── inc/                     # Project headers
│   ├── config.h             # Pinout, scaling constants, robot dimensions
│   ├── bsp_motor.h
│   ├── bsp_encoder.h
│   ├── bsp_icm20948.h
│   ├── bsp_uart.h
│   ├── bsp_adc.h            # battery
│   ├── bsp_led.h
│   ├── bsp_systime.h
│   ├── kinematics.h
│   ├── pid.h
│   ├── control_loop.h
│   ├── protocol.h
│   └── stm32f1xx_hal_conf.h # HAL config
├── src/
│   ├── main.c
│   ├── system_stm32f1xx.c   # CMSIS clock setup
│   ├── stm32f1xx_it.c       # IRQ handlers
│   ├── bsp_motor.c
│   ├── bsp_encoder.c
│   ├── bsp_icm20948.c
│   ├── bsp_uart.c
│   ├── bsp_adc.c
│   ├── bsp_led.c
│   ├── bsp_systime.c
│   ├── kinematics.c
│   ├── pid.c
│   ├── control_loop.c
│   ├── protocol.c
│   └── stubs.c              # _write/_read for printf via UART
├── startup/
│   └── startup_stm32f103xe.s
├── linker/
│   └── stm32f103rc.ld
├── scripts/
│   ├── flash.sh             # openocd one-liner
│   ├── debug.sh             # gdb + openocd
│   ├── erase.sh
│   └── reset.sh
└── vendor/
    └── STM32CubeF1/         # gitignored, fetched by `make hal`
```

## Software Layers

```
┌──────────────────────────────────────────────────────────────┐
│  Application                                                  │
│  ┌────────────┐  ┌───────────┐  ┌─────────────┐              │
│  │ control_   │  │ protocol  │  │ kinematics  │              │
│  │  loop      │  │ (parser+  │  │ (FK/IK)     │              │
│  │  (100 Hz)  │  │  reports) │  │ + pid.c     │              │
│  └────────────┘  └───────────┘  └─────────────┘              │
├──────────────────────────────────────────────────────────────┤
│  BSP (board support package)                                  │
│  motor   encoder   icm20948   uart(DMA TX/IRQ RX)   adc   led │
├──────────────────────────────────────────────────────────────┤
│  STM32F1xx HAL (vendored, slim subset)                        │
├──────────────────────────────────────────────────────────────┤
│  CMSIS (core_cm3.h, system_stm32f1xx.c, startup .s)           │
└──────────────────────────────────────────────────────────────┘
```

## Real-Time Schedule

| Source | Period | Action |
|--------|--------|--------|
| SysTick | 1 ms | bump `g_tick_ms` |
| TIM7 IRQ | 10 ms | control loop tick: read IMU + encoders → FK → PID → motor PWM |
| Auto-report sub-tick | 10 ms (rotating) | emit 1 of 4 RX packets each tick |
| USART1 RX IRQ | per byte | append to RX ring buffer |
| USART1 TX | DMA | drain TX ring buffer |
| Watchdog | 500 ms | if no MOTION packet in window → zero target → ramp stop |
| Battery ADC | 100 ms | low-rate sample |

The control loop and auto-report tick share the 10 ms TIM7 ISR. To keep the
ISR short, only timestamps and "due flags" are set there — actual work runs in
the main loop.

## Numerical Pipeline (Forward)

```
host /cmd_vel  --(USART1)-->  protocol.c  --[vx,vy,wz m/s,rad/s]-->
   kinematics.c IK  --[w_FL, w_FR, w_RL, w_RR rad/s]-->
   pid.c (per-wheel incremental, target vs measured)  --[duty -3600..+3600]-->
   bsp_motor.c (TIM1+TIM8 PWM)
```

## Numerical Pipeline (Reverse)

```
encoders (TIM2/3/4/5 hw counter) --[Δcounts/10ms]-->
   kinematics.c counts→rad/s  --[per-wheel ω]-->
   FK  --[vx,vy,wz body frame]-->
   protocol.c REPORT_SPEED packet (×1000 scaling)  --(USART1 DMA)--> host
```

## Configuration Constants (`config.h`)

Compile-time defines for current hardware. Runtime override via flash-stored
calibration in a future revision — for now, edit `config.h` and re-flash.

```c
/* Wheel + chassis (measured 2026-03-16 / TBD chassis v2) */
#define WHEEL_RADIUS_M      0.0397f      // 39.7 mm
#define WHEEL_BASE_X_M      0.0965f      // half wheelbase, fwd-back
#define WHEEL_BASE_Y_M      0.1025f      // half track, left-right
#define WHEEL_LXLY_M        (WHEEL_BASE_X_M + WHEEL_BASE_Y_M)

/* Encoder */
#define ENCODER_CPR         1320         // 30:1 × 11 PPR × 4×

/* Motor */
#define MOTOR_PWM_MAX       3600
#define MOTOR_DEADZONE      30           // pulses below this don't overcome stiction

/* Loop rates */
#define CONTROL_LOOP_HZ     100
#define AUTOREP_TICK_HZ     100          // 4 packets / 40 ms

/* Safety */
#define CMDVEL_TIMEOUT_MS   500
#define BAT_LOW_VOLTS       9.0f
#define MAX_DECEL_LIN       1.0f
#define MAX_DECEL_ANG       3.0f
```

## Mecanum Equations

Inverse (cmd_vel → wheel angular velocities):

```
ω_FL = (vx − vy − (lx+ly)·wz) / r
ω_FR = (vx + vy + (lx+ly)·wz) / r
ω_RL = (vx + vy − (lx+ly)·wz) / r
ω_RR = (vx − vy + (lx+ly)·wz) / r
```

Forward (wheel ω → body velocities):

```
vx = (ω_FL + ω_FR + ω_RL + ω_RR) · r / 4
vy = (−ω_FL + ω_FR + ω_RL − ω_RR) · r / 4
wz = (−ω_FL + ω_FR − ω_RL + ω_RR) · r / (4·(lx+ly))
```

## PID

Incremental form per wheel:

```
e = setpoint - measured
ΔU = Kp·(e − e_prev) + Ki·e + Kd·(e − 2·e_prev + e_pp)
U = clamp(U + ΔU, -PWM_MAX, +PWM_MAX)
```

Tunable runtime via FUNC_SET_MOTOR_PID; persisted to flash on save=0x5F.

## IMU axis correction

The board is mounted 180° rotated on chassis. CLAUDE.md says we negate
`ax,ay,az` and `gx,gy,gz`, keep `mz` and negate `mx,my`. Encoded in
`bsp_icm20948.c` so the ROS layer no longer needs to apply it.

## What v1 firmware does NOT do

- Servos (PWM and serial bus) — no-op handlers
- RGB LED bar — no-op
- CAN, SBUS — not initialised
- Madgwick/Mahony onboard fusion — emit `FUNC_REPORT_IMU_ATT` with zeros, leave
  fusion to host `imu_filter_madgwick` (matches current behaviour)
- Flash persistence of PID + car_type — punt to v2 (compile-time defaults)

## Acceptance Criteria for v1

- [ ] Builds with `make` to a single .elf and .bin
- [ ] Boots cleanly, blinks LED at 1 Hz (heartbeat)
- [ ] Responds to `set_auto_report_state(True)` by emitting all 4 RX packets at 10 ms cadence
- [ ] `set_car_motion(0.1, 0, 0)` → wheels turn forward at correct speed under PID
- [ ] Encoders count up when wheels turn forward, down when reverse
- [ ] `/odom_raw` integrates to a physically plausible pose during a 1 m forward drive
- [ ] IMU az = +9.81 ± 0.2 when board flat, gz sign matches "left turn → positive yaw rate"
- [ ] cmd_vel watchdog: kill /cmd_vel publisher → motors stop within 500 ms
- [ ] Existing `roscar_driver` Python code starts and runs without modification

## Out-of-Scope Risks (will need follow-up tasks)

- TIM5 encoder pin remap not yet verified — bench-test required.
- Buzzer pin not located in PDFs — defer until needed.
- Battery ADC channel + divider ratio TBD from schematic.
- Accel scaling (×10000 vs raw LSB) needs empirical verification on hardware.
