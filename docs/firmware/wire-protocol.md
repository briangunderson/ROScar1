# YB-ERF01 Wire Protocol (USART1 ↔ Host)

Byte-level spec of the Yahboom Rosmaster serial protocol that our firmware must
emit and parse so the existing ROS2 driver (`roscar_driver/driver_node.py` →
`Rosmaster_Lib`) works unchanged. Reverse-engineered from
[`Rosmaster_Lib.py` v3.3.1](https://www.yahboom.net/study/ROS-Driver-Board).

## Link Layer
- **USART1**, 115200 baud, 8N1, no flow control.
- Host (RPi) talks to the on-board CH340 at `/dev/ttyUSB0` (or our udev symlink
  `/dev/roscar_board`).
- All multi-byte numeric fields are **little-endian** (`struct.pack('h', …)`).

## Frame Format

```
+------+----------+-----+------+-----------------+----------+
| HEAD | DEVICE   | LEN | FUNC |    PAYLOAD      | CHECKSUM |
| 0xFF | (1 byte) |  L  |  F   |   N bytes       |  1 byte  |
+------+----------+-----+------+-----------------+----------+
```

| Field    | Size | Notes |
|----------|------|-------|
| HEAD     | 1    | Always `0xFF`. |
| DEVICE   | 1    | TX (host→firmware): `0xFC`. RX (firmware→host): `0xFB` (= `DEVICE_ID − 1`). |
| LEN      | 1    | Number of bytes after LEN, **including** FUNC, PAYLOAD, and CHECKSUM. So `LEN = N + 2`. |
| FUNC     | 1    | Function code (see tables below). |
| PAYLOAD  | N    | Function-specific. |
| CHECKSUM | 1    | `(LEN + FUNC + sum(PAYLOAD)) mod 256` |

> The Python source uses `sum(cmd, COMPLEMENT) & 0xff` where
> `COMPLEMENT = 257 − DEVICE_ID = 5` and `cmd` is `[HEAD, DEVICE, LEN, FUNC, …PAYLOAD]`.
> The HEAD + DEVICE + COMPLEMENT terms add to `0xFF + 0xFC + 5 = 0x200 ≡ 0 mod 256`,
> so checksum reduces to `(LEN + FUNC + sum(PAYLOAD)) mod 256`. Verified against
> the RX path in `__receive_data` which only sums LEN + FUNC + payload bytes.

### Checksum pseudocode (C)

```c
static uint8_t checksum(uint8_t len, uint8_t func, const uint8_t *payload, size_t n) {
    uint32_t s = (uint32_t)len + (uint32_t)func;
    for (size_t i = 0; i < n; ++i) s += payload[i];
    return (uint8_t)(s & 0xFF);
}
```

## Function Codes

### Required for ROScar1 (driver_node.py uses these)

| Code | Name | Dir | Payload | Notes |
|------|------|-----|---------|-------|
| 0x01 | `FUNC_AUTO_REPORT`     | TX | `enable u8`, `forever u8` | `enable=1` starts the four-packet auto-report stream; `forever=0x5F` writes to flash. |
| 0x0F | `FUNC_RESET_STATE`     | TX | `0x5F` (magic) | Stop motors, lights off, buzzer off. |
| 0x12 | `FUNC_MOTION`          | TX | `car_type u8`, `vx i16`, `vy i16`, `wz i16` | Velocities in **mm/s** and **mrad/s** (×1000). Bit 7 of `car_type` (0x80) enables MCU yaw-PID assist. |
| 0x22 | `FUNC_UART_SERVO_TORQUE` | TX | `enable u8` | Driver sends this once at startup (`Rosmaster.__init__` → `set_uart_servo_torque(1)`). Firmware can no-op. |

### Auto-report packets (firmware → host, broadcast every 10 ms)

The firmware sends four packets per cycle, one per 10 ms tick = ~40 ms full cycle.
Recommended order: SPEED, ICM_RAW, IMU_ATT (or MAG), ENCODER.

| Code | Name | Payload | Layout |
|------|------|---------|--------|
| 0x0A | `FUNC_REPORT_SPEED`    | 7 B | `vx i16`, `vy i16`, `wz i16`, `bat_x10 u8` — velocities ×1000, battery in 0.1 V units (e.g. 84 = 8.4 V). |
| 0x0E | `FUNC_REPORT_ICM_RAW`  | 18 B | `gx,gy,gz` i16 ×1000 (rad/s × 1000), `ax,ay,az` i16 ×10000 (m/s² × ?), `mx,my,mz` i16 ×1000. |
| 0x0D | `FUNC_REPORT_ENCODER`  | 16 B | `m1,m2,m3,m4` int32 (cumulative encoder counts). |
| 0x0C | `FUNC_REPORT_IMU_ATT`  | 6 B  | `roll, pitch, yaw` i16 ×10000 (radians). Optional. |
| 0x09 | `FUNC_REPORT_MAG`      | 6 B  | `mx, my, mz` i16 ×10000. Optional (duplicates ICM_RAW mag). |
| 0x0B | `FUNC_REPORT_MPU_RAW`  | 18 B | Legacy MPU9250 scaling — **don't emit on V3.0**. Driver auto-detects which packet arrives. |

### Optional / nice-to-have (commented in driver, useful later)

| Code | Name | Dir | Payload |
|------|------|-----|---------|
| 0x02 | `FUNC_BEEP`            | TX | `on_time i16` (ms; 1 = on; 0 = off; ≥10 = auto-off after N ms) |
| 0x05 | `FUNC_RGB`             | TX | `led_id u8, r u8, g u8, b u8` |
| 0x06 | `FUNC_RGB_EFFECT`      | TX | `effect u8, speed u8, parm u8` |
| 0x10 | `FUNC_MOTOR`           | TX | 4× signed i8 (open-loop PWM, −100..100) |
| 0x11 | `FUNC_CAR_RUN`         | TX | `car_type u8, state u8, speed i16` (state-machine motion) |
| 0x13 | `FUNC_SET_MOTOR_PID`   | TX | `kp i16, ki i16, kd i16, save u8` (×1000, save=0x5F → flash) |
| 0x15 | `FUNC_SET_CAR_TYPE`    | TX | `car_type u8, save u8` |
| 0x50 | `FUNC_REQUEST_DATA`    | TX | `function u8, parm u8` (request a one-shot report) |
| 0x51 | `FUNC_VERSION`         | bidir | TX: empty. RX: `H u8, L u8` (e.g. 03 03 = v3.3) |
| 0xA0 | `FUNC_RESET_FLASH`     | TX | `0x5F` magic — factory reset |

### Codes to reject silently (servo/arm — not present on ROScar1)

`0x03 PWM_SERVO`, `0x04 PWM_SERVO_ALL`, `0x07 PORT_ON_OFF`, `0x14 SET_YAW_PID`,
`0x20 UART_SERVO`, `0x21 UART_SERVO_ID`, `0x23 ARM_CTRL`, `0x24 ARM_OFFSET`,
`0x30 AKM_DEF_ANGLE`, `0x31 AKM_STEER_ANGLE`. Read frame, validate checksum,
discard payload — do not crash.

## Init Sequence (host startup)

In order:

1. Open `/dev/ttyUSB0` at 115200.
2. Wait for serial port up (CH340 enumerates instantly).
3. Send `FUNC_UART_SERVO_TORQUE { enable=1 }` — fire-and-forget, no reply expected.
4. Driver starts its RX thread (`create_receive_threading`).
5. Send `FUNC_AUTO_REPORT { enable=1, forever=0 }` — firmware MUST start
   broadcasting the auto-report packets immediately on receiving this.
6. Driver subscribes to `/cmd_vel` and starts publishing.

The firmware should:
- On reset, **NOT** auto-report by default — wait for the explicit
  `FUNC_AUTO_REPORT { enable=1 }`. (This matches Rosmaster_Lib expectation that
  the host enables it post-connect, and gives a clean startup log.)
- Persist `enable` to flash if `forever=0x5F`.
- Tolerate the `set_uart_servo_torque(1)` packet — accept and discard.

## Wire Examples

### `set_car_motion(0.3, 0.0, 0.0)` (drive forward 300 mm/s, X3 mecanum)
```
FF FC 0A 12 01 2C 01 00 00 00 00 4A
└─ ─┘ ─┘ ─┘ ─┘ ──── ──── ──── ─┘
HD DEV LN FN ct  vx   vy   wz  CK
```
- `LEN = 0x0A = 10` (FUNC + 7-byte payload + checksum)
- `car_type = 0x01` (CARTYPE_X3)
- `vx = 0x012C = 300` mm/s, `vy = 0`, `wz = 0`
- `CHECKSUM = (10 + 0x12 + 1 + 0x2C + 1 + 0 + 0 + 0 + 0) mod 256 = 0x4A` ✓

### `set_auto_report_state(enable=True, forever=False)`
```
FF FC 05 01 01 00 07
```
- `LEN = 5`, payload `01 00`, `CHECKSUM = (5+1+1+0) = 7` ✓

### `reset_car_state()`
```
FF FC 04 0F 5F 72
```
- `LEN = 4`, payload `5F`, `CHECKSUM = (4+0x0F+0x5F) mod 256 = 0x72` ✓

### `FUNC_REPORT_SPEED` (firmware → host, vx=200 mm/s, batt=11.8 V)
```
FF FB 0A 0A C8 00 00 00 00 00 76 4E
```
- DEVICE = `0xFB` (firmware → host)
- `LEN = 10`, FUNC = `0x0A`, payload = `vx=200, vy=0, wz=0, bat=118 (=11.8 V × 10)`
- `CHECKSUM = (10+10+200+0+0+0+0+0+118) mod 256 = 0x4E` ✓

## Receiver State Machine (for our STM32 firmware)

```
IDLE  →  read 1 byte
         │
         ├── 0xFF → state HEAD1
         └── else → discard, stay IDLE

HEAD1 →  read 1 byte
         │
         ├── 0xFC → state LEN
         └── else → state IDLE  (resync; do NOT consume — protocol has only one valid header)

LEN   →  read 1 byte → save as `len`
         if len < 3 or len > MAX_LEN: state IDLE
         else state FUNC

FUNC  →  read 1 byte → save as `func`
         payload_count = len - 2     // bytes left = payload + checksum
         state PAYLOAD

PAYLOAD → read until payload_count == 0
          last byte = checksum, prior bytes = payload
          if checksum matches: dispatch(func, payload, payload_len)
          state IDLE
```

A 1-byte timeout between bytes can also reset to IDLE to recover from glitches.

## Numeric Conventions Reference

| Quantity | On wire | Unit | Example |
|----------|---------|------|---------|
| Linear velocity | i16 | mm/s | 300 = 0.3 m/s |
| Angular velocity | i16 | mrad/s | 1000 = 1.0 rad/s |
| Gyro (ICM20948) | i16 ×1000 | rad/s | 1500 = 1.5 rad/s |
| Accel (ICM20948) | i16 ×10000 | m/s² | 98100 (≈ +1 g)? CLAUDE.md notes accel ratio = 1/10000, so 9810 = 0.981 m/s² ... TBD verify with `ros2 topic echo /imu/data_raw` |
| Mag (ICM20948) | i16 ×1000 | µT (or sensor LSB?) | TBD |
| Encoder | i32 | counts | 1320 / rev |
| Battery | u8 | 0.1 V | 118 = 11.8 V |
| Roll/Pitch/Yaw | i16 ×10000 | rad | 7853 ≈ π/4 |

> ⚠️ Accel scaling needs hardware verification — the Python ratio `1/10000` implies
> the firmware sends m/s² × 10000, but the ICM20948 native units (LSB at ±16g
> = 2048 LSB/g) suggest the original firmware may emit raw LSB counts that
> *happen* to divide cleanly. We'll measure on bench and document the chosen
> convention here.
