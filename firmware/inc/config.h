/* ROScar1 firmware compile-time configuration.
 * Edit and re-flash to retune.
 */
#ifndef ROSCAR_CONFIG_H
#define ROSCAR_CONFIG_H

/* ── Robot geometry (measured 2026-03-16; chassis v1) ──────────────────── */
#define WHEEL_RADIUS_M      0.0397f
#define HALF_WHEELBASE_M    0.04825f   /* (wheelbase / 2) — fwd-back */
#define HALF_TRACK_M        0.05125f   /* (track / 2)     — left-right */
#define LXLY_M              (HALF_WHEELBASE_M + HALF_TRACK_M)

/* ── Encoder ───────────────────────────────────────────────────────────── */
#define ENCODER_CPR         1320       /* 30:1 gear × 11 PPR × 4× quad */

/* ── Motor PWM ─────────────────────────────────────────────────────────── */
#define MOTOR_PWM_MAX       3600
#define MOTOR_DEADZONE      30         /* counts below this can't break stiction */

/* ── Velocity limits (firmware-side clamp) ─────────────────────────────── */
#define VX_MAX              1.0f
#define VY_MAX              1.0f
#define WZ_MAX              5.0f

/* ── Loop rates ────────────────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ     100        /* 10 ms tick */
#define CONTROL_LOOP_DT     (1.0f / (float)CONTROL_LOOP_HZ)

/* ── Safety ────────────────────────────────────────────────────────────── */
#define CMDVEL_TIMEOUT_MS   500u
#define BAT_LOW_VOLTS       9.0f       /* refuse motor commands below this */
#define BAT_DIVIDER_RATIO   4.0f       /* TBD: from schematic; placeholder */
#define MAX_DECEL_LIN       1.0f       /* m/s^2 */
#define MAX_DECEL_ANG       3.0f       /* rad/s^2 */

/* ── PID defaults (per-wheel, target = wheel rad/s, output = PWM) ──────── */
#define PID_KP_DEFAULT      120.0f
#define PID_KI_DEFAULT      400.0f
#define PID_KD_DEFAULT      0.0f
#define PID_INT_LIMIT       1500.0f    /* anti-windup */

/* ── IMU axis correction (board mounted 180° rotated) ──────────────────── */
#define IMU_NEGATE_AX       1
#define IMU_NEGATE_AY       1
#define IMU_NEGATE_AZ       1
#define IMU_NEGATE_GX       1
#define IMU_NEGATE_GY       1
#define IMU_NEGATE_GZ       1
#define IMU_NEGATE_MX       1
#define IMU_NEGATE_MY       1
#define IMU_NEGATE_MZ       0

/* ── Wire protocol scaling (must match Rosmaster_Lib) ──────────────────── */
#define VEL_WIRE_SCALE      1000.0f    /* m/s   → mm/s   in i16 */
#define ANG_WIRE_SCALE      1000.0f    /* rad/s → mrad/s in i16 */
#define GYRO_WIRE_SCALE     1000.0f    /* rad/s × 1000 */
#define ACCEL_WIRE_SCALE    10000.0f   /* m/s² × 10000 (per Python lib) */
#define MAG_WIRE_SCALE      1000.0f
#define ATT_WIRE_SCALE      10000.0f   /* radians × 10000 */
#define BAT_WIRE_SCALE      10.0f      /* volts × 10 (1 byte) */

/* ── Protocol ──────────────────────────────────────────────────────────── */
#define PROTO_HEAD          0xFFu
#define PROTO_DEV_TX        0xFCu      /* host → firmware */
#define PROTO_DEV_RX        0xFBu      /* firmware → host */
#define PROTO_MAX_PAYLOAD   32u
#define PROTO_FLASH_MAGIC   0x5Fu

#endif /* ROSCAR_CONFIG_H */
