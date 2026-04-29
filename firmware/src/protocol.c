#include "protocol.h"
#include "bsp_uart.h"
#include "bsp_systime.h"
#include "bsp_encoder.h"
#include "bsp_icm20948.h"
#include "bsp_adc.h"
#include "control_loop.h"
#include "config.h"

#include <string.h>

/* ── Receive parser state machine ─────────────────────────────────────── */
typedef enum {
    RX_IDLE = 0,
    RX_HEAD1,
    RX_LEN,
    RX_FUNC,
    RX_PAYLOAD,
} rx_state_t;

static rx_state_t rx_state;
static uint8_t    rx_len;        /* bytes-after-LEN as advertised */
static uint8_t    rx_func;
static uint8_t    rx_payload[PROTO_MAX_PAYLOAD];
static uint8_t    rx_count;      /* payload bytes received so far  */

/* ── Auto-report scheduler ────────────────────────────────────────────── */
static bool     ar_enabled;
static uint32_t ar_last_ms;
static uint8_t  ar_phase;        /* rotates 0..3, one packet per 10 ms tick */

static void send_report_speed(void);
static void send_report_icm(void);
static void send_report_encoder(void);
static void send_report_imu_att(void);

/* ── Frame send ──────────────────────────────────────────────────────── */
void protocol_send(uint8_t func, const uint8_t *payload, uint8_t payload_len) {
    if (payload_len > PROTO_MAX_PAYLOAD) return;
    uint8_t frame[5 + PROTO_MAX_PAYLOAD];
    uint8_t len = (uint8_t)(payload_len + 2);   /* func + payload + checksum */
    frame[0] = PROTO_HEAD;
    frame[1] = PROTO_DEV_RX;
    frame[2] = len;
    frame[3] = func;
    memcpy(&frame[4], payload, payload_len);

    uint32_t s = (uint32_t)len + (uint32_t)func;
    for (uint8_t i = 0; i < payload_len; ++i) s += payload[i];
    frame[4 + payload_len] = (uint8_t)(s & 0xFFu);

    uart_tx_write(frame, (size_t)(5 + payload_len));
}

/* ── Handlers for inbound frames ──────────────────────────────────────── */
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static void on_motion(const uint8_t *payload, uint8_t len) {
    if (len < 7) return;
    /* payload[0] = car_type (high bit = adjust enable) — we ignore type for
     * mecanum since that's the only kinematics this firmware implements.
     * The adjust bit (yaw-PID) is also a no-op for now. */
    int16_t vx_mm  = rd_i16(&payload[1]);
    int16_t vy_mm  = rd_i16(&payload[3]);
    int16_t wz_mr  = rd_i16(&payload[5]);

    float vx = (float)vx_mm / VEL_WIRE_SCALE;
    float vy = (float)vy_mm / VEL_WIRE_SCALE;
    float wz = (float)wz_mr / ANG_WIRE_SCALE;

    control_loop_set_target(vx, vy, wz);
}

static void on_auto_report(const uint8_t *payload, uint8_t len) {
    if (len < 2) return;
    /* payload[0] = enable, payload[1] = forever (0x5F = persist). For v1 we
     * don't persist to flash — every reboot starts disabled. */
    protocol_set_auto_report(payload[0] != 0);
}

static void on_reset_state(const uint8_t *payload, uint8_t len) {
    (void)payload; (void)len;
    control_loop_estop();
}

static void on_uart_servo_torque(const uint8_t *payload, uint8_t len) {
    /* No-op — Yahboom's Rosmaster.__init__ sends this once at startup. */
    (void)payload; (void)len;
}

static void on_version(const uint8_t *payload, uint8_t len) {
    (void)payload; (void)len;
    /* Reply with firmware version. */
    uint8_t reply[2] = { 99, 1 };   /* 99.1 → "ROScar1 v1" */
    protocol_send(FUNC_VERSION, reply, 2);
}

/* ── Dispatch ────────────────────────────────────────────────────────── */
static void dispatch_frame(uint8_t func, const uint8_t *payload, uint8_t payload_len) {
    switch (func) {
        case FUNC_MOTION:           on_motion(payload, payload_len); break;
        case FUNC_AUTO_REPORT:      on_auto_report(payload, payload_len); break;
        case FUNC_RESET_STATE:      on_reset_state(payload, payload_len); break;
        case FUNC_UART_SERVO_TORQUE:on_uart_servo_torque(payload, payload_len); break;
        case FUNC_VERSION:          on_version(payload, payload_len); break;

        /* Recognized but unused on ROScar1 — silently accept. */
        case FUNC_BEEP:
        case FUNC_RGB:
        case FUNC_RGB_EFFECT:
        case FUNC_PORT_ON_OFF:
        case FUNC_PWM_SERVO:
        case FUNC_PWM_SERVO_ALL:
        case FUNC_MOTOR:
        case FUNC_CAR_RUN:
        case FUNC_SET_MOTOR_PID:    /* TODO: live tuning support */
        case FUNC_SET_CAR_TYPE:
        case FUNC_UART_SERVO:
        case FUNC_UART_SERVO_ID:
        case FUNC_ARM_CTRL:
        case FUNC_ARM_OFFSET:
        case FUNC_AKM_DEF_ANGLE:
        case FUNC_AKM_STEER_ANGLE:
        case FUNC_REQUEST_DATA:
        case FUNC_RESET_FLASH:
            break;

        default:
            /* Unknown function code — drop. */
            break;
    }
}

/* ── State machine ───────────────────────────────────────────────────── */
void protocol_feed(uint8_t b) {
    switch (rx_state) {
        case RX_IDLE:
            if (b == PROTO_HEAD) rx_state = RX_HEAD1;
            break;

        case RX_HEAD1:
            if (b == PROTO_DEV_TX) {
                rx_state = RX_LEN;
            } else if (b == PROTO_HEAD) {
                /* still HEAD — stay here */
            } else {
                rx_state = RX_IDLE;
            }
            break;

        case RX_LEN:
            if (b < 3 || (b - 2) > PROTO_MAX_PAYLOAD) {
                rx_state = RX_IDLE;
            } else {
                rx_len = b;
                rx_state = RX_FUNC;
            }
            break;

        case RX_FUNC:
            rx_func  = b;
            rx_count = 0;
            rx_state = RX_PAYLOAD;
            break;

        case RX_PAYLOAD: {
            uint8_t data_left = (uint8_t)(rx_len - 2u - rx_count);  /* still to come incl checksum */
            if (data_left == 1u) {
                /* This byte is the checksum. */
                uint32_t s = (uint32_t)rx_len + (uint32_t)rx_func;
                for (uint8_t i = 0; i < rx_count; ++i) s += rx_payload[i];
                if ((uint8_t)(s & 0xFFu) == b) {
                    dispatch_frame(rx_func, rx_payload, rx_count);
                }
                rx_state = RX_IDLE;
            } else {
                if (rx_count < PROTO_MAX_PAYLOAD) {
                    rx_payload[rx_count++] = b;
                } else {
                    rx_state = RX_IDLE;
                }
            }
            break;
        }
    }
}

/* ── Auto-report packets ─────────────────────────────────────────────── */
static void wr_i16(uint8_t *p, int32_t v) {
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void wr_i32(uint8_t *p, int32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >>  8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void send_report_speed(void) {
    uint8_t pl[7];
    wr_i16(&pl[0], (int32_t)(g_state.vx_meas * VEL_WIRE_SCALE));
    wr_i16(&pl[2], (int32_t)(g_state.vy_meas * VEL_WIRE_SCALE));
    wr_i16(&pl[4], (int32_t)(g_state.wz_meas * ANG_WIRE_SCALE));
    float bv = battery_volts();
    int32_t b10 = (int32_t)(bv * BAT_WIRE_SCALE + 0.5f);
    if (b10 < 0) b10 = 0;
    if (b10 > 255) b10 = 255;
    pl[6] = (uint8_t)b10;
    protocol_send(FUNC_REPORT_SPEED, pl, sizeof(pl));
}

static void send_report_icm(void) {
    imu_raw_t g, a, m;
    icm20948_wire_values(&g, &a, &m);
    uint8_t pl[18];
    wr_i16(&pl[0],  g.x); wr_i16(&pl[2],  g.y); wr_i16(&pl[4],  g.z);
    wr_i16(&pl[6],  a.x); wr_i16(&pl[8],  a.y); wr_i16(&pl[10], a.z);
    wr_i16(&pl[12], m.x); wr_i16(&pl[14], m.y); wr_i16(&pl[16], m.z);
    protocol_send(FUNC_REPORT_ICM_RAW, pl, sizeof(pl));
}

static void send_report_encoder(void) {
    uint8_t pl[16];
    wr_i32(&pl[0],  encoder_total(MOTOR_FL));
    wr_i32(&pl[4],  encoder_total(MOTOR_RL));
    wr_i32(&pl[8],  encoder_total(MOTOR_FR));
    wr_i32(&pl[12], encoder_total(MOTOR_RR));
    protocol_send(FUNC_REPORT_ENCODER, pl, sizeof(pl));
}

static void send_report_imu_att(void) {
    /* Onboard fusion is out-of-scope for v1; emit zeros so the host gets a
     * well-formed packet but relies on imu_filter_madgwick for orientation. */
    uint8_t pl[6] = {0};
    protocol_send(FUNC_REPORT_IMU_ATT, pl, sizeof(pl));
}

/* ── Pump ────────────────────────────────────────────────────────────── */
void protocol_init(void) {
    rx_state = RX_IDLE;
    ar_enabled = false;
    ar_last_ms = 0;
    ar_phase   = 0;
}

void protocol_set_auto_report(bool enable) {
    ar_enabled = enable;
    ar_phase   = 0;
    ar_last_ms = millis();
}

void protocol_pump(uint32_t now_ms) {
    /* Drain RX into the parser. */
    while (uart_rx_available()) {
        uint16_t v = uart_rx_pop();
        if (v <= 0xFFu) protocol_feed((uint8_t)v);
    }

    /* Auto-report cadence: 1 packet every 10 ms, 4-packet rotation. */
    if (!ar_enabled) return;
    if ((uint32_t)(now_ms - ar_last_ms) < 10u) return;
    ar_last_ms += 10;        /* fixed cadence; falls behind on overrun */
    switch (ar_phase) {
        case 0: send_report_speed();    break;
        case 1: send_report_icm();      break;
        case 2: send_report_imu_att();  break;
        case 3: send_report_encoder();  break;
    }
    ar_phase = (uint8_t)((ar_phase + 1u) & 0x03u);
}
