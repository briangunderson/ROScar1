#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* Function codes — must match Rosmaster_Lib exactly */
enum {
    FUNC_AUTO_REPORT      = 0x01,
    FUNC_BEEP             = 0x02,
    FUNC_PWM_SERVO        = 0x03,
    FUNC_PWM_SERVO_ALL    = 0x04,
    FUNC_RGB              = 0x05,
    FUNC_RGB_EFFECT       = 0x06,
    FUNC_PORT_ON_OFF      = 0x07,
    FUNC_REPORT_MAG       = 0x09,
    FUNC_REPORT_SPEED     = 0x0A,
    FUNC_REPORT_MPU_RAW   = 0x0B,
    FUNC_REPORT_IMU_ATT   = 0x0C,
    FUNC_REPORT_ENCODER   = 0x0D,
    FUNC_REPORT_ICM_RAW   = 0x0E,
    FUNC_RESET_STATE      = 0x0F,
    FUNC_MOTOR            = 0x10,
    FUNC_CAR_RUN          = 0x11,
    FUNC_MOTION           = 0x12,
    FUNC_SET_MOTOR_PID    = 0x13,
    FUNC_SET_YAW_PID      = 0x14,
    FUNC_SET_CAR_TYPE     = 0x15,
    FUNC_UART_SERVO       = 0x20,
    FUNC_UART_SERVO_ID    = 0x21,
    FUNC_UART_SERVO_TORQUE= 0x22,
    FUNC_ARM_CTRL         = 0x23,
    FUNC_ARM_OFFSET       = 0x24,
    FUNC_AKM_DEF_ANGLE    = 0x30,
    FUNC_AKM_STEER_ANGLE  = 0x31,
    FUNC_REQUEST_DATA     = 0x50,
    FUNC_VERSION          = 0x51,
    FUNC_RESET_FLASH      = 0xA0,
};

void protocol_init(void);

/* Feed bytes from UART RX into the parser. Process complete frames. */
void protocol_feed(uint8_t b);

/* Call from the main loop ~every 1 ms. Emits the next auto-report packet
 * if the auto-report cadence is due. */
void protocol_pump(uint32_t now_ms);

/* Force-enable / -disable auto-reporting. */
void protocol_set_auto_report(bool enable);

/* Build + send a single frame (used by reports and ad-hoc replies). */
void protocol_send(uint8_t func, const uint8_t *payload, uint8_t payload_len);

#endif
