#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include "bsp_motor.h"

typedef struct {
    float vx, vy, wz;       /* most recent body twist setpoint (m/s, rad/s) */
    float vx_meas, vy_meas, wz_meas;  /* FK from encoders */
    float wheel_setpoint[MOTOR_COUNT];   /* target per-wheel ω (rad/s) */
    float wheel_measured[MOTOR_COUNT];   /* measured per-wheel ω (rad/s) */
    int32_t pwm[MOTOR_COUNT];            /* last PID output (signed PWM)   */
    uint32_t last_cmd_ms;                /* timestamp of last MOTION packet */
    uint8_t  cmd_active;                 /* 1 if within watchdog window     */
} robot_state_t;

extern robot_state_t g_state;

void control_loop_init(void);

/* Set new target body twist (called from protocol parser on FUNC_MOTION). */
void control_loop_set_target(float vx, float vy, float wz);

/* Trigger an immediate stop (FUNC_RESET_STATE). */
void control_loop_estop(void);

/* Run one 100 Hz tick: encoder_sample → FK → PID → motor_set.
 * Should be called from main loop when the 10 ms tick flag is set. */
void control_loop_tick(void);

#endif
