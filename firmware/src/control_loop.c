#include "control_loop.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_systime.h"
#include "kinematics.h"
#include "pid.h"
#include "config.h"

#include <math.h>

robot_state_t g_state;
static pid_t  g_pid[MOTOR_COUNT];

/* Smoothed setpoint with deceleration limiter (per-axis). */
static float vx_cur, vy_cur, wz_cur;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void control_loop_init(void) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        pid_init(&g_pid[i],
                 PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                 -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    }
    g_state.last_cmd_ms = millis();
    g_state.cmd_active  = 0;
    vx_cur = vy_cur = wz_cur = 0.0f;
}

void control_loop_set_target(float vx, float vy, float wz) {
    g_state.vx = clampf(vx, -VX_MAX, VX_MAX);
    g_state.vy = clampf(vy, -VY_MAX, VY_MAX);
    g_state.wz = clampf(wz, -WZ_MAX, WZ_MAX);
    g_state.last_cmd_ms = millis();
    g_state.cmd_active  = 1;
}

void control_loop_estop(void) {
    g_state.vx = g_state.vy = g_state.wz = 0.0f;
    vx_cur = vy_cur = wz_cur = 0.0f;
    g_state.cmd_active = 0;
    motor_stop_all(0);
    for (int i = 0; i < MOTOR_COUNT; ++i) pid_reset(&g_pid[i]);
}

/* Approach `target` from `cur` using a per-step rate limit. */
static float approach(float cur, float target, float max_step) {
    float diff = target - cur;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return cur + diff;
}

void control_loop_tick(void) {
    /* 1. cmd_vel watchdog */
    if (elapsed(g_state.last_cmd_ms, CMDVEL_TIMEOUT_MS)) {
        g_state.cmd_active = 0;
        g_state.vx = g_state.vy = g_state.wz = 0.0f;
    }

    /* 2. Deceleration limiter on the body twist setpoint. Acceleration
     *    is unrestricted (matches the existing roscar_driver.py behaviour). */
    const float dt = CONTROL_LOOP_DT;
    float lin_step = MAX_DECEL_LIN * dt;
    float ang_step = MAX_DECEL_ANG * dt;

    /* Only limit when slowing down toward zero / past zero. Otherwise step
     * directly to target. */
    if (fabsf(g_state.vx) < fabsf(vx_cur)) vx_cur = approach(vx_cur, g_state.vx, lin_step);
    else                                   vx_cur = g_state.vx;
    if (fabsf(g_state.vy) < fabsf(vy_cur)) vy_cur = approach(vy_cur, g_state.vy, lin_step);
    else                                   vy_cur = g_state.vy;
    if (fabsf(g_state.wz) < fabsf(wz_cur)) wz_cur = approach(wz_cur, g_state.wz, ang_step);
    else                                   wz_cur = g_state.wz;

    /* 3. Sample encoders, compute measured wheel ω. */
    encoder_sample();
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        g_state.wheel_measured[i] = encoder_delta_to_radps(encoder_delta(i), dt);
    }

    /* 4. IK on smoothed setpoint */
    mecanum_ik(vx_cur, vy_cur, wz_cur, g_state.wheel_setpoint);

    /* 5. FK on measured for /odom_raw publication */
    mecanum_fk(g_state.wheel_measured,
               &g_state.vx_meas, &g_state.vy_meas, &g_state.wz_meas);

    /* 6. PID per wheel → PWM */
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        float u = pid_step(&g_pid[i],
                           g_state.wheel_setpoint[i],
                           g_state.wheel_measured[i]);
        g_state.pwm[i] = (int32_t)u;
        motor_set((motor_id_t)i, g_state.pwm[i]);
    }
}
