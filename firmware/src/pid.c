#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd, float u_min, float u_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->u_min = u_min; p->u_max = u_max;
    pid_reset(p);
}

void pid_reset(pid_t *p) {
    p->e_prev = 0.0f;
    p->e_pp   = 0.0f;
    p->u      = 0.0f;
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd;
}

/* Incremental PID:
 *   Δu = Kp(e − e_prev) + Ki·e + Kd(e − 2·e_prev + e_pp)
 *   u  = clamp(u + Δu)
 */
float pid_step(pid_t *p, float setpoint, float measured) {
    float e   = setpoint - measured;
    float du  = p->kp * (e - p->e_prev)
              + p->ki * e
              + p->kd * (e - 2.0f * p->e_prev + p->e_pp);
    float u   = p->u + du;
    if (u > p->u_max) u = p->u_max;
    if (u < p->u_min) u = p->u_min;
    p->u      = u;
    p->e_pp   = p->e_prev;
    p->e_prev = e;
    return u;
}
