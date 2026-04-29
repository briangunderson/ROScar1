#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float e_prev;       /* e(k-1)  */
    float e_pp;         /* e(k-2)  */
    float u;            /* clamped output */
    float u_min, u_max;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd, float u_min, float u_max);
void  pid_reset(pid_t *p);
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);
float pid_step(pid_t *p, float setpoint, float measured);

#endif
