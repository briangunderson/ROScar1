#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "bsp_motor.h"

/* Mecanum inverse kinematics: body twist (vx, vy, wz) → wheel ω (rad/s).
 * Output array indexed by motor_id_t. */
void mecanum_ik(float vx, float vy, float wz, float wheel_rads[MOTOR_COUNT]);

/* Forward kinematics: per-wheel ω (rad/s) → body twist (vx, vy, wz). */
void mecanum_fk(const float wheel_rads[MOTOR_COUNT],
                float *vx, float *vy, float *wz);

/* Convert encoder count delta over `dt_s` to wheel angular velocity (rad/s). */
float encoder_delta_to_radps(int32_t delta_counts, float dt_s);

#endif
