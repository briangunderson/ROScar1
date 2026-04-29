#include "kinematics.h"
#include "config.h"

#include <math.h>

void mecanum_ik(float vx, float vy, float wz, float wheel_rads[MOTOR_COUNT]) {
    const float r  = WHEEL_RADIUS_M;
    const float k  = LXLY_M;

    /* Standard mecanum (front-wheel A pattern, ABBA install).
     * FL/RR are A wheels; FR/RL are B wheels. */
    wheel_rads[MOTOR_FL] = (vx - vy - k * wz) / r;
    wheel_rads[MOTOR_FR] = (vx + vy + k * wz) / r;
    wheel_rads[MOTOR_RL] = (vx + vy - k * wz) / r;
    wheel_rads[MOTOR_RR] = (vx - vy + k * wz) / r;
}

void mecanum_fk(const float w[MOTOR_COUNT], float *vx, float *vy, float *wz) {
    const float r  = WHEEL_RADIUS_M;
    const float k  = LXLY_M;
    *vx = ( w[MOTOR_FL] + w[MOTOR_FR] + w[MOTOR_RL] + w[MOTOR_RR]) * r / 4.0f;
    *vy = (-w[MOTOR_FL] + w[MOTOR_FR] + w[MOTOR_RL] - w[MOTOR_RR]) * r / 4.0f;
    *wz = (-w[MOTOR_FL] + w[MOTOR_FR] - w[MOTOR_RL] + w[MOTOR_RR]) * r / (4.0f * k);
}

float encoder_delta_to_radps(int32_t delta, float dt_s) {
    /* counts / dt → rev/s → rad/s */
    return ((float)delta / (float)ENCODER_CPR) * (2.0f * (float)M_PI) / dt_s;
}
