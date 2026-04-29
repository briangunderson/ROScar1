#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_FL = 0,   /* M1: front-left  (TIM1_CH1 / TIM8_CH1) */
    MOTOR_RL,       /* M2: rear-left   (TIM1_CH2N / TIM8_CH2) */
    MOTOR_FR,       /* M3: front-right (TIM1_CH3N / TIM8_CH3) */
    MOTOR_RR,       /* M4: rear-right  (TIM1_CH4 / TIM8_CH4) */
    MOTOR_COUNT = 4
} motor_id_t;

void motor_init(void);

/* signed_pwm: −MOTOR_PWM_MAX … +MOTOR_PWM_MAX; sign sets direction. */
void motor_set(motor_id_t m, int32_t signed_pwm);

/* Stop all motors. brake=1 → short-brake (both half-bridges low),
 *                  brake=0 → free-run (both PWM duty=0).            */
void motor_stop_all(int brake);

#endif
