#include "stm32f1xx_hal.h"
#include "bsp_motor.h"
#include "config.h"

/* Per-motor: 2 PWM signals, one on TIM1, one on TIM8. The AM2861 H-bridge takes
 * a pair of inputs (IN1, IN2). To go forward, drive IN1=PWM, IN2=0. To go
 * reverse, drive IN1=0, IN2=PWM. To brake (short across motor), drive both
 * IN1=PWM_MAX and IN2=PWM_MAX. To coast, both 0.
 *
 * Per Yahboom motor PDF:
 *   M1 (FL): TIM1_CH1  (PA8)  + TIM8_CH1 (PC6)
 *   M2 (RL): TIM1_CH2N (PB0)  + TIM8_CH2 (PC7)
 *   M3 (FR): TIM1_CH3N (PB1)  + TIM8_CH3 (PC8)
 *   M4 (RR): TIM1_CH4  (PA11) + TIM8_CH4 (PC9)
 *
 * TIM1 is an advanced-control timer — channels driven by CH2N/CH3N use the
 * complementary output, which requires MOE and dead-time configuration.
 */

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim8;

typedef struct {
    TIM_HandleTypeDef *htim_a;     /* TIM1 channel — IN1 side */
    uint32_t           ch_a;
    int                ch_a_is_complementary;
    TIM_HandleTypeDef *htim_b;     /* TIM8 channel — IN2 side */
    uint32_t           ch_b;
} motor_pair_t;

static motor_pair_t pairs[MOTOR_COUNT] = {
    [MOTOR_FL] = { &htim1, TIM_CHANNEL_1, 0, &htim8, TIM_CHANNEL_1 },
    [MOTOR_RL] = { &htim1, TIM_CHANNEL_2, 1, &htim8, TIM_CHANNEL_2 },
    [MOTOR_FR] = { &htim1, TIM_CHANNEL_3, 1, &htim8, TIM_CHANNEL_3 },
    [MOTOR_RR] = { &htim1, TIM_CHANNEL_4, 0, &htim8, TIM_CHANNEL_4 },
};

static void gpio_init_pwm(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    /* TIM1 outputs: PA8 (CH1), PA11 (CH4), PB0 (CH2N), PB1 (CH3N) */
    g.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &g);

    /* TIM8 outputs: PC6 (CH1), PC7 (CH2), PC8 (CH3), PC9 (CH4) */
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &g);
}

static void timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *inst) {
    htim->Instance               = inst;
    htim->Init.Prescaler         = 0;                       /* 72 MHz tick */
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = MOTOR_PWM_MAX - 1;       /* 20 kHz @ 72MHz */
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(htim);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity= TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_4);

    /* Advanced-control extras for TIM1 / TIM8: enable Main Output, configure
     * dead-time generator (small DT — AM2861 doesn't actually need it, but
     * MOE must be set for any output to drive the pin). */
    if (inst == TIM1 || inst == TIM8) {
        TIM_BreakDeadTimeConfigTypeDef bd = {0};
        bd.OffStateRunMode  = TIM_OSSR_DISABLE;
        bd.OffStateIDLEMode = TIM_OSSI_DISABLE;
        bd.LockLevel        = TIM_LOCKLEVEL_OFF;
        bd.DeadTime         = 0;
        bd.BreakState       = TIM_BREAK_DISABLE;
        bd.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
        bd.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
        HAL_TIMEx_ConfigBreakDeadTime(htim, &bd);
    }
}

void motor_init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    gpio_init_pwm();
    timer_init(&htim1, TIM1);
    timer_init(&htim8, TIM8);

    /* Start all 4 channels (and complementary outputs on TIM1) at 0 duty. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    motor_stop_all(0);
}

static void set_compare(TIM_HandleTypeDef *htim, uint32_t ch, uint32_t val) {
    __HAL_TIM_SET_COMPARE(htim, ch, val);
}

void motor_set(motor_id_t m, int32_t signed_pwm) {
    if (m >= MOTOR_COUNT) return;

    /* Clamp and apply deadzone. */
    int32_t v = signed_pwm;
    if (v >  MOTOR_PWM_MAX) v =  MOTOR_PWM_MAX;
    if (v < -MOTOR_PWM_MAX) v = -MOTOR_PWM_MAX;
    if (v >  0 && v <  MOTOR_DEADZONE) v = 0;
    if (v <  0 && v > -MOTOR_DEADZONE) v = 0;

    uint32_t mag = (uint32_t)(v < 0 ? -v : v);

    motor_pair_t *p = &pairs[m];
    if (v >= 0) {
        /* Forward: IN1 = duty, IN2 = 0 */
        set_compare(p->htim_a, p->ch_a, mag);
        set_compare(p->htim_b, p->ch_b, 0);
    } else {
        /* Reverse: IN1 = 0, IN2 = duty */
        set_compare(p->htim_a, p->ch_a, 0);
        set_compare(p->htim_b, p->ch_b, mag);
    }
}

void motor_stop_all(int brake) {
    uint32_t v = brake ? (uint32_t)MOTOR_PWM_MAX : 0u;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_pair_t *p = &pairs[i];
        set_compare(p->htim_a, p->ch_a, v);
        set_compare(p->htim_b, p->ch_b, v);
    }
}
