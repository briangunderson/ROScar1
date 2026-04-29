#include "stm32f1xx_hal.h"
#include "bsp_encoder.h"
#include "config.h"

/* Per Yahboom encoder PDF:
 *   M1 (FL) → TIM2  CH1+CH2  (PA0, PA1)
 *   M2 (RL) → TIM4  CH1+CH2  (PB6, PB7)
 *   M3 (FR) → TIM5  CH1+CH2  (PA0, PA1) — overlap with TIM2! must use remap
 *   M4 (RR) → TIM3  CH1+CH2  (PA6, PA7)
 *
 * The TIM2/TIM5 overlap is resolved at the schematic level by partial remap.
 * For the V3.0 board, encoder pin assignments need bench verification —
 * we'll start with default-mode assumptions and adjust once we can see counts.
 *
 * Encoder TI1+TI2 mode 4× quad: hardware accumulates ±counts in CNT (16-bit).
 * We sample every 10 ms and accumulate into a 32-bit software counter to
 * handle wrap.
 */

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static TIM_HandleTypeDef htim5;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t prev_cnt;
    int32_t  total;
    int32_t  delta;
} enc_state_t;

static enc_state_t state[MOTOR_COUNT];

static void enc_tim_init(TIM_HandleTypeDef *htim, TIM_TypeDef *inst) {
    htim->Instance               = inst;
    htim->Init.Prescaler         = 0;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = 0xFFFFu;        /* 16-bit free-running */
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;        /* 4× quadrature */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection= TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler= TIM_ICPSC_DIV1;
    enc.IC1Filter   = 4;                            /* light filter */
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection= TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler= TIM_ICPSC_DIV1;
    enc.IC2Filter   = 4;
    HAL_TIM_Encoder_Init(htim, &enc);

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

static void enc_gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    /* TIM2 default: PA0 (CH1), PA1 (CH2)
     * TIM3 default: PA6 (CH1), PA7 (CH2)
     * TIM4 default: PB6 (CH1), PB7 (CH2)
     * TIM5: PA0/PA1 — overlaps TIM2; needs schematic-specific remap.
     *       For now we configure these but the remap is BOARD-DEPENDENT and
     *       must be verified before claiming v1 stable.
     */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);
}

void encoder_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    enc_gpio_init();
    enc_tim_init(&htim2, TIM2);
    enc_tim_init(&htim3, TIM3);
    enc_tim_init(&htim4, TIM4);
    enc_tim_init(&htim5, TIM5);

    state[MOTOR_FL].htim = &htim2;
    state[MOTOR_RL].htim = &htim4;
    state[MOTOR_FR].htim = &htim5;
    state[MOTOR_RR].htim = &htim3;

    encoder_reset();
}

static inline int32_t signed_diff(uint16_t now, uint16_t prev) {
    /* 16-bit wrap-aware delta. */
    return (int32_t)(int16_t)((uint16_t)(now - prev));
}

void encoder_sample(void) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        enc_state_t *s = &state[i];
        uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(s->htim);
        s->delta = signed_diff(cnt, s->prev_cnt);
        s->prev_cnt = cnt;
        s->total += s->delta;
    }
}

int32_t encoder_total(motor_id_t m) { return (m < MOTOR_COUNT) ? state[m].total : 0; }
int32_t encoder_delta(motor_id_t m) { return (m < MOTOR_COUNT) ? state[m].delta : 0; }

void encoder_reset(void) {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        enc_state_t *s = &state[i];
        s->total    = 0;
        s->delta    = 0;
        s->prev_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(s->htim);
    }
}
