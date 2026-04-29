/* Battery voltage via ADC.
 * Pin: TBD from schematic — placeholder PA4 (ADC1_IN4). Most YB boards use a
 * resistor divider with a 4:1 ratio so 12V → 3V at the ADC pin (within VDD).
 *
 * Once schematic is in hand, update CHANNEL and BAT_DIVIDER_RATIO.
 */
#include "stm32f1xx_hal.h"
#include "bsp_adc.h"
#include "config.h"

static ADC_HandleTypeDef hadc1;
static float bat_v_filt = 0.0f;

#define BAT_ADC_PORT   GPIOA
#define BAT_ADC_PIN    GPIO_PIN_4
#define BAT_CHANNEL    ADC_CHANNEL_4

void adc_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin  = BAT_ADC_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(BAT_ADC_PORT, &g);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = BAT_CHANNEL;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ch);

    HAL_ADCEx_Calibration_Start(&hadc1);
}

void battery_sample(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) return;
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* 12-bit ADC, VREF = 3.3 V. */
    float pin_v = (raw / 4095.0f) * 3.3f;
    float bat_v = pin_v * BAT_DIVIDER_RATIO;

    /* Light low-pass: alpha = 0.2 */
    bat_v_filt = bat_v_filt == 0.0f ? bat_v : (0.8f * bat_v_filt + 0.2f * bat_v);
}

float battery_volts(void) {
    return bat_v_filt;
}
