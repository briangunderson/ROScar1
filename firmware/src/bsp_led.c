#include "stm32f1xx_hal.h"
#include "bsp_led.h"

/* On-board status LED — pin TBD, using PC13 (common Bluepill convention) as
 * placeholder. Actual pin to be confirmed against schematic — board has
 * "indicators" at item ⑧ which are likely on PB5/PB6 or PC13. We'll bench-test.
 */
#define LED_PORT  GPIOC
#define LED_PIN   GPIO_PIN_13
#define LED_RCC   __HAL_RCC_GPIOC_CLK_ENABLE

void led_init(void) {
    LED_RCC();
    GPIO_InitTypeDef g = {0};
    g.Pin   = LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    led_set(0);
}

void led_set(int on) {
    /* Many on-board LEDs are active-low. Adjust polarity here once verified. */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led_toggle(void) {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void led_heartbeat(uint32_t now_ms) {
    static uint32_t last = 0;
    if ((uint32_t)(now_ms - last) >= 500u) {
        last = now_ms;
        led_toggle();
    }
}
