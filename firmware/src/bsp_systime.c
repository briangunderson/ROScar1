#include "stm32f1xx_hal.h"
#include "bsp_systime.h"

uint32_t millis(void) {
    return HAL_GetTick();
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
