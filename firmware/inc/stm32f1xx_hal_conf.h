/* Slim HAL config — enable only the modules we use */
#ifndef STM32F1xx_HAL_CONF_H
#define STM32F1xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED

/* Clock config */
#define HSE_VALUE              ((uint32_t)8000000U)
#define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)
#define HSI_VALUE              ((uint32_t)8000000U)
#define LSI_VALUE              ((uint32_t)40000U)
#define LSE_VALUE              ((uint32_t)32768U)
#define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)

#define VDD_VALUE              ((uint32_t)3300U)
#define TICK_INT_PRIORITY      ((uint32_t)0x0FU)
#define USE_RTOS               0U
#define PREFETCH_ENABLE        1U

/* Assert macro — disable in release */
/* #define USE_FULL_ASSERT 1U */

#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1xx_hal_iwdg.h"

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#endif /* STM32F1xx_HAL_CONF_H */
