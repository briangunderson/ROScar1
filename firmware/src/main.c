/* ROScar1 firmware entry point.
 * Targets: STM32F103RCT6 on Yahboom YB-ERF01-V3.0
 */
#include "stm32f1xx_hal.h"
#include "config.h"
#include "bsp_systime.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_icm20948.h"
#include "bsp_adc.h"
#include "control_loop.h"
#include "protocol.h"

/* ── System clock: HSE 8 MHz × PLL9 → 72 MHz ──────────────────────────── */
static void system_clock_72mhz(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.HSIState       = RCC_HSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;        /* 36 MHz max */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;        /* 72 MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

int main(void) {
    HAL_Init();
    system_clock_72mhz();

    led_init();
    uart_init();
    motor_init();
    encoder_init();
    adc_init();

    /* IMU is optional — keep booting if it fails so we still get serial. */
    bool imu_ok = icm20948_init();
    if (imu_ok) {
        icm20948_calibrate_gyro_bias();
    }

    control_loop_init();
    protocol_init();

    uint32_t last_control = 0;
    uint32_t last_battery = 0;

    while (1) {
        uint32_t now = millis();
        led_heartbeat(now);

        /* 100 Hz control loop tick. */
        if ((uint32_t)(now - last_control) >= 10u) {
            last_control += 10;
            control_loop_tick();
        }

        /* 10 Hz battery sample. */
        if ((uint32_t)(now - last_battery) >= 100u) {
            last_battery = now;
            battery_sample();
        }

        protocol_pump(now);
    }
}
