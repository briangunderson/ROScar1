/* IRQ vector handlers — minimal set, delegate to HAL. */
#include "stm32f1xx_hal.h"
#include "bsp_uart.h"

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void)        {}
void DebugMon_Handler(void)   {}
void PendSV_Handler(void)     {}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart1.hdmatx);
}
