#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f1xx_hal.h"

/* USART1 (PA9 TX, PA10 RX), 115200 8N1.
 * RX: byte-at-a-time IRQ, pushed into a ring buffer.
 * TX: DMA1_Channel4 from a ring buffer for non-blocking writes.
 */
void uart_init(void);

/* True if at least one byte is available in the RX ring. */
int uart_rx_available(void);

/* Pop one byte from RX ring. Returns 0xFFFF if empty. */
uint16_t uart_rx_pop(void);

/* Queue `len` bytes onto the TX ring. Drops bytes if ring full.
 * Returns number of bytes accepted. */
size_t uart_tx_write(const uint8_t *buf, size_t len);

/* Write a single byte (blocking on full ring is bounded — falls back to drop). */
void uart_tx_byte(uint8_t b);

/* Called from DMA-complete IRQ to schedule next chunk. */
void uart_tx_dma_complete_isr(void);

/* Called from USART1 IRQ on RXNE. */
void uart_rx_isr_byte(uint8_t b);

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_tx;

#endif
