#include "stm32f1xx_hal.h"
#include "bsp_uart.h"
#include <string.h>

#define RX_RING_SZ 256u
#define TX_RING_SZ 512u

UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_tx;

static volatile uint8_t  rx_ring[RX_RING_SZ];
static volatile uint16_t rx_head, rx_tail;

static volatile uint8_t  tx_ring[TX_RING_SZ];
static volatile uint16_t tx_head, tx_tail;
static volatile uint8_t  tx_dma_busy;

/* Single-byte RX through HAL_UART_Receive_IT. */
static volatile uint8_t rx_byte;

/* Forward */
static void uart_kick_tx(void);

void uart_init(void) {
    /* Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PA9 = TX (AF push-pull), PA10 = RX (input floating) */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin  = GPIO_PIN_10;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    /* DMA1 Channel4 = USART1_TX */
    hdma_usart1_tx.Instance                 = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    /* USART1: 115200 8N1, no flow control */
    huart1.Instance        = USART1;
    huart1.Init.BaudRate   = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Enable IRQs: USART1 for RX-byte, DMA1_Ch4 for TX-complete */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* Kick off RX-by-IRQ on rx_byte */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
}

/* ── RX path ───────────────────────────────────────────────────────────── */

void uart_rx_isr_byte(uint8_t b) {
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SZ);
    if (next != rx_tail) {
        rx_ring[rx_head] = b;
        rx_head = next;
    } /* else drop on overflow */
}

int uart_rx_available(void) {
    return rx_head != rx_tail;
}

uint16_t uart_rx_pop(void) {
    if (rx_head == rx_tail) return 0xFFFFu;
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SZ);
    return b;
}

/* HAL callback fires after each byte — re-arm immediately and queue. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_rx_isr_byte(rx_byte);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
    }
}

/* ── TX path ───────────────────────────────────────────────────────────── */

size_t uart_tx_write(const uint8_t *buf, size_t len) {
    size_t accepted = 0;
    for (size_t i = 0; i < len; ++i) {
        uint16_t next = (uint16_t)((tx_head + 1u) % TX_RING_SZ);
        if (next == tx_tail) break;     /* full */
        tx_ring[tx_head] = buf[i];
        tx_head = next;
        ++accepted;
    }
    uart_kick_tx();
    return accepted;
}

void uart_tx_byte(uint8_t b) {
    uart_tx_write(&b, 1);
}

/* Drain a contiguous chunk of the ring via DMA. */
static void uart_kick_tx(void) {
    if (tx_dma_busy) return;
    if (tx_head == tx_tail) return;

    uint16_t start = tx_tail;
    uint16_t end   = tx_head;
    uint16_t chunk = (end > start) ? (uint16_t)(end - start)
                                   : (uint16_t)(TX_RING_SZ - start);
    tx_dma_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_ring[start], chunk);
}

void uart_tx_dma_complete_isr(void) {
    /* Advance tail by however much DMA shipped (CNDTR=0 means full chunk). */
    uint16_t consumed = (uint16_t)(hdma_usart1_tx.Instance->CNDTR);
    /* HAL stores transfer size in XferSize; subtract remaining. */
    uint16_t shipped  = (uint16_t)(huart1.TxXferSize - consumed);
    tx_tail = (uint16_t)((tx_tail + shipped) % TX_RING_SZ);
    tx_dma_busy = 0;
    uart_kick_tx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_tx_dma_complete_isr();
    }
}
