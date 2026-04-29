#ifndef BSP_SYSTIME_H
#define BSP_SYSTIME_H

#include <stdint.h>

/* Free-running millisecond counter, fed by SysTick at 1 kHz.
 * Wraps after 49.7 days; subtract using uint32_t arithmetic to handle wrap.
 */
uint32_t millis(void);

/* Spin-wait for `ms` milliseconds. Use sparingly — ISR-safe but blocks. */
void delay_ms(uint32_t ms);

/* Returns true if `millis() - since >= dt_ms`, with wrap-safe comparison. */
static inline int elapsed(uint32_t since, uint32_t dt_ms) {
    return (uint32_t)(millis() - since) >= dt_ms;
}

#endif
