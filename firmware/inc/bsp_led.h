#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

void led_init(void);
void led_set(int on);
void led_toggle(void);

/* Drive a 1 Hz heartbeat. Call from main loop. */
void led_heartbeat(uint32_t now_ms);

#endif
