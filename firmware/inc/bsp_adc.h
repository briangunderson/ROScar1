#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

void adc_init(void);

/* Returns most recent battery voltage in volts (filtered). */
float battery_volts(void);

/* Triggers a fresh sample. Call at ~10 Hz from main loop. */
void battery_sample(void);

#endif
