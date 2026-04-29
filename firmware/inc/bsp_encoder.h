#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include "bsp_motor.h"

void encoder_init(void);

/* Sample all 4 hardware counters, accumulate into 32-bit software counters,
 * and capture per-tick delta. Call from 100 Hz control loop. */
void encoder_sample(void);

/* Total counts since boot (latched at last encoder_sample). */
int32_t encoder_total(motor_id_t m);

/* Counts during the most recent encoder_sample tick. */
int32_t encoder_delta(motor_id_t m);

/* Reset all software counters to zero. */
void encoder_reset(void);

#endif
