#ifndef BSP_ICM20948_H
#define BSP_ICM20948_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t x, y, z;
} imu_raw_t;

typedef struct {
    float ax, ay, az;       /* m/s²  (already axis-corrected) */
    float gx, gy, gz;       /* rad/s (axis-corrected, bias-subtracted) */
    float mx, my, mz;       /* µT    (axis-corrected) */
    bool  mag_valid;
} imu_sample_t;

/* Initialise SPI2 + reset/config the ICM20948 + AK09916 magnetometer.
 * Returns true if WHO_AM_I matches; otherwise the caller can decide whether
 * to retry or boot without IMU. */
bool icm20948_init(void);

/* Sample at 100 Hz. Fills `out` with axis-corrected SI values. Always reads
 * accel + gyro; magnetometer is fetched at lower rate (every Nth call). */
void icm20948_sample(imu_sample_t *out);

/* Wire-format helpers used by protocol.c — pre-scaled int16 values matching
 * Rosmaster_Lib's parse expectations (gyro ×1000, accel ×10000, mag ×1000). */
void icm20948_wire_values(imu_raw_t *gyro_w, imu_raw_t *accel_w, imu_raw_t *mag_w);

/* Run gyro bias calibration: average ~200 samples, subtract from subsequent
 * reads. Robot must be stationary. */
void icm20948_calibrate_gyro_bias(void);

#endif
