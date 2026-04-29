/* ICM20948 (with AK09916 mag) over SPI2.
 * Pinout (V3.0): PB13 SCK, PB14 MISO, PB15 MOSI, PB12 NSS (software).
 * Reference: Yahboom V3.0 IMU PDF — uses SPI mode (not I2C).
 *
 * The chip has 4 user banks (0..3) accessed by writing reg 0x7F. We always
 * select bank before transfer and leave bank=0 between transfers.
 *
 * Magnetometer (AK09916) is internal to the ICM20948 and accessed via the
 * I2C-master block. We program the I2C-master to do periodic reads into the
 * EXT_SENS_DATA registers, then read those over SPI.
 */
#include "stm32f1xx_hal.h"
#include "bsp_icm20948.h"
#include "config.h"
#include <math.h>
#include <string.h>

/* ── Pin / SPI handle ─────────────────────────────────────────────────── */
#define ICM_CS_PORT  GPIOB
#define ICM_CS_PIN   GPIO_PIN_12

static SPI_HandleTypeDef hspi2;

/* ── Register map (subset) ────────────────────────────────────────────── */
#define REG_BANK_SEL          0x7F
#define READ_FLAG             0x80

/* Bank 0 */
#define B0_WHO_AM_I           0x00      /* expect 0xEA */
#define B0_USER_CTRL          0x03
#define B0_PWR_MGMT_1         0x06
#define B0_PWR_MGMT_2         0x07
#define B0_INT_PIN_CFG        0x0F
#define B0_ACCEL_XOUT_H       0x2D
#define B0_GYRO_XOUT_H        0x33
#define B0_EXT_SLV_SENS_DATA_00 0x3B
/* Bank 2 */
#define B2_GYRO_SMPLRT_DIV    0x00
#define B2_GYRO_CONFIG_1      0x01
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG       0x14
/* Bank 3 (I2C master) */
#define B3_I2C_MST_CTRL       0x01
#define B3_I2C_SLV0_ADDR      0x03
#define B3_I2C_SLV0_REG       0x04
#define B3_I2C_SLV0_CTRL      0x05
#define B3_I2C_SLV0_DO        0x06

/* AK09916 magnetometer (I2C addr 0x0C) */
#define MAG_I2C_ADDR          0x0C
#define MAG_WIA2              0x01      /* expect 0x09 */
#define MAG_HXL               0x11
#define MAG_ST2               0x18
#define MAG_CNTL2             0x31
#define MAG_CNTL3             0x32

/* Sensitivity (full-scale ±2000 dps, ±16 g) */
static const float GYRO_LSB_TO_RADPS  = (float)((2000.0 / 32768.0) * (M_PI / 180.0));
static const float ACCEL_LSB_TO_MS2   = (float)((16.0 * 9.80665) / 32768.0);
static const float MAG_LSB_TO_UT      = 0.15f;

/* ── State ────────────────────────────────────────────────────────────── */
static uint8_t  current_bank = 0xFF;
static int16_t  gyro_bias[3] = {0, 0, 0};
static imu_raw_t last_gyro_raw, last_accel_raw, last_mag_raw;
static bool     last_mag_valid;

/* ── Low-level SPI helpers ────────────────────────────────────────────── */
static inline void cs_low(void)  { HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET); }

static void select_bank(uint8_t bank) {
    if (bank == current_bank) return;
    uint8_t tx[2] = { REG_BANK_SEL, (uint8_t)(bank << 4) };
    cs_low();
    HAL_SPI_Transmit(&hspi2, tx, 2, 10);
    cs_high();
    current_bank = bank;
}

static void reg_write(uint8_t bank, uint8_t reg, uint8_t val) {
    select_bank(bank);
    uint8_t tx[2] = { reg, val };
    cs_low();
    HAL_SPI_Transmit(&hspi2, tx, 2, 10);
    cs_high();
}

static uint8_t reg_read(uint8_t bank, uint8_t reg) {
    select_bank(bank);
    uint8_t addr = reg | READ_FLAG;
    uint8_t rx;
    cs_low();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 10);
    HAL_SPI_Receive(&hspi2, &rx, 1, 10);
    cs_high();
    return rx;
}

static void reg_read_burst(uint8_t bank, uint8_t reg, uint8_t *buf, uint8_t len) {
    select_bank(bank);
    uint8_t addr = reg | READ_FLAG;
    cs_low();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 10);
    HAL_SPI_Receive(&hspi2, buf, len, 50);
    cs_high();
}

/* ── Magnetometer-via-I2C-master helpers ──────────────────────────────── */
static void mag_write(uint8_t reg, uint8_t val) {
    reg_write(3, B3_I2C_SLV0_ADDR, MAG_I2C_ADDR);
    reg_write(3, B3_I2C_SLV0_REG,  reg);
    reg_write(3, B3_I2C_SLV0_DO,   val);
    reg_write(3, B3_I2C_SLV0_CTRL, 0x81);   /* enable, 1 byte */
    HAL_Delay(2);
}

static void mag_read_setup(uint8_t reg, uint8_t len) {
    reg_write(3, B3_I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80);  /* read */
    reg_write(3, B3_I2C_SLV0_REG,  reg);
    reg_write(3, B3_I2C_SLV0_CTRL, (uint8_t)(0x80 | len));
}

/* ── Init ─────────────────────────────────────────────────────────────── */
static void spi_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    /* PB13 SCK + PB15 MOSI: AF push-pull */
    g.Pin   = GPIO_PIN_13 | GPIO_PIN_15;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    /* PB14 MISO: input pull-up */
    g.Pin  = GPIO_PIN_14;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
    /* PB12 NSS: push-pull output, idle high */
    g.Pin  = GPIO_PIN_12;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    cs_high();

    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_HIGH;     /* mode 3 */
    hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 36 MHz / 8 = 4.5 MHz, ICM max 7 MHz */
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi2);
}

bool icm20948_init(void) {
    spi_init();

    /* Soft reset */
    reg_write(0, B0_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    /* Wake up, auto-select clock */
    reg_write(0, B0_PWR_MGMT_1, 0x01);
    HAL_Delay(20);
    reg_write(0, B0_PWR_MGMT_2, 0x00);   /* enable accel + gyro */

    /* WHO_AM_I check */
    uint8_t who = reg_read(0, B0_WHO_AM_I);
    if (who != 0xEA) {
        return false;
    }

    /* Gyro: ±2000 dps, DLPF off, ODR ÷1 */
    reg_write(2, B2_GYRO_CONFIG_1, (3 << 1));      /* FS_SEL=3 (±2000), DLPF disabled */
    reg_write(2, B2_GYRO_SMPLRT_DIV, 0);

    /* Accel: ±16 g, DLPF off, ODR max */
    reg_write(2, B2_ACCEL_CONFIG, (3 << 1));
    reg_write(2, B2_ACCEL_SMPLRT_DIV_1, 0);
    reg_write(2, B2_ACCEL_SMPLRT_DIV_2, 0);

    /* Enable I2C master @ 400 kHz to talk to AK09916 */
    reg_write(0, B0_USER_CTRL, 0x20);              /* I2C_MST_EN */
    reg_write(3, B3_I2C_MST_CTRL, 0x07);           /* 400 kHz, stop between reads */

    /* AK09916: soft-reset then continuous mode 4 (100 Hz) */
    mag_write(MAG_CNTL3, 0x01);
    HAL_Delay(10);
    mag_write(MAG_CNTL2, 0x08);                    /* mode 4 = 100 Hz */
    HAL_Delay(10);

    /* Set up SLV0 to auto-read 8 bytes (HXL..HZH + ST2) into EXT_SENS_DATA_00 */
    mag_read_setup(MAG_HXL, 8);

    return true;
}

/* ── Sampling ─────────────────────────────────────────────────────────── */
static void read_accel_gyro(void) {
    uint8_t buf[12];
    reg_read_burst(0, B0_ACCEL_XOUT_H, buf, 12);
    last_accel_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    last_accel_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    last_accel_raw.z = (int16_t)((buf[4] << 8) | buf[5]);
    last_gyro_raw.x  = (int16_t)((buf[6] << 8) | buf[7]) - gyro_bias[0];
    last_gyro_raw.y  = (int16_t)((buf[8] << 8) | buf[9]) - gyro_bias[1];
    last_gyro_raw.z  = (int16_t)((buf[10] << 8) | buf[11]) - gyro_bias[2];
}

static void read_mag(void) {
    uint8_t buf[8];
    reg_read_burst(0, B0_EXT_SLV_SENS_DATA_00, buf, 8);
    /* AK09916 is little-endian */
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
    uint8_t st2 = buf[7];
    last_mag_valid = !(st2 & 0x08);   /* HOFL = overflow */
    if (last_mag_valid) {
        last_mag_raw.x = x;
        last_mag_raw.y = y;
        last_mag_raw.z = z;
    }
}

void icm20948_sample(imu_sample_t *out) {
    static uint32_t mag_div = 0;
    read_accel_gyro();
    if ((++mag_div & 0x03u) == 0u) {       /* mag every 4th tick = ~25 Hz */
        read_mag();
    }

    float ax = last_accel_raw.x * ACCEL_LSB_TO_MS2;
    float ay = last_accel_raw.y * ACCEL_LSB_TO_MS2;
    float az = last_accel_raw.z * ACCEL_LSB_TO_MS2;
    float gx = last_gyro_raw.x  * GYRO_LSB_TO_RADPS;
    float gy = last_gyro_raw.y  * GYRO_LSB_TO_RADPS;
    float gz = last_gyro_raw.z  * GYRO_LSB_TO_RADPS;
    float mx = last_mag_raw.x   * MAG_LSB_TO_UT;
    float my = last_mag_raw.y   * MAG_LSB_TO_UT;
    float mz = last_mag_raw.z   * MAG_LSB_TO_UT;

#if IMU_NEGATE_AX
    ax = -ax;
#endif
#if IMU_NEGATE_AY
    ay = -ay;
#endif
#if IMU_NEGATE_AZ
    az = -az;
#endif
#if IMU_NEGATE_GX
    gx = -gx;
#endif
#if IMU_NEGATE_GY
    gy = -gy;
#endif
#if IMU_NEGATE_GZ
    gz = -gz;
#endif
#if IMU_NEGATE_MX
    mx = -mx;
#endif
#if IMU_NEGATE_MY
    my = -my;
#endif
#if IMU_NEGATE_MZ
    mz = -mz;
#endif

    out->ax = ax; out->ay = ay; out->az = az;
    out->gx = gx; out->gy = gy; out->gz = gz;
    out->mx = mx; out->my = my; out->mz = mz;
    out->mag_valid = last_mag_valid;
}

void icm20948_wire_values(imu_raw_t *gyro_w, imu_raw_t *accel_w, imu_raw_t *mag_w) {
    /* Send the most recent sample, with axis correction baked in, scaled per
     * Rosmaster_Lib's expected divisors (gyro ÷1000, accel ÷10000, mag ÷1000).
     *
     * Equivalent to: wire = SI_value * SCALE.
     * gyro:  SI rad/s → wire = rad/s × 1000
     * accel: SI m/s²  → wire = m/s² × 10000
     * mag:   SI µT    → wire = µT × 1000
     */
    imu_sample_t s;
    icm20948_sample(&s);
    int32_t gx = (int32_t)(s.gx * GYRO_WIRE_SCALE);
    int32_t gy = (int32_t)(s.gy * GYRO_WIRE_SCALE);
    int32_t gz = (int32_t)(s.gz * GYRO_WIRE_SCALE);
    int32_t ax = (int32_t)(s.ax * ACCEL_WIRE_SCALE);
    int32_t ay = (int32_t)(s.ay * ACCEL_WIRE_SCALE);
    int32_t az = (int32_t)(s.az * ACCEL_WIRE_SCALE);
    int32_t mx = (int32_t)(s.mx * MAG_WIRE_SCALE);
    int32_t my = (int32_t)(s.my * MAG_WIRE_SCALE);
    int32_t mz = (int32_t)(s.mz * MAG_WIRE_SCALE);

    /* Saturate to int16 */
    #define SAT16(v) ((v) > 32767 ? 32767 : ((v) < -32768 ? -32768 : (int16_t)(v)))
    gyro_w->x = SAT16(gx); gyro_w->y = SAT16(gy); gyro_w->z = SAT16(gz);
    accel_w->x = SAT16(ax); accel_w->y = SAT16(ay); accel_w->z = SAT16(az);
    mag_w->x = SAT16(mx); mag_w->y = SAT16(my); mag_w->z = SAT16(mz);
    #undef SAT16
}

void icm20948_calibrate_gyro_bias(void) {
    enum { N = 200 };
    int32_t sx = 0, sy = 0, sz = 0;
    /* zero existing bias to read raw */
    gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0;
    for (int i = 0; i < N; ++i) {
        read_accel_gyro();
        sx += last_gyro_raw.x;
        sy += last_gyro_raw.y;
        sz += last_gyro_raw.z;
        HAL_Delay(5);
    }
    gyro_bias[0] = (int16_t)(sx / N);
    gyro_bias[1] = (int16_t)(sy / N);
    gyro_bias[2] = (int16_t)(sz / N);
}
