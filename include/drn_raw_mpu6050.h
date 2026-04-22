/**
 * @file    drn_raw_mpu6050.h
 * @brief   MPU6050/MPU6500 — Raw verification driver (NO complementary filter)
 *          Pipeline stops at Step 4: atan2 accel-only angle.
 *          Purpose: compare raw/noisy accel angle BEFORE applying the filter
 *          in drn_mpu6050 so the effect of the complementary filter is visible.
 *
 * @target  Dual-platform: ESP32-S3-SuperMini / STM32F103C8T6
 * @version 1.0
 *
 * Pipeline implemented here:
 *   ① I2C Burst Read (14 bytes)
 *   ② Calibration    (1000 samples → offset[])
 *   ③ Scale          (raw − offset) → g  and  °/s
 *   ④ Accel Angle    atan2 → roll_raw_deg / pitch_raw_deg   ← STOPS HERE
 *
 * What is intentionally OMITTED:
 *   ✗ Complementary filter  (no 0.98/0.02 fusion)
 *   ✗ Fused roll_deg / pitch_deg / yaw_deg
 */
#ifndef DRN_RAW_MPU6050_H
#define DRN_RAW_MPU6050_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────────────────────────────────
 * Hardware constants — identical to drn_mpu6050.h
 * ────────────────────────────────────────────────────────────────────────── */

/** @brief I2C address (AD0 pin low). */
#define DRN_RAW_MPU6050_I2C_ADDR     0x68

/**
 * @brief Scale factors for the configured hardware ranges:
 *        Accelerometer : ±8g      → 4096 LSB/g
 *        Gyroscope     : ±500 dps → 65.5 LSB/dps
 */
#define DRN_RAW_MPU6050_ACCEL_SCALE  4096.0f
#define DRN_RAW_MPU6050_GYRO_SCALE   65.5f

/* ──────────────────────────────────────────────────────────────────────────
 * Data structure
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Motion data structure — raw pipeline only (Steps 1–4).
 *
 * Naming mirrors drn_mpu_motion_t so the two structs can be placed side-by-side
 * in a serial log or oscilloscope capture for direct comparison.
 */
typedef struct {

    /* ── Step ①  Raw data from sensor (14-byte I2C burst) ── */
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;           /**< Chip temperature (read but not used). */
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    /* ── Step ②  Calibration offsets (computed once at startup) ── */
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;     /**< = mean(az) − 4096  (1g compensation). */
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;

    /* ── Step ③  Scaled physical values ── */
    float accel_x_g;            /**< Acceleration X  [g]. */
    float accel_y_g;            /**< Acceleration Y  [g]. */
    float accel_z_g;            /**< Acceleration Z  [g]. */
    float gyro_x_dps;           /**< Angular rate X  [°/s]. */
    float gyro_y_dps;           /**< Angular rate Y  [°/s]. */
    float gyro_z_dps;           /**< Angular rate Z  [°/s]. */

    /* ── Step ④  Accel-only angle (NO filter) ── */
    float roll_raw_deg;         /**< Roll  from atan2 — noisy under vibration. */
    float pitch_raw_deg;        /**< Pitch from atan2 — noisy under vibration. */
    /* NOTE: yaw_raw_deg is intentionally absent.
     *       Accelerometer cannot measure heading; gyro-only yaw drifts too fast
     *       to be useful here without the complementary filter. */

    /* ── Timing ── */
    uint32_t last_update_ms;    /**< Timestamp of the last successful read. */

    /* ── Diagnostics ── */
    uint32_t bytes_per_second;  /**< I2C throughput counter (optional). */
    uint8_t  is_calibrated;     /**< 1 once DRN_RawMPU6050_Calibrate() succeeds. */

} drn_raw_mpu_motion_t;

/* ──────────────────────────────────────────────────────────────────────────
 * Public instance
 * ────────────────────────────────────────────────────────────────────────── */

/** @brief Global data instance — access from main or logging task. */
extern drn_raw_mpu_motion_t drn_raw_mpu_data;

/* ──────────────────────────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize I2C bus and configure the sensor registers.
 *        Must be called once at startup before any other function.
 *        Identical register config to drn_mpu6050:
 *          PWR_MGMT_1 = 0x00  (wake up)
 *          CONFIG     = 0x03  (DLPF ~41 Hz)
 *          GYRO_CFG   = 0x08  (±500 dps)
 *          ACCEL_CFG  = 0x10  (±8g)
 *
 * @return ESP_OK on success.
 */
esp_err_t DRN_RawMPU6050_Init(void);

/**
 * @brief Calibrate sensor offsets.
 *        Device must be placed still and perfectly level during this call.
 *        Collects 1000 samples and computes mean for each axis.
 *        Blocks for approximately 1 second.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t DRN_RawMPU6050_Calibrate(void);

/**
 * @brief Read a 14-byte burst from the sensor.
 *        Updates raw values only. Does NOT compute angles.
 *
 * @return ESP_OK if read succeeded.
 */
esp_err_t DRN_RawMPU6050_Read_Raw(void);

/**
 * @brief Compute scaled values and accel-only angles (Steps ③ and ④).
 *        Applies offset subtraction, unit scaling, and atan2.
 *        Does NOT apply complementary filter.
 *        Must be called after a successful DRN_RawMPU6050_Read_Raw().
 */
void DRN_RawMPU6050_Compute(void);

/**
 * @brief High-level periodic task: read + compute at ~200 Hz.
 *        Designed to run from the super-loop or a FreeRTOS task alongside
 *        DRN_MPU6050_Run_Task() for side-by-side comparison.
 *
 * @param current_time_ms  Timestamp from DRN_Millis().
 */
void DRN_RawMPU6050_Run_Task(uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif
#endif /* DRN_RAW_MPU6050_H */
