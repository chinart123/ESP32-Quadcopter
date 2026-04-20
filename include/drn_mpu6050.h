/**
 * @file    drn_mpu6050.h
 * @brief   MPU6050/MPU6500 IMU sensor HAL — Project Windify
 * @target  Dual-platform: ESP32-S3-SuperMini / STM32F103C8T6
 * @version 1.0
 */
#ifndef DRN_MPU6050_H
#define DRN_MPU6050_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C address of the MPU6050/MPU6500 (AD0 pin low).
 */
#define DRN_MPU6050_I2C_ADDR     0x68

/**
 * @brief Scale factors for the configured ranges:
 *        Accelerometer: ±8g  -> 4096 LSB/g
 *        Gyroscope:     ±500 dps -> 65.5 LSB/dps
 */
#define DRN_MPU6050_ACCEL_SCALE  4096.0f
#define DRN_MPU6050_GYRO_SCALE   65.5f

/**
 * @brief Complementary filter coefficient (98% gyro, 2% accel).
 */
#define DRN_MPU6050_FILTER_ALPHA 0.98f

/**
 * @brief Motion data structure containing raw, scaled, and fused values.
 */
typedef struct {
    // ---- Raw data from sensor (14 bytes burst) ----
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    // ---- Calibration offsets (determined at startup) ----
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;

    // ---- Scaled physical values (g, degrees/sec) ----
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    // ---- Fused angles (degrees) ----
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    // ---- Timing (for integration) ----
    float delta_time_s;         // Time since last update
    uint32_t last_update_ms;    // Timestamp of last successful read

    // ---- Diagnostic ----
    uint32_t bytes_per_second;  // Data throughput (optional)
    uint8_t is_calibrated;      // Flag indicating calibration completed
} drn_mpu_motion_t;

/**
 * @brief Public motion data instance (accessible from main).
 */
extern drn_mpu_motion_t drn_mpu_data;

/**
 * @brief Initialize the I2C bus and configure the MPU6050.
 *        Must be called once during system startup.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t DRN_MPU6050_Init(void);

/**
 * @brief Perform sensor calibration.
 *        Device must be placed still and level during this call.
 *        This function blocks for approximately 1 second.
 *
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t DRN_MPU6050_Calibrate(void);

/**
 * @brief Read a burst of 14 bytes from the sensor (non‑blocking read attempt).
 *        Updates raw values only. Does NOT compute angles.
 *
 * @return esp_err_t ESP_OK if read succeeded, error otherwise.
 */
esp_err_t DRN_MPU6050_Read_Raw(void);

/**
 * @brief Compute scaled values and fused angles using complementary filter.
 *        Must be called after a successful raw read.
 */
void DRN_MPU6050_Compute(void);

/**
 * @brief High‑level periodic task: read, compute, and update.
 *        Designed to be called from the super loop.
 *
 * @param current_time_ms Timestamp from DRN_Millis().
 */
void DRN_MPU6050_Run_Task(uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif
#endif // DRN_MPU6050_H