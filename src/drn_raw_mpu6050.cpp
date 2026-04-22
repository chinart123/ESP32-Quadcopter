/**
 * @file    drn_raw_mpu6050.cpp
 * @brief   MPU6050/MPU6500 — Raw verification driver (NO complementary filter)
 *          Pipeline stops at Step 4: atan2 accel-only angle.
 *
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.0
 *
 * HOW TO USE FOR COMPARISON:
 *   In your main loop / FreeRTOS task, call both drivers at the same rate:
 *
 *     DRN_RawMPU6050_Run_Task(DRN_Millis());   // accel-only → noisy
 *     DRN_MPU6050_Run_Task(DRN_Millis());       // filtered   → smooth
 *
 *   Then log both over UART:
 *     printf("RAW  Roll=%.2f Pitch=%.2f\n",
 *             drn_raw_mpu_data.roll_raw_deg,
 *             drn_raw_mpu_data.pitch_raw_deg);
 *     printf("FILT Roll=%.2f Pitch=%.2f\n",
 *             drn_mpu_data.roll_deg,
 *             drn_mpu_data.pitch_deg);
 */
#include "drn_raw_mpu6050.h"
#include "drn_main_board_choose.h"
#include "soc/gpio_num.h"
#include <math.h>
#include <string.h>

#ifdef ESP32_S3_SUPERMINI
    #include "driver/i2c.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 bare-metal / HAL logic here
#endif

static const char* TAG = "DRN_RAW_MPU6050";

/* ──────────────────────────────────────────────────────────────────────────
 * I2C configuration — identical to drn_mpu6050.cpp
 * ────────────────────────────────────────────────────────────────────────── */
#ifdef ESP32_S3_SUPERMINI
    #define I2C_MASTER_NUM          I2C_NUM_0
    #define I2C_MASTER_FREQ_HZ      400000
    #define I2C_MASTER_SDA_IO       GPIO_NUM_8
    #define I2C_MASTER_SCL_IO       GPIO_NUM_9
    #define I2C_MASTER_TIMEOUT_MS   100
#endif

/* ──────────────────────────────────────────────────────────────────────────
 * MPU register addresses — identical to drn_mpu6050.cpp
 * ────────────────────────────────────────────────────────────────────────── */
#define MPU_REG_WHO_AM_I        0x75
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B

#define MPU6050_WHO_AM_I_VAL    0x68
#define MPU6500_WHO_AM_I_VAL    0x70

/* ──────────────────────────────────────────────────────────────────────────
 * Public data instance
 * ────────────────────────────────────────────────────────────────────────── */
drn_raw_mpu_motion_t drn_raw_mpu_data;

/* ──────────────────────────────────────────────────────────────────────────
 * Internal state
 * ────────────────────────────────────────────────────────────────────────── */
static bool     xx_raw_initialized     = false;
static uint32_t xx_raw_last_read_ms    = 0;
static uint32_t xx_raw_byte_counter    = 0;
static uint32_t xx_raw_last_bps_ms     = 0;

/* ──────────────────────────────────────────────────────────────────────────
 * Static I2C helpers — identical to drn_mpu6050.cpp
 * ────────────────────────────────────────────────────────────────────────── */
#ifdef ESP32_S3_SUPERMINI

static esp_err_t i2c_raw_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, DRN_RAW_MPU6050_I2C_ADDR,
                                      write_buf, sizeof(write_buf),
                                      pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t i2c_raw_read_multi(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, DRN_RAW_MPU6050_I2C_ADDR,
                                        &reg_addr, 1,
                                        data, len,
                                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

#endif /* ESP32_S3_SUPERMINI */

/* ══════════════════════════════════════════════════════════════════════════
 * Public API
 * ══════════════════════════════════════════════════════════════════════════ */

/* ──────────────────────────────────────────────────────────────────────────
 * DRN_RawMPU6050_Init
 * ────────────────────────────────────────────────────────────────────────── */
esp_err_t DRN_RawMPU6050_Init(void) {
#ifdef ESP32_S3_SUPERMINI

    /* 1. Configure I2C bus */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master           = { .clk_speed = I2C_MASTER_FREQ_HZ },
        .clk_flags        = 0
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. Verify sensor presence via WHO_AM_I */
    uint8_t who_am_i = 0;
    ret = i2c_raw_read_multi(MPU_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VAL && who_am_i != MPU6500_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU sensor found (WHO_AM_I = 0x%02X)", who_am_i);

    /* 3. Wake up and configure registers
     *    Settings are kept identical to drn_mpu6050 so both drivers
     *    read from the same hardware configuration. */
    ret = i2c_raw_write_reg(MPU_REG_PWR_MGMT_1, 0x00);   /* Exit sleep mode      */
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));                        /* Allow stabilisation  */

    ret = i2c_raw_write_reg(MPU_REG_CONFIG,      0x03);   /* DLPF ~41 Hz          */
    if (ret != ESP_OK) return ret;
    ret = i2c_raw_write_reg(MPU_REG_GYRO_CONFIG, 0x08);   /* ±500 dps             */
    if (ret != ESP_OK) return ret;
    ret = i2c_raw_write_reg(MPU_REG_ACCEL_CONFIG, 0x10);  /* ±8g                  */
    if (ret != ESP_OK) return ret;

    /* 4. Clear data struct */
    memset(&drn_raw_mpu_data, 0, sizeof(drn_raw_mpu_data));
    xx_raw_initialized  = true;
    xx_raw_last_read_ms = DRN_Millis();

    ESP_LOGI(TAG, "DRN_RawMPU6050 initialized (filter-free mode)");
    return ESP_OK;

#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return 0;
#else
    return -1;
#endif
}

/* ──────────────────────────────────────────────────────────────────────────
 * DRN_RawMPU6050_Calibrate
 * ────────────────────────────────────────────────────────────────────────── */
esp_err_t DRN_RawMPU6050_Calibrate(void) {
    if (!xx_raw_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting raw calibration (keep device still and level)...");

    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int num_samples = 1000;
    uint8_t buffer[14];

    for (int i = 0; i < num_samples; i++) {
#ifdef ESP32_S3_SUPERMINI
        esp_err_t ret = i2c_raw_read_multi(MPU_REG_ACCEL_XOUT_H, buffer, 14);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed at sample %d", i);
            return ret;
        }
#elif defined(STM32F103C8T6)
        // [STM32 PLACEHOLDER]
        return -1;
#endif
        /* Accumulate raw values (big-endian → host) */
        sum_ax += (int16_t)((buffer[0] << 8) | buffer[1]);
        sum_ay += (int16_t)((buffer[2] << 8) | buffer[3]);
        sum_az += (int16_t)((buffer[4] << 8) | buffer[5]);
        sum_gx += (int16_t)((buffer[8] << 8) | buffer[9]);
        sum_gy += (int16_t)((buffer[10] << 8) | buffer[11]);
        sum_gz += (int16_t)((buffer[12] << 8) | buffer[13]);

        vTaskDelay(pdMS_TO_TICKS(1));   /* ~1ms between samples */
    }

    /* Compute mean offsets */
    drn_raw_mpu_data.accel_x_offset = (int16_t)(sum_ax / num_samples);
    drn_raw_mpu_data.accel_y_offset = (int16_t)(sum_ay / num_samples);
    /* Z axis reads +1g (4096 LSB) when level — compensate */
    drn_raw_mpu_data.accel_z_offset = (int16_t)((sum_az / num_samples) - 4096);
    drn_raw_mpu_data.gyro_x_offset  = (int16_t)(sum_gx / num_samples);
    drn_raw_mpu_data.gyro_y_offset  = (int16_t)(sum_gy / num_samples);
    drn_raw_mpu_data.gyro_z_offset  = (int16_t)(sum_gz / num_samples);

    drn_raw_mpu_data.is_calibrated   = 1;
    drn_raw_mpu_data.last_update_ms  = DRN_Millis();

    ESP_LOGI(TAG, "Raw calibration complete");
    ESP_LOGI(TAG, "Accel offsets: X=%d  Y=%d  Z=%d",
             drn_raw_mpu_data.accel_x_offset,
             drn_raw_mpu_data.accel_y_offset,
             drn_raw_mpu_data.accel_z_offset);
    ESP_LOGI(TAG, "Gyro  offsets: X=%d  Y=%d  Z=%d",
             drn_raw_mpu_data.gyro_x_offset,
             drn_raw_mpu_data.gyro_y_offset,
             drn_raw_mpu_data.gyro_z_offset);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────────────────
 * DRN_RawMPU6050_Read_Raw  (Step ①)
 * ────────────────────────────────────────────────────────────────────────── */
esp_err_t DRN_RawMPU6050_Read_Raw(void) {
    if (!xx_raw_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

#ifdef ESP32_S3_SUPERMINI
    uint8_t buffer[14];
    esp_err_t ret = i2c_raw_read_multi(MPU_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Raw read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Parse big-endian data into signed 16-bit integers */
    drn_raw_mpu_data.accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    drn_raw_mpu_data.accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    drn_raw_mpu_data.accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    drn_raw_mpu_data.temp_raw    = (int16_t)((buffer[6] << 8) | buffer[7]);
    drn_raw_mpu_data.gyro_x_raw  = (int16_t)((buffer[8] << 8) | buffer[9]);
    drn_raw_mpu_data.gyro_y_raw  = (int16_t)((buffer[10] << 8) | buffer[11]);
    drn_raw_mpu_data.gyro_z_raw  = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Update bytes-per-second diagnostic counter */
    uint32_t now = DRN_Millis();
    xx_raw_byte_counter += 14;
    if (now - xx_raw_last_bps_ms >= 1000) {
        drn_raw_mpu_data.bytes_per_second = xx_raw_byte_counter;
        xx_raw_byte_counter  = 0;
        xx_raw_last_bps_ms   = now;
    }

    return ESP_OK;

#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return -1;
#else
    return -1;
#endif
}

/* ──────────────────────────────────────────────────────────────────────────
 * DRN_RawMPU6050_Compute  (Steps ② ③ ④ — no filter)
 * ────────────────────────────────────────────────────────────────────────── */
void DRN_RawMPU6050_Compute(void) {
    if (!drn_raw_mpu_data.is_calibrated) {
        /* Cannot produce meaningful angles without calibration */
        return;
    }

    /* ── Step ③  Scale: subtract offset and convert to physical units ── */
    drn_raw_mpu_data.accel_x_g = (float)(drn_raw_mpu_data.accel_x_raw - drn_raw_mpu_data.accel_x_offset)
                                  / DRN_RAW_MPU6050_ACCEL_SCALE;
    drn_raw_mpu_data.accel_y_g = (float)(drn_raw_mpu_data.accel_y_raw - drn_raw_mpu_data.accel_y_offset)
                                  / DRN_RAW_MPU6050_ACCEL_SCALE;
    drn_raw_mpu_data.accel_z_g = (float)(drn_raw_mpu_data.accel_z_raw - drn_raw_mpu_data.accel_z_offset)
                                  / DRN_RAW_MPU6050_ACCEL_SCALE;

    drn_raw_mpu_data.gyro_x_dps = (float)(drn_raw_mpu_data.gyro_x_raw - drn_raw_mpu_data.gyro_x_offset)
                                   / DRN_RAW_MPU6050_GYRO_SCALE;
    drn_raw_mpu_data.gyro_y_dps = (float)(drn_raw_mpu_data.gyro_y_raw - drn_raw_mpu_data.gyro_y_offset)
                                   / DRN_RAW_MPU6050_GYRO_SCALE;
    drn_raw_mpu_data.gyro_z_dps = (float)(drn_raw_mpu_data.gyro_z_raw - drn_raw_mpu_data.gyro_z_offset)
                                   / DRN_RAW_MPU6050_GYRO_SCALE;

    /* Update timestamp */
    drn_raw_mpu_data.last_update_ms = DRN_Millis();

    /* ── Step ④  Accel-only angle via atan2  (pipeline ends here) ──
     *
     *  roll_raw_deg  : angle around X axis — lateral tilt
     *  pitch_raw_deg : angle around Y axis — fore/aft tilt
     *
     *  Both values are computed from the accelerometer ONLY.
     *  They are accurate when the board is stationary but become very
     *  noisy under propeller/motor vibration — that is exactly what
     *  this driver is meant to demonstrate before the filter is applied.
     *
     *  57.2957795f = 180 / π  (radian → degree conversion)
     */
    drn_raw_mpu_data.roll_raw_deg =
        atan2f(drn_raw_mpu_data.accel_y_g, drn_raw_mpu_data.accel_z_g)
        * 57.2957795f;

    drn_raw_mpu_data.pitch_raw_deg =
        atan2f(-drn_raw_mpu_data.accel_x_g,
               sqrtf(drn_raw_mpu_data.accel_y_g * drn_raw_mpu_data.accel_y_g +
                     drn_raw_mpu_data.accel_z_g * drn_raw_mpu_data.accel_z_g))
        * 57.2957795f;

    /* No complementary filter step — function returns here intentionally. */
}

/* ──────────────────────────────────────────────────────────────────────────
 * DRN_RawMPU6050_Run_Task  (high-level periodic wrapper)
 * ────────────────────────────────────────────────────────────────────────── */
void DRN_RawMPU6050_Run_Task(uint32_t current_time_ms) {
    /* Throttle to ~200 Hz (5 ms interval) — same rate as drn_mpu6050 */
    if ((current_time_ms - xx_raw_last_read_ms) >= 5) {
        xx_raw_last_read_ms = current_time_ms;

        if (DRN_RawMPU6050_Read_Raw() == ESP_OK) {
            DRN_RawMPU6050_Compute();
        }
    }
}
