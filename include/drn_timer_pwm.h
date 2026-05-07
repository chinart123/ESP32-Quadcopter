/**
 * @file    drn_timer_pwm.h
 * @brief   LEDC PWM hardware abstraction layer — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.0
 */
#ifndef DRN_TIMER_PWM_H
#define DRN_TIMER_PWM_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of PWM channels supported by this HAL.
 */
#define DRN_PWM_MAX_CHANNELS 8

/**
 * @brief PWM configuration structure.
 */
typedef struct {
    uint8_t gpio_num;       // GPIO pin number
    uint8_t channel;        // LEDC channel (0–7)
    uint32_t freq_hz;       // PWM frequency in Hz
    uint8_t duty_resolution;// LEDC duty resolution (e.g., LEDC_TIMER_10_BIT)
} drn_pwm_channel_config_t;

/**
 * @brief Initialize the LEDC timer (shared among channels).
 * @param timer_num Timer index (typically LEDC_TIMER_0).
 * @param freq_hz   PWM frequency for all channels using this timer.
 * @param duty_res  Duty resolution (e.g., LEDC_TIMER_10_BIT).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t DRN_Timer_PWM_InitTimer(uint8_t timer_num, uint32_t freq_hz, uint8_t duty_res);

/**
 * @brief Initialize a single PWM channel.
 * @param config Pointer to configuration.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t DRN_Timer_PWM_InitChannel(const drn_pwm_channel_config_t *config);

/**
 * @brief Set duty cycle for a specific channel.
 * @param channel LEDC channel (0–7).
 * @param duty    Duty value (0 to (2^resolution - 1)).
 */
void DRN_Timer_PWM_SetDuty(uint8_t channel, uint32_t duty);

/**
 * @brief Set duty cycle as percentage (0–100).
 * @param channel LEDC channel.
 * @param percent Percentage (0–100).
 */
void DRN_Timer_PWM_SetDutyPercent(uint8_t channel, uint8_t percent);

#ifdef __cplusplus
}
#endif
#endif // DRN_TIMER_PWM_H