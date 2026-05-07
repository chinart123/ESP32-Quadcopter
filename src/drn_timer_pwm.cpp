/**
 * @file    drn_timer_pwm.cpp
 * @brief   LEDC PWM hardware abstraction layer — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.1
 */
#include "drn_timer_pwm.h"
#include "drn_main_board_choose.h"

#ifdef ESP32_S3_SUPERMINI
    #include "driver/ledc.h"
    #include "esp_log.h"
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 bare-metal / HAL logic here
#endif

static const char* TAG = "DRN_TIMER_PWM";

static uint32_t xx_channel_max_duty[DRN_PWM_MAX_CHANNELS] = {0};
static bool     xx_timer_initialized = false;

esp_err_t DRN_Timer_PWM_InitTimer(uint8_t timer_num, uint32_t freq_hz, uint8_t duty_res) {
#ifdef ESP32_S3_SUPERMINI
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = (ledc_timer_bit_t)duty_res;
    ledc_timer.timer_num = (ledc_timer_t)timer_num;
    ledc_timer.freq_hz = freq_hz;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timer %d config failed: %s", timer_num, esp_err_to_name(ret));
        return ret;
    }
    xx_timer_initialized = true;
    ESP_LOGI(TAG, "LEDC timer %d initialized at %lu Hz", timer_num, freq_hz);
    return ESP_OK;
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return 0;
#else
    return -1;
#endif
}

esp_err_t DRN_Timer_PWM_InitChannel(const drn_pwm_channel_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
#ifdef ESP32_S3_SUPERMINI
    if (!xx_timer_initialized) {
        ESP_LOGE(TAG, "Timer not initialized before channel config!");
        return ESP_ERR_INVALID_STATE;
    }

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num   = config->gpio_num;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = (ledc_channel_t)config->channel;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel  = LEDC_TIMER_0;  // Assume timer 0 for all channels
    ledc_channel.duty       = 0;
    ledc_channel.hpoint     = 0;
    // ledc_channel.flags.output_invert = 0; // Default is 0

    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Channel %d config failed: %s", config->channel, esp_err_to_name(ret));
        return ret;
    }

    uint32_t max_duty = (1 << config->duty_resolution) - 1;
    xx_channel_max_duty[config->channel] = max_duty;

    ESP_LOGI(TAG, "Channel %d initialized on GPIO %d", config->channel, config->gpio_num);
    return ESP_OK;
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return 0;
#else
    return -1;
#endif
}

void DRN_Timer_PWM_SetDuty(uint8_t channel, uint32_t duty) {
    if (channel >= DRN_PWM_MAX_CHANNELS) return;
#ifdef ESP32_S3_SUPERMINI
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
#endif
}

void DRN_Timer_PWM_SetDutyPercent(uint8_t channel, uint8_t percent) {
    if (channel >= DRN_PWM_MAX_CHANNELS || percent > 100) return;
    uint32_t max_duty = xx_channel_max_duty[channel];
    if (max_duty == 0) {
        max_duty = 1023; // fallback to 10-bit
    }
    uint32_t duty = (uint32_t)((percent * max_duty) / 100);
    DRN_Timer_PWM_SetDuty(channel, duty);
}