/**
 * @file    drn_time.cpp
 * @brief   HAL timing utilities — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.0
 */
#include "drn_time.h"
#include "drn_main_board_choose.h"

#ifdef ESP32_S3_SUPERMINI
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_timer.h"
    #include "esp_log.h"
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 bare-metal / HAL logic here
#endif

static const char* TAG = "DRN_TIME";

esp_err_t DRN_Time_Init(void) {
#ifdef ESP32_S3_SUPERMINI
    // esp_timer is initialized automatically by ESP-IDF
    ESP_LOGI(TAG, "Time module initialized.");
    return ESP_OK;
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return 0;
#else
    return -1;
#endif
}

uint32_t DRN_Millis(void) {
#ifdef ESP32_S3_SUPERMINI
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
    return 0;
#else
    return 0;
#endif
}

void DRN_Delay_ms(uint32_t ms) {
#ifdef ESP32_S3_SUPERMINI
    vTaskDelay(pdMS_TO_TICKS(ms));
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER]
#endif
}