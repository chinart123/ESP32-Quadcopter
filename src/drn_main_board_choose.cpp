/**
 * @file    drn_main_board_choose.cpp
 * @brief   Board selection and global HAL initialization — Project Windify
 * @target  Dual-platform: ESP32-S3-SuperMini / STM32F103C8T6
 * @version 1.0
 */
#include "drn_main_board_choose.h"

#ifdef ESP32_S3_SUPERMINI
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 bare-metal / HAL logic here
#endif

static const char* TAG = "DRN_MAIN";

void drn_main_board_choose_Init(void) {
#ifdef ESP32_S3_SUPERMINI
    ESP_LOGI(TAG, "Initializing ESP32-S3-SuperMini target...");
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 init message here
#endif

    // Initialization order matters: time first, then peripherals
    DRN_Time_Init();
    DRN_Motor_PWM_Init();
    DRN_Button_Init();
	// XÓA dòng DRN_MPU6050_Init() độc lập cũ đi.
    // CHỈ DÙNG 1 block lệnh này để khởi tạo và calibrate:
    if (DRN_MPU6050_Init() == ESP_OK) {
        DRN_MPU6050_Calibrate(); 
    }
}

void drn_main_board_choose_Delay_ms(unsigned int ms) {
#ifdef STM32F103C8T6
    uint32_t start_time = DRN_Millis();
    while ((DRN_Millis() - start_time) < ms) {}
#elif defined(ESP32_S3_SUPERMINI)
    vTaskDelay(pdMS_TO_TICKS(ms));
#endif
}