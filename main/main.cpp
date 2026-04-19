/**
 * @file    main.cpp
 * @brief   Raw GPIO test for buttons — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "BTN_TEST";

#define BTN_PA0_GPIO GPIO_NUM_4
#define BTN_PA1_GPIO GPIO_NUM_5

extern "C" void app_main(void)
{
    // Configure GPIOs as inputs with pull‑up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_PA0_GPIO) | (1ULL << BTN_PA1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIO test ready. Press buttons on GPIO4 and GPIO5.");

    int last_pa0 = 1;
    int last_pa1 = 1;

    while (1) {
        int pa0 = gpio_get_level(BTN_PA0_GPIO);
        int pa1 = gpio_get_level(BTN_PA1_GPIO);

        if (pa0 != last_pa0) {
            ESP_LOGI(TAG, "GPIO4 changed: %d", pa0);
            last_pa0 = pa0;
        }
        if (pa1 != last_pa1) {
            ESP_LOGI(TAG, "GPIO5 changed: %d", pa1);
            last_pa1 = pa1;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

