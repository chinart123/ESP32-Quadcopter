/**
 * @file    drn_button.cpp
 * @brief   Button HAL — ESP32 implementation (matches STM32 logic)
 * @target  ESP32-S3-SuperMini
 * @version 2.0
 */
#include "drn_button.h"
#include "drn_main_board_choose.h"

#ifdef ESP32_S3_SUPERMINI
    #include "driver/gpio.h"
    #include "esp_log.h"
#endif

static const char* TAG = "DRN_BUTTON";

#define BTN_PA0_GPIO GPIO_NUM_4
#define BTN_PA1_GPIO GPIO_NUM_5
// Buttons are active low (0 = pressed)

// Public instances
DRN_Button_Context drn_btn_PA0;
DRN_Button_Context drn_btn_PA1;

void DRN_Button_Init(void) {
#ifdef ESP32_S3_SUPERMINI
    // Configure GPIOs with internal pull‑up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_PA0_GPIO) | (1ULL << BTN_PA1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize context structures (match STM32 initial values)
    drn_btn_PA0 = (DRN_Button_Context){
        .pin_state = 1,
        .press_duration = 0,
        .release_duration = 0,
        .click_count = 0,
        .event_code = BTN_EVENT_NONE,
        .last_update_tick = 0,
        .hold_target = 50      // 50 * 10ms = 500ms
    };
    drn_btn_PA1 = (DRN_Button_Context){
        .pin_state = 1,
        .press_duration = 0,
        .release_duration = 0,
        .click_count = 0,
        .event_code = BTN_EVENT_NONE,
        .last_update_tick = 0,
        .hold_target = 20      // 20 * 10ms = 200ms (as in STM32 example)
    };

    ESP_LOGI(TAG, "Buttons initialized (STM32‑compatible FSM)");
#endif
}

void DRN_Button_State_Hardware_Scan(void) {
#ifdef ESP32_S3_SUPERMINI
    // Read raw pin (0 = pressed because active low)
    drn_btn_PA0.pin_state = gpio_get_level(BTN_PA0_GPIO);
    drn_btn_PA1.pin_state = gpio_get_level(BTN_PA1_GPIO);
#endif
}

void DRN_Button_FSM_Process(DRN_Button_Context *btn, uint32_t current_tick) {
    // Process only every 10ms (matching STM32 timing)
    if ((current_tick - btn->last_update_tick) >= 10) {
        btn->last_update_tick = current_tick;

        if (btn->pin_state == 0) {   // Button is pressed (active low)
            btn->release_duration = 0;
            if (btn->press_duration < 60000) {
                btn->press_duration++;
            }

            // Check for HOLD event
            if (btn->press_duration == btn->hold_target) {
                btn->event_code = BTN_EVENT_HOLD;
                btn->click_count = 0;
                ESP_LOGD(TAG, "GPIO %d HOLD", (btn == &drn_btn_PA0) ? 4 : 5);
            }
        }
        else {   // Button released
            if (btn->press_duration > 0) {
                // Valid press (longer than debounce) but not hold
                if (btn->press_duration > BTN_TICK_DEBOUNCE && btn->press_duration < btn->hold_target) {
                    btn->click_count++;
                    ESP_LOGD(TAG, "GPIO %d click count: %d", (btn == &drn_btn_PA0) ? 4 : 5, btn->click_count);
                }
                btn->press_duration = 0;
            }

            if (btn->click_count > 0) {
                if (btn->release_duration < 60000) {
                    btn->release_duration++;
                }

                // Double click detected immediately when second click occurs
                if (btn->click_count == 2) {
                    btn->event_code = BTN_EVENT_DOUBLE_CLICK;
                    btn->click_count = 0;
                    btn->release_duration = 0;
                    ESP_LOGI(TAG, "GPIO %d DOUBLE_CLICK", (btn == &drn_btn_PA0) ? 4 : 5);
                }
                // Single click after double‑click gap timeout
                else if (btn->release_duration > BTN_TICK_DOUBLE_GAP) {
                    if (btn->click_count == 1) {
                        btn->event_code = BTN_EVENT_SINGLE_CLICK;
                        ESP_LOGI(TAG, "GPIO %d SINGLE_CLICK", (btn == &drn_btn_PA0) ? 4 : 5);
                    }
                    btn->click_count = 0;
                    btn->release_duration = 0;
                }
            } else {
                btn->release_duration = 0;
            }
        }
    }
}