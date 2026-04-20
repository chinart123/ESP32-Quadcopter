/**
 * @file    main.cpp
 * @brief   Main application — Windify
 */
#include "drn_main_board_choose.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting Windify with STM32‑style button FSM");
    drn_main_board_choose_Init();

    while (1) {
        uint32_t now = DRN_Millis();

        DRN_Button_State_Hardware_Scan();
        DRN_Button_FSM_Process(&drn_btn_PA0, now);
        DRN_Button_FSM_Process(&drn_btn_PA1, now);

        // Motor logic uses button events
        DRN_Motor_Update_Logic(drn_btn_PA1.event_code, drn_btn_PA0.event_code);

        // Clear events after use
        drn_btn_PA1.event_code = BTN_EVENT_NONE;
        drn_btn_PA0.event_code = BTN_EVENT_NONE;

        DRN_Motor_Run_Task(now);

        vTaskDelay(pdMS_TO_TICKS(1));   // yield to IDLE
    }
}