/**
 * @file    main.cpp
 * @brief   Motor PWM Test — stripped, no MPU6050
 *
 * Đổi MOTOR_TEST_ID để test riêng từng motor:
 *   0 = Motor 1 (GPIO 13)   1 = Motor 2 (GPIO 12)
 *   2 = Motor 3 (GPIO 11)   3 = Motor 4 (GPIO 10)
 *   -1 = Tất cả 4 motor cùng lúc
 *
 * GPIO 4 → Single click → Toggle Gate (mở/đóng)
 * GPIO 5 → Single click → Cycle duty: 0→20→50→70→90→0 (%)
 */

#include "drn_main_board_choose.h"
#include "drn_motor_pwm.h"
#include "drn_button.h"
#include "drn_time.h"


#ifdef ESP32_S3_SUPERMINI
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
#endif

static const char* TAG = "MOTOR_TEST";

// ── ĐỔI DÒNG NÀY ĐỂ TEST 1 HOẶC 4 MOTOR ──
#define MOTOR_TEST_ID  -1   // -1 = cả 4 | 0,1,2,3 = riêng từng motor

static const uint8_t DUTY_TABLE[] = {0, 20, 50, 70, 90};
static int duty_idx = 0; // Bắt đầu ở 0%

static void apply_duty(uint8_t percent) {
    if (MOTOR_TEST_ID == -1) {
        for (int i = 0; i < 4; i++) {
            DRN_Motor_Set_Duty_Percent(i, percent);
        }
        ESP_LOGI(TAG, "All motors → %d%%", percent);
    } else {
        DRN_Motor_Set_Duty_Percent(MOTOR_TEST_ID, percent);
        ESP_LOGI(TAG, "Motor %d (GPIO %d) → %d%%",
                 MOTOR_TEST_ID + 1,
                 13 - MOTOR_TEST_ID,   // GPIO 13,12,11,10
                 percent);
    }
}

extern "C" void app_main(void) {
    drn_main_board_choose_Init(); // Không còn MPU6050 sau khi fix

    ESP_LOGI(TAG, "=== MOTOR TEST (no MPU) ===");
    ESP_LOGI(TAG, "Gate: CLOSED | Nhấn GPIO4 để mở");
    ESP_LOGI(TAG, "Motor test ID: %s",
             MOTOR_TEST_ID == -1 ? "ALL (1-2-3-4)" : "Single");

    while (true) {
        uint32_t now = DRN_Millis();

        // ── Đọc + chạy FSM 2 nút ──
        DRN_Button_State_Hardware_Scan();
        DRN_Button_FSM_Process(&drn_btn_PA0, now); // GPIO 4
        DRN_Button_FSM_Process(&drn_btn_PA1, now); // GPIO 5

        // ── GPIO 4: Toggle Gate ──
        if (drn_btn_PA0.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA0.event_code = BTN_EVENT_NONE;

            // btn5_event = tham số đầu của Update_Logic → điều khiển Gate
            DRN_Motor_Update_Logic(BTN_EVENT_SINGLE_CLICK, BTN_EVENT_NONE);

            uint8_t g = DRN_Motor_Get_Gate_State();
            ESP_LOGI(TAG, "GPIO4 → Gate %s",
                     g == 0x01 ? "OPEN ✓ (motor bật)" : "CLOSE (motor tắt hết)");
        }

        // ── GPIO 5: Cycle duty ──
        if (drn_btn_PA1.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA1.event_code = BTN_EVENT_NONE;

            duty_idx = (duty_idx + 1) % 5; // 0→20→50→70→90→0
            apply_duty(DUTY_TABLE[duty_idx]);

            if (DRN_Motor_Get_Gate_State() != 0x01) {
                ESP_LOGW(TAG, "⚠ Gate đang ĐÓNG — nhấn GPIO4 trước!");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}