#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drn_main_board_choose.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "APP_MAIN";

// Các biến trạng thái State Machine
uint8_t xx_mpu_state = 0x00;       
bool is_scraping_data = false;      // Trạng thái cào/dừng cào dữ liệu
uint32_t state_0x01_start_time = 0; // Bộ đếm thời gian 1.5s

extern "C" void app_main(void) {
    // 1. Khởi tạo toàn bộ hệ thống
    drn_main_board_choose_Init();

    // 2. Vòng lặp siêu hệ thống (Super Loop)
    while (1) {
        uint32_t current_tick = DRN_Millis();

        // 3. Quét phần cứng nút bấm và chạy FSM (chu kỳ 10ms do bạn định nghĩa)
        DRN_Button_State_Hardware_Scan();
        DRN_Button_FSM_Process(&drn_btn_PA0, current_tick); // GPIO 4 (Nút cào data)
        DRN_Button_FSM_Process(&drn_btn_PA1, current_tick); // GPIO 5 (Nút đóng/mở cổng)

        // ---------------------------------------------------------
        // LOGIC NÚT 1 (GPIO 5 - drn_btn_PA1): ĐÓNG / MỞ CỔNG
        // ---------------------------------------------------------
        if (drn_btn_PA1.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA1.event_code = BTN_EVENT_NONE; // Clear cờ

            if (xx_mpu_state == 0x00 || xx_mpu_state == 0x04) {
                // Mở cổng
                xx_mpu_state = 0x01;
                state_0x01_start_time = current_tick;
                is_scraping_data = false; // Đảm bảo trạng thái ban đầu là chưa quét
                ESP_LOGI(TAG, "STATE = 0x01: Cong da MO (GPIO 5), cho 1.5s...");
            } 
            else if (xx_mpu_state == 0x01) {
                // Đóng cổng
                xx_mpu_state = 0x04;
                is_scraping_data = false; // Ngừng cào dữ liệu
                
                // Reset dữ liệu góc và gia tốc về 0 (Tránh memset cả struct để giữ lại data Calibrate)
                drn_mpu_data.roll_deg = 0.0f;
                drn_mpu_data.pitch_deg = 0.0f;
                drn_mpu_data.yaw_deg = 0.0f;
                drn_mpu_data.accel_x_g = 0.0f; 
                drn_mpu_data.accel_y_g = 0.0f; 
                drn_mpu_data.accel_z_g = 0.0f;
                drn_mpu_data.gyro_x_dps = 0.0f; 
                drn_mpu_data.gyro_y_dps = 0.0f; 
                drn_mpu_data.gyro_z_dps = 0.0f;
                
                ESP_LOGI(TAG, "STATE = 0x04: Cong DONG (GPIO 5). Da reset data MPU.");
                
                // Ngay lập tức về 0x00 theo logic luân phiên
                xx_mpu_state = 0x00;
                ESP_LOGI(TAG, "STATE = 0x00: Tro ve trang thai ranh.");
            }
        }

        // ---------------------------------------------------------
        // LOGIC NÚT 2 (GPIO 4 - drn_btn_PA0): BẬT / DỪNG CÀO DỮ LIỆU
        // ---------------------------------------------------------
        if (drn_btn_PA0.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA0.event_code = BTN_EVENT_NONE; // Clear cờ

            // Chỉ cho phép thao tác nếu cổng đang mở (0x01) VÀ đã qua 1.5s (1500ms)
            if (xx_mpu_state == 0x01 && (current_tick - state_0x01_start_time >= 1500)) {
                is_scraping_data = !is_scraping_data; // Luân phiên trạng thái (Toggle)
                
                if (is_scraping_data) {
                    ESP_LOGI(TAG, "RESUME: Dang cao du lieu... (GPIO 4)");
                } else {
                    ESP_LOGI(TAG, "CAPTURE: Da dung terminal, luu ket qua. (GPIO 4)");
                }
            } else if (xx_mpu_state == 0x01) {
                ESP_LOGW(TAG, "Chua du 1.5s tu khi mo cong! Hay doi them...");
            }
        }

        // ---------------------------------------------------------
        // LOGIC CẬP NHẬT VÀ IN DỮ LIỆU MPU
        // ---------------------------------------------------------
        // Task của MPU chỉ chạy khi cổng mở (0x01)
        if (xx_mpu_state == 0x01) {
            
            // Cập nhật tính toán data liên tục dưới background (nếu cần cho các task khác)
            // Hoặc bạn có thể cho nó vào if (is_scraping_data) nếu chỉ muốn tính khi quét
            DRN_MPU6050_Run_Task(current_tick);
            
            // Chỉ in ra terminal khi cờ cào dữ liệu bật
            if (is_scraping_data) {
                static uint32_t last_print_ms = 0;
                // Throttle tốc độ in (100ms in 1 lần) để terminal không bị tràn
                if (current_tick - last_print_ms >= 100) {
                    last_print_ms = current_tick;
                    printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n", 
                           drn_mpu_data.roll_deg, drn_mpu_data.pitch_deg, drn_mpu_data.yaw_deg);
                }
            }
        }

        // Nhường CPU cho FreeRTOS (Chống watch-dog reset)
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}