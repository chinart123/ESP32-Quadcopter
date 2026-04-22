#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drn_main_board_choose.h"
#include "drn_raw_mpu6050.h"   // Using Raw Library
#include "esp_log.h"
#include <string.h>

static const char* TAG = "APP_MAIN_RAW";

// State Machine variables
uint8_t xx_mpu_state = 0x00;       
bool is_scraping_data = false;      
uint32_t state_0x01_start_time = 0; 

extern "C" void app_main(void) {
    // 1. Initialize the entire system
    drn_main_board_choose_Init();

    while (1) {
        uint32_t current_tick = DRN_Millis();

        DRN_Button_State_Hardware_Scan();
        DRN_Button_FSM_Process(&drn_btn_PA0, current_tick); // Scraping button
        DRN_Button_FSM_Process(&drn_btn_PA1, current_tick); // Gate toggle button

        // BUTTON 1 LOGIC (GPIO 5): OPEN / CLOSE GATE
        if (drn_btn_PA1.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA1.event_code = BTN_EVENT_NONE; 

            if (xx_mpu_state == 0x00 || xx_mpu_state == 0x04) {
                xx_mpu_state = 0x01;
                state_0x01_start_time = current_tick;
                is_scraping_data = false; 
                ESP_LOGI(TAG, "STATE = 0x01: Gate OPENED - RAW mode (Accel only)");
            } 
            else if (xx_mpu_state == 0x01) {
                xx_mpu_state = 0x04;
                is_scraping_data = false; 
                
                // Reset raw data
                drn_raw_mpu_data.roll_raw_deg = 0.0f;
                drn_raw_mpu_data.pitch_raw_deg = 0.0f;
                drn_raw_mpu_data.accel_x_g = 0.0f; 
                drn_raw_mpu_data.accel_y_g = 0.0f; 
                drn_raw_mpu_data.accel_z_g = 0.0f;
                
                ESP_LOGI(TAG, "STATE = 0x04: Gate CLOSED. RAW data reset.");
                xx_mpu_state = 0x00;
            }
        }

        // BUTTON 2 LOGIC (GPIO 4): START / STOP SCRAPING
        if (drn_btn_PA0.event_code == BTN_EVENT_SINGLE_CLICK) {
            drn_btn_PA0.event_code = BTN_EVENT_NONE; 

            if (xx_mpu_state == 0x01 && (current_tick - state_0x01_start_time >= 1500)) {
                is_scraping_data = !is_scraping_data; 
                if (is_scraping_data) {
                    ESP_LOGI(TAG, "RESUME: Scraping RAW data...");
                } else {
                    ESP_LOGI(TAG, "CAPTURE: Terminal stopped.");
                }
            }
        }

        // MPU UPDATE AND PRINT LOGIC
        if (xx_mpu_state == 0x01) {
            // Run Raw task
            DRN_RawMPU6050_Run_Task(current_tick);
            
            if (is_scraping_data) {
                static uint32_t last_print_ms = 0;
                if (current_tick - last_print_ms >= 100) {
                    last_print_ms = current_tick;
                    // Note: Raw version does not have yaw_deg
                    printf("RAW_Roll: %6.2f | RAW_Pitch: %6.2f\n", 
                           drn_raw_mpu_data.roll_raw_deg, drn_raw_mpu_data.pitch_raw_deg);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}