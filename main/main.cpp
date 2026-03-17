#include "nvs_flash.h"
#include <stdio.h>

#include "../_src/button.h"
#include "../_src/hal_timer_pwm.h" 
#include "../_src/i2c_mpu_debug.h"
#include "../_src/pid_control.h"   
#include "../_src/xx_mpu_data_fusion.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry.h"

// ==========================================================
// VÒNG LẶP CON: TRÁI TIM CỦA DRONE (Tần số 500Hz - 2ms/nhịp)
// ==========================================================
void drone_core_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); 
    unsigned int current_tick = 0; 

    while(1) {
        button_state_hardware_scan();
        button_fsm_process(&btn_0, current_tick);
        button_fsm_process(&btn_1, current_tick);

        // --- KHỐI 1: NÚT CHA (Chân 5) ---
        if (btn_1.event_code == BTN_EVENT_SINGLE_CLICK) {
            if (xx == 0x00 || xx == 0x04) {
                MPU_WakeUp();                               
                MPU_Read_WhoAmI(&xx_mpu_state, &xx_mpu_id); 
                
                if (xx_mpu_state == 0x99) {
                    MPU_Fusion_Init();                      
                    MPU_Fusion_Calibrate();                 
                    xLastWakeTime = xTaskGetTickCount(); 
                }
                xx = 0x01; 
            } 
            else {
                MPU_Sleep();       
                Motor_Mixer(0); 
                Drone_IMU.Roll = 0.0f; Drone_IMU.Pitch = 0.0f; Drone_IMU.Yaw = 0.0f;
                xx = 0x04; 
            }
            btn_1.event_code = BTN_EVENT_NONE; 
        }

        // --- KHỐI 2: NÚT CON (Chân 4) ---
        if (btn_0.event_code == BTN_EVENT_SINGLE_CLICK) {
            if (xx == 0x01 || xx == 0x03) {
                xx = 0x02; 
            } 
            else if (xx == 0x02) {
                Motor_Mixer(0); 
                xx = 0x03; 
            }
            btn_0.event_code = BTN_EVENT_NONE; 
        }

        // --- KHỐI 3: THỰC THI PID TẠI TRẠNG THÁI 0x02 ---
        switch (xx) {
            case 0x02:
                if (MPU_Fusion_Read_Burst() == 1) {
                    MPU_Fusion_Compute();
                    PID_Compute(Drone_IMU.Roll, Drone_IMU.Pitch, Drone_IMU.Yaw, Drone_IMU.dt);
                    Motor_Mixer(150); 
                }
                break;
            default:
                break;
        }

        current_tick += 2; 
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// ==========================================================
// VÒNG LẶP CHA: GIAO DIỆN HIỂN THỊ (Cập nhật 10 lần/s)
// ==========================================================
extern "C" void app_main(void) {
    printf("--- BOOTING ESP32-S3 (FLIGHT CONTROLLER) ---\n");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Khởi tạo phần cứng và Bluetooth
    button_hardware_init(); 
    I2C_MPU_Init(); 
    HAL_PWM_Init(); 
    PID_Init();     
    BLE_Init();
    MPU_Sleep();

    xTaskCreatePinnedToCore(drone_core_task, "DroneTask", 4096, NULL, 10, NULL, 1);

    while (1) {
        char ble_buffer[100]; // Đệm chứa chuỗi trước khi gửi

        if (xx == 0x00 || xx == 0x04) {
            sprintf(ble_buffer, "[STATE %02X] He thong NGU. Bam nut CHA (Chan 5) de khoi dong!\n", xx);
            printf("%s", ble_buffer);
            BLE_Send_Data(ble_buffer);
        } 
        else if (xx == 0x01 || xx == 0x03) {
            sprintf(ble_buffer, "[STATE %02X] MPU ID: 0x%02X | Calibrated OK! | Bam nut CON de do goc!\n", 
                    xx, xx_mpu_id);
            printf("%s", ble_buffer);
            BLE_Send_Data(ble_buffer);
        }
        else if (xx == 0x02) {
            sprintf(ble_buffer, "ROLL: %6.2f | PITCH: %6.2f | PID_R: %5.1f | PID_P: %5.1f\n", 
                    Drone_IMU.Roll, Drone_IMU.Pitch, PID_Roll.out, PID_Pitch.out);
            printf("%s", ble_buffer);
            BLE_Send_Data(ble_buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms tương đương 10 khung hình/s
    }
}