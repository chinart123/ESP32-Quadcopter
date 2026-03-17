#include "hal_timer_pwm.h"
#include "driver/ledc.h" // Vũ khí băm xung của ESP-IDF

// ========================================================
// SẾP TỰ DO ĐỔI 4 CHÂN NÀY THEO SƠ ĐỒ HÀN MẠCH CỦA SẾP NHÉ:
#define MOTOR1_PIN GPIO_NUM_10 // Trái-Trước (M1)
#define MOTOR2_PIN GPIO_NUM_11 // Phải-Trước (M2)
#define MOTOR3_PIN GPIO_NUM_12 // Phải-Sau (M3)
#define MOTOR4_PIN GPIO_NUM_13 // Trái-Sau (M4)
// ========================================================

void HAL_PWM_Init(void) {
    // 1. Cấu hình lõi Timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT; // Phân giải 10-bit (0 đến 1023) -> Quá khớp với mức ga 999 của sếp!
    
    // [LƯU Ý CỰC KỲ QUAN TRỌNG VỀ TẦN SỐ]:
    // - Nếu sếp xài Động cơ chổi than (Drone đồ chơi Coreless): Để 5000 (5kHz)
    // - Nếu sếp xài Động cơ Không chổi than (ESC cục to đùng): Sửa thành 400 (400Hz)
    ledc_timer.freq_hz          = 5000;              
    
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

// 2. Cấu hình 4 Kênh xuất xung ra 4 chân GPIO
    ledc_channel_config_t ledc_channel[4] = {}; // Khởi tạo mảng trống

    // Kênh 0 (Motor 1)
    ledc_channel[0].gpio_num       = MOTOR1_PIN;
    ledc_channel[0].speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel[0].channel        = LEDC_CHANNEL_0;
    ledc_channel[0].intr_type      = LEDC_INTR_DISABLE;
    ledc_channel[0].timer_sel      = LEDC_TIMER_0;
    ledc_channel[0].duty           = 0;
    ledc_channel[0].hpoint         = 0;

    // Kênh 1 (Motor 2)
    ledc_channel[1].gpio_num       = MOTOR2_PIN;
    ledc_channel[1].speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel[1].channel        = LEDC_CHANNEL_1;
    ledc_channel[1].intr_type      = LEDC_INTR_DISABLE;
    ledc_channel[1].timer_sel      = LEDC_TIMER_0;
    ledc_channel[1].duty           = 0;
    ledc_channel[1].hpoint         = 0;

    // Kênh 2 (Motor 3)
    ledc_channel[2].gpio_num       = MOTOR3_PIN;
    ledc_channel[2].speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel[2].channel        = LEDC_CHANNEL_2;
    ledc_channel[2].intr_type      = LEDC_INTR_DISABLE;
    ledc_channel[2].timer_sel      = LEDC_TIMER_0;
    ledc_channel[2].duty           = 0;
    ledc_channel[2].hpoint         = 0;

    // Kênh 3 (Motor 4)
    ledc_channel[3].gpio_num       = MOTOR4_PIN;
    ledc_channel[3].speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel[3].channel        = LEDC_CHANNEL_3;
    ledc_channel[3].intr_type      = LEDC_INTR_DISABLE;
    ledc_channel[3].timer_sel      = LEDC_TIMER_0;
    ledc_channel[3].duty           = 0;
    ledc_channel[3].hpoint         = 0;

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}
// Hàm này đóng vai trò "Kẻ thế mạng" cho hàm HAL_TIM3 cũ của STM32
void HAL_TIM3_PWM_SetDuty(uint8_t motor_id, uint16_t duty) {
    // Chốt chặn an toàn: Hệ thống 10-bit không cho phép số > 1023
    if (duty > 1023) duty = 1023;

    switch (motor_id) {
        case 1: 
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty); 
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); 
            break;
        case 2: 
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty); 
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); 
            break;
        case 3: 
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty); 
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); 
            break;
        case 4: 
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty); 
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); 
            break;
    }
}