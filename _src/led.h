#ifndef LED_H
#define LED_H

#include <stdint.h>
#include "driver/gpio.h" // Thư viện chân IO của ESP

// Định nghĩa chân LED cho ESP32-S3 (Sếp tự do chọn chân)
#define LED_0_PIN GPIO_NUM_10 // Thay cho PB12
#define LED_1_PIN GPIO_NUM_11 // Thay cho PC13

// 1. Cấu hình hằng số (Giữ nguyên của Sếp)
typedef enum {
    LED_CFG_MAX_BRIGHTNESS = 20,   
    LED_CFG_FADE_SPEED     = 50    
} LedConfig_t;

// 2. Các chế độ hoạt động (Giữ nguyên)
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,          
    LED_MODE_FADE_IN,     
    LED_MODE_FADE_OUT,    
    LED_MODE_ASYNC_BLINK  
} LedMode_TypeDef;

// 3. Hồ sơ Đèn LED (Giữ nguyên)
typedef struct {
    LedMode_TypeDef mode;
    unsigned int    brightness;     
    unsigned int    last_slow_tick; 
    unsigned char   is_on_phase;    
    
    unsigned int    time_on;        
    unsigned int    time_off;       
} Led_Context;

// Khai báo 2 con LED mới
extern Led_Context led_0;
extern Led_Context led_1;

// Khai báo hàm
void led_hardware_init(void); // Thêm hàm khởi tạo chân
void led_set_mode(Led_Context *led, LedMode_TypeDef new_mode, unsigned int t_on, unsigned int t_off, unsigned int current_tick);
void led_process(Led_Context *led, unsigned int current_tick);
void led_hardware_update(unsigned int current_tick); 

#endif // LED_H