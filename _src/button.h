#ifndef BUTTON_H
#define BUTTON_H
 
#include <stdint.h>
#include "driver/gpio.h" // Thư viện điều khiển chân IO của ESP-IDF

// Định nghĩa chân Nút bấm cho ESP32-S3 (Sếp đổi chân tùy ý)
#define BTN_0_PIN GPIO_NUM_4  // Thay thế cho PA0 cũ
#define BTN_1_PIN GPIO_NUM_5  // Thay thế cho PA1 cũ

// 1. Nhóm cấu hình thời gian (Dùng chung cho TẤT CẢ các nút)
typedef enum {
    BTN_TICK_DEBOUNCE   = 3,     // 3 nhịp = 30ms (Lọc nhiễu)
    BTN_TICK_DOUBLE_GAP = 30     // 30 nhịp = 300ms (Chờ click lần 2)
} ButtonTiming_t;

// 2. Nhóm sự kiện đầu ra
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_HOLD
} ButtonEvent_TypeDef;

// 3. Hồ sơ bệnh án của Nút bấm
typedef struct {
    unsigned char       pin_state;        
    unsigned int        press_duration;   
    unsigned int        release_duration; 
    unsigned char       click_count;      
    ButtonEvent_TypeDef event_code;       
    unsigned int        last_update_tick; 
    unsigned int        hold_target;      
} Button_Context;

// Khai báo 2 nút để main.cpp có thể gọi
extern Button_Context btn_0;
extern Button_Context btn_1;

// Khai báo hàm
void button_hardware_init(void); // Hàm mới thêm để khởi tạo chân ESP32
void button_state_hardware_scan(void);
void button_fsm_process(Button_Context *btn, unsigned int current_tick);

#endif // BUTTON_H