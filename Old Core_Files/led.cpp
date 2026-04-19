#include "../_src/led.h"

// Khởi tạo 2 bóng đèn (Tắt, sáng 0)
Led_Context led_0 = {LED_MODE_OFF, 0, 0, 0, 0, 0};
Led_Context led_1 = {LED_MODE_OFF, 0, 0, 0, 0, 0};

// =========================================================
// HÀM MỚI: KHỞI TẠO PHẦN CỨNG CHÂN LED (Output)
// =========================================================
void led_hardware_init(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;      
    io_conf.mode = GPIO_MODE_OUTPUT;            // Chế độ OUTPUT cho LED
    io_conf.pin_bit_mask = (1ULL << LED_0_PIN) | (1ULL << LED_1_PIN); 
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;   
    
    gpio_config(&io_conf);
    
    // Đảm bảo LED tắt lúc mới cấp nguồn (Active-Low nên xuất mức 1 là Tắt)
    gpio_set_level(LED_0_PIN, 1);
    gpio_set_level(LED_1_PIN, 1);
}

// =========================================================
// MẢNG THUẬT TOÁN (GIỮ NGUYÊN 100% CỦA SẾP)
// =========================================================
void led_set_mode(Led_Context *led, LedMode_TypeDef new_mode, unsigned int t_on, unsigned int t_off, unsigned int current_tick) {
    if (led->mode == new_mode && led->time_on == t_on && led->time_off == t_off) return; 
    
    led->mode = new_mode;
    led->time_on = t_on;
    led->time_off = t_off;
    led->last_slow_tick = current_tick;
    
    if (new_mode == LED_MODE_OFF) {
        led->brightness = 0;
    } 
    else if (new_mode == LED_MODE_ON) {
        led->brightness = LED_CFG_MAX_BRIGHTNESS;
    }
    else if (new_mode == LED_MODE_ASYNC_BLINK) {
        led->brightness = LED_CFG_MAX_BRIGHTNESS; 
        led->is_on_phase = 1; 
    }
}

void led_process(Led_Context *led, unsigned int current_tick) {
    if (led->mode == LED_MODE_ASYNC_BLINK) {
        unsigned int target_delay = (led->is_on_phase) ? led->time_on : led->time_off;
        if (current_tick - led->last_slow_tick >= target_delay) {
            led->last_slow_tick = current_tick;
            led->is_on_phase = !led->is_on_phase;
            led->brightness = (led->is_on_phase) ? LED_CFG_MAX_BRIGHTNESS : 0;
        }
    }
    else if (led->mode == LED_MODE_FADE_IN) {
        if (current_tick - led->last_slow_tick >= LED_CFG_FADE_SPEED) {
            led->last_slow_tick = current_tick;
            if (led->brightness < LED_CFG_MAX_BRIGHTNESS) led->brightness++;
        }
    }
    else if (led->mode == LED_MODE_FADE_OUT) {
        if (current_tick - led->last_slow_tick >= LED_CFG_FADE_SPEED) {
            led->last_slow_tick = current_tick;
            if (led->brightness > 0) led->brightness--;
        }
    }
}

// =========================================================
// MẢNG PHẦN CỨNG: BĂM XUNG SOFTWARE PWM (Đã thay bằng API ESP)
// =========================================================
void led_hardware_update(unsigned int current_tick) {
    unsigned int pwm_phase = current_tick % LED_CFG_MAX_BRIGHTNESS; 
    
    // Cập nhật LED 0 (Active-Low: Kéo xuống 0 là sáng)
    if (pwm_phase < led_0.brightness) {
        gpio_set_level(LED_0_PIN, 0); // Tương đương GPIOB->BRR bên STM32
    } else {
        gpio_set_level(LED_0_PIN, 1); // Tương đương GPIOB->BSRR bên STM32
    }

    // Cập nhật LED 1 (Active-Low: Kéo xuống 0 là sáng)
    if (pwm_phase < led_1.brightness) {
        gpio_set_level(LED_1_PIN, 0); 
    } else {
        gpio_set_level(LED_1_PIN, 1); 
    }
}