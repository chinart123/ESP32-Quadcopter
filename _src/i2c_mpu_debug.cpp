#include "../_src/i2c_mpu_debug.h"

#include "esp_log.h"
#include "esp_timer.h" // THƯ VIỆN ĐẾM THỜI GIAN CỦA ESP (Thay thế TIM4 của STM32)

static const char *TAG = "I2C_MPU"; 

volatile uint8_t xx = 0;           
volatile uint8_t xx_mpu_state = 0; 
volatile uint8_t xx_mpu_id = 0;    

void I2C_MPU_Init(void) {
    i2c_config_t conf = {}; 
    
    conf.mode = I2C_MODE_MASTER;                   
    conf.sda_io_num = I2C_MASTER_SDA_IO;           
    conf.scl_io_num = I2C_MASTER_SCL_IO;           
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;       
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;       
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;    
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Khoi tao I2C thanh cong (SDA:%d, SCL:%d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    } else {
        ESP_LOGE(TAG, "Loi khoi tao I2C!");
    }
}

uint8_t MPU_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) return 0x99; 
    return 0x07; 
}

void MPU_WakeUp(void) { MPU_WriteReg(0x6B, 0x00); }
void MPU_Sleep(void)  { MPU_WriteReg(0x6B, 0x40); }

void MPU_Read_WhoAmI(volatile uint8_t *state, volatile uint8_t *id) {
    uint8_t reg_who_am_i = 0x75; 
    uint8_t rx_data = 0;
    
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU_ADDR, &reg_who_am_i, 1, &rx_data, 1, 1000 / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        *id = rx_data;
        *state = 0x99; 
    } else {
        *state = 0x06; 
    }
}

// =================================================================
// [CẬP NHẬT TỪ STAGE 7]: HÀM ĐỌC NHIỀU BYTE (TRẢ VỀ 1 HOẶC 0)
// =================================================================
uint8_t MPU_Read_Multi(uint8_t reg, uint8_t *data, uint8_t size) {
    // Gọi hàm I2C của ESP-IDF với thời gian Timeout là 20ms (giống hệ số 500.000 cũ của sếp)
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU_ADDR, &reg, 1, data, size, 20 / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        xx_mpu_state = 0x99; 
        return 1; // [Thành công]: Trả về 1 để PID hút data
    } else {
        xx_mpu_state = 0x08; 
        // ESP-IDF tự lo vụ Reset Bus I2C, sếp không cần gọi hàm I2C_Bus_Recovery() nữa!
        return 0; // [Đứt dây]: Trả về 0 báo cáo lỗi, ngăn chặn tính toán rác
    }
}

// =================================================================
// [CẬP NHẬT TỪ STAGE 7]: HÀM ĐẾM THỜI GIAN MICRO-GIÂY
// =================================================================
// Bên STM32 sếp dùng thanh ghi TIM4->CNT (trả về uint16_t bị tràn nhanh).
// Bên ESP32 sếp dùng hàm hệ thống, trả về uint32_t (đếm được liên tục 71 phút mới tràn).
uint32_t micros(void) {
    // esp_timer_get_time() trả về số micro-giây tính từ lúc chip vừa khởi động
    return (uint32_t)esp_timer_get_time();
}