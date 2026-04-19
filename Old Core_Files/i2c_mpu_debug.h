#ifndef I2C_MPU_DEBUG_H
#define I2C_MPU_DEBUG_H

#include <stdint.h>
#include "driver/i2c.h" 

// =================================================================
// CẤU HÌNH CHÂN I2C CHO ESP32-S3
// =================================================================
#define I2C_MASTER_SDA_IO           8       // Chân SDA 
#define I2C_MASTER_SCL_IO           9       // Chân SCL
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          100000  // Đổi từ 400000 xuống 100000 để đi chậm mà chắc trên testboard
#define MPU_ADDR                    0x68    // Địa chỉ I2C của MPU-6500

// Từ điển trạng thái (Giữ nguyên)
// 0x05 : Treo Bus I2C
// 0x06 : Không tìm thấy địa chỉ MPU
// 0x07 : Lỗi truyền tải
// 0x08 : MPU không chịu nhả dữ liệu
// 0x99 : THÀNH CÔNG

extern volatile uint8_t xx;           
extern volatile uint8_t xx_mpu_state; 
extern volatile uint8_t xx_mpu_id;    

void I2C_MPU_Init(void);
void MPU_WakeUp(void);
void MPU_Sleep(void);
uint8_t MPU_WriteReg(uint8_t reg, uint8_t data);
void MPU_Read_WhoAmI(volatile uint8_t *state, volatile uint8_t *id);

// [CẬP NHẬT TỪ STAGE 7]: Đổi kiểu trả về thành uint8_t
uint8_t MPU_Read_Multi(uint8_t reg, uint8_t *data, uint8_t size);

// [CẬP NHẬT TỪ STAGE 7]: Hàm đếm thời gian Micro-giây
uint32_t micros(void);

#endif