#ifndef XX_MPU_DATA_FUSION_H
#define XX_MPU_DATA_FUSION_H

#include <stdint.h> // Đổi stm32f10x.h thành thư viện C chuẩn

typedef struct {
    // 1. Raw Data
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Temp_RAW;      
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // 2. Offset Data
    int16_t Accel_X_Offset;
    int16_t Accel_Y_Offset;
    int16_t Accel_Z_Offset;
    int16_t Gyro_X_Offset;
    int16_t Gyro_Y_Offset;
    int16_t Gyro_Z_Offset;

    // 3. Scaled Data
    float Ax, Ay, Az;      
    float Gx, Gy, Gz;      

    // 4. Fused Data (Góc bay)
    float Roll;            
    float Pitch;           
    float Yaw;             

    // [ĐÃ SỬA LỖI TRÀN BIẾN]: Nâng lên 32-bit cho ESP32
    float dt;              
    uint32_t last_time; 
} MPU_Motion_t;

extern MPU_Motion_t Drone_IMU; 

void MPU_Fusion_Init(void);       
uint8_t MPU_Fusion_Read_Burst(void);
void MPU_Fusion_Compute(void);    
uint8_t MPU_Fusion_Calibrate(void);

#endif