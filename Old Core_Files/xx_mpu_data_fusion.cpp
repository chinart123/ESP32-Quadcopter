#include "xx_mpu_data_fusion.h"
#include "i2c_mpu_debug.h" 
#include <math.h> 
#include "freertos/FreeRTOS.h" // Nhúng FreeRTOS để xài vTaskDelay
#include "freertos/task.h"

MPU_Motion_t Drone_IMU;

void MPU_Fusion_Init(void) {
    MPU_WriteReg(0x6B, 0x00);
    
    // [ĐÃ SỬA]: Dùng hàm Delay chuẩn của FreeRTOS (10ms) thay cho vòng lặp For mù
    vTaskDelay(pdMS_TO_TICKS(10)); 

    MPU_WriteReg(0x1A, 0x03); // DLPF ~41Hz
    MPU_WriteReg(0x1B, 0x08); // Gyro ±500 dps
    MPU_WriteReg(0x1C, 0x10); // Accel ±8g
    
    Drone_IMU.Roll = 0.0f;
    Drone_IMU.Pitch = 0.0f;
    Drone_IMU.Yaw = 0.0f;
}

uint8_t MPU_Fusion_Read_Burst(void) {
    uint8_t buffer[14];
    
    if (MPU_Read_Multi(0x3B, buffer, 14) == 0) return 0;
    
    Drone_IMU.Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    Drone_IMU.Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    Drone_IMU.Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    Drone_IMU.Temp_RAW    = (int16_t)((buffer[6] << 8) | buffer[7]);
    Drone_IMU.Gyro_X_RAW  = (int16_t)((buffer[8] << 8) | buffer[9]);
    Drone_IMU.Gyro_Y_RAW  = (int16_t)((buffer[10] << 8) | buffer[11]);
    Drone_IMU.Gyro_Z_RAW  = (int16_t)((buffer[12] << 8) | buffer[13]);
    return 1; 
}

uint8_t MPU_Fusion_Calibrate(void) {
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t num_samples = 1000; 
    uint8_t buffer[14]; 
    
    for (uint16_t i = 0; i < num_samples; i++) {
        if (MPU_Read_Multi(0x3B, buffer, 14) == 0) return 0;

        sum_ax += (int16_t)((buffer[0] << 8) | buffer[1]);
        sum_ay += (int16_t)((buffer[2] << 8) | buffer[3]);
        sum_az += (int16_t)((buffer[4] << 8) | buffer[5]);
        sum_gx += (int16_t)((buffer[8] << 8) | buffer[9]);
        sum_gy += (int16_t)((buffer[10] << 8) | buffer[11]);
        sum_gz += (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // [ĐÃ SỬA]: Delay 1ms chuẩn FreeRTOS chờ MPU nặn data
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    Drone_IMU.Accel_X_Offset = sum_ax / num_samples;
    Drone_IMU.Accel_Y_Offset = sum_ay / num_samples;
    Drone_IMU.Accel_Z_Offset = (sum_az / num_samples) - 4096; 
    
    Drone_IMU.Gyro_X_Offset = sum_gx / num_samples;
    Drone_IMU.Gyro_Y_Offset = sum_gy / num_samples;
    Drone_IMU.Gyro_Z_Offset = sum_gz / num_samples;

    Drone_IMU.last_time = micros();
    return 1; 
}

void MPU_Fusion_Compute(void) {
    Drone_IMU.Ax = (float)(Drone_IMU.Accel_X_RAW - Drone_IMU.Accel_X_Offset) / 4096.0f;
    Drone_IMU.Ay = (float)(Drone_IMU.Accel_Y_RAW - Drone_IMU.Accel_Y_Offset) / 4096.0f;
    Drone_IMU.Az = (float)(Drone_IMU.Accel_Z_RAW - Drone_IMU.Accel_Z_Offset) / 4096.0f;

    Drone_IMU.Gx = (float)(Drone_IMU.Gyro_X_RAW - Drone_IMU.Gyro_X_Offset) / 65.5f;
    Drone_IMU.Gy = (float)(Drone_IMU.Gyro_Y_RAW - Drone_IMU.Gyro_Y_Offset) / 65.5f;
    Drone_IMU.Gz = (float)(Drone_IMU.Gyro_Z_RAW - Drone_IMU.Gyro_Z_Offset) / 65.5f;

    // [ĐÃ SỬA LỖI TRÀN BIẾN]: Biến thời gian 32-bit
    uint32_t current_time = micros();
    Drone_IMU.dt = (float)(current_time - Drone_IMU.last_time) / 1000000.0f; 
    Drone_IMU.last_time = current_time;

    float Accel_Roll  = atan2(Drone_IMU.Ay, Drone_IMU.Az) * 57.2957795f;
    float Accel_Pitch = atan2(-Drone_IMU.Ax, sqrt(Drone_IMU.Ay * Drone_IMU.Ay + Drone_IMU.Az * Drone_IMU.Az)) * 57.2957795f;
    
    Drone_IMU.Roll  = 0.98f * (Drone_IMU.Roll  + Drone_IMU.Gx * Drone_IMU.dt) + 0.02f * Accel_Roll;
    Drone_IMU.Pitch = 0.98f * (Drone_IMU.Pitch + Drone_IMU.Gy * Drone_IMU.dt) + 0.02f * Accel_Pitch;
    Drone_IMU.Yaw   = Drone_IMU.Yaw + Drone_IMU.Gz * Drone_IMU.dt;
}