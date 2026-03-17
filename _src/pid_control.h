#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h> // Đã xóa thư viện STM32

typedef struct {
    float Kp;         
    float Ki;         
    float Kd;         
    float setpoint;   
    float error;      
    float integral;   
    float prev_error; 
    float out;        
    float out_max;    
} PID_Controller_t;

extern PID_Controller_t PID_Roll;
extern PID_Controller_t PID_Pitch;
extern PID_Controller_t PID_Yaw;

void PID_Init(void);
void PID_Compute(float current_roll, float current_pitch, float current_yaw, float dt);
void Motor_Mixer(uint16_t base_throttle);

#endif