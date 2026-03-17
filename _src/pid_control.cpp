#include "pid_control.h"
#include "hal_timer_pwm.h" 

PID_Controller_t PID_Roll;
PID_Controller_t PID_Pitch;
PID_Controller_t PID_Yaw;

void PID_Init(void) {
    PID_Roll.Kp = 1.2f; PID_Roll.Ki = 0.0f; PID_Roll.Kd = 0.0f;
    PID_Roll.integral = 0; PID_Roll.prev_error = 0; PID_Roll.out_max = 400.0f; 

    PID_Pitch.Kp = 1.2f; PID_Pitch.Ki = 0.0f; PID_Pitch.Kd = 0.0f;
    PID_Pitch.integral = 0; PID_Pitch.prev_error = 0; PID_Pitch.out_max = 400.0f;

    PID_Yaw.Kp = 2.0f; PID_Yaw.Ki = 0.0f; PID_Yaw.Kd = 0.0f;
    PID_Yaw.integral = 0; PID_Yaw.prev_error = 0; PID_Yaw.out_max = 400.0f;
}

static float Calculate_Single_PID(PID_Controller_t *pid, float current_val, float dt) {
    pid->error = pid->setpoint - current_val;
    
    pid->integral += pid->error * dt;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    else if (pid->integral < -pid->out_max) pid->integral = -pid->out_max;

    float derivative = (pid->error - pid->prev_error) / dt;
    pid->prev_error = pid->error;

    pid->out = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    
    if (pid->out > pid->out_max) pid->out = pid->out_max;
    else if (pid->out < -pid->out_max) pid->out = -pid->out_max;

    return pid->out;
}

void PID_Compute(float current_roll, float current_pitch, float current_yaw, float dt) {
    Calculate_Single_PID(&PID_Roll, current_roll, dt);
    Calculate_Single_PID(&PID_Pitch, current_pitch, dt);
    Calculate_Single_PID(&PID_Yaw, current_yaw, dt);
}

void Motor_Mixer(uint16_t base_throttle) {
    if (base_throttle < 50) {
        HAL_TIM3_PWM_SetDuty(1, 0); 
        HAL_TIM3_PWM_SetDuty(2, 0);
        HAL_TIM3_PWM_SetDuty(3, 0); 
        HAL_TIM3_PWM_SetDuty(4, 0);
        return;
    }

    int16_t m1 = base_throttle + PID_Pitch.out + PID_Roll.out - PID_Yaw.out;
    int16_t m2 = base_throttle + PID_Pitch.out - PID_Roll.out + PID_Yaw.out;
    int16_t m3 = base_throttle - PID_Pitch.out - PID_Roll.out - PID_Yaw.out;
    int16_t m4 = base_throttle - PID_Pitch.out + PID_Roll.out + PID_Yaw.out;

	// Ép an toàn 10-bit cho ESP32 (Đã thêm ngoặc nhọn để chiều ý Compiler)
    if (m1 > 1023) { m1 = 1023; } else if (m1 < 0) { m1 = 0; }
    if (m2 > 1023) { m2 = 1023; } else if (m2 < 0) { m2 = 0; }
    if (m3 > 1023) { m3 = 1023; } else if (m3 < 0) { m3 = 0; }
    if (m4 > 1023) { m4 = 1023; } else if (m4 < 0) { m4 = 0; }

    HAL_TIM3_PWM_SetDuty(1, m1);
    HAL_TIM3_PWM_SetDuty(2, m2);
    HAL_TIM3_PWM_SetDuty(3, m3);
    HAL_TIM3_PWM_SetDuty(4, m4);
}