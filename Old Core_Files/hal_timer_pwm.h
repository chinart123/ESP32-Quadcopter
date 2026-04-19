#ifndef HAL_TIMER_PWM_H
#define HAL_TIMER_PWM_H

#include <stdint.h>

void HAL_PWM_Init(void); 

// Cố tình giữ lại cái tên hàm "đậm chất STM32" này để đánh lừa file pid_control.c của sếp!
void HAL_TIM3_PWM_SetDuty(uint8_t motor_id, uint16_t duty);

#endif