/**
 * @file    drn_motor_pwm.h
 * @brief   Motor PWM control HAL — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.1
 */
#ifndef DRN_MOTOR_PWM_H
#define DRN_MOTOR_PWM_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gate control states (Button 1)
 */
#define GATE_OPEN   0x01
#define GATE_CLOSE  0x04

/**
 * @brief Sensor acquisition states (Button 2)
 */
typedef enum {
    SENSOR_IDLE = 0,
    SENSOR_ACQUIRING,
    SENSOR_CAPTURED
} sensor_state_t;

/**
 * @brief Initialize LEDC PWM channels for motors.
 *        Configures GPIOs, 16kHz frequency, 10-bit resolution.
 *
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t DRN_Motor_PWM_Init(void);

/**
 * @brief Set raw duty cycle (0–1023) for a specific motor channel.
 *
 * @param motor_id Motor index (0–3).
 * @param duty Duty cycle value (0 = 0%, 1023 = 100%).
 */
void DRN_Motor_Set_Duty(uint8_t motor_id, uint32_t duty);

/**
 * @brief Set duty cycle as percentage (0–100) for a specific motor.
 *
 * @param motor_id Motor index (0–3).
 * @param percent Duty percentage (0–100).
 */
void DRN_Motor_Set_Duty_Percent(uint8_t motor_id, uint8_t percent);

/**
 * @brief Get current gate state (0x01 = OPEN, 0x04 = CLOSE).
 */
uint8_t DRN_Motor_Get_Gate_State(void);

/**
 * @brief Get current sensor acquisition state.
 */
sensor_state_t DRN_Motor_Get_Sensor_State(void);

/**
 * @brief Update motor logic based on button events (non‑blocking FSM step).
 *        Called from the super loop.
 *
 * @param btn5_event Event code from button 5 (GPIO5) – Gate control.
 * @param btn4_event Event code from button 4 (GPIO4) – Sensor control.
 */
void DRN_Motor_Update_Logic(uint8_t btn5_event, uint8_t btn4_event);

/**
 * @brief Apply motor state to hardware and handle time‑dependent actions.
 *        Must be called periodically from the super loop.
 *
 * @param current_time Timestamp from DRN_Millis().
 */
void DRN_Motor_Run_Task(uint32_t current_time);

#ifdef __cplusplus
}
#endif
#endif // DRN_MOTOR_PWM_H