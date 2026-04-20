/**
 * @file    drn_main_board_choose.h
 * @brief   Board selection and global HAL initialization — Project Windify
 * @target  Dual-platform: ESP32-S3-SuperMini / STM32F103C8T6
 * @version 1.1
 */
#ifndef DRN_MAIN_BOARD_CHOOSE_H
#define DRN_MAIN_BOARD_CHOOSE_H

// ============================================================
// BOARD SELECTION — Uncomment exactly ONE target
// ============================================================
// #define STM32F103C8T6
#define ESP32_S3_SUPERMINI

// ============================================================
// Platform-specific OS headers (FreeRTOS / STM32 HAL)
// ============================================================
#ifdef ESP32_S3_SUPERMINI
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#elif defined(STM32F103C8T6)
    // [STM32 PLACEHOLDER] Insert STM32 HAL includes here
#endif

// ============================================================
// God Header — Includes all HAL modules
// ============================================================
#include "drn_time.h"
#include "drn_motor_pwm.h"
#include "drn_button.h"
#include "drn_mpu6050.h"   

#ifdef __cplusplus
extern "C" {
#endif

void drn_main_board_choose_Init(void);
void drn_main_board_choose_Delay_ms(unsigned int ms);

#ifdef __cplusplus
}
#endif

#endif // DRN_MAIN_BOARD_CHOOSE_H