/**
 * @file    drn_time.h
 * @brief   HAL timing utilities — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.0
 */
#ifndef DRN_TIME_H
#define DRN_TIME_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t DRN_Time_Init(void);
uint32_t  DRN_Millis(void);
void      DRN_Delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif // DRN_TIME_H