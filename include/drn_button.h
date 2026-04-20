/**
 * @file    drn_button.h
 * @brief   Button HAL — Project Windify (dual‑platform)
 * @target  ESP32-S3 / STM32F103
 * @version 2.0
 */
#ifndef DRN_BUTTON_H
#define DRN_BUTTON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BTN_TICK_DEBOUNCE   = 3,
    BTN_TICK_DOUBLE_GAP = 30
} DRN_ButtonTiming_t;

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_HOLD
} DRN_ButtonEvent_t;

typedef struct {
    uint8_t             pin_state;          // raw pin state (1 = released, 0 = pressed)
    uint32_t            press_duration;     // counter of 10ms ticks while pressed
    uint32_t            release_duration;   // counter of 10ms ticks while released
    uint8_t             click_count;        // number of clicks detected in sequence
    DRN_ButtonEvent_t   event_code;         // latest event
    uint32_t            last_update_tick;   // last time FSM was processed (ms)
    uint32_t            hold_target;        // tick count for hold event
} DRN_Button_Context;

extern DRN_Button_Context drn_btn_PA0;
extern DRN_Button_Context drn_btn_PA1;

void DRN_Button_Init(void);
void DRN_Button_State_Hardware_Scan(void);
void DRN_Button_FSM_Process(DRN_Button_Context *btn, uint32_t current_tick);

#ifdef __cplusplus
}
#endif
#endif