/**
 * @file    drn_motor_pwm.cpp
 * @brief   Motor PWM control HAL — Project Windify
 * @target  ESP32-S3-SuperMini (ESP-IDF v5.x)
 * @version 1.4
 */
#include "drn_motor_pwm.h"
#include "drn_main_board_choose.h"
#include "drn_button.h"
#include "drn_timer_pwm.h"

#ifdef ESP32_S3_SUPERMINI
    #include "driver/ledc.h"   // for LEDC_TIMER_0, LEDC_TIMER_10_BIT
    #include "esp_log.h"
#endif

static const char* TAG = "DRN_MOTOR";

#define MOTOR_COUNT            4
#define MOTOR_PWM_FREQ_HZ      16000
#define MOTOR_PWM_DUTY_RES     LEDC_TIMER_10_BIT
#define MOTOR_PWM_TIMER        LEDC_TIMER_0

static const uint8_t motor_gpios[MOTOR_COUNT] = {1, 2, 3, 4};
static const uint8_t motor_channels[MOTOR_COUNT] = {0, 1, 2, 3};

static uint32_t xx_duties[MOTOR_COUNT] = {0};
static uint8_t  xx_gate_state = GATE_CLOSE;
static bool     xx_gate_lockout_active = false;
static uint32_t xx_gate_last_action_time = 0;
static sensor_state_t xx_sensor_state = SENSOR_IDLE;

esp_err_t DRN_Motor_PWM_Init(void) {
#ifdef ESP32_S3_SUPERMINI
    esp_err_t ret = DRN_Timer_PWM_InitTimer(MOTOR_PWM_TIMER, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_RES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timer PWM init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        drn_pwm_channel_config_t cfg = {
            .gpio_num = motor_gpios[i],
            .channel = motor_channels[i],
            .freq_hz = MOTOR_PWM_FREQ_HZ,
            .duty_resolution = MOTOR_PWM_DUTY_RES
        };
        ret = DRN_Timer_PWM_InitChannel(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d channel init failed: %s", i, esp_err_to_name(ret));
            return ret;
        }
        xx_duties[i] = 0;
    }

    xx_gate_state = GATE_CLOSE;
    xx_gate_lockout_active = false;
    xx_sensor_state = SENSOR_IDLE;

    ESP_LOGI(TAG, "4 motors PWM initialized at %d Hz", MOTOR_PWM_FREQ_HZ);
    return ESP_OK;
#else
    return -1;
#endif
}

void DRN_Motor_Set_Duty(uint8_t motor_id, uint32_t duty) {
    if (motor_id >= MOTOR_COUNT) return;
    if (duty > 1023) duty = 1023;
    xx_duties[motor_id] = duty;
    if (xx_gate_state == GATE_OPEN) {
        DRN_Timer_PWM_SetDuty(motor_channels[motor_id], duty);
    }
}

void DRN_Motor_Set_Duty_Percent(uint8_t motor_id, uint8_t percent) {
    if (motor_id >= MOTOR_COUNT || percent > 100) return;
    uint32_t duty = (uint32_t)((percent * 1023) / 100);
    DRN_Motor_Set_Duty(motor_id, duty);
}

uint8_t DRN_Motor_Get_Gate_State(void) {
    return xx_gate_state;
}

sensor_state_t DRN_Motor_Get_Sensor_State(void) {
    return xx_sensor_state;
}

void DRN_Motor_Update_Logic(uint8_t btn5_event, uint8_t btn4_event) {
    if (btn5_event == BTN_EVENT_SINGLE_CLICK) {
        if (!xx_gate_lockout_active) {
            if (xx_gate_state == GATE_CLOSE) {
                xx_gate_state = GATE_OPEN;
                ESP_LOGI(TAG, "Gate OPEN (0x01)");
                xx_gate_lockout_active = true;
                xx_gate_last_action_time = DRN_Millis();
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    DRN_Timer_PWM_SetDuty(motor_channels[i], xx_duties[i]);
                }
            } else {
                xx_gate_state = GATE_CLOSE;
                ESP_LOGI(TAG, "Gate CLOSE (0x04) - data reset");
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    DRN_Timer_PWM_SetDuty(motor_channels[i], 0);
                    xx_duties[i] = 0;
                }
            }
        } else {
            ESP_LOGW(TAG, "Gate button ignored (lockout active)");
        }
    }

    if (btn4_event == BTN_EVENT_SINGLE_CLICK) {
        if (xx_sensor_state == SENSOR_IDLE) {
            xx_sensor_state = SENSOR_ACQUIRING;
            ESP_LOGI(TAG, "Sensor acquisition STARTED");
        } else if (xx_sensor_state == SENSOR_ACQUIRING) {
            xx_sensor_state = SENSOR_CAPTURED;
            ESP_LOGI(TAG, "Sensor data CAPTURED (snapshot)");
        } else {
            xx_sensor_state = SENSOR_ACQUIRING;
            ESP_LOGI(TAG, "Sensor acquisition RESUMED");
        }
    }
}

void DRN_Motor_Run_Task(uint32_t current_time) {
    if (xx_gate_lockout_active) {
        if ((current_time - xx_gate_last_action_time) >= 1500) {
            xx_gate_lockout_active = false;
            ESP_LOGI(TAG, "Gate lockout period ended");
        }
    }

    static uint32_t last_sensor_read = 0;
    if (xx_sensor_state == SENSOR_ACQUIRING) {
        if ((current_time - last_sensor_read) >= 10) {
            last_sensor_read = current_time;
            // TODO: sensor reading
        }
    }
}