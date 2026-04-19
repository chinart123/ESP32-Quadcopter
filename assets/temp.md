Software Architect's Operational Notes
1. Temporal Isolation (Core Pinning)

Core 0 (PRO_CPU): Handles the LwIP stack and Bluetooth LE. This core is subject to Wi-Fi Beacon Jitter (TCP/IP stack delays of 10-50ms). 
By pinning it to Core 0, we guarantee that vTaskDelay or select() calls in the networking stack cannot preempt the flight controller logic.

Core 1 (APP_CPU): Dedicated Flight Core Task. This task runs at a priority higher than configMAX_PRIORITIES - 2. It spends 99% of its time blocked on the Binary Semaphore.

2. The 500Hz Synchronized Loop (Orange Path)
The architecture shifts from a Super Loop to an Event-Driven model:

ISR Constraint: drn_mpu_isr_handler (marked orange) is extremely short. It only checks the interrupt status register, clears the pin, and calls xSemaphoreGiveFromISR.

Critical Timing: The time delta (esp_timer_get_time) is captured immediately upon waking from the semaphore. 
This ensures the dt used in the PID loop matches the exact 2ms interval of the MPU6500 Data Ready Interrupt (500Hz), eliminating temporal drift caused by I2C read variance.

3. I2C Error Handling (Non-Blocking)
Unlike the Bare-metal blocking architecture where I2C_Bus_Recovery freezes the motor output, the HW_I2C node here relies on the ESP32-S3's Hardware Timeout mechanism. If the MPU6500 hangs the SDA line low:

The I2C driver returns ESP_ERR_TIMEOUT.

The FlightTask logs the error and immediately loops back to xSemaphoreTake, without calling the manual bit-bang recovery inline.

This ensures the LEDC PWM (16kHz) continues to update on schedule, maintaining motor stability even during sensor glitches.