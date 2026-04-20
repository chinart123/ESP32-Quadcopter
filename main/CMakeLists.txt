# ============================================================
# Component: main
# Description: Main application component for Windify Quadcopter
# Target: ESP32-S3 (ESP-IDF v5.x)
# ============================================================
idf_component_register(
    SRCS
        "main.cpp"
        "../src/drn_main_board_choose.cpp"
        "../src/drn_time.cpp"
        "../src/drn_button.cpp"
        "../src/drn_timer_pwm.cpp"
        "../src/drn_motor_pwm.cpp"
    INCLUDE_DIRS
        "."
        "../include"
    REQUIRES
        driver
        freertos
        esp_timer
)
# Optional: Add compile definitions or flags for this component
# target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DDEBUG_LEVEL=3")