I (26) boot: ESP-IDF v5.3.1-dirty 2nd stage bootloader
I (27) boot: compile time Apr 20 2026 07:50:16
I (27) boot: Multicore bootloader
I (30) boot: chip revision: v0.2
I (34) boot.esp32s3: Boot SPI Speed : 80MHz
I (39) boot.esp32s3: SPI Mode       : DIO
I (44) boot.esp32s3: SPI Flash Size : 2MB
I (48) boot: Enabling RNG early entropy source...
I (54) boot: Partition Table:
I (57) boot: ## Label            Usage          Type ST Offset   Length
I (65) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (72) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (80) boot:  2 factory          factory app      00 00 00010000 00100000
I (87) boot: End of partition table
I (91) esp_image: segment 0: paddr=00010020 vaddr=3c030020 size=0cd54h ( 52564) map
I (109) esp_image: segment 1: paddr=0001cd7c vaddr=3fc94000 size=02c7ch ( 11388) load
I (112) esp_image: segment 2: paddr=0001fa00 vaddr=40374000 size=00618h (  1560) load
I (117) esp_image: segment 3: paddr=00020020 vaddr=42000020 size=203b8h (132024) map
I (149) esp_image: segment 4: paddr=000403e0 vaddr=40374618 size=0f960h ( 63840) load
I (170) boot: Loaded app from partition at offset 0x10000
I (170) boot: Disabling RNG early entropy source...
I (182) cpu_start: Multicore app
I (191) cpu_start: Pro cpu start user code
I (191) cpu_start: cpu freq: 160000000 Hz
I (191) app_init: Application information:
I (194) app_init: Project name:     ESP32-Quadcopter
I (199) app_init: App version:      e80c73b-dirty
I (205) app_init: Compile time:     Apr 20 2026 07:51:27
I (211) app_init: ELF file SHA256:  6d393ce8a...
I (216) app_init: ESP-IDF:          v5.3.1-dirty
I (221) efuse_init: Min chip rev:     v0.0
I (226) efuse_init: Max chip rev:     v0.99 
I (231) efuse_init: Chip rev:         v0.2
I (236) heap_init: Initializing. RAM available for dynamic allocation:
I (243) heap_init: At 3FC97668 len 000520A8 (328 KiB): RAM
I (249) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (255) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (262) heap_init: At 600FE100 len 00001EE8 (7 KiB): RTCRAM
I (269) spi_flash: detected chip: generic
I (273) spi_flash: flash io: dio
W (276) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
W (290) i2c: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`
I (300) sleep: Configure to isolate all GPIO pins in sleep state
I (307) sleep: Enable automatic switching of GPIO sleep configuration
I (314) coexist: coex firmware version: 4482466
I (320) coexist: coexist rom version e7ae62f
I (325) main_task: Started on CPU0
I (329) main_task: Calling app_main()
I (333) DRN_MAIN: Initializing ESP32-S3-SuperMini target...
I (339) DRN_TIME: Time module initialized.
I (344) DRN_TIMER_PWM: LEDC timer 0 initialized at 16000 Hz
I (350) DRN_TIMER_PWM: Channel 0 initialized on GPIO 1
I (356) DRN_TIMER_PWM: Channel 1 initialized on GPIO 2
I (362) DRN_TIMER_PWM: Channel 2 initialized on GPIO 3
I (368) DRN_TIMER_PWM: Channel 3 initialized on GPIO 4
I (374) DRN_MOTOR: 4 motors PWM initialized at 16000 Hz
I (380) gpio: GPIO[4]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
I (389) gpio: GPIO[5]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
I (398) DRN_BUTTON: Buttons initialized (STM32ï¿½ï¿½ï¿½compatible FSM)
I (405) DRN_MPU6050: MPU sensor found (WHO_AM_I = 0x70)
I (421) DRN_MPU6050: MPU6050 initialized successfully
I (421) DRN_MPU6050: Starting calibration (keep device still and level)...
I (1424) DRN_MPU6050: Calibration complete
I (1424) DRN_MPU6050: Accel offsets: X=-18 Y=-506 Z=484
I (1424) DRN_MPU6050: Gyro offsets: X=91 Y=54 Z=-71
I (16379) DRN_BUTTON: GPIO 5 SINGLE_CLICK
I (16379) APP_MAIN: STATE = 0x01: Cong da MO (GPIO 5), cho 1.5s...
I (18859) DRN_BUTTON: GPIO 4 SINGLE_CLICK
I (18859) APP_MAIN: RESUME: Dang cao du lieu... (GPIO 4)
Roll:   4.62 | Pitch:   5.68 | Yaw: -10.39
Roll:   4.88 | Pitch:   5.35 | Yaw: -10.21
Roll:   4.71 | Pitch:   5.20 | Yaw: -10.19
Roll:   4.98 | Pitch:   5.61 | Yaw: -10.28
Roll:   4.82 | Pitch:   5.57 | Yaw: -10.31
Roll:   4.93 | Pitch:   5.61 | Yaw: -10.35
Roll:   4.98 | Pitch:   5.65 | Yaw: -10.31
Roll:   5.01 | Pitch:   5.74 | Yaw: -10.32
Roll:   5.11 | Pitch:   5.78 | Yaw: -10.32
Roll:   5.09 | Pitch:   5.88 | Yaw: -10.31
Roll:   5.01 | Pitch:   5.84 | Yaw: -10.29
Roll:   4.98 | Pitch:   5.60 | Yaw: -10.30
Roll:   5.07 | Pitch:   5.69 | Yaw: -10.30
Roll:   5.14 | Pitch:   5.66 | Yaw: -10.26
Roll:   5.25 | Pitch:   5.70 | Yaw: -10.17
Roll:   5.44 | Pitch:   5.70 | Yaw: -10.13
Roll:   5.30 | Pitch:   5.67 | Yaw: -10.10
Roll:   5.32 | Pitch:   5.67 | Yaw: -10.13
Roll:   5.23 | Pitch:   5.61 | Yaw: -10.24
Roll:   4.45 | Pitch:   5.84 | Yaw: -10.16
Roll:   4.08 | Pitch:   5.68 | Yaw:  -9.97
Roll:   5.21 | Pitch:   5.23 | Yaw:  -9.97
Roll:   5.16 | Pitch:   5.41 | Yaw:  -9.94
Roll:   5.09 | Pitch:   5.59 | Yaw:  -9.96
Roll:   4.94 | Pitch:   5.47 | Yaw: -10.02
Roll:   4.38 | Pitch:   5.15 | Yaw: -10.09
Roll:   3.20 | Pitch:   5.22 | Yaw: -10.13
Roll:  -3.15 | Pitch:   4.20 | Yaw:  -9.57
Roll: -11.96 | Pitch:   4.14 | Yaw:  -9.39
Roll: -17.59 | Pitch:   4.19 | Yaw: -10.72
Roll: -23.08 | Pitch:   4.58 | Yaw: -11.34
Roll: -28.27 | Pitch:   5.16 | Yaw: -10.23
Roll: -34.87 | Pitch:   6.67 | Yaw:  -9.27
Roll: -41.44 | Pitch:   7.40 | Yaw:  -8.61
Roll: -47.60 | Pitch:   7.80 | Yaw:  -7.25
Roll: -52.44 | Pitch:   8.69 | Yaw:  -6.92
Roll: -57.43 | Pitch:   9.16 | Yaw:  -6.34
Roll: -61.08 | Pitch:   9.42 | Yaw:  -5.46
Roll: -63.82 | Pitch:  10.25 | Yaw:  -5.82
Roll: -66.31 | Pitch:  10.38 | Yaw:  -6.08
Roll: -68.48 | Pitch:  10.24 | Yaw:  -6.60
Roll: -69.59 | Pitch:   9.79 | Yaw:  -6.94
Roll: -70.53 | Pitch:   9.60 | Yaw:  -7.25
Roll: -71.38 | Pitch:   9.38 | Yaw:  -7.61
Roll: -71.85 | Pitch:   9.08 | Yaw:  -7.88
Roll: -72.52 | Pitch:   9.07 | Yaw:  -8.13
Roll: -72.56 | Pitch:   8.79 | Yaw:  -7.80
Roll: -73.09 | Pitch:   8.44 | Yaw:  -7.71
Roll: -73.89 | Pitch:   8.51 | Yaw:  -7.93
Roll: -74.31 | Pitch:   8.55 | Yaw:  -8.13
Roll: -73.90 | Pitch:   8.44 | Yaw:  -8.54
Roll: -71.92 | Pitch:   7.70 | Yaw:  -9.29
Roll: -66.73 | Pitch:   5.99 | Yaw: -10.83
Roll: -55.74 | Pitch:   3.88 | Yaw: -12.27
Roll: -40.57 | Pitch:   3.44 | Yaw: -13.61
Roll: -25.68 | Pitch:   3.34 | Yaw: -14.99
Roll: -15.14 | Pitch:   3.24 | Yaw: -16.76
Roll:  -6.41 | Pitch:   2.29 | Yaw: -17.41
Roll:  -1.95 | Pitch:   2.69 | Yaw: -17.05
Roll:   4.59 | Pitch:   2.63 | Yaw: -17.01
Roll:  11.14 | Pitch:   2.74 | Yaw: -19.08
Roll:  13.04 | Pitch:   2.16 | Yaw: -19.53
Roll:  13.81 | Pitch:   2.92 | Yaw: -18.74
Roll:  15.19 | Pitch:   3.04 | Yaw: -17.71
Roll:  15.04 | Pitch:   3.45 | Yaw: -16.89
Roll:   9.70 | Pitch:   2.19 | Yaw: -17.03
Roll:   9.65 | Pitch:   1.73 | Yaw: -16.65
Roll:  13.20 | Pitch:   1.45 | Yaw: -16.15
Roll:  15.48 | Pitch:   2.29 | Yaw: -14.87
Roll:  18.01 | Pitch:   2.93 | Yaw: -13.94
Roll:  21.65 | Pitch:   3.63 | Yaw: -11.72
Roll:  27.53 | Pitch:   2.71 | Yaw:  -9.93
Roll:  32.94 | Pitch:   0.69 | Yaw:  -8.32
Roll:  38.47 | Pitch:  -0.71 | Yaw:  -8.48
Roll:  44.72 | Pitch:  -1.77 | Yaw: -10.68
Roll:  53.72 | Pitch:  -2.53 | Yaw: -18.68
Roll:  62.59 | Pitch:  -1.42 | Yaw: -25.31
Roll:  66.20 | Pitch:   0.74 | Yaw: -24.74
Roll:  67.47 | Pitch:   1.63 | Yaw: -20.69
Roll:  69.58 | Pitch:   1.53 | Yaw: -18.30
Roll:  71.25 | Pitch:   1.14 | Yaw: -16.13
Roll:  74.63 | Pitch:   0.29 | Yaw: -17.69
Roll:  78.32 | Pitch:  -0.09 | Yaw: -18.47
Roll:  83.21 | Pitch:   0.60 | Yaw: -20.96
Roll:  88.48 | Pitch:   1.56 | Yaw: -22.46
Roll:  92.01 | Pitch:   2.77 | Yaw: -22.07
Roll:  93.75 | Pitch:   3.09 | Yaw: -22.05
Roll:  94.90 | Pitch:   3.06 | Yaw: -21.24
Roll:  95.24 | Pitch:   3.01 | Yaw: -21.37
Roll:  95.50 | Pitch:   2.96 | Yaw: -21.60
Roll:  95.28 | Pitch:   3.58 | Yaw: -22.27
Roll:  94.92 | Pitch:   3.68 | Yaw: -22.35
Roll:  94.27 | Pitch:   3.44 | Yaw: -21.93
Roll:  94.60 | Pitch:   3.44 | Yaw: -20.84
Roll:  94.49 | Pitch:   3.60 | Yaw: -19.26
Roll:  94.19 | Pitch:   2.94 | Yaw: -18.75
Roll:  93.50 | Pitch:   2.32 | Yaw: -18.99
Roll:  91.93 | Pitch:   2.21 | Yaw: -19.34
Roll:  87.93 | Pitch:   2.38 | Yaw: -19.81
Roll:  76.60 | Pitch:   4.27 | Yaw: -19.64
Roll:  65.00 | Pitch:   4.03 | Yaw: -20.58
Roll:  48.06 | Pitch:   5.00 | Yaw: -22.81
Roll:  35.37 | Pitch:   5.36 | Yaw: -22.70
Roll:  17.61 | Pitch:   7.24 | Yaw: -19.35
Roll:   6.04 | Pitch:   6.29 | Yaw: -15.95
Roll:   7.45 | Pitch:   5.62 | Yaw: -14.35
Roll:   6.13 | Pitch:   5.98 | Yaw: -13.14
Roll:   5.75 | Pitch:   4.31 | Yaw: -18.09
Roll:   5.27 | Pitch:   4.11 | Yaw: -19.35
Roll:   3.73 | Pitch:   3.12 | Yaw: -19.83
Roll:   2.18 | Pitch:   3.20 | Yaw: -19.44
Roll:   1.16 | Pitch:   3.52 | Yaw: -19.30
Roll:   0.56 | Pitch:   3.13 | Yaw: -19.89
Roll:  -0.13 | Pitch:   2.69 | Yaw: -21.58
Roll:  -0.21 | Pitch:   1.99 | Yaw: -22.74
Roll:  -0.13 | Pitch:   1.44 | Yaw: -23.71
Roll:   0.33 | Pitch:   1.15 | Yaw: -24.02
Roll:   0.14 | Pitch:  -0.06 | Yaw: -24.50
Roll:  -0.74 | Pitch:  -1.59 | Yaw: -26.36
Roll:  -1.25 | Pitch:  -5.00 | Yaw: -28.74
Roll:  -2.05 | Pitch:  -9.71 | Yaw: -29.24
Roll:  -0.99 | Pitch: -13.10 | Yaw: -29.46
Roll:  -1.57 | Pitch: -18.08 | Yaw: -30.77
Roll:  -3.38 | Pitch: -23.49 | Yaw: -30.23
Roll:  -5.75 | Pitch: -27.12 | Yaw: -29.40
Roll:  -7.31 | Pitch: -32.88 | Yaw: -30.33
Roll:  -7.84 | Pitch: -38.32 | Yaw: -33.94
Roll:  -6.78 | Pitch: -41.86 | Yaw: -36.30
Roll:  -7.60 | Pitch: -46.36 | Yaw: -38.63
Roll: -10.14 | Pitch: -54.27 | Yaw: -40.13
Roll:  -8.87 | Pitch: -58.53 | Yaw: -40.67
Roll:  -8.58 | Pitch: -61.25 | Yaw: -46.14
Roll:  -4.13 | Pitch: -65.39 | Yaw: -58.61
Roll:   2.07 | Pitch: -64.19 | Yaw: -60.32
Roll:  11.45 | Pitch: -66.13 | Yaw: -61.73
Roll:  14.20 | Pitch: -65.25 | Yaw: -60.81
Roll:  19.76 | Pitch: -69.84 | Yaw: -61.01
Roll:  32.86 | Pitch: -75.99 | Yaw: -62.64
Roll:  44.01 | Pitch: -76.75 | Yaw: -61.11
Roll:  40.59 | Pitch: -76.86 | Yaw: -58.73
Roll:  41.57 | Pitch: -77.03 | Yaw: -57.70
Roll:  43.61 | Pitch: -79.31 | Yaw: -57.00
Roll:  49.12 | Pitch: -80.52 | Yaw: -56.14
Roll:  41.09 | Pitch: -80.63 | Yaw: -55.30
Roll:  52.20 | Pitch: -81.79 | Yaw: -55.49
Roll:  65.09 | Pitch: -82.31 | Yaw: -56.39
Roll:  69.18 | Pitch: -81.96 | Yaw: -56.99
Roll:  72.08 | Pitch: -81.25 | Yaw: -57.34
Roll:  71.76 | Pitch: -78.91 | Yaw: -55.36
Roll:  72.54 | Pitch: -78.36 | Yaw: -51.72
Roll:  60.68 | Pitch: -79.78 | Yaw: -50.60
Roll:  51.50 | Pitch: -79.39 | Yaw: -49.43
Roll:  40.58 | Pitch: -79.21 | Yaw: -48.98
Roll:  30.41 | Pitch: -76.81 | Yaw: -48.08
Roll:  19.76 | Pitch: -74.65 | Yaw: -46.75
Roll:   8.62 | Pitch: -66.51 | Yaw: -45.05
Roll:   3.33 | Pitch: -60.18 | Yaw: -45.29
Roll:  -1.09 | Pitch: -52.13 | Yaw: -49.11
Roll:   0.02 | Pitch: -40.44 | Yaw: -51.63
Roll:   0.24 | Pitch: -28.32 | Yaw: -51.07
Roll:   4.65 | Pitch: -15.65 | Yaw: -49.86
Roll:   6.20 | Pitch:  -5.67 | Yaw: -48.72
Roll:  10.89 | Pitch:   4.27 | Yaw: -45.58
Roll:  13.15 | Pitch:  10.52 | Yaw: -44.34
Roll:   8.60 | Pitch:   9.74 | Yaw: -42.87
Roll:   7.15 | Pitch:  10.27 | Yaw: -42.09
Roll:   6.50 | Pitch:  14.20 | Yaw: -41.75
Roll:   5.04 | Pitch:  21.33 | Yaw: -39.85
Roll:   6.32 | Pitch:  32.34 | Yaw: -41.38
Roll:   4.48 | Pitch:  44.27 | Yaw: -46.30
Roll:  -0.23 | Pitch:  51.58 | Yaw: -47.67
Roll:   2.03 | Pitch:  63.94 | Yaw: -48.50
Roll:   2.27 | Pitch:  72.75 | Yaw: -45.82
Roll:   4.44 | Pitch:  78.69 | Yaw: -45.17
Roll:  17.10 | Pitch:  81.61 | Yaw: -44.93
Roll:  14.65 | Pitch:  83.39 | Yaw: -45.78
Roll:  20.09 | Pitch:  83.92 | Yaw: -46.04
Roll:  22.35 | Pitch:  84.05 | Yaw: -46.44
Roll:  17.64 | Pitch:  84.18 | Yaw: -47.80
Roll:   2.59 | Pitch:  84.34 | Yaw: -48.42
Roll:   0.30 | Pitch:  85.11 | Yaw: -48.08
Roll:  -3.26 | Pitch:  84.59 | Yaw: -47.81
Roll:  -0.43 | Pitch:  86.51 | Yaw: -48.46
Roll: -23.48 | Pitch:  88.88 | Yaw: -50.15
Roll: -53.58 | Pitch:  87.58 | Yaw: -50.39
Roll: -69.44 | Pitch:  87.55 | Yaw: -50.19
Roll: -74.70 | Pitch:  87.20 | Yaw: -50.64
Roll: -86.79 | Pitch:  86.83 | Yaw: -50.83
Roll: -94.48 | Pitch:  85.78 | Yaw: -51.28
Roll: -92.96 | Pitch:  85.27 | Yaw: -51.67
Roll: -76.09 | Pitch:  78.93 | Yaw: -53.00
Roll: -55.14 | Pitch:  69.89 | Yaw: -52.43
Roll: -43.52 | Pitch:  59.20 | Yaw: -51.74
Roll: -37.07 | Pitch:  48.70 | Yaw: -52.54
Roll: -32.15 | Pitch:  37.76 | Yaw: -53.53
Roll: -27.30 | Pitch:  25.77 | Yaw: -51.39
Roll: -26.01 | Pitch:  20.72 | Yaw: -47.42
Roll: -22.57 | Pitch:  16.34 | Yaw: -46.66
Roll: -19.16 | Pitch:  13.19 | Yaw: -47.60
Roll: -19.09 | Pitch:   8.60 | Yaw: -49.62
Roll: -14.23 | Pitch:  -2.70 | Yaw: -54.68
Roll: -15.24 | Pitch:  -3.47 | Yaw: -53.57
Roll: -16.19 | Pitch:  -1.13 | Yaw: -53.69
Roll: -15.22 | Pitch:  -1.78 | Yaw: -54.47
Roll: -15.43 | Pitch:  -1.51 | Yaw: -55.41
Roll: -12.32 | Pitch:   0.97 | Yaw: -55.49
Roll:  -9.47 | Pitch:  -3.36 | Yaw: -60.67
Roll:  -8.12 | Pitch:  -4.84 | Yaw: -62.49
Roll:  -7.60 | Pitch: -10.48 | Yaw: -55.58
Roll: -11.65 | Pitch: -21.40 | Yaw: -45.15
Roll: -13.74 | Pitch: -29.14 | Yaw: -44.14
Roll: -14.23 | Pitch: -36.73 | Yaw: -45.09
Roll: -14.39 | Pitch: -47.34 | Yaw: -48.65
Roll: -12.74 | Pitch: -56.55 | Yaw: -49.87
Roll: -15.60 | Pitch: -66.73 | Yaw: -50.02
Roll: -20.03 | Pitch: -77.91 | Yaw: -50.75
Roll: -31.90 | Pitch: -86.42 | Yaw: -50.93
Roll: -58.18 | Pitch: -92.15 | Yaw: -49.70
Roll: -80.83 | Pitch: -89.63 | Yaw: -49.74
Roll: -101.47 | Pitch: -85.60 | Yaw: -49.83
Roll: -116.70 | Pitch: -81.87 | Yaw: -50.07
Roll: -126.46 | Pitch: -78.93 | Yaw: -49.98
Roll: -132.25 | Pitch: -77.14 | Yaw: -49.97
Roll: -136.52 | Pitch: -76.39 | Yaw: -50.05
Roll: -139.37 | Pitch: -75.46 | Yaw: -50.35
Roll: -140.51 | Pitch: -75.00 | Yaw: -50.64
Roll: -141.84 | Pitch: -74.25 | Yaw: -51.54
Roll: -141.24 | Pitch: -75.71 | Yaw: -51.41
Roll: -141.67 | Pitch: -75.80 | Yaw: -51.85
Roll: -141.83 | Pitch: -76.52 | Yaw: -52.23
Roll: -141.72 | Pitch: -77.76 | Yaw: -52.23
Roll: -141.77 | Pitch: -77.95 | Yaw: -52.41
Roll: -137.92 | Pitch: -73.50 | Yaw: -52.40
Roll: -113.43 | Pitch: -62.92 | Yaw: -50.78
Roll: -84.57 | Pitch: -52.07 | Yaw: -49.87
Roll: -62.62 | Pitch: -43.23 | Yaw: -49.39
Roll: -46.61 | Pitch: -39.69 | Yaw: -48.45
Roll: -37.41 | Pitch: -34.43 | Yaw: -48.29
Roll: -31.20 | Pitch: -30.71 | Yaw: -46.67
Roll: -27.00 | Pitch: -29.33 | Yaw: -44.94
Roll: -22.19 | Pitch: -24.70 | Yaw: -46.65
Roll: -18.40 | Pitch: -19.57 | Yaw: -48.29
Roll: -13.65 | Pitch: -15.99 | Yaw: -47.84
Roll: -12.81 | Pitch: -13.88 | Yaw: -43.42
Roll: -11.56 | Pitch:  -6.86 | Yaw: -34.37
Roll:  -9.14 | Pitch:  -2.37 | Yaw: -35.66
Roll:  -6.21 | Pitch:  -5.86 | Yaw: -48.67
Roll:  -5.55 | Pitch:  -6.68 | Yaw: -50.87
Roll:  -4.26 | Pitch:  -6.54 | Yaw: -51.68
Roll:  -0.14 | Pitch:  -5.58 | Yaw: -51.46
Roll:   2.52 | Pitch:  -5.45 | Yaw: -47.61
Roll:   3.80 | Pitch:  -5.13 | Yaw: -47.86
Roll:   6.22 | Pitch:  -3.47 | Yaw: -49.82
Roll:   5.69 | Pitch:  -3.47 | Yaw: -52.28
Roll:   5.00 | Pitch:  -4.27 | Yaw: -52.33
Roll:   4.04 | Pitch:  -4.93 | Yaw: -48.88
Roll:   1.58 | Pitch:  -4.48 | Yaw: -46.21
Roll:   0.54 | Pitch:  -3.95 | Yaw: -44.41
Roll:  -1.65 | Pitch:  -5.24 | Yaw: -41.79
Roll:  -3.48 | Pitch:  -5.38 | Yaw: -41.09
Roll:  -4.50 | Pitch:  -5.40 | Yaw: -40.66
Roll:  -5.98 | Pitch:  -5.85 | Yaw: -40.14
Roll:  -8.98 | Pitch:  -6.75 | Yaw: -39.58
Roll: -14.57 | Pitch:  -5.91 | Yaw: -39.19
Roll: -26.45 | Pitch:  -3.63 | Yaw: -39.40
Roll: -35.91 | Pitch:  -1.75 | Yaw: -49.53
Roll: -45.40 | Pitch:  -7.00 | Yaw: -45.96
Roll: -54.54 | Pitch:  -8.32 | Yaw: -43.37
Roll: -60.11 | Pitch: -12.12 | Yaw: -40.36
Roll: -67.38 | Pitch: -12.27 | Yaw: -31.68
Roll: -71.65 | Pitch:  -9.20 | Yaw: -33.34
Roll: -75.98 | Pitch:  -7.92 | Yaw: -32.74
Roll: -78.44 | Pitch:  -6.80 | Yaw: -36.11
Roll: -81.59 | Pitch:  -5.58 | Yaw: -35.20
Roll: -83.98 | Pitch:  -5.71 | Yaw: -30.60
Roll: -86.14 | Pitch:  -3.52 | Yaw: -28.82
Roll: -88.74 | Pitch:  -0.55 | Yaw: -24.73
Roll: -89.70 | Pitch:   0.40 | Yaw: -30.93
Roll: -91.36 | Pitch:   0.40 | Yaw: -33.95
Roll: -92.49 | Pitch:  -0.44 | Yaw: -34.67
Roll: -93.21 | Pitch:  -1.24 | Yaw: -34.12
Roll: -93.75 | Pitch:  -1.25 | Yaw: -33.55
Roll: -94.61 | Pitch:  -1.77 | Yaw: -33.08
Roll: -95.18 | Pitch:  -1.83 | Yaw: -32.85
Roll: -95.83 | Pitch:  -1.52 | Yaw: -32.71
Roll: -96.23 | Pitch:  -1.53 | Yaw: -32.66
Roll: -96.47 | Pitch:  -1.63 | Yaw: -32.75
Roll: -96.28 | Pitch:  -1.55 | Yaw: -32.36
Roll: -94.93 | Pitch:  -1.26 | Yaw: -32.58
Roll: -90.71 | Pitch:  -0.87 | Yaw: -32.39
Roll: -82.31 | Pitch:   0.56 | Yaw: -35.40
Roll: -73.22 | Pitch:   0.24 | Yaw: -37.80
Roll: -63.77 | Pitch:   0.54 | Yaw: -37.89
Roll: -60.91 | Pitch:  -0.04 | Yaw: -36.18
Roll: -56.95 | Pitch:  -2.47 | Yaw: -33.44
Roll: -52.10 | Pitch:  -5.23 | Yaw: -37.80
Roll: -41.88 | Pitch:  -6.90 | Yaw: -47.28
Roll: -33.17 | Pitch:  -5.86 | Yaw: -51.31
Roll: -27.80 | Pitch:  -2.54 | Yaw: -56.61
Roll: -26.51 | Pitch:  -4.00 | Yaw: -56.56
Roll: -24.36 | Pitch:  -9.80 | Yaw: -53.35
Roll: -22.72 | Pitch:  -5.39 | Yaw: -51.35
Roll: -24.52 | Pitch:  -2.92 | Yaw: -54.58
Roll: -20.59 | Pitch:  -6.86 | Yaw: -50.42
Roll: -22.16 | Pitch:  -6.95 | Yaw: -50.46
Roll: -20.89 | Pitch:  -4.92 | Yaw: -50.94
Roll: -16.00 | Pitch:  -2.89 | Yaw: -48.79
Roll: -12.83 | Pitch:  -0.17 | Yaw: -45.42
Roll:  -9.52 | Pitch:   2.69 | Yaw: -42.44
Roll:  -7.96 | Pitch:   3.52 | Yaw: -40.52
Roll:  -7.67 | Pitch:   3.66 | Yaw: -39.81
Roll:  -6.37 | Pitch:   5.11 | Yaw: -38.82
Roll:  -7.23 | Pitch:   5.30 | Yaw: -38.82
Roll:  -7.44 | Pitch:   7.11 | Yaw: -37.31
Roll:  -1.26 | Pitch:   9.46 | Yaw: -35.03
Roll:   3.80 | Pitch:  11.15 | Yaw: -32.24
Roll:   8.01 | Pitch:  12.29 | Yaw: -30.26
Roll:  13.10 | Pitch:  12.99 | Yaw: -31.51
Roll:  25.32 | Pitch:  17.18 | Yaw: -30.88
Roll:  36.14 | Pitch:  19.57 | Yaw: -30.96
Roll:  44.17 | Pitch:  21.64 | Yaw: -31.08
Roll:  50.12 | Pitch:  22.61 | Yaw: -28.55
Roll:  57.55 | Pitch:  20.65 | Yaw: -26.20
Roll:  64.68 | Pitch:  20.22 | Yaw: -27.94
Roll:  69.19 | Pitch:  21.01 | Yaw: -28.19
Roll:  73.32 | Pitch:  22.25 | Yaw: -28.59
Roll:  77.31 | Pitch:  22.87 | Yaw: -28.00
Roll:  80.56 | Pitch:  22.51 | Yaw: -27.41
Roll:  83.68 | Pitch:  19.78 | Yaw: -26.59
Roll:  84.20 | Pitch:  18.47 | Yaw: -27.76
Roll:  83.28 | Pitch:  17.88 | Yaw: -26.14
Roll:  83.54 | Pitch:  18.43 | Yaw: -24.07
Roll:  84.43 | Pitch:  18.48 | Yaw: -23.71
Roll:  84.93 | Pitch:  17.77 | Yaw: -22.78
Roll:  85.10 | Pitch:  16.32 | Yaw: -21.95
Roll:  85.20 | Pitch:  14.66 | Yaw: -21.34
Roll:  85.43 | Pitch:  13.17 | Yaw: -20.48
Roll:  85.64 | Pitch:  12.57 | Yaw: -19.84
Roll:  85.46 | Pitch:  12.19 | Yaw: -19.69
Roll:  86.20 | Pitch:  12.08 | Yaw: -18.89
Roll:  86.32 | Pitch:  11.82 | Yaw: -18.33
Roll:  87.26 | Pitch:  10.44 | Yaw: -17.56
Roll:  87.47 | Pitch:  10.32 | Yaw: -17.06
Roll:  85.96 | Pitch:  10.69 | Yaw: -17.57
Roll:  83.72 | Pitch:  10.62 | Yaw: -17.63
Roll:  82.89 | Pitch:  10.03 | Yaw: -16.87
Roll:  80.37 | Pitch:  10.65 | Yaw: -16.68
Roll:  73.60 | Pitch:   9.40 | Yaw: -16.91
Roll:  54.32 | Pitch:   9.70 | Yaw: -20.08
Roll:  41.88 | Pitch:  10.51 | Yaw: -21.80
Roll:  23.46 | Pitch:   6.84 | Yaw: -28.50
Roll:  16.77 | Pitch:   6.97 | Yaw: -26.01
Roll:   8.04 | Pitch:   6.50 | Yaw: -23.22
Roll:  -0.78 | Pitch:   3.35 | Yaw: -20.22
Roll:  -7.34 | Pitch:   3.87 | Yaw: -20.74
Roll: -13.09 | Pitch:   3.30 | Yaw: -20.09
Roll: -16.95 | Pitch:   2.59 | Yaw: -20.91
Roll: -16.59 | Pitch:   2.59 | Yaw: -19.16
Roll: -14.57 | Pitch:   2.56 | Yaw: -15.49
Roll: -16.28 | Pitch:   2.75 | Yaw: -20.09
Roll: -17.58 | Pitch:   2.71 | Yaw: -19.82
Roll: -18.02 | Pitch:   4.39 | Yaw: -20.71
Roll: -17.95 | Pitch:   5.89 | Yaw: -17.85
Roll: -19.67 | Pitch:   6.73 | Yaw: -16.46
Roll: -21.90 | Pitch:   6.66 | Yaw: -14.65
Roll: -21.54 | Pitch:   3.26 | Yaw: -14.14
Roll: -23.87 | Pitch:   2.04 | Yaw: -16.38
Roll: -23.58 | Pitch:   3.72 | Yaw: -16.85
Roll: -22.99 | Pitch:   4.18 | Yaw: -18.97
Roll: -21.20 | Pitch:   4.43 | Yaw: -18.66
Roll: -18.64 | Pitch:   5.43 | Yaw: -19.18
Roll: -14.97 | Pitch:   6.23 | Yaw: -19.14
Roll: -12.77 | Pitch:   5.53 | Yaw: -19.45
Roll: -11.75 | Pitch:   3.82 | Yaw: -19.04
Roll: -10.78 | Pitch:   4.11 | Yaw: -18.93
Roll: -11.26 | Pitch:   3.78 | Yaw: -19.68
Roll: -18.21 | Pitch:   7.22 | Yaw: -14.90
Roll: -22.37 | Pitch:   1.06 | Yaw: -18.71
Roll: -13.29 | Pitch:  -9.73 | Yaw: -21.84
Roll:  -8.10 | Pitch: -12.31 | Yaw: -22.32
Roll:  -7.24 | Pitch: -12.01 | Yaw: -21.15
Roll:  -5.49 | Pitch:  -8.86 | Yaw: -22.77
Roll:  -4.05 | Pitch:  -7.74 | Yaw: -23.10
Roll:  -4.25 | Pitch:  -6.28 | Yaw: -24.07
Roll:  -5.26 | Pitch:  -6.05 | Yaw: -24.85
Roll:  -2.95 | Pitch:  -3.97 | Yaw: -26.01
Roll:  -2.71 | Pitch:  -3.33 | Yaw: -25.82
Roll:  -3.35 | Pitch:  -3.09 | Yaw: -25.46
Roll:  -2.04 | Pitch:  -3.57 | Yaw: -25.74
Roll:  -2.09 | Pitch:  -4.03 | Yaw: -25.91
Roll:  -3.07 | Pitch:  -3.34 | Yaw: -25.44
Roll:  -2.03 | Pitch:  -2.85 | Yaw: -25.54
Roll:  -1.49 | Pitch:  -2.60 | Yaw: -25.79
I (58159) DRN_BUTTON: GPIO 4 SINGLE_CLICK
I (58159) APP_MAIN: CAPTURE: Da dung terminal, luu ket qua. (GPIO 4)
I (59899) DRN_BUTTON: GPIO 5 SINGLE_CLICK
I (59899) APP_MAIN: STATE = 0x04: Cong DONG (GPIO 5). Da reset data MPU.
I (59900) APP_MAIN: STATE = 0x00: Tro ve trang thai ranh.