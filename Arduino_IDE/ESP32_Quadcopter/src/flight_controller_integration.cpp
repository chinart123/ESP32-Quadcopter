// =============================================================================
// flight_controller_integration.cpp
// Glue code: HungVo_IMU (Arduino Wire) → Madgwick Filter → PID
//
// WINDIFY PROJECT — Đây là file GLUE CODE duy nhất cần sửa khi migrate.
// madgwick_filter.h/.c: KHÔNG đụng vào (pure math, hardware-agnostic).
// HungVo_IMU.h/.cpp:    Chỉ thêm 3 hàm getAccelX/Y/Z (Task 0).
//
// Cách dùng:
//   - Copy toàn bộ nội dung section SETUP vào setup() của droneflightcode.ino
//   - Copy toàn bộ nội dung section LOOP vào vị trí thay thế Kalman cũ
//   - Hoặc dùng nguyên như FreeRTOS task (xem phần cuối file)
//
// ─────────────────────────────────────────────────────────────────────────────
// TRẠNG THÁI CÁC OPEN ISSUES (cập nhật khi giải quyết xong)
// ─────────────────────────────────────────────────────────────────────────────
//
//  [ ] TASK 0: Thêm getAccelX/Y/Z vào HungVo_IMU.h/.cpp
//              → Chưa làm = file này KHÔNG compile được
//
//  [ ] TASK 1: HungVo_IMU.cpp dòng writeReg(0x1A, 0x05) → đổi thành 0x03
//              → Chưa làm = Madgwick hoạt động nhưng chậm hội tụ
//
//  [?] TASK 4: IMU gắn vật lý 45° so với khung không?
//              → Chưa xác nhận = rotation block bên dưới đang bị comment
//              → Xem: ROTATION_45DEG_BLOCK
//
//  [?] SIGN CONVENTION: Dấu gx/gy/gz và ax/ay/az chưa test thực tế
//              → Xem: SIGN_CONVENTION_BLOCK
//
//  [?] YAW DIRECTION: Chờ quyết định từ nhóm (Hướng A hay B)
//              → Xem: YAW_DECISION_BLOCK
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "HungVo_IMU.h"
#include "madgwick_filter.h"

// =============================================================================
// FREQUENCY — khớp với droneflightcode.ino hiện tại
// =============================================================================
// droneflightcode.ino dùng const float dt = 0.004 (250Hz cứng trong loop)
// Nếu sau này chuyển sang FreeRTOS task: đổi DT_SECONDS = 0.002 (500Hz)
#define DT_SECONDS  0.004f      // 250Hz — khớp với dt trong droneflightcode.ino

// =============================================================================
// ĐƠN VỊ — HungVo_IMU output
// =============================================================================
// getGyroX/Y/Z() → dps (degrees per second)
// getAccelX/Y/Z() → g  (đơn vị trọng lực, ±8g range, scale 4096 LSB/g)
//
// Madgwick cần:
//   gyro → rad/s  : nhân với DEG_TO_RAD = π/180 = 0.017453293
//   accel → g     : KHÔNG cần đổi (Madgwick tự normalize nội bộ)
#define DEG_TO_RAD  0.017453293f

// =============================================================================
// OBJECTS
// =============================================================================
extern HungVo_IMU myIMU;    // đã khai báo trong droneflightcode.ino
MadgwickFilter_t  madgwick; // instance bộ lọc — state sống ở đây

// =============================================================================
// OUTPUT — góc đã lọc (degrees), đưa vào PID như cũ
// =============================================================================
float fRoll  = 0.0f;
float fPitch = 0.0f;
float fYaw   = 0.0f;

// =============================================================================
// SECTION: SETUP
// Gọi 1 lần trong setup() của droneflightcode.ino,
// SAU KHI myIMU.begin() và myIMU.calibrate() đã chạy xong.
// =============================================================================
void madgwick_setup(void)
{
    madgwick_init(&madgwick, MADGWICK_BETA_DEFAULT, DT_SECONDS);
    // Filter khởi động ở identity quaternion [1, 0, 0, 0]
    // Tương đương: drone nằm phẳng, mũi nhìn về phía Bắc (chưa có magnetometer)
}

// =============================================================================
// SECTION: LOOP
// Gọi mỗi chu kỳ trong loop() của droneflightcode.ino,
// thay thế hoàn toàn đoạn Kalman cũ.
//
// Code cũ cần XÓA:
//   kalmanP.getAngle(rawP, -myIMU.getGyroX(), dt)
//   kalmanR.getAngle(rawR,  myIMU.getGyroY(), dt)
//   fRoll  = (fR_raw * 0.7071) + (fP_raw * 0.7071)
//   fPitch = (fP_raw * 0.7071) - (fR_raw * 0.7071)
//   fYaw  -= gz_clean * dt
// =============================================================================
void madgwick_loop(void)
{
    // ------------------------------------------------------------------
    // BƯỚC 1: Đọc IMU
    // myIMU.update() đã được gọi đầu loop() — không gọi lại ở đây
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // BƯỚC 2: Lấy gyro và đổi đơn vị dps → rad/s
    //
    // [SIGN_CONVENTION_BLOCK]
    // Dấu dưới đây là giả định ban đầu. Cần test thực tế:
    //   Nghiêng drone sang PHẢI → fRoll phải DƯƠNG
    //   Chúi mũi xuống        → fPitch phải DƯƠNG (theo convention ZYX)
    // Nếu ngược: thêm dấu âm trước myIMU.getGyroX() hoặc myIMU.getAccelX()
    // ------------------------------------------------------------------
    float gx =  myIMU.getGyroX() * DEG_TO_RAD;   // [rad/s]
    float gy =  myIMU.getGyroY() * DEG_TO_RAD;
    float gz =  myIMU.getGyroZ() * DEG_TO_RAD;

    // ------------------------------------------------------------------
    // BƯỚC 3: Lấy accel vector thô (đơn vị g)
    //
    // [TASK 0 — CHƯA LÀM]: getAccelX/Y/Z() chưa tồn tại trong HungVo_IMU.
    // Thêm vào HungVo_IMU.h/.cpp trước khi compile:
    //
    //   float HungVo_IMU::getAccelX() { return ((float)_ax/4096.0f) - _offAX; }
    //   float HungVo_IMU::getAccelY() { return ((float)_ay/4096.0f) - _offAY; }
    //   float HungVo_IMU::getAccelZ() { return ((float)_az/4096.0f) - (_offAZ-1.0f); }
    //
    // Lưu ý scale: HungVo_IMU dùng ±8g → 4096 LSB/g (KHÔNG phải 8192)
    // ------------------------------------------------------------------
    float ax =  myIMU.getAccelX();   // [g]
    float ay =  myIMU.getAccelY();
    float az =  myIMU.getAccelZ();

    // ------------------------------------------------------------------
    // BƯỚC 4: Chạy Madgwick (~15µs trên ESP32-S3 FPU @ 240MHz)
    // ------------------------------------------------------------------
    madgwick_update_imu(&madgwick, gx, gy, gz, ax, ay, az);

    // ------------------------------------------------------------------
    // BƯỚC 5: Lấy Euler angles (radians → degrees)
    // ------------------------------------------------------------------
    EulerAngles_t euler;
    madgwick_get_euler(&madgwick, &euler);
    fRoll  = rad2deg(euler.roll);
    fPitch = rad2deg(euler.pitch);
    fYaw   = rad2deg(euler.yaw);

    // ------------------------------------------------------------------
    // [ROTATION_45DEG_BLOCK]
    // Nếu MPU6050 gắn vật lý lệch 45° so với khung drone:
    // Uncomment đoạn dưới đây, và XÓA đoạn 0.7071 cũ trong droneflightcode.ino
    //
    // Áp dụng SAU get_euler (không phải trước như code cũ):
    //
    // float roll_frame  = fRoll * 0.7071f + fPitch * 0.7071f;
    // float pitch_frame = fPitch * 0.7071f - fRoll * 0.7071f;
    // fRoll  = roll_frame;
    // fPitch = pitch_frame;
    // (fYaw không thay đổi — heading không bị ảnh hưởng bởi mount orientation)
    //
    // Nếu KHÔNG gắn 45°: bỏ comment này và xóa 0.7071 trong code cũ luôn.
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // [YAW_DECISION_BLOCK]
    // Hiện tại fYaw = Madgwick integrated yaw — drift ~0.5–2°/phút.
    // Đây là Hướng A (chấp nhận drift).
    //
    // Nếu nhóm chọn Hướng B (PMW3901 correction):
    //   → Cần PMW3901 Motion Mode >100Hz trước (Milestone 2)
    //   → Sau đó thêm outer PID loop ở đây
    // ------------------------------------------------------------------
}

// =============================================================================
// SECTION: FreeRTOS Task (tùy chọn — nếu muốn chạy riêng trên Core 1)
// Thay cho việc gọi madgwick_loop() trong loop() của Arduino
// =============================================================================
#ifdef USE_FREERTOS_TASK

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Shared attitude struct (đọc từ pid_task)
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} SharedAttitude_t;

volatile SharedAttitude_t g_attitude = {0, 0, 0};
SemaphoreHandle_t g_attitude_mutex   = NULL;

// IMU Fusion Task — Core 1, priority 5, 500Hz
void imu_fusion_task(void *arg)
{
    // Reinit tại 500Hz nếu dùng FreeRTOS task
    madgwick_init(&madgwick, MADGWICK_BETA_DEFAULT, 0.002f);

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        myIMU.update();
        madgwick_loop();   // gọi logic bên trên

        if (xSemaphoreTake(g_attitude_mutex, 0) == pdTRUE) {
            g_attitude.roll_deg  = fRoll;
            g_attitude.pitch_deg = fPitch;
            g_attitude.yaw_deg   = fYaw;
            xSemaphoreGive(g_attitude_mutex);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2)); // 500Hz
    }
}

// Gọi trong setup() để tạo task
void madgwick_task_init(void)
{
    g_attitude_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(imu_fusion_task, "imu_fusion",
                            4096, NULL, 5, NULL, 1);
}

#endif // USE_FREERTOS_TASK
