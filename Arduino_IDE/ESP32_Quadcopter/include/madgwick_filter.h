#pragma once
// =============================================================================
// madgwick_filter.h
// Madgwick AHRS Filter — Optimized for ESP32-S3 FPU (bare-metal / FreeRTOS)
// Ref: S. Madgwick, "An efficient orientation filter for IMUs and MARGs", 2010
//
// -----------------------------------------------------------------------------
// WINDIFY PROJECT — INTEGRATION NOTES (đọc trước khi dùng)
// -----------------------------------------------------------------------------
//
// [PREREQUISITE — CHƯA LÀM] Trước khi gọi madgwick_update_imu():
//   HungVo_IMU.h/.cpp phải được bổ sung 3 hàm sau (hiện CHƯA có):
//     float getAccelX();   // trả về g, scale ±8g = 4096 LSB/g
//     float getAccelY();
//     float getAccelZ();
//   Xem chi tiết: Windify_Architecture_Roadmap_v2.md — Task 0
//
// [PREREQUISITE — CHƯA LÀM] HungVo_IMU.cpp dòng writeReg(0x1A, 0x05):
//   Đổi thành writeReg(0x1A, 0x03) để giảm DLPF từ 10Hz → 44Hz.
//   10Hz gây phase delay ~70ms — Madgwick sẽ hội tụ chậm với cấu hình hiện tại.
//   Xem chi tiết: Windify_Architecture_Roadmap_v2.md — Task 1
//
// [CẦU HỎI MỞ] MPU6050 gắn vật lý 45° so với khung drone không?
//   Nếu CÓ: sau madgwick_get_euler() cần thêm rotation matrix 45°.
//   Nếu KHÔNG: xóa bỏ đoạn 0.7071 trong droneflightcode.ino là đúng.
//   Xem chi tiết: Windify_Architecture_Roadmap_v2.md — Task 4, Câu hỏi A
//
// [CẦU HỎI MỞ] Dấu trục (sign convention) chưa được test thực tế:
//   Sau khi gắn vào drone, kiểm tra: nghiêng phải → fRoll dương?
//   Nếu ngược: đổi dấu gx hoặc ax trong flight_controller_integration.
//   Xem chi tiết: Windify_Architecture_Roadmap_v2.md — Task 3
// =============================================================================

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// -----------------------------------------------------------------------------
// Filter configuration — tune these for your hardware
// -----------------------------------------------------------------------------

// beta: gradient descent step size (convergence gain)
//   - Higher  → trusts accelerometer more, faster convergence, noisier output
//   - Lower   → trusts gyroscope more, smoother but slower correction
//   - Typical range for brushed coreless drone: 0.033f – 0.15f
//   - Start with 0.1f, reduce if hover oscillates, increase if sluggish
#define MADGWICK_BETA_DEFAULT   0.1f

// Sample period in seconds — MUST match the rate at which you call madgwick_update_imu()
//
// droneflightcode.ino hiện tại dùng dt = 0.004 (250Hz)
// Nếu chạy FreeRTOS imu_fusion_task ở 500Hz: đổi thành 500.0f
// Rule: MADGWICK_SAMPLE_FREQ_HZ phải khớp với vTaskDelayUntil period
#define MADGWICK_SAMPLE_FREQ_HZ 250.0f          // khớp với droneflightcode.ino (dt=0.004)
#define MADGWICK_DT             (1.0f / MADGWICK_SAMPLE_FREQ_HZ)

// -----------------------------------------------------------------------------
// Quaternion state structure
// q0 = w (scalar), q1 = x, q2 = y, q3 = z
// Always normalized: q0^2 + q1^2 + q2^2 + q3^2 = 1
// -----------------------------------------------------------------------------
typedef struct {
    float q0, q1, q2, q3;  // quaternion components [w, x, y, z]
    float beta;             // convergence gain
    float dt;               // sample period (seconds)
} MadgwickFilter_t;

// -----------------------------------------------------------------------------
// Euler angles output (radians)
// -----------------------------------------------------------------------------
typedef struct {
    float roll;   // rotation around X-axis (phi)   [-π,  π]
    float pitch;  // rotation around Y-axis (theta) [-π/2, π/2]
    float yaw;    // rotation around Z-axis (psi)   [-π,  π]
} EulerAngles_t;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

/**
 * @brief  Initialize filter to identity quaternion (level, north-facing)
 * @param  f     Pointer to filter instance
 * @param  beta  Convergence gain (use MADGWICK_BETA_DEFAULT to start)
 * @param  dt    Sample period in seconds
 */
void madgwick_init(MadgwickFilter_t *f, float beta, float dt);

/**
 * @brief  Update filter with raw IMU data (6-DOF, no magnetometer)
 *
 * Call this at a FIXED rate equal to 1/dt.
 * Units: gyro in rad/s, accel in any consistent unit (m/s^2 or g)
 * Accel does NOT need to be normalized before calling — handled internally.
 *
 * @param  f              Filter instance
 * @param  gx, gy, gz     Gyroscope  [rad/s]  — bias-corrected preferred
 * @param  ax, ay, az     Accelerometer [m/s^2 or g]
 */
void madgwick_update_imu(MadgwickFilter_t *f,
                         float gx, float gy, float gz,
                         float ax, float ay, float az);

/**
 * @brief  Convert quaternion state → Euler angles (ZYX convention)
 * @param  f    Filter instance (reads q0..q3)
 * @param  out  Output struct filled with roll/pitch/yaw in radians
 */
void madgwick_get_euler(const MadgwickFilter_t *f, EulerAngles_t *out);

/**
 * @brief  Convert radians to degrees (inline helper)
 */
static inline float rad2deg(float r) { return r * 57.29577951f; }

#ifdef __cplusplus
}
#endif
