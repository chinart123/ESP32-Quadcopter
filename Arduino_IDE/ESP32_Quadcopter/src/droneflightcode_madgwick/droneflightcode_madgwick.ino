/* ============================================================================
 * PROJECT: 1S DRONE FLIGHT CONTROLLER (ESP32-S3 SUPERMINI)
 * DEVELOPER: HUNG VO | UPDATE: MAY 2026
 * MIGRATION: Kalman 1D (Euler) → Madgwick 3D (Quaternion)
 *
 * THAY ĐỔI SO VỚI BẢN CŨ:
 *   [XÓA] #include <Kalman.h>
 *   [XÓA] Kalman kalmanR, kalmanP
 *   [XÓA] kalman.setRmeasure / setQangle / setQbias trong setup()
 *   [XÓA] rawP, rawR, fP_raw, fR_raw, kalman.getAngle() trong loop()
 *   [THÊM] #include "madgwick_filter.h"
 *   [THÊM] MadgwickFilter_t madgwick
 *   [THÊM] madgwick_init() trong setup() và smartCalibrate()
 *   [THÊM] madgwick_update_imu() + madgwick_get_euler() trong loop()
 *
 * KHÔNG ĐỔI: ISR, PID cascade, motor mixer, BMS, LED, arming logic
 *
 * OPEN ISSUES CẦN XÁC NHẬN:
 *   [ ] Dấu trục sau khi chạy: nghiêng phải → fRoll dương? (SIGN_CHECK)
 *   [ ] 0.7071 mix: giữ lại vì IMU mount lệch 45° (chưa xác nhận vật lý)
 *   [ ] Yaw direction: Hướng A (drift accepted) hay B (PMW3901 correction)?
 * ============================================================================
 */

#include <Wire.h>
// [XÓA] #include <Kalman.h>
#include <Adafruit_NeoPixel.h>
#include "HungVo_IMU.h"
#include "madgwick_filter.h"   // [THÊM] thay thế Kalman.h

// =============================================================================
// 1. CẤU HÌNH PIN
// =============================================================================
const int motorPins[] = {1, 4, 2, 3};
const int rcPins[]    = {16, 15, 14, 39, 40, 41};
#define PIN_ADC  7
#define PIN_RGB  48

// =============================================================================
// 2. BIẾN OFFSET & TELEMETRY
// =============================================================================
#define REF_VOLTAGE    3.10
#define ADC_RESOLUTION 4095.0
#define DIVIDER_RATIO  2.0
#define CALIB_FACTOR   1.1455
float filtered_voltage  = 4.2;
float pitchOffset       = 0;
float rollOffset        = 0;
float yawGyroOffset     = 0;
float targetYawHeading  = 0;

bool isArmed         = false;
bool safetyReleased  = false;
bool isReadyToFly    = false;
unsigned long armStartTime = 0;
const int idleSpeed  = 120;

// =============================================================================
// 3. RX INTERRUPT — giữ nguyên hoàn toàn
// =============================================================================
volatile unsigned long pulseStart[6];
volatile int           rcValue[6];
void IRAM_ATTR isr0() { if(digitalRead(rcPins[0])) pulseStart[0]=micros(); else rcValue[0]=micros()-pulseStart[0]; }
void IRAM_ATTR isr1() { if(digitalRead(rcPins[1])) pulseStart[1]=micros(); else rcValue[1]=micros()-pulseStart[1]; }
void IRAM_ATTR isr2() { if(digitalRead(rcPins[2])) pulseStart[2]=micros(); else rcValue[2]=micros()-pulseStart[2]; }
void IRAM_ATTR isr3() { if(digitalRead(rcPins[3])) pulseStart[3]=micros(); else rcValue[3]=micros()-pulseStart[3]; }
void IRAM_ATTR isr4() { if(digitalRead(rcPins[4])) pulseStart[4]=micros(); else rcValue[4]=micros()-pulseStart[4]; }
void IRAM_ATTR isr5() { if(digitalRead(rcPins[5])) pulseStart[5]=micros(); else rcValue[5]=micros()-pulseStart[5]; }

// =============================================================================
// 4. IMU & FILTER
// =============================================================================
HungVo_IMU        myIMU(Wire);

// [XÓA] Kalman kalmanR, kalmanP;
MadgwickFilter_t  madgwick;       // [THÊM] instance filter — state sống ở đây

float fRoll  = 0.0f;
float fPitch = 0.0f;
float fYaw   = 0.0f;

const float dt = 0.004f;          // 250Hz — giữ nguyên, dùng cho PID và madgwick_init

// Hệ số đổi đơn vị gyro: dps → rad/s
// HungVo_IMU.getGyroX/Y/Z() trả về degrees/second
// Madgwick cần rad/s
#define DEG_TO_RAD 0.017453293f

// =============================================================================
// 5. PID TUNING — giữ nguyên hoàn toàn
// =============================================================================
float PAngle    = 5.6;
float PRate     = 0.85, IRate = 0.04, DRate = 0.025;
float PAngleYaw = 2.0;
float PRateYaw  = 3.5;
float iTermRoll, iTermPitch, lastErrorRoll, lastErrorPitch;

Adafruit_NeoPixel pixels(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

// =============================================================================
// HÀM CALI THÔNG MINH (2s Tím → Trắng → Zero)
// Thay đổi: kalmanP.setAngle(0); kalmanR.setAngle(0) → madgwick_init()
// =============================================================================
void smartCalibrate() {
    pixels.setPixelColor(0, pixels.Color(255, 0, 255)); pixels.show();
    Serial.println(">>> PURPLE: WAIT 2 SECONDS...");
    delay(2000);

    int   validSamples = 0;
    float sumP = 0, sumR = 0, sumGZ = 0;
    const float gyroThreshold = 15.0;

    Serial.println(">>> WHITE: SAMPLING 3000 MS...");
    while (validSamples < 3000) {
        myIMU.update();
        if (abs(myIMU.getGyroX()) > gyroThreshold || abs(myIMU.getGyroY()) > gyroThreshold) {
            // Drone bị rung/di chuyển → reset mẫu, chờ yên
            validSamples = 0; sumP = 0; sumR = 0; sumGZ = 0;
            pixels.setPixelColor(0, pixels.Color(255, 0, 255)); pixels.show();
        } else {
            validSamples++;
            sumP  += -myIMU.getRawRollAngle();
            sumR  +=  myIMU.getRawPitchAngle();
            sumGZ +=  myIMU.getGyroZ();
            pixels.setPixelColor(0, pixels.Color(255, 255, 255)); pixels.show();
        }
        delay(2);
    }

    pitchOffset      = sumP  / 3000.0f;
    rollOffset       = sumR  / 3000.0f;
    yawGyroOffset    = sumGZ / 3000.0f;

    // [XÓA] kalmanP.setAngle(0); kalmanR.setAngle(0);
    // [THÊM] Reset Madgwick về identity quaternion (tương đương đặt góc về 0)
    madgwick_init(&madgwick, MADGWICK_BETA_DEFAULT, dt);

    fYaw = 0.0f; targetYawHeading = 0.0f;
    isReadyToFly = true;
    Serial.println(">>> CALI DONE. LEVEL OK!");
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(921600);
    pixels.begin(); pixels.setBrightness(100);

    // Wire SDA=5, SCL=6 — giữ nguyên
    Wire.begin(5, 6, 400000);
    Wire.setTimeOut(50);
    if (!myIMU.begin(5, 6)) { while (1); }

    // [XÓA] Toàn bộ kalman.setRmeasure / setQangle / setQbias
    // Lý do: Madgwick chỉ có 1 tham số duy nhất là beta (trong MADGWICK_BETA_DEFAULT)
    // Tune beta: giảm nếu nhiễu motor ảnh hưởng góc, tăng nếu góc drift chậm
    // Điểm bắt đầu: 0.1f — xem madgwick_filter.h để biết range

    // [THÊM] Khởi tạo Madgwick — gọi trước smartCalibrate()
    madgwick_init(&madgwick, MADGWICK_BETA_DEFAULT, dt);

    // Calibrate offset và reset filter về 0
    smartCalibrate();

    // RC interrupt setup — giữ nguyên
    for (int i = 0; i < 6; i++) {
        pinMode(rcPins[i], INPUT_PULLDOWN);
        if (i == 0) attachInterrupt(rcPins[0], isr0, CHANGE);
        if (i == 1) attachInterrupt(rcPins[1], isr1, CHANGE);
        if (i == 2) attachInterrupt(rcPins[2], isr2, CHANGE);
        if (i == 3) attachInterrupt(rcPins[3], isr3, CHANGE);
        if (i == 4) attachInterrupt(rcPins[4], isr4, CHANGE);
        if (i == 5) attachInterrupt(rcPins[5], isr5, CHANGE);
    }

    // Motor PWM 16kHz 10-bit — giữ nguyên
    for (int i = 0; i < 4; i++) {
        ledcAttach(motorPins[i], 16000, 10);
        ledcWrite(motorPins[i], 0);
    }
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
    unsigned long startLoop = micros();
    myIMU.update();   // burst read 14 bytes từ MPU6050

    // =========================================================================
    // A. CẬP NHẬT GÓC — Madgwick thay thế Kalman
    // =========================================================================

    // --- BƯỚC 1: Lấy gyro, đổi đơn vị dps → rad/s ---
    //
    // Lưu ý axis mapping (giống code cũ):
    //   IMU Y-axis → drone Roll axis  (getGyroY dùng cho Roll PID bên dưới)
    //   IMU X-axis → drone Pitch axis (getGyroX dùng cho Pitch PID, đảo dấu)
    //   Đây là kết quả của việc IMU được gắn lệch trục vật lý.
    //
    // [SIGN_CHECK] Sau khi chạy lần đầu: nghiêng drone sang PHẢI → fRoll phải DƯƠNG.
    //   Nếu ngược dấu: đổi dấu gx ↔ gy hoặc ax ↔ ay bên dưới.
    float gx =  myIMU.getGyroY() * DEG_TO_RAD;   // drone Roll  axis [rad/s]
    float gy = -myIMU.getGyroX() * DEG_TO_RAD;   // drone Pitch axis [rad/s] (đảo dấu)
    float gz =  myIMU.getGyroZ() * DEG_TO_RAD;   // drone Yaw   axis [rad/s]

    // --- BƯỚC 2: Lấy accel vector thô ---
    //
    // [PREREQUISITE — Task 0] getAccelX/Y/Z() phải được thêm vào HungVo_IMU trước:
    //   float HungVo_IMU::getAccelX() { return ((float)_ax/4096.0f) - _offAX; }
    //   float HungVo_IMU::getAccelY() { return ((float)_ay/4096.0f) - _offAY; }
    //   float HungVo_IMU::getAccelZ() { return ((float)_az/4096.0f) - (_offAZ-1.0f); }
    //
    // Axis mapping giống gyro: Y→Roll, -X→Pitch
    float ax =  myIMU.getAccelY();   // drone Roll  axis [g]
    float ay = -myIMU.getAccelX();   // drone Pitch axis [g] (đảo dấu)
    float az =  myIMU.getAccelZ();   // Z giữ nguyên     [g]

    // --- BƯỚC 3: Chạy Madgwick (~15µs trên ESP32-S3 FPU) ---
    madgwick_update_imu(&madgwick, gx, gy, gz, ax, ay, az);

    // --- BƯỚC 4: Lấy Euler angles ---
    EulerAngles_t euler;
    madgwick_get_euler(&madgwick, &euler);
    float roll_raw  = rad2deg(euler.roll);
    float pitch_raw = rad2deg(euler.pitch);
    fYaw            = rad2deg(euler.yaw);

    // --- BƯỚC 5: Rotation matrix 45° ---
    //
    // Giữ lại đoạn 0.7071 từ code cũ vì IMU có thể gắn lệch 45° vật lý.
    // Đây là bước xoay hệ tọa độ từ IMU frame → drone frame.
    //
    // [?] Nếu sau khi test thực tế fRoll/fPitch đã đúng chiều mà không cần mix:
    //     comment out 4 dòng dưới và dùng trực tiếp roll_raw / pitch_raw.
    fRoll  = (roll_raw  * 0.7071f) + (pitch_raw * 0.7071f);
    fPitch = (pitch_raw * 0.7071f) - (roll_raw  * 0.7071f);

    // --- BƯỚC 6: Trừ offset sau khi có góc cuối ---
    //
    // Code cũ trừ offset TRƯỚC khi vào Kalman (rawP = angle - offset).
    // Code mới trừ offset SAU khi có output Madgwick.
    // Kết quả tương đương vì Madgwick linear ở góc nhỏ (hover condition).
    fRoll  -= rollOffset;
    fPitch -= pitchOffset;
    // fYaw đã được reset về 0 trong smartCalibrate() → không cần trừ offset

    // gz_clean dùng cho Yaw PID rate term (giữ nguyên logic cũ)
    float gz_clean = myIMU.getGyroZ() - yawGyroOffset;

    // =========================================================================
    // B. AN TOÀN — giữ nguyên
    // =========================================================================
    if (isArmed && (abs(fRoll) > 45 || abs(fPitch) > 45)) {
        isArmed = false; safetyReleased = false;
    }

    // =========================================================================
    // C. ARMING LOGIC — giữ nguyên
    // =========================================================================
    if (!isArmed && isReadyToFly && rcValue[5] > 1490 && rcValue[5] < 1510)
        safetyReleased = true;

    if (!isArmed && safetyReleased && rcValue[4] > 1550 && rcValue[2] < 1100) {
        isArmed          = true;
        armStartTime     = millis();
        fYaw             = 0.0f;
        targetYawHeading = 0.0f;
        iTermRoll        = 0.0f;
        iTermPitch       = 0.0f;
        // Reset Madgwick về identity khi arm để loại bỏ drift tích lũy từ idle
        madgwick_init(&madgwick, MADGWICK_BETA_DEFAULT, dt);
    }

    if (rcValue[4] < 1450) { isArmed = false; safetyReleased = false; }

    // =========================================================================
    // D. PID CASCADE — giữ nguyên hoàn toàn
    // =========================================================================
    float targetR    = map(rcValue[0], 1000, 2000, -25, 25);
    float targetP    = map(rcValue[1], 1000, 2000, -25, 25);
    int   throttle   = map(rcValue[2], 1000, 2000, 0, 850);

    float stickYawRate = map(rcValue[3], 1000, 2000, 150, -150);
    if (abs(stickYawRate) < 15) {
        stickYawRate = 0;
    } else {
        targetYawHeading = fYaw;
    }

    if (abs(rcValue[0] - 1500) < 15) targetR = 0;
    if (abs(rcValue[1] - 1500) < 15) targetP = 0;

    // Roll PID
    float tarRateR = PAngle * (targetR - fRoll);
    float errR     = tarRateR - myIMU.getGyroY();
    if (throttle < 50) iTermRoll = 0;
    else iTermRoll = constrain(iTermRoll + IRate * errR * dt, -200, 200);
    float outRoll  = (PRate * errR) + iTermRoll + DRate * (errR - lastErrorRoll) / dt;
    lastErrorRoll  = errR;

    // Pitch PID
    float tarRateP = PAngle * (targetP - fPitch);
    float errP     = tarRateP - (-myIMU.getGyroX());
    if (throttle < 50) iTermPitch = 0;
    else iTermPitch = constrain(iTermPitch + IRate * errP * dt, -200, 200);
    float outPitch = (PRate * errP) + iTermPitch + DRate * (errP - lastErrorPitch) / dt;
    lastErrorPitch = errP;

    // Yaw Heading Hold
    // fYaw giờ đến từ Madgwick (thay vì tích phân thô)
    // Vẫn drift theo thời gian (không có magnetometer) — Hướng A
    float yawError = targetYawHeading - fYaw;
    float outYaw   = (PAngleYaw * yawError) + (PRateYaw * (stickYawRate - gz_clean));

    // =========================================================================
    // E. MOTOR MIXER — giữ nguyên hoàn toàn
    // =========================================================================
    if (isArmed) {
        int baseThr = (throttle < 25) ? idleSpeed : throttle;
        ledcWrite(3, constrain(baseThr - outPitch + outRoll + outYaw, 30, 1023)); // FL
        ledcWrite(4, constrain(baseThr - outPitch - outRoll - outYaw, 30, 1023)); // FR
        ledcWrite(2, constrain(baseThr + outPitch + outRoll - outYaw, 30, 1023)); // RL
        ledcWrite(1, constrain(baseThr + outPitch - outRoll + outYaw, 30, 1023)); // RR
    } else {
        for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], 0);
    }

    updateBMS();
    updateLEDs();

    // Giữ chu kỳ 4000µs = 250Hz — giữ nguyên
    while (micros() - startLoop < 4000);
}

// =============================================================================
// BMS — giữ nguyên hoàn toàn
// =============================================================================
void updateBMS() {
    int   r = analogRead(PIN_ADC);
    float v = (r / 4095.0f) * 3.1f * 2.0f * 1.1455f;
    filtered_voltage = filtered_voltage * 0.95f + v * 0.05f;
}

// =============================================================================
// LED — giữ nguyên hoàn toàn
// =============================================================================
void updateLEDs() {
    if (!isArmed) {
        if (safetyReleased) pixels.setPixelColor(0, pixels.Color(0, 0, 255));    // Xanh = sẵn sàng arm
        else                pixels.setPixelColor(0, pixels.Color(255, 100, 0));  // Cam = chờ safety
    } else {
        if (millis() - armStartTime < 5000) {
            pixels.setPixelColor(0, pixels.Color(0, 255, 0));   // Xanh lá = vừa arm
        } else {
            if      (filtered_voltage > 3.85f) pixels.setPixelColor(0, pixels.Color(0, 255, 0));   // Pin đầy
            else if (filtered_voltage > 3.65f) pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // Pin trung
            else                               pixels.setPixelColor(0, pixels.Color(255, 0, 0));   // Pin yếu
        }
    }
    pixels.show();
}
