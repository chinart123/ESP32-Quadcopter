# MISSION CONTEXT: SENSOR FUSION MERGE & YAW ANCHORING
**Role:** Senior Embedded Flight Control Engineer
**Target Platform:** ESP32-S3 Supermini (Dual-core, FPU enabled)
**Actuation:** 8520 Coreless Motors running at **8kHz PWM**

## 1. HISTORICAL CODEBASE & LIBRARY CONTEXT
The flight controller relies on a custom hardware abstraction library for the IMU, combined with the main `.ino` logic. None of the main flight codes have ever integrated the PMW3901 Optical Flow sensor.

### 1.1 The IMU Library (`HungVo_IMU.h` & `HungVo_IMU.cpp`)
This custom library handles I2C communication and raw data extraction. Crucially, it contains the **Static Gyro Bias Calibration** logic that fixes ~60% of the Yaw drift by averaging thousands of samples at startup.
*Snippet from `HungVo_IMU.cpp`:*
```cpp
void HungVo_IMU::calibrate(int samples) {
  float sumGX=0, sumGY=0, sumGZ=0;
  for(int i=0; i<samples; i++) {
    readBurst();
    sumGX += (float)_gx/65.5; sumGY += (float)_gy/65.5; sumGZ += (float)_gz/65.5;
    delay(1);
  }
  _offGX = sumGX/samples; 
  _offGY = sumGY/samples; 
  _offGZ = sumGZ/samples; // Capturing static Yaw drift offset
}
```

### 1.2 The Legacy Main Code (`droneflightcode.ino`)
The previous stable version. It utilized a basic 1D Kalman filter and a highly aggressive 16kHz PWM output.
- **Flaw:** Only featured P and D terms for Yaw, and did not dynamically update the Gyro Bias during flight. It also lacked SPI initialization for any optical sensor.

### 1.3 The Current Main Code (`flycode_Yaw_ok_thang5.ino`)
The latest iteration. It isolates the I2C buses (`I2C_IMU` and `I2C_TOF`), reduces PWM to **8kHz** for better torque, and introduces an I-term PID for Yaw alongside dynamic bias filtering.
- **Flaw:** Still solely relies on MPU6050 for heading. Yaw macro-drift still exists over time due to the lack of an absolute spatial reference.

## 2. OPTICAL FLOW SUBSYSTEM STATUS (`Optical Sensor-v5` tab)
- **Sensor:** PMW3901 (Optical Flow via SPI).
- **Current State:** Successfully initialized and running in **Motion Mode**. The frame-grab bottleneck is resolved. The non-blocking `millis()` loop currently prints $\Delta X$ and $\Delta Y$ at 100Hz.
- **Deficiency:** Fully isolated from `flycode_Yaw_ok_thang5.ino`.

## 3. THE OBJECTIVE (YOUR TASK)
I need you (Claude) to merge the SPI Optical Flow logic into `flycode_Yaw_ok_thang5.ino` to execute **Yaw Anchoring (Cross-track Error Correction)**, fixing the remaining Yaw drift.

Please provide the fully merged C++ code fulfilling these requirements:
1. **SPI Integration:** Move the PMW3901 initialization and 100Hz non-blocking read logic into `flycode_Yaw_ok_thang5.ino`.
2. **Velocity Scaling:** Scale the raw $\Delta X, \Delta Y$ using `filteredHeight` from the VL53L1X.
3. **Yaw Anchoring Math (Outer Loop):** When the RC input commands strictly forward flight (high Pitch, near-zero Roll), use `atan2f(Delta_X, Delta_Y)` to detect the cross-track slip angle.
4. **PID Injection:** Inject this computed slip angle as an error correction term into the existing Yaw PID controller.

## 4. TARGET SYSTEM ARCHITECTURE (VISUALIZED)
```mermaid
graph TD
    classDef hw fill:#2d3436,color:#fff,stroke:#b2bec3,stroke-width:2px;
    classDef sw fill:#0984e3,color:#fff,stroke:#74b9ff,stroke-width:2px;
    classDef pid fill:#00b894,color:#fff,stroke:#55efc4,stroke-width:2px;

    S1[PMW3901 SPI @ 100Hz]:::hw -->|ΔX, ΔY| O1(Outer Loop: Yaw Anchoring):::sw
    S2[VL53L1X I2C]:::hw -->|Height| O1
    RC[ELRS RC Input]:::hw -->|Pitch/Roll Cmd| O1

    O1 -->|Cross-track Error| P1{Yaw PID + Bias}:::pid

    S3[MPU6050 I2C @ 400kHz]:::hw -->|Raw Gyro Z| P1
    S3 -->|Roll/Pitch| P2{Attitude PID}:::pid

    P1 --> M[Motor Mixer]:::sw
    P2 --> M
    M --> PWM[PWM Actuation @ 8kHz]:::hw
```