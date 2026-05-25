# TECHNICAL CONTEXT: WINDIFY 6-DOF QUADCOPTER FLIGHT CONTROLLER MIGRATION

## 1. System Mathematical Foundation & Dynamics

### The General State-Space Form
The mathematical representation of the physical dynamics within this estimation system is governed by the discrete-time linear State Extrapolation Equation (or State-Space Presentation):
$$x_{n+1} = F x_n + G u_n$$
* $x_n$: The State Vector at step $n$, encapsulating the system's physical quantities.
* $F$: The State Transition Matrix, dictating how the states evolve naturally over time step $\Delta t$.
* $u_n$: The Control Input Vector.
* $G$: The Control Input Matrix, mapping control actions to state transitions.

### Current Linear 1D Model (Euler-Based)
The current implementation runs two decoupled 1D Kalman filters for the Roll and Pitch axes independently. To isolate the severe mechanical high-frequency vibrations induced by the 8520 coreless brushed motors, Gyro Bias ($\beta$) is included as a state variable to eliminate integration drift:
$$\begin{bmatrix} \theta_{n+1} \\ \beta_{n+1} \end{bmatrix} = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \theta_n \\ \beta_n \end{bmatrix} + \begin{bmatrix} \Delta t \\ 0 \end{bmatrix} \dot{\theta}_{gyro}$$
* **Limitation:** This decoupled model completely ignores 3D rotational cross-coupling, defined by non-linear trigonometric dependencies: $\dot{\phi} = p + q \sin\phi \tan\theta + r \cos\phi \tan\theta$. Furthermore, it is highly vulnerable to Gimbal Lock (mathematical singularity) when the Pitch angle $\theta$ approaches $\pm90^\circ$.

### Target Non-Linear 3D Model (Quaternion-Based)
To eliminate Gimbal Lock and properly account for cross-axis coupling, the orientation state must be represented via unit Quaternions ($q = [q_0, q_1, q_2, q_3]^T$), mapping a single rotation angle $\theta$ around an arbitrary spatial vector axis $\vec{v} = (x, y, z)$. The kinematic propagation equation is purely algebraic, avoiding expensive trigonometric computations on the MCU:
$$\begin{bmatrix} \dot{q_0} \\ \dot{q_1} \\ \dot{q_2} \\ \dot{q_3} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} \begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}$$

---

## 2. Codebase Evolution History

### Phase 1: Bare-Metal STM32 & Early ESP-IDF (Static Complementary Filter)
* **Logic:** Fixed-weight fusion utilizing a High-pass filter for short-term Gyro responsiveness and a Low-pass filter for long-term Accelerometer gravity reference.
* **Source Reference:** `xx_mpu_data_fusion.c`, `drn_mpu6050.cpp`
```c
// Static alpha weighting (98% Gyro trust, 2% Accel trust)
// Critically failed during flight due to coreless motor vibrations shifting the gravity vector
Drone_IMU.roll_deg  = DRN_MPU6050_FILTER_ALPHA * (Drone_IMU.roll_deg  + Drone_IMU.gyro_x_dps * Drone_IMU.delta_time_s) + (1.0f - DRN_MPU6050_FILTER_ALPHA) * accel_roll;
Drone_IMU.pitch_deg = DRN_MPU6050_FILTER_ALPHA * (Drone_IMU.pitch_deg + Drone_IMU.gyro_y_dps * Drone_IMU.delta_time_s) + (1.0f - DRN_MPU6050_FILTER_ALPHA) * accel_pitch;

// Pure integration for Yaw axis - completely lacks absolute sensor reference, causing continuous drift
Drone_IMU.yaw_deg  += Drone_IMU.gyro_z_dps * Drone_IMU.delta_time_s;
```

### Phase 2: Arduino API Framework on ESP32-S3 (Dynamic 1D Kalman Filter)
* **Logic:** Dynamic weighting based on runtime noise estimation matrices ($Q$ for process noise, $R$ for measurement noise) via an external library.
* **Source Reference:** `droneflightcode.ino`
```cpp
#include <Kalman.h>
Kalman kalmanR, kalmanP; // Instantiating two independent 1D Kalman filters
const float dt = 0.004;  // Fixed loop cycle time (250Hz)

void setup() {
  // Fine-tuning the covariance matrices specifically to damp 8520 coreless motor noise
  kalmanR.setRmeasure(0.13); // High R value = less trust in noisy accelerometer data
  kalmanP.setRmeasure(0.13);
  kalmanR.setQangle(0.003);  kalmanP.setQangle(0.003);
  kalmanR.setQbias(0.003);   kalmanP.setQbias(0.003);
}

void loop() {
  // Step 1: Extract absolute tilt reference from Accelerometer as measurement input (z_k)
  float rawP = (-myIMU.getRawRollAngle()) - pitchOffset;
  float rawR = (myIMU.getRawPitchAngle()) - rollOffset;

  // Step 2: Run internal Predict and Update loop cycles via library function calls
  // Passes raw gyro rate (\omega_k) and dt to compute the optimized Kalman Gain K_k
  float fP_raw = kalmanP.getAngle(rawP, -myIMU.getGyroX(), dt);
  float fR_raw = kalmanR.getAngle(rawR, myIMU.getGyroY(), dt);

  fRoll  = (fR_raw * 0.7071) + (fP_raw * 0.7071);
  fPitch = (fP_raw * 0.7071) - (fR_raw * 0.7071);

  // Unfiltered Yaw axis: Continues to rely on vulnerable open-loop pure integration
  float gz_clean = myIMU.getGyroZ() - yawGyroOffset;
  fYaw  -= gz_clean * dt; 
}
```

---

## 3. Peripheral Sensors & Low-Level Hardware Execution

### Hardware Constraints of MPU6050 (6-DOF)
The MPU6050 houses a 3-axis accelerometer and a 3-axis gyroscope. Because the accelerometer can only detect the Earth's gravity vector pointing downwards, it provides a stable static reference for Roll and Pitch but is fundamentally blind to rotations around the gravity vector itself (Yaw). Consequently, any algorithm bound strictly to an MPU6050 cannot perform full sensor fusion on the Yaw axis, resulting in unchecked heading drift over time. 

To overcome this without an onboard magnetometer, the system incorporates an auxiliary sensor array:
1.  **VL53L1X (Time-of-Flight):** Collects highly accurate Z-axis distance data to drive the Altitude Hold PID loop.
2.  **PMW3901 (Optical Flow):** Tracks relative pixel shifts on the floor to calculate horizontal velocity vectors ($\Delta X$, $\Delta Y$) for Position Hold.

### PMW3901 SPI Bottleneck & Control Mismatch
* **Current State:** The system is constrained within a low-level **Frame Grab Mode** used for optical lens debugging. Due to highly conservative timing overheads introduced during raw pixel polling (e.g., a 200µs delay after pulling the Chip Select line high), retrieving a single 35x35 pixel frame takes approximately ~990ms, limiting throughput to **~1 FPS**.
* **Control Loop Mismatch Dilemma:** The brushed coreless motor driver (`drn_timer_pwm.c`) modulates throttle at a rapid **4kHz PWM frequency**. Developers must address how a slow-sampling position sensor can effectively interface with a high-frequency motor actuation scheme without destabilizing the system.
* **Target State:** The sensor must be transitioned into **Motion Mode** to bypass raw frame fetching, allowing it to output velocity delta vectors over SPI at its native rate of **>100Hz**.

### Automated Hardware Acceleration (ESP32-S3 FPU)
The ESP32-S3 features dual Xtensa LX7 cores with a hardware Single-Precision Floating-Point Unit (FPU). Within the Arduino IDE and ESP-IDF compilation frameworks, developers do not need to manipulate bare-metal registers to unlock this feature. The underlying GCC toolchain automatically injects compiler optimization flags (specifically `-mhard-float` and `-mfpu=fpv5-sp-d16`), mapping standard C/C++ `float` operations (`+`, `-`, `*`, `/`) and mathematical macros directly to dedicated hardware FPU instructions rather than generating slow software emulation routines.

---

## 4. User Requirement for Project Sketching Plan
*(Still suffering from architectural issues and ambiguous integration points)*

The Principal Architect must explicitly evaluate and resolve the following compilation of engineering requirements and structural design doubts:

1.  **General Form Discrepancy:** Verify how textbook state-space notations ($x_{n+1} = Fx_n + Gu_n$) directly translate into real-time C/C++ matrix arrays inside execution loops.
2.  **Slide Formulas vs. Actual 1D Code:** Contrast the linear equations found in the M2 presentation against the true 1D implementation found inside `droneflightcode.ino`. Detail the physical implications of ignoring axis cross-coupling during dynamic maneuvers.
3.  **Register vs. Software Abstraction:** Confirm that migrating to a 3D Quaternion-based filter (such as Madgwick or Mahony) executes purely in the software layer of the MCU using raw sensor bytes, and requires zero low-level register configuration changes inside the MPU6050 itself.
4.  **The Pure Integration Yaw Vulnerability:** Validate that the current heading estimation (`fYaw -= gz_clean * dt;`) mirrors the drifting logic found in legacy STM32 and ESP-IDF drivers. Formulate the explicit mathematical or sensor-based architecture needed to fix the Yaw axis using the PMW3901 + VL53L1X array in the absence of a Magnetometer.
5.  **Complementary vs. Kalman Architecture:** Provide a definitive technical breakdown of how a static-weighted filter differs from the dynamic covariance-driven state estimation used in the current codebase.
6.  **Cascaded Loop Time-Scale Separation:** Architect the solution to the speed mismatch problem. Explain how a ~100Hz Motion Mode Optical Flow sensor can successfully guide a 4kHz PWM motor control setup using a Cascaded (Dual-Loop) PID control scheme (Outer Loop Position vs. Inner Loop Attitude).
7.  **FPU Optimization Constraints:** Ensure that the new Quaternion equations are structured to maximize single-precision float hardware acceleration on the Xtensa LX7 core.
8.  **Quaternion Library Mapping:** Analyze the 3 newly generated Madgwick/Mahony quaternion files. Map out exactly where and how they replace the current `<Kalman.h>` function calls (`kalmanP.getAngle(...)`), and show how to feed raw data from the existing `HungVo_IMU` driver into the new quaternion update cycles.