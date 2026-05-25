// =============================================================================
// madgwick_filter.c
// Madgwick AHRS Filter — Optimized for ESP32-S3 FPU (bare-metal / FreeRTOS)
//
// Key optimizations for ESP32-S3 Xtensa LX7:
//   1. Uses fast inverse square root (recipSqrt) — avoids slow sqrtf()
//   2. All arithmetic on float32 → maps directly to FPU instructions
//   3. Zero dynamic allocation — all state in MadgwickFilter_t struct
//   4. No transcendental calls (sin/cos) in update loop
//   5. Accel normalization guard prevents divide-by-zero on free-fall
// =============================================================================

#include "madgwick_filter.h"
#include <math.h>   // only used in euler conversion (atan2f, asinf)

// -----------------------------------------------------------------------------
// Fast reciprocal square root  — 1/sqrt(x)
// Original Quake III trick, refined for IEEE-754 float32.
// On ESP32-S3 FPU this is ~4x faster than 1.0f/sqrtf(x).
// -----------------------------------------------------------------------------
static inline float recip_sqrt(float x)
{
    // Use hardware FPU sqrt if available and the compiler supports it,
    // otherwise fall back to the bit-hack approximation + Newton-Raphson.
    // ESP-IDF with -mfpu=fp16 will auto-vectorize sqrtf; we keep the
    // bit-hack here for maximum portability across ESP32 variants.

    float half_x = 0.5f * x;
    union { float f; uint32_t i; } conv = { .f = x };
    conv.i = 0x5F3759DFu - (conv.i >> 1);          // magic constant
    conv.f *= (1.5f - half_x * conv.f * conv.f);   // one Newton-Raphson step
    conv.f *= (1.5f - half_x * conv.f * conv.f);   // two steps → ~6 decimal digits
    return conv.f;
}

// =============================================================================
// madgwick_init
// =============================================================================
void madgwick_init(MadgwickFilter_t *f, float beta, float dt)
{
    f->q0   = 1.0f;   // identity quaternion: no rotation
    f->q1   = 0.0f;
    f->q2   = 0.0f;
    f->q3   = 0.0f;
    f->beta = beta;
    f->dt   = dt;
}

// =============================================================================
// madgwick_update_imu  (6-DOF — gyro + accel, no magnetometer)
//
// Algorithm overview (per Madgwick 2010, eq. 33):
//   1. Integrate gyroscope rate → quaternion derivative (qdot)
//   2. If accel valid: compute gradient of objective function f(q, a_ref)
//      that measures the error between estimated gravity direction and measured
//   3. Subtract beta * gradient from qdot  (gradient descent step)
//   4. Integrate: q += qdot * dt
//   5. Renormalize quaternion
// =============================================================================
void madgwick_update_imu(MadgwickFilter_t *f,
                         float gx, float gy, float gz,
                         float ax, float ay, float az)
{
    // -- Cache quaternion components locally for readability & register use --
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // -------------------------------------------------------------------------
    // STEP 1: Quaternion derivative from gyroscope (pure integration, eq. 12)
    //   qdot = 0.5 * q ⊗ [0, gx, gy, gz]
    // -------------------------------------------------------------------------
    float qdot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qdot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qdot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qdot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // -------------------------------------------------------------------------
    // STEP 2: Gradient descent correction using accelerometer
    // Skip if accel magnitude is near zero (free-fall, collision spike, etc.)
    // -------------------------------------------------------------------------
    float accel_sq = ax * ax + ay * ay + az * az;

    if (accel_sq > 1e-6f)   // ~0.001 m/s^2 threshold
    {
        // Normalize accelerometer vector
        float inv_norm = recip_sqrt(accel_sq);
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        // Pre-compute repeated products (saves ~12 multiplications)
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // Objective function gradient ∇f (eq. 25 in Madgwick 2010)
        // Measures how far estimated gravity direction is from measured accel
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Normalize gradient vector
        float inv_s = recip_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= inv_s;
        s1 *= inv_s;
        s2 *= inv_s;
        s3 *= inv_s;

        // Apply gradient descent correction to qdot (eq. 33)
        qdot0 -= f->beta * s0;
        qdot1 -= f->beta * s1;
        qdot2 -= f->beta * s2;
        qdot3 -= f->beta * s3;
    }

    // -------------------------------------------------------------------------
    // STEP 3: Integrate quaternion derivative → new quaternion
    // -------------------------------------------------------------------------
    q0 += qdot0 * f->dt;
    q1 += qdot1 * f->dt;
    q2 += qdot2 * f->dt;
    q3 += qdot3 * f->dt;

    // -------------------------------------------------------------------------
    // STEP 4: Re-normalize (mandatory — integration accumulates float errors)
    // -------------------------------------------------------------------------
    float inv_norm_q = recip_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    f->q0 = q0 * inv_norm_q;
    f->q1 = q1 * inv_norm_q;
    f->q2 = q2 * inv_norm_q;
    f->q3 = q3 * inv_norm_q;
}

// =============================================================================
// madgwick_get_euler
// Converts quaternion → Euler angles using ZYX (aerospace) convention:
//   Roll  (phi)   — rotation about X
//   Pitch (theta) — rotation about Y
//   Yaw   (psi)   — rotation about Z
//
// NOTE: atan2f and asinf are only called ONCE per PID cycle (not in filter
// update), so transcendental cost is acceptable at 500Hz.
//
// Singularity at pitch = ±90° (Gimbal Lock) is handled by clamping sinp.
// =============================================================================
void madgwick_get_euler(const MadgwickFilter_t *f, EulerAngles_t *out)
{
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // -- Roll (X-axis rotation) --
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    out->roll = atan2f(sinr_cosp, cosr_cosp);

    // -- Pitch (Y-axis rotation) — clamp to avoid asinf domain error --
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    out->pitch = asinf(sinp);

    // -- Yaw (Z-axis rotation) --
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    out->yaw = atan2f(siny_cosp, cosy_cosp);
}
