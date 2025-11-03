#include <filters.h>

// Apply zero level to filter out steady state noise
static inline int32_t apply_noise_floor(int32_t value, int32_t threshold)
{
    return (value > threshold || value < -threshold) ? value : 0;
}

// Fast inverse square root
static float invSqrt(float x)
{
    union
    {
        float f;
        uint32_t i;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;
    y = y * (1.5f - 0.5f * x * y * y);
    y = y * (1.5f - 0.5f * x * y * y); // Second iteration for more precision
    return y;
}

// // fast inverse square root
// static float invSqrt(float x)
// {
//     // Using the fast inverse square root algorithm
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long *)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float *)&i;
//     y = y * (1.5f - (halfx * y * y));
//     y = y * (1.5f - (halfx * y * y)); // Second iteration for more precision
//     return y;
// }

// Madgwick estimation

void Madgwick_init(Madgwick_est *filter, float sample_frequency, float beta)
{
    filter->beta = beta;
    filter->sample_period = 1.0f / sample_frequency;

    // Initialize quaternion to identity (no rotation)
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;

    // Initialize angles to zero
    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;
}

void Madgwick_updateIMU(Madgwick_est *filter, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
    qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    // Compute feedback only if accelerometer measurement is valid (avoids NaN)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalize accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * filter->q0;
        _2q1 = 2.0f * filter->q1;
        _2q2 = 2.0f * filter->q2;
        _2q3 = 2.0f * filter->q3;
        _4q0 = 4.0f * filter->q0;
        _4q1 = 4.0f * filter->q1;
        _4q2 = 4.0f * filter->q2;
        _8q1 = 8.0f * filter->q1;
        _8q2 = 8.0f * filter->q2;
        q0q0 = filter->q0 * filter->q0;
        q1q1 = filter->q1 * filter->q1;
        q2q2 = filter->q2 * filter->q2;
        q3q3 = filter->q3 * filter->q3;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * filter->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * filter->q3 - _2q1 * ax + 4.0f * q2q2 * filter->q3 - _2q2 * ay;

        // Normalize the corrective step
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply new external quaternion rate of change
        qDot1 -= filter->beta * s0;
        qDot2 -= filter->beta * s1;
        qDot3 -= filter->beta * s2;
        qDot4 -= filter->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    filter->q0 += qDot1 * filter->sample_period;
    filter->q1 += qDot2 * filter->sample_period;
    filter->q2 += qDot3 * filter->sample_period;
    filter->q3 += qDot4 * filter->sample_period;

    // Normalize quaternion
    recipNorm = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

void Madgwick_computeEulerAngles(Madgwick_est *filter)
{
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    filter->roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
    {
        filter->pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    }
    else
    {
        filter->pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    filter->yaw = atan2f(siny_cosp, cosy_cosp);
}

// 9axis version
void Madgwick_update(Madgwick_est *filter, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    gx *= mdps_to_radss;
    gy *= mdps_to_radss;
    gz *= mdps_to_radss;

    // Use 6-axis update if magnetometer measurement is invalid (avoids NaN)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick_updateIMU(filter, gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
    qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    // Compute feedback only if accelerometer measurement is valid (avoids NaN)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalize accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalize magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * filter->q0 * mx;
        _2q0my = 2.0f * filter->q0 * my;
        _2q0mz = 2.0f * filter->q0 * mz;
        _2q1mx = 2.0f * filter->q1 * mx;
        _2q0 = 2.0f * filter->q0;
        _2q1 = 2.0f * filter->q1;
        _2q2 = 2.0f * filter->q2;
        _2q3 = 2.0f * filter->q3;
        _2q0q2 = 2.0f * filter->q0 * filter->q2;
        _2q2q3 = 2.0f * filter->q2 * filter->q3;
        q0q0 = filter->q0 * filter->q0;
        q0q1 = filter->q0 * filter->q1;
        q0q2 = filter->q0 * filter->q2;
        q0q3 = filter->q0 * filter->q3;
        q1q1 = filter->q1 * filter->q1;
        q1q2 = filter->q1 * filter->q2;
        q1q3 = filter->q1 * filter->q3;
        q2q2 = filter->q2 * filter->q2;
        q2q3 = filter->q2 * filter->q3;
        q3q3 = filter->q3 * filter->q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * filter->q3 + _2q0mz * filter->q2 + mx * q1q1 + _2q1 * my * filter->q2 + _2q1 * mz * filter->q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * filter->q3 + my * q0q0 - _2q0mz * filter->q1 + _2q1mx * filter->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * filter->q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * filter->q2 + _2q0my * filter->q1 + mz * q0q0 + _2q1mx * filter->q3 - mz * q1q1 + _2q2 * my * filter->q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * filter->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * filter->q3 + _2bz * filter->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * filter->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * filter->q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * filter->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * filter->q2 + _2bz * filter->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * filter->q3 - _4bz * filter->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * filter->q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * filter->q2 - _2bz * filter->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * filter->q1 + _2bz * filter->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * filter->q0 - _4bz * filter->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * filter->q3 + _2bz * filter->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * filter->q0 + _2bz * filter->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * filter->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalize the corrective step
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply new external quaternion rate of change
        qDot1 -= filter->beta * s0;
        qDot2 -= filter->beta * s1;
        qDot3 -= filter->beta * s2;
        qDot4 -= filter->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    filter->q0 += qDot1 * filter->sample_period;
    filter->q1 += qDot2 * filter->sample_period;
    filter->q2 += qDot3 * filter->sample_period;
    filter->q3 += qDot4 * filter->sample_period;

    // Normalize quaternion
    recipNorm = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

// (E) Kalman filter
