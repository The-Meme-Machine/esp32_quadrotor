// Apply zero level
static inline int32_t apply_noise_floor(int32_t value, int32_t threshold)
{
    return (value > threshold || value < -threshold) ? value : 0;
}

xl_x = apply_noise_floor(xl_x, ACCEL_ZERO);
xl_y = apply_noise_floor(xl_y, ACCEL_ZERO);
xl_z = apply_noise_floor(xl_z, ACCEL_ZERO);
g_x = apply_noise_floor(g_x, RATE_ZERO);
g_y = apply_noise_floor(g_y, RATE_ZERO);
g_z = apply_noise_floor(g_z, RATE_ZERO);

// Madgwick estimation

// (E) Kalman filter
