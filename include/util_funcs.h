
static inline int32_t apply_noise_floor(int32_t value, int32_t threshold)
{
    return (value > threshold || value < -threshold) ? value : 0;
}

static inline uint16_t clamp_throttle(uint16_t input)
{
    if (input < DSHOT_THROTTLE_MIN)
        return DSHOT_THROTTLE_MIN;
    if (input > DSHOT_THROTTLE_MAX / 2)
        return DSHOT_THROTTLE_MAX / 2;
    return input;
}

// cap is as percent of max throttle
static inline uint16_t clamp_throttle_limit(uint16_t input, uint8_t cap)
{
    if (input < DSHOT_THROTTLE_MIN)
        return DSHOT_THROTTLE_MIN;
    if (input > DSHOT_THROTTLE_MAX * cap / 100)
        return DSHOT_THROTTLE_MAX * cap / 100;
    return input;
}

void logging_func(void *args)
{
    log_data *data = (log_data *)args;
    // for teleplot
    printf(">g_x:%li\n>g_y:%li\n>g_z:%li\n>xl_x:%li\n>xl_y:%li\n>xl_z:%li\n>thr_1:%d>thr_2:%d>thr_3:%d>thr_4:%d",
           data->g_x, data->g_y, data->g_z,
           data->xl_x, data->xl_y, data->xl_z,
           data->throttles[0], data->throttles[1], data->throttles[2], data->throttles[3]);
}
