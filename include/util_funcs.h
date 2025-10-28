
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

// Log IMU and throttle data for teleplot
void logging_func(void *args)
{
    log_data *data = (log_data *)args;
    // for teleplot
    printf(">g_x:%li\n>g_y:%li\n>g_z:%li\n>xl_x:%li\n>xl_y:%li\n>xl_z:%li\n>thr_1:%d>thr_2:%d>thr_3:%d>thr_4:%d",
           data->g_x, data->g_y, data->g_z,
           data->xl_x, data->xl_y, data->xl_z,
           data->throttles[0], data->throttles[1], data->throttles[2], data->throttles[3]);
}

// Print vector for debugging
void print_vector(const char *name, const float *vector, int len)
{
    printf("  %s: [", name);
    for (int i = 0; i < len; i++)
    {
        printf(" %.2f", vector[i]);
        if (i < len - 1)
            printf(",");
    }
    printf(" ]\n");
}

// Reciever boolean (switch)
static inline bool reciever_switch(uint16_t channel_value, uint16_t threshold)
{
    return channel_value > threshold;
}

// Reciever 3 position switch
static inline uint8_t reciever_3pos_switch(uint16_t channel_value, uint16_t deadband)
{
    if (channel_value > (1000 + deadband))
        return 0; // Position 1
    else if (channel_value < (1000 - deadband))
        return 2; // Position 2
    return 1;     // Center position
}

// Reciever stick position normalized to -1 to 1
static inline float reciever_stick_normalized(uint16_t channel_value, uint16_t deadband)
{
    if (channel_value > (1000 + deadband))
        return (float)(channel_value - (1000 + deadband)) / (1800 - (1000 + deadband));
    else if (channel_value < (1000 - deadband))
        return (float)(channel_value - (1000 - deadband)) / ((1000 - deadband) - 200);
    return 0.0f;
}