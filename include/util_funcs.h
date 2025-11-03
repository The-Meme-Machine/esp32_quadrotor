
static inline int32_t apply_noise_floor(int32_t value, int32_t threshold)
{
    return (value > threshold || value < -threshold) ? value : 0;
}

static inline float zero_value_clamp(float value, float min)
{
    if (value < min)
        return 0.0f;
    return value;
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

float f_max(float a, float b)
{
    return (a > b) ? a : b;
}

float f_min(float a, float b)
{
    return (a < b) ? a : b;
}

void scale_and_offset_motors(float *motor_outputs, float min_throttle, float max_throttle)
{
    // 1. Find min and max raw motor values from the input array
    float min_raw = f_min(f_min(motor_outputs[0], motor_outputs[1]), f_min(motor_outputs[2], motor_outputs[3]));
    float max_raw = f_max(f_max(motor_outputs[0], motor_outputs[1]), f_max(motor_outputs[2], motor_outputs[3]));

    // 2. Calculate range and check for saturation
    float raw_range = max_raw - min_raw;
    const float output_range = max_throttle - min_throttle;

    // 3. Handle saturation if necessary
    // This logic scales the mix down if the total range is too large.
    if (raw_range > output_range)
    {
        float scale = output_range / raw_range;
        // Find the average (or "throttle" component)
        float avg = (min_raw + max_raw) / 2.0f;
        // Scale each motor's deviation from the average
        motor_outputs[0] = avg + (motor_outputs[0] - avg) * scale;
        motor_outputs[1] = avg + (motor_outputs[1] - avg) * scale;
        motor_outputs[2] = avg + (motor_outputs[2] - avg) * scale;
        motor_outputs[3] = avg + (motor_outputs[3] - avg) * scale;

        // Recalculate min/max after scaling
        min_raw = f_min(f_min(motor_outputs[0], motor_outputs[1]), f_min(motor_outputs[2], motor_outputs[3]));
        max_raw = f_max(f_max(motor_outputs[0], motor_outputs[1]), f_max(motor_outputs[2], motor_outputs[3]));
    }

    // 4. Apply offset
    // Shift all motor values up or down to fit within the [min, max] range.
    // This handles the "no negative throttle" requirement dynamically.
    float offset = 0.0f;
    if (min_raw < min_throttle)
    {
        offset = min_throttle - min_raw;
    }
    else if (max_raw > max_throttle)
    {
        offset = max_throttle - max_raw;
    }

    // 5. Apply offset and final clamp (as a safeguard)
    motor_outputs[0] = fmax(min_throttle, fmin(max_throttle, motor_outputs[0] + offset));
    motor_outputs[1] = fmax(min_throttle, fmin(max_throttle, motor_outputs[1] + offset));
    motor_outputs[2] = fmax(min_throttle, fmin(max_throttle, motor_outputs[2] + offset));
    motor_outputs[3] = fmax(min_throttle, fmin(max_throttle, motor_outputs[3] + offset));
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

// Reciever stick position normalized to 0 to 1
static inline float reciever_stick_normalized_absolute(uint16_t channel_value, uint16_t deadband)
{
    return (channel_value - deadband) / 1600.0f;
}