#include <main.h>

static const char *TAG = "MAIN";

static volatile bool imu_drdy_flag = false;
// volatile uint32_t int_count = 0;

void IRAM_ATTR drdy_intr_flag(void *args)
{
    imu_drdy_flag = true;
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

// Run the main control loop every 602us (~1.66kHZ)
// Unsubscribe the idle1 task watchdog
void control_loop()
{
    while (1)
    {
        if (imu_drdy_flag)
        {
            // int_count++;
            static uint32_t loop_count = 0;

            // uint64_t start = esp_timer_get_time();

            IMU_data imu_data = read_IMU();

            // Fixed point conversion for speed
            int32_t xl_x = imu_data.xl_x * ACCEL_SENS; // mdps
            int32_t xl_y = imu_data.xl_y * ACCEL_SENS;
            int32_t xl_z = imu_data.xl_z * ACCEL_SENS;
            int32_t g_x = imu_data.g_x * RATE_SENS; // mg
            int32_t g_y = imu_data.g_y * RATE_SENS;
            int32_t g_z = imu_data.g_z * RATE_SENS;

            // Apply zero level
            xl_x = apply_noise_floor(xl_x, ACCEL_ZERO);
            xl_y = apply_noise_floor(xl_y, ACCEL_ZERO);
            xl_z = apply_noise_floor(xl_z, ACCEL_ZERO);
            g_x = apply_noise_floor(g_x, RATE_ZERO);
            g_y = apply_noise_floor(g_y, RATE_ZERO);
            g_z = apply_noise_floor(g_z, RATE_ZERO);

            if (loop_count < 5000)
            {
                throttles[0] = 0;
                throttles[1] = 0;
                throttles[2] = 0;
                throttles[3] = 0;
            }
            else
            {
                throttles[0] = clamp_throttle_limit((loop_count - 5000) / 800, 50);
                throttles[1] = clamp_throttle_limit((loop_count - 5000) / 800, 50);
                throttles[2] = clamp_throttle_limit((loop_count - 5000) / 800, 50);
                throttles[3] = clamp_throttle_limit((loop_count - 5000) / 800, 50);
            }

            // if (loop_count < 2500)
            // {
            //     throttles[0] = 0;
            //     throttles[1] = 0;
            //     throttles[2] = 0;
            //     throttles[3] = 0;
            // }
            // else if (loop_count >= 2500 && loop_count < 4000)
            // {
            //     throttles[0] = 100;
            //     throttles[1] = 100;
            //     throttles[2] = 100;
            //     throttles[3] = 100;
            // }
            // else if (loop_count >= 4000 && loop_count < 6000)
            // {
            //     throttles[0] = 200;
            //     throttles[1] = 200;
            //     throttles[2] = 200;
            //     throttles[3] = 200;
            // }
            // else if (loop_count >= 6000 && loop_count < 8000)
            // {
            //     throttles[0] = 300;
            //     throttles[1] = 300;
            //     throttles[2] = 300;
            //     throttles[3] = 300;
            // }
            // else if (loop_count >= 8000 && loop_count < 10000)
            // {
            //     throttles[0] = 400;
            //     throttles[1] = 400;
            //     throttles[2] = 400;
            //     throttles[3] = 400;
            // }
            // else
            // {
            //     throttles[0] = 200;
            //     throttles[1] = 200;
            //     throttles[2] = 200;
            //     throttles[3] = 200;
            // }

            send_dshot_frame(&throttles, TELEMETRY);
            loop_count++;

            // log_data log_args = {
            //     .g_x = g_x,
            //     .g_y = g_y,
            //     .g_z = g_z,
            //     .xl_x = xl_x,
            //     .xl_y = xl_y,
            //     .xl_z = xl_z,
            //     .throttles = throttles,
            // };

            // esp_ipc_call(0, logging_func, &log_args);

            // uint64_t end = esp_timer_get_time();
            // printf("Control loop took %llu us... \n", (end - start));

            imu_drdy_flag = false;
        }
    }
}

// void check_int_pin()
// {
//     // ESP_LOGI(TAG, "INT1 level: %d", gpio_get_level(IMU_int_pin));

//     // check interrupt freq
//     while (1)
//     {
//         uint32_t start = int_count;
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         uint32_t end = int_count;
//         ESP_LOGI(TAG, "Interrupt frequency: %lu Hz", end - start);
//     }
// }

void app_main()
{
    // esp_log_level_set("*", ESP_LOG_INFO); // Set global log level
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Program is running...");

    setup_rmt_channels(motor_pins);
    // setup_imu_mag(IMU_data_pin, IMU_clock_pin);

    throttles[0] = 0;
    throttles[1] = 0;
    throttles[2] = 0;
    throttles[3] = 0;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Trigger on rising edge
        // .intr_type = GPIO_INTR_HIGH_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << IMU_int_pin,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // 0 = default config
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_int_pin, drdy_intr_flag, (void *)IMU_int_pin));

    setup_imu_mag(IMU_data_pin, IMU_clock_pin);

    // Pin main control loop to second core
    // Leave first open for navigation or communication
    ESP_LOGI(TAG, "Creating main ctrl loop interrupt...");
    xTaskCreatePinnedToCore(
        control_loop,
        "ctrl_loop",
        4096,
        NULL,
        10,
        NULL,
        1 // Core 1
    );

    ESP_LOGI(TAG, "Finished setup.");

    // const esp_timer_create_args_t timer_args = {
    //     .callback = &check_int_pin,
    //     .name = "Check Interrupt Pin",
    //     .dispatch_method = ESP_TIMER_TASK};

    // esp_timer_handle_t timer;
    // esp_timer_create(&timer_args, &timer);
    // esp_timer_start_periodic(timer, 1000000); // 1 sec
    // esp_timer_start_periodic(timer, 60); // 1 sec

    // check_int_pin();
};