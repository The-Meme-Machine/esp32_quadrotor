#include <main.h>
#include <util_funcs.h>

static const char *TAG = "MAIN";

// volatile uint32_t int_count = 0;

// Check interrupt pin frequency
// void check_int_pin()
// {
//     // ESP_LOGI(TAG, "INT1 level: %d", gpio_get_level(IMU_int_pin));

//     while (1)
//     {
//         uint32_t start = int_count;
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         uint32_t end = int_count;
//         ESP_LOGI(TAG, "Interrupt frequency: %lu Hz", end - start);
//     }
// }

void IRAM_ATTR drdy_intr_flag(void *args)
{
    imu_drdy_flag = true;
    // int_count++;
}

// Stability control loop (critical) IRAM_ATTR
void control_loop()
{
    static uint64_t loop_count = 0;

    while (1)
    {
        if (imu_drdy_flag)
        {
            imu_drdy_flag = false;

            // uint64_t start = esp_timer_get_time();

            IMU_packet *imu_data = read_IMU();

            // // Fixed point conversion for speed
            float xl_x = imu_data->xl_x * ACCEL_SENS; // mdps
            float xl_y = imu_data->xl_y * ACCEL_SENS;
            float xl_z = imu_data->xl_z * ACCEL_SENS;
            float g_x = imu_data->g_x * RATE_SENS; // mg
            float g_y = imu_data->g_y * RATE_SENS;
            float g_z = imu_data->g_z * RATE_SENS;

            // Filters
            // Apply zero level
            xl_x = apply_noise_floor(xl_x, ACCEL_ZERO);
            xl_y = apply_noise_floor(xl_y, ACCEL_ZERO);
            xl_z = apply_noise_floor(xl_z, ACCEL_ZERO);
            g_x = apply_noise_floor(g_x, RATE_ZERO);
            g_y = apply_noise_floor(g_y, RATE_ZERO);
            g_z = apply_noise_floor(g_z, RATE_ZERO);

            // Check for new

            // PID Loop
            // Axes are uncoupled, so three separate PIDs are adquate
            static float roll_integral = 0.0f, pitch_integral = 0.0f, yaw_integral = 0.0f;
            static float prev_roll_error = 0.0f, prev_pitch_error = 0.0f, prev_yaw_error = 0.0f;

            // Setpoints (desired angles/rates)
            float roll_setpoint = 0.0f;
            float pitch_setpoint = 0.0f;
            float yaw_setpoint = 0.0f;

            // Measured values (replace with actual sensor fusion output if available)
            float roll_measured = g_x;
            float pitch_measured = g_y;
            float yaw_measured = g_z;

            // PID gains (tune as needed)
            const float Kp = 0.8f, Ki = 0.01f, Kd = 0.05f;

            // Calculate errors
            float roll_error = roll_setpoint - roll_measured;
            float pitch_error = pitch_setpoint - pitch_measured;
            float yaw_error = yaw_setpoint - yaw_measured;

            // Integrate errors
            roll_integral += roll_error;
            pitch_integral += pitch_error;
            yaw_integral += yaw_error;

            // Derivative terms
            float roll_derivative = roll_error - prev_roll_error;
            float pitch_derivative = pitch_error - prev_pitch_error;
            float yaw_derivative = yaw_error - prev_yaw_error;

            // PID outputs
            float roll_output = Kp * roll_error + Ki * roll_integral + Kd * roll_derivative;
            float pitch_output = Kp * pitch_error + Ki * pitch_integral + Kd * pitch_derivative;
            float yaw_output = Kp * yaw_error + Ki * yaw_integral + Kd * yaw_derivative;

            // Save errors for next iteration
            prev_roll_error = roll_error;
            prev_pitch_error = pitch_error;
            prev_yaw_error = yaw_error;

            // Use roll_output, pitch_output, yaw_output in your mixer logic

            // Mixer

            // Feedforward

            if (motor_armed_flag)
            {

                // Test mode
                // Send zero throttle to arm motors
                if (loop_count < 10000)
                {
                    throttles[0] = 0;
                    throttles[1] = 0;
                    throttles[2] = 0;
                    throttles[3] = 0;
                }
                // Send actual throttle
                else
                {
                    uint16_t throttle_setting = clamp_throttle_limit((loop_count + 78000) / 800, throttle_max);
                    // uint16_t throttle_setting = 500;
                    throttles[0] = throttle_setting;
                    throttles[1] = throttle_setting;
                    throttles[2] = throttle_setting;
                    throttles[3] = throttle_setting;

                    if (loop_count % 5000 == 0)
                    {
                        printf("Throttle: %d\n", throttle_setting);
                    }
                }

                send_dshot_frame(&throttles, TELEMETRY);
                loop_count++;
            }

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
        }
        // else {

        // }
    }
}

void test_motor_func()
{
    static uint64_t loop_count = 0;
    if (loop_count < 2000)
    {
        throttles[0] = 0;
        throttles[1] = 0;
        throttles[2] = 0;
        throttles[3] = 0;
    }
    else
    {
        uint16_t throttle_setting = clamp_throttle_limit((loop_count - 5000 + 1600) / 800, 50);
        // uint16_t throttle_setting = 500;
        throttles[0] = throttle_setting;
        throttles[1] = throttle_setting;
        throttles[2] = throttle_setting;
        throttles[3] = throttle_setting;
    }

    send_dshot_frame(&throttles, TELEMETRY);
    loop_count++;
}

void app_main()
{
    // esp_log_level_set("*", ESP_LOG_INFO); // Set global log level
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Program is running...");

    setup_rmt_channels(motor_pins);

    ESP_LOGI(TAG, "Setting up GPIO...");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Trigger on rising edge
        // .intr_type = GPIO_INTR_HIGH_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << IMU_int_pin,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG, "Setting up IMU DRDY interrupt...");
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // 0 = default config
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_int_pin, drdy_intr_flag, (void *)IMU_int_pin));

    setup_imu_mag(IMU_data_pin, IMU_clock_pin);

    ESP_LOGI(TAG, "Finished setup.");

    // Pin main control loop to second core
    // Leave first open for navigation or communication
    // ESP_LOGI(TAG, "Created main control loop...");
    xTaskCreatePinnedToCore(
        control_loop,
        "ctrl_loop",
        4096,
        NULL,
        10,
        NULL,
        1 // Core 1
    );

    // Verify interrupt timing
    // const esp_timer_create_args_t timer_args = {
    //     .callback = &check_int_pin,
    //     .name = "Check Interrupt Pin",
    //     .dispatch_method = ESP_TIMER_TASK};

    // esp_timer_handle_t timer;
    // esp_timer_create(&timer_args, &timer);
    // esp_timer_start_periodic(timer, 1000000); // 1 sec
    // esp_timer_start_periodic(timer, 60);      // 1 sec

    // check_int_pin();

    // const esp_timer_create_args_t timer_args = {
    //     .callback = &test_motor_func,
    //     .name = "ctrl loop",
    //     .dispatch_method = ESP_TIMER_TASK};

    // esp_timer_handle_t timer;
    // esp_timer_create(&timer_args, &timer);
    // esp_timer_start_periodic(timer, 1200); // 1 sec

    motor_armed_flag = true;
};