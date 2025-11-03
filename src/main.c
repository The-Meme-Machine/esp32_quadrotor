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

// IMU data ready interrupt handler
// Control loop runs on DRDY flag toggle
void IRAM_ATTR drdy_intr_flag(void *args)
{
    imu_drdy_flag = true;
    // int_count++;
}

// Stability control loop (critical)
void control_loop()
{
    static uint64_t loop_count = 0;

    // Throttle values
    uint16_t throttles[NUM_MOTORS] = {DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN};

    // Radio commands
    crsf_channels_t commands = {0};

    // Madgwick estimation
    Madgwick_est madgwick_filter;
    Madgwick_init(&madgwick_filter, IMU_SAMPLE_FREQUENCY, 0.13f);

    float ctrl_inputs[4] = {0};            // Thrust, Roll, Pitch, Yaw
    float motor_outputs[NUM_MOTORS] = {0}; // Motor outputs after mixing

    pid_controller_t roll_rate_pid;
    pid_controller_t pitch_rate_pid;
    pid_controller_t yaw_rate_pid;

    // Initialize rate PIDs
    pid_init(&roll_rate_pid, 0.15f, 0.0f, 0.002f, -1.0f, 1.0f);
    pid_init(&pitch_rate_pid, 0.15f, 0.0f, 0.002f, -1.0f, 1.0f);
    pid_init(&yaw_rate_pid, 0.2f, 0.0f, 0.003f, -1.0f, 1.0f);

    static float prev_roll = 0.0f;
    static float prev_pitch = 0.0f;
    static float prev_yaw = 0.0f;

    while (1)
    {
        // Run the loop when new IMU data is ready
        // Could replace with semaphore if more CPU cycles are needed
        if (imu_drdy_flag)
        {
            imu_drdy_flag = false;

            // Measure loop time
            int64_t start = esp_timer_get_time();

            // Fetch IMU data
            IMU_packet *imu_data = read_IMU();

            // Fetch magnetometer data
            mag_packet *mag_data = read_mag();

            // Fetch radio commands
            CRSF_receive_channels(&commands);

            // 32bit float for FPU acceleration - Avoid floating point division
            // Convert to physical units
            float xl_x = imu_data->xl_x * ACCEL_SENS; // mdps
            float xl_y = imu_data->xl_y * ACCEL_SENS;
            float xl_z = imu_data->xl_z * ACCEL_SENS;
            float g_x = zero_value_clamp(imu_data->g_x * RATE_SENS, RATE_ZERO); // mg
            float g_y = zero_value_clamp(imu_data->g_y * RATE_SENS, RATE_ZERO);
            float g_z = zero_value_clamp(imu_data->g_z * RATE_SENS, RATE_ZERO);
            float m_x = mag_data->m_x * MAG_SENS; // gauss
            float m_y = mag_data->m_y * MAG_SENS; // gauss
            float m_z = mag_data->m_z * MAG_SENS; // gauss

            // Filters
            // Madgwick update
            Madgwick_update(&madgwick_filter, g_x, g_y, g_z, xl_x, xl_y, xl_z, m_x, m_y, m_z);
            Madgwick_computeEulerAngles(&madgwick_filter);

            // Flight modes (Channel 6 switch)
            flight_mode = (flight_mode_t)reciever_3pos_switch(commands.ch6, 100);
            switch (flight_mode) // calculate motor commands based on flight mode
            {
            case FLIGHT_MODE_ANGLE:
                ctrl_inputs[0] = reciever_stick_normalized_absolute(commands.ch3, 160);                                            // Throttle
                ctrl_inputs[1] = pid_compute(&roll_rate_pid, reciever_stick_normalized(commands.ch1, 50), madgwick_filter.roll);   // Roll
                ctrl_inputs[2] = pid_compute(&pitch_rate_pid, reciever_stick_normalized(commands.ch2, 50), madgwick_filter.pitch); // Pitch
                ctrl_inputs[3] = pid_compute(&yaw_rate_pid, reciever_stick_normalized(commands.ch4, 50), madgwick_filter.yaw);     // Yaw

                break;

            case FLIGHT_MODE_HYBRID:
                break;

            case FLIGHT_MODE_RATE:
                ctrl_inputs[0] = reciever_stick_normalized_absolute(commands.ch3, 160);                                                         // Throttle
                ctrl_inputs[1] = pid_compute(&roll_rate_pid, reciever_stick_normalized(commands.ch1, 50), madgwick_filter.roll - prev_roll);    // Roll
                ctrl_inputs[2] = pid_compute(&pitch_rate_pid, reciever_stick_normalized(commands.ch2, 50), madgwick_filter.pitch - prev_pitch); // Pitch
                ctrl_inputs[3] = pid_compute(&yaw_rate_pid, reciever_stick_normalized(commands.ch4, 50), madgwick_filter.yaw - prev_yaw);       // Yaw

                break;

                // case FLIGHT_MODE_FF:
                //     break;

                // case FLIGHT_MODE_ALT_HOLD:
                //     break;

                // case FLIGHT_MODE_POS_HOLD:
                //     break;

            default:
                break;
            }

            // Save Previous angular positions for rate calculations
            prev_roll = madgwick_filter.roll;
            prev_pitch = madgwick_filter.pitch;
            prev_yaw = madgwick_filter.yaw;

            // Motor mixer
            // Optimized DSP matrix multiplication
            dspm_mult_f32(&mixer_matrix[0], &ctrl_inputs[0], &motor_outputs[0], 4, 4, 1);
            // Scale and offset motors to min/max throttle
            scale_and_offset_motors(&motor_outputs[0], ((float)(DSHOT_THROTTLE_MIN)) / 2000.0f, 1.0f);

            // check if armed
            motor_armed_flag = reciever_switch(commands.ch5, 1500);
            if (motor_armed_flag)
            {
                // Send zero throttle to arm motors (BLHeli S requirement)
                if (loop_count < 10000)
                {
                    throttles[0] = 0;
                    throttles[1] = 0;
                    throttles[2] = 0;
                    throttles[3] = 0;
                }
                // Send actual throttle once armed
                else
                {
                    for (int i = 0; i < NUM_MOTORS; i++)
                    {
                        //   motor_outputs[i] = (motor_outputs[i] > 1.0f) ? 1.0f : motor_outputs[i];
                        throttles[i] = (uint16_t)(motor_outputs[i] * 2000.0f); // Scale to DSHOT range
                    }
                }

                send_dshot_frame(&throttles, TELEMETRY);
            }
            else
            {
                // Ensure motors are off
                throttles[0] = 0;
                throttles[1] = 0;
                throttles[2] = 0;
                throttles[3] = 0;
                send_dshot_frame(&throttles, TELEMETRY);
            }

            loop_count++;

            // Measure loop time
            int64_t end = esp_timer_get_time();
            // printf("Control loop took %llu us... \n", (end - start));

            // Send telemetry at 1Hz (Can go up to 10Hz if needed)
            if (loop_count % 400 == 0)
            {
                // Add data to telemetry queue
                telemetry_data_t new_telem_data = {
                    .armed = motor_armed_flag,
                    .flight_mode = (uint8_t)flight_mode,
                    .loop_time_us = end - start,
                    .g_x = g_x,
                    .g_y = g_y,
                    .g_z = g_z,
                    .xl_x = xl_x,
                    .xl_y = xl_y,
                    .xl_z = xl_z,
                    .m_x = m_x,
                    .m_y = m_y,
                    .m_z = m_z,
                    .thr_1 = throttles[0],
                    .thr_2 = throttles[1],
                    .thr_3 = throttles[2],
                    .thr_4 = throttles[3],
                    .roll = ctrl_inputs[1],
                    .pitch = ctrl_inputs[2],
                    .yaw = ctrl_inputs[3],
                    .throttle = ctrl_inputs[0],
                    .ch1 = commands.ch1,
                    .ch2 = commands.ch2,
                    .ch3 = commands.ch3,
                    .ch4 = commands.ch4,
                    .ch5 = commands.ch5,
                    .ch6 = commands.ch6,
                    .ch7 = commands.ch7,
                    .ch8 = commands.ch8,
                    .est_roll = madgwick_filter.roll,
                    .est_pitch = madgwick_filter.pitch,
                    .est_yaw = madgwick_filter.yaw};

                if (xQueueSend(telemetry_tx_queue, &new_telem_data, (TickType_t)0) != pdTRUE)
                {
                    ESP_LOGI(TAG, "Telemetry queue full. Dropping packet...");
                }
            }
        }
        // Absent fresh IMU data,
        else
        {
            // if (loop_count % 5000 == 0)
            // {
            //     printf("Channel 3 (Throttle): %d\n", commands.ch3);
            //     printf("Channel 5 (Armed): %d\n", commands.ch5);
            // }
        }
    }

    // Should never reach here
    vTaskDelete(NULL);
}

void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Program is running...");

    setup_rmt_channels(motor_pins);

    // Setting up WiFi telemetry
    ESP_LOGI(TAG, "Setting up WiFi telemetry dashboard...");
    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    start_webserver();

    // if (server)
    // {
    //     xTaskCreate(telemetry_sender_task, "telemetry_sender", 4096, NULL, 5, NULL);
    // }

    // Setting up CRSF receiver
    ESP_LOGI(TAG, "Setting up CRSF receiver...");
    crsf_config_t crsf_config = {
        .uart_num = UART_NUM_1,
        .tx_pin = CRSF_UART_TX_PIN,
        .rx_pin = CRSF_UART_RX_PIN};
    CRSF_init(&crsf_config);

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

    // Create telemetry server task
    ESP_LOGI(TAG, "Starting telemetry task...");
    telemetry_tx_queue = xQueueCreate(10, sizeof(telemetry_data_t));
    wifi_telemetry_queue_init(&telemetry_tx_queue);
    xTaskCreatePinnedToCore(
        websocket_telemetry_task,
        "telemetry_server",
        4096,
        NULL,
        8,
        NULL,
        0 // Core 0
    );

    ESP_LOGI(TAG, "Finished setup.");

    // Pin main control loop to second core
    // Leave first open for navigation or communication
    ESP_LOGI(TAG, "Starting control loop...");
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