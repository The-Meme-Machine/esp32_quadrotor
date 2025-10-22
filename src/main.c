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
// Avoid floating point operations in ISR
void control_loop()
{
    static uint64_t loop_count = 0;

    crsf_channels_t commands = {0};

    while (1)
    {
        // Run the loop when new IMU data is ready
        // Could also use a semaphore here but its slower for the critical loop
        if (imu_drdy_flag)
        {
            imu_drdy_flag = false;

            // uint64_t start = esp_timer_get_time();

            IMU_packet *imu_data = read_IMU();

            // 32bit float for FPU acceleration - Avoid floating point division
            // Convert to physical units
            float xl_x = imu_data->xl_x * ACCEL_SENS; // mdps
            float xl_y = imu_data->xl_y * ACCEL_SENS;
            float xl_z = imu_data->xl_z * ACCEL_SENS;
            float g_x = imu_data->g_x * RATE_SENS; // mg
            float g_y = imu_data->g_y * RATE_SENS;
            float g_z = imu_data->g_z * RATE_SENS;

            // Send telemetry at 10Hz
            if (loop_count % 80 == 0)
            {
                // Add data to telemetry queue
            }

            // Filters

            // Check for new commands

            // Flight modes
            switch (flight_mode)
            {
            case FLIGHT_MODE_ANGLE:
                break;

            case FLIGHT_MODE_RATE:
                break;

            case FLIGHT_MODE_FF:
                break;

            case FLIGHT_MODE_ALT_HOLD:
                break;

            case FLIGHT_MODE_POS_HOLD:
                break;

            default:
                break;
            }

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
                    uint16_t throttle_setting = clamp_throttle_limit((loop_count + 78000) / 800, throttle_limit);
                    throttles[0] = throttle_setting;
                    throttles[1] = throttle_setting;
                    throttles[2] = throttle_setting;
                    throttles[3] = throttle_setting;

                    // Log throttle values for debugging
                    if (loop_count % 5000 == 0)
                    {
                        printf("Throttle: %d\n", throttle_setting);
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

            // uint64_t end = esp_timer_get_time();
            // printf("Control loop took %llu us... \n", (end - start));
        }
        // Absent fresh IMU data,
        else
        {
            // Fetch radio commands
            // 50Hz update rate nominal
            CRSF_receive_channels(&commands);

            if (loop_count % 5000 == 0)
            {
                printf("Channel 3 (Throttle): %d\n", commands.ch3);
                printf("Channel 5 (Armed): %d\n", commands.ch5);
            }

            // for (uint8_t channel = 1; channel < 17; channel++)
            // {
            //     printf(">Channel %d: %d\n", channel, commands.);
            // }
        }
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

    // Setting up WiFi telemetry
    ESP_LOGI(TAG, "Setting up WiFi telemetry dashboard...");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // wifi_init_softap();
    // server = start_webserver();

    // if (server)
    // {
    //     xTaskCreate(telemetry_sender_task, "telemetry_sender", 4096, NULL, 5, NULL);
    // }

    // Setting up CRSF reciever
    ESP_LOGI(TAG, "Setting up CRSF reciever...");
    crsf_config_t crsf_config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 11,
        .rx_pin = 10};
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