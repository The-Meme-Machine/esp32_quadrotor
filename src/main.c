#include <main.h>

static const char *TAG = "MAIN";

// Run the main control loop every 605us (~1.66kHZ)
void IRAM_ATTR control_loop()
{
    static uint32_t loop_count = 0;
    // ESP_LOGI(TAG, "Control loop called");
    uint64_t start = esp_timer_get_time();

    if (loop_count < 1250)
    {
        throttles[0] = 0;
        throttles[1] = 0;
        throttles[2] = 0;
        throttles[3] = 0;
    }
    else if (loop_count >= 1250 && loop_count < 1500)
    {
        throttles[0] = 100;
        throttles[1] = 100;
        throttles[2] = 100;
        throttles[3] = 100;
    }
    else if (loop_count >= 1500 && loop_count < 1900)
    {
        throttles[0] = 200;
        throttles[1] = 200;
        throttles[2] = 200;
        throttles[3] = 200;
    }
    else if (loop_count >= 1900 && loop_count < 2300)
    {
        throttles[0] = 300;
        throttles[1] = 300;
        throttles[2] = 300;
        throttles[3] = 300;
    }
    else if (loop_count >= 2300 && loop_count < 2700)
    {
        throttles[0] = 400;
        throttles[1] = 400;
        throttles[2] = 400;
        throttles[3] = 400;
    }
    else
    {
        throttles[0] = 200;
        throttles[1] = 200;
        throttles[2] = 200;
        throttles[3] = 200;
    }

    send_dshot_frame(&throttles, TELEMETRY);
    loop_count++;

    uint64_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "Control loop took %ull ms", ((end - start) / 1000));
    printf("Control loop took %llu us... \n", (end - start));
}

void app_main()
{
    setup_rmt_channels(motor_pins);

    throttles[0] = 0;
    throttles[1] = 0;
    throttles[2] = 0;
    throttles[3] = 0;

    // Do nothing for 3sec
    ESP_LOGI(TAG, "Program is running...");

    // Call control loop on a timer
    // In the future, call control loop on I2C interrupt
    const esp_timer_create_args_t timer_args = {
        .callback = &control_loop,
        .name = "Main Control Loop",
        .dispatch_method = ESP_TIMER_TASK};

    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 1200); // 1200 microseconds - 800Hz
};