#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <dshot.h>
#include <driver/gpio.h>

// #define NUM_MOTORS 4
#define TELEMETRY false // toggle telemetry to false

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

gpio_num_t motor_pins[NUM_MOTORS] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};

uint16_t throttles[NUM_MOTORS] = {DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN};

static const char *TAG = "MAIN";

void command_motors(uint16_t throttle)
{
    throttles[0] = throttle;
    // throttles[1] = throttle;
    // throttles[2] = throttle;
    // throttles[3] = throttle;

    send_dshot_frame(&throttles, TELEMETRY);
    // vTaskDelay(pdMS_TO_TICKS(delay));
}

// Run the main control loop every 605us (~1.66kHZ)
void IRAM_ATTR control_loop()
{
    static uint32_t loop_count = 0;
    // ESP_LOGI(TAG, "Control loop called");
    uint64_t start = esp_timer_get_time();

    if (loop_count < 1250)
    {
        command_motors(0);
    }
    else if (loop_count >= 1250 && loop_count < 1500)
    {
        command_motors(100);
    }
    else if (loop_count >= 1500 && loop_count < 1900)
    {
        command_motors(200);
    }
    else if (loop_count >= 1900 && loop_count < 2300)
    {
        command_motors(300);
    }
    else if (loop_count >= 2300 && loop_count < 2700)
    {
        command_motors(500);
    }
    else
    {
        command_motors(550);
    }

    loop_count++;

    uint64_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "Control loop took %ull ms", ((end - start) / 1000));
    printf("Control loop took %llu us... \n", (end - start));
}

void app_main()
{
    setup_rmt_channels(motor_pins);

    // throttles[0] = throttle;
    throttles[1] = 0;
    throttles[2] = 0;
    throttles[3] = 0;

    // Do nothing for 3sec
    ESP_LOGI(TAG, "Program is running...");
    ESP_LOGI(TAG, "Waiting for 3 seconds.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    const esp_timer_create_args_t timer_args = {
        .callback = &control_loop,
        .name = "Main Control Loop",
        .dispatch_method = ESP_TIMER_TASK};

    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 1200); // 1200 microseconds - 800Hz
};