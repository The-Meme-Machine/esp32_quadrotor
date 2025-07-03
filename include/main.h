#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_ipc.h>
#include <esp_dsp.h>
#include <dshot.h>
#include <ws2812.h>
#include <IMU.h>

#define NUM_MOTORS 4
#define TELEMETRY false // toggle telemetry to false
// #define LOG_LOCAL_LEVEL ESP_LOG_INFO // log verbosity

// GPIO wiring
gpio_num_t motor_pins[NUM_MOTORS] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};
gpio_num_t esc_c_pin = GPIO_NUM_5;
gpio_num_t esc_telemetry_pin = GPIO_NUM_6;
gpio_num_t IMU_clock_pin = GPIO_NUM_7; // yellow
gpio_num_t IMU_data_pin = GPIO_NUM_8;  // blue
gpio_num_t IMU_int_pin = GPIO_NUM_9;   // IMU interrupt
gpio_num_t WS2812_pin = GPIO_NUM_21;   // Integral WS2812

typedef struct
{
    int32_t g_x;
    int32_t g_y;
    int32_t g_z;
    int32_t xl_x;
    int32_t xl_y;
    int32_t xl_z;
    uint16_t *throttles;
} log_data;

// Throttle values
uint16_t throttles[NUM_MOTORS] = {DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN};
