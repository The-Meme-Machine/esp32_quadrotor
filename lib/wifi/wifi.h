#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_http_server.h>
#include <cJSON.h> // For creating JSON data
#include <esp_log.h>
// #include <esp_spiffs.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define SSID "QuadTelemetryAP"
#define PASSWORD "slicr2002" // password must be at least 8 characters
#define WIFI_CHANNEL 1
#define MAX_CONNECTIONS 1

typedef struct
{
    // Arming status
    bool armed;

    // Flight mode
    uint8_t flight_mode;

    // Loop time
    int64_t loop_time_us;

    // IMU data
    float g_x;
    float g_y;
    float g_z;
    float xl_x;
    float xl_y;
    float xl_z;

    // Commanded throttle
    uint16_t thr_1;
    uint16_t thr_2;
    uint16_t thr_3;
    uint16_t thr_4;

    // Control loop data
    float roll;
    float pitch;
    float yaw;
    float throttle;

    // Radio channels
    uint16_t ch1; // Roll
    uint16_t ch2; // Pitch
    uint16_t ch3; // Throttle
    uint16_t ch4; // Yaw
    uint16_t ch5; // Arm switch
    uint16_t ch6; // Flight mode switch
    uint16_t ch7;
    uint16_t ch8; // Altitude Hold switch

} telemetry_data_t;

extern void wifi_init_softap();

extern void wifi_telemetry_queue_init(QueueHandle_t *telemetry_queue);

extern void start_webserver();

extern void stop_webserver();

extern void websocket_telemetry_task(void *pvParameters);

extern esp_err_t websocket_handler(httpd_req_t *req);

// /**
//  * @brief Initializes Wi-Fi AP, starts the web server, and enables WebSocket handling.
//  */
// void wifi_manager_start(void);

// /**
//  * @brief Sends a string of data to all connected WebSocket clients.
//  * @param data The null-terminated string to send.
//  */
// void wifi_manager_send_ws_data(const char *data);

// // typedef union {

// // } telem_packet_t;