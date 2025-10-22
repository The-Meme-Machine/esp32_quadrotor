#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_http_server.h>
#include <cJSON.h> // For creating JSON data
#include <esp_log.h>
#include <esp_spiffs.h>

/**
 * @brief Initializes Wi-Fi AP, starts the web server, and enables WebSocket handling.
 */
void wifi_manager_start(void);

/**
 * @brief Sends a string of data to all connected WebSocket clients.
 * @param data The null-terminated string to send.
 */
void wifi_manager_send_ws_data(const char *data);

// typedef union {

// } telem_packet_t;