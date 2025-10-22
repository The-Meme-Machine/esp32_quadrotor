#include <wifi.h>

static const char *TAG = "WIFI_TELEMETRY";
// static httpd_handle_t server = NULL;

// // --- HTTP & WebSocket Handlers (Internal to this component) ---

// // HTTP GET handler for serving the main page
// static esp_err_t http_get_handler(httpd_req_t *req)
// {
//     extern const unsigned char index_html_start[] asm("_binary_index_html_start");
//     extern const unsigned char index_html_end[] asm("_binary_index_html_end");
//     const size_t index_html_size = (index_html_end - index_html_start);

//     httpd_resp_set_type(req, "text/html");
//     httpd_resp_send(req, (const char *)index_html_start, index_html_size);
//     return ESP_OK;
// }

// // WebSocket handler
// static esp_err_t ws_handler(httpd_req_t *req)
// {
//     if (req->method == HTTP_GET)
//     {
//         ESP_LOGI(TAG, "Handshake done, new client connected");
//         return ESP_OK;
//     }
//     return ESP_OK;
// }

// // --- Server Initialization (Internal to this component) ---

// static httpd_handle_t start_webserver(void)
// {
//     httpd_handle_t server_handle = NULL;
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//     config.uri_match_fn = httpd_uri_match_wildcard;

//     if (httpd_start(&server_handle, &config) == ESP_OK)
//     {
//         httpd_uri_t root_uri = {.uri = "/", .method = HTTP_METHOD_GET, .handler = http_get_handler};
//         httpd_register_uri_handler(server_handle, &root_uri);

//         httpd_uri_t ws_uri = {.uri = "/ws", .method = HTTP_METHOD_GET, .handler = ws_handler, .is_websocket = true};
//         httpd_register_uri_handler(server_handle, &ws_uri);
//         ESP_LOGI(TAG, "Web server started");
//     }
//     else
//     {
//         ESP_LOGE(TAG, "Error starting web server!");
//     }
//     return server_handle;
// }

// // --- Public Functions (defined in wifi_manager.h) ---

// void wifi_manager_start(void)
// {
//     // Wi-Fi AP Initialization
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = "TelemetryAP", .password = "slicr2", .max_connection = 2, .authmode = WIFI_AUTH_WPA_WPA2_PSK},
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
//     ESP_LOGI(TAG, "Wi-Fi AP Initialized. SSID: %s", "DroneTelemetryAP");

//     // Start Web Server
//     server = start_webserver();
// }

// void telemetry_sender_task(void *pvParameters)
// {
//     while (1)
//     {
//         vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz

//         // --- GATHER YOUR REAL TELEMETRY DATA HERE ---
//         // float altitude = 20.5f + (float)(esp_random() % 100) / 100.0f;
//         // float roll = -5.2f + (float)(esp_random() % 100) / 50.0f;

//         // Create JSON payload
//         // cJSON *root = cJSON_CreateObject();
//         // cJSON_AddNumberToObject(root, "alt", altitude);
//         // cJSON_AddNumberToObject(root, "roll", roll);
//         // cJSON_AddNumberToObject(root, "pitch", 3.1);
//         // cJSON_AddNumberToObject(root, "yaw", 90.0);
//         // const char *json_string = cJSON_PrintUnformatted(root);

//         // // Send the data using the wifi_manager API
//         // wifi_manager_send_ws_data(json_string);

//         // // Clean up
//         // cJSON_Delete(root);
//         // free((void *)json_string);
//     }
// }

// void wifi_manager_send_ws_data(const char *data)
// {
//     if (server == NULL)
//     {
//         ESP_LOGE(TAG, "Web server not started, cannot send data");
//         return;
//     }

//     httpd_ws_frame_t ws_pkt;
//     memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
//     ws_pkt.payload = (uint8_t *)data;
//     ws_pkt.len = strlen(data);
//     ws_pkt.type = HTTPD_WS_TYPE_TEXT;

//     // This is a simple broadcast to all connected clients
//     httpd_ws_send_frame_to_all(server, &ws_pkt);
// }
