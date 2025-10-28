#include <wifi.h>

static const char *TAG = "WIFI_TELEMETRY";

static httpd_handle_t server = NULL;
static QueueHandle_t *telemetry_tx_queue = NULL;

const httpd_uri_t ws = {
    .uri = "/telemetry",
    .method = HTTP_GET,
    .handler = websocket_handler,
    .user_ctx = NULL,
    .is_websocket = true};

void wifi_init_softap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .password = PASSWORD, // password must be at least 8 characters
            .max_connection = MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP Initialized. SSID: %s PASS: %s", SSID, PASSWORD);
}

// Get pointer to telemetry queue
void wifi_telemetry_queue_init(QueueHandle_t *telemetry_queue)
{
    telemetry_tx_queue = telemetry_queue;
}

void start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting web server on port: %d...", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers...");
        httpd_register_uri_handler(server, &ws);
        ESP_LOGI(TAG, "Web server started.");
    }
}

void stop_webserver()
{
    if (server)
    {
        ESP_LOGI(TAG, "Stopping web server...");
        httpd_stop(server);
        server = NULL;
    }
}

esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "WebSocket handshake done, new client connected");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // Process received data if needed
    return ESP_OK;
}

void websocket_telemetry_task(void *pvParameters)
{
    telemetry_data_t data;

    ESP_LOGI(TAG, "Telemetry task started. Waiting for data...");

    for (;;)
    {
        // 1. Wait indefinitely for a struct to appear in the queue
        if (xQueueReceive(*telemetry_tx_queue, &data, portMAX_DELAY) == pdTRUE)
        {
            // 2. Check if the server is even running
            if (server == NULL)
            {
                ESP_LOGE(TAG, "Webserver handle not registered. Cannot send data.");
                continue; // Wait for the next item
            }

            // 3. Create the cJSON object from the struct
            cJSON *root = cJSON_CreateObject();
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to create cJSON root object");
                continue;
            }

            cJSON_AddBoolToObject(root, "armed", data.armed);
            cJSON_AddNumberToObject(root, "flight_mode", data.flight_mode);
            cJSON_AddNumberToObject(root, "loop_time_us", (double)data.loop_time_us); // Cast int64_t to double for cJSON
            cJSON_AddNumberToObject(root, "g_x", data.g_x);
            cJSON_AddNumberToObject(root, "g_y", data.g_y);
            cJSON_AddNumberToObject(root, "g_z", data.g_z);
            cJSON_AddNumberToObject(root, "xl_x", data.xl_x);
            cJSON_AddNumberToObject(root, "xl_y", data.xl_y);
            cJSON_AddNumberToObject(root, "xl_z", data.xl_z);
            cJSON_AddNumberToObject(root, "thr_1", data.thr_1);
            cJSON_AddNumberToObject(root, "thr_2", data.thr_2);
            cJSON_AddNumberToObject(root, "thr_3", data.thr_3);
            cJSON_AddNumberToObject(root, "thr_4", data.thr_4);
            cJSON_AddNumberToObject(root, "roll", data.roll);
            cJSON_AddNumberToObject(root, "pitch", data.pitch);
            cJSON_AddNumberToObject(root, "yaw", data.yaw);
            cJSON_AddNumberToObject(root, "throttle", data.throttle);
            cJSON_AddNumberToObject(root, "ch1", data.ch1);
            cJSON_AddNumberToObject(root, "ch2", data.ch2);
            cJSON_AddNumberToObject(root, "ch3", data.ch3);
            cJSON_AddNumberToObject(root, "ch4", data.ch4);
            cJSON_AddNumberToObject(root, "ch5", data.ch5);
            cJSON_AddNumberToObject(root, "ch6", data.ch6);
            cJSON_AddNumberToObject(root, "ch7", data.ch7);
            cJSON_AddNumberToObject(root, "ch8", data.ch8);

            // 4. Convert JSON object to a string
            char *json_string = cJSON_PrintUnformatted(root);
            if (json_string == NULL)
            {
                ESP_LOGE(TAG, "Failed to print cJSON to string");
                cJSON_Delete(root);
                continue;
            }

            // 5. Free the cJSON object (we only need the string now)
            cJSON_Delete(root);

            // 6. Prepare the WebSocket frame
            httpd_ws_frame_t ws_pkt;
            memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
            ws_pkt.payload = (uint8_t *)json_string;
            ws_pkt.len = strlen(json_string);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT; // We are sending text

            // 7. Get the list of all connected clients
            // We need to allocate an array for the client file descriptors (fds)
            size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP; // Max clients from menuconfig
            int *client_fds = calloc(max_clients, sizeof(int));
            if (client_fds == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for client FDs");
                free(json_string);
                continue;
            }

            size_t num_clients = max_clients;
            esp_err_t ret = httpd_get_client_list(server, &num_clients, client_fds);

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to get client list: %s", esp_err_to_name(ret));
            }
            else
            {
                // 8. Loop over all clients and send the frame
                for (size_t i = 0; i < num_clients; i++)
                {
                    int client_fd = client_fds[i];

                    // Check if this client is a WebSocket client
                    if (httpd_ws_get_fd_info(server, client_fd) == HTTPD_WS_CLIENT_WEBSOCKET)
                    {
                        // Send asynchronously. This is non-blocking and prevents
                        // this high-priority task from getting stuck on a slow client.
                        httpd_ws_send_frame_async(server, client_fd, &ws_pkt);
                    }
                }
            }

            // 9. Clean up
            free(client_fds);
            free(json_string);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(35)); // Avoid busy loop if no data
        }
    }

    // Should never be reached
    vTaskDelete(NULL);
}