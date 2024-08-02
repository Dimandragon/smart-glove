#include <stdio.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_event.h"
#include <cJSON.h>

class WebSockets{
    static constexpr char *TAG = "websocket";
    static TimerHandle_t shutdown_signal_timer;
    static esp_websocket_client_handle_t client;
    static esp_websocket_client_config_t websocket_cfg;

    static void log_error_if_nonzero(const char *message, int error_code)
    {
        if (error_code != 0) {
            ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
        }
    }

    static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
    {
        esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
        switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
            if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
            }
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
            ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
            if (data->op_code == 0x2) { // Opcode 0x2 indicates binary data
                ESP_LOG_BUFFER_HEX("Received binary data", data->data_ptr, data->data_len);
            } else if (data->op_code == 0x08 && data->data_len == 2) {
                ESP_LOGW(TAG, "Received closed message with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
            } else {
                ESP_LOGW(TAG, "Received=%.*s\n\n", data->data_len, (char *)data->data_ptr);
            }

            ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

            xTimerReset(shutdown_signal_timer, portMAX_DELAY);
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
            log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
            if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
            }
            break;
        }
    }

    static void websocket_app_start(void)
    {
        websocket_cfg.uri = CONFIG_WEBSOCKET_URI;

        websocket_cfg.skip_cert_common_name_check = true;

        ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

        client = esp_websocket_client_init(&websocket_cfg);
        esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

        esp_websocket_client_start(client);
    }

public:
    static void Init(){
        ESP_LOGI(TAG, "[APP] Startup..");
        ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
        ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
        esp_log_level_set("*", ESP_LOG_INFO);
        esp_log_level_set("websocket_client", ESP_LOG_DEBUG);
        esp_log_level_set("transport_ws", ESP_LOG_DEBUG);
        esp_log_level_set("trans_tcp", ESP_LOG_DEBUG);

        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
         * Read "Establishing Wi-Fi or Ethernet Connection" section in
         * examples/protocols/README.md for more information about this function.
         */

        websocket_app_start();
    }
    static void deinit(){
        esp_websocket_client_close(client, portMAX_DELAY);
        ESP_LOGI(TAG, "Websocket Stopped");
        esp_websocket_client_destroy(client);
    }

    static void sendData(const char * data, size_t len){
        esp_websocket_client_send_bin(client, data, len, 0);
    }
};


esp_websocket_client_handle_t WebSockets::client;
esp_websocket_client_config_t WebSockets::websocket_cfg;