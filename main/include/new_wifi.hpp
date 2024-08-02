#pragma once

#include <string.h>
#include "protocol_examples_common.h"
#include "example_common_private.h"
#include "esp_log.h"

class Wifi{
    static constexpr char *TAG = "example_connect";
    static esp_netif_t *s_example_sta_netif;
    static SemaphoreHandle_t s_semph_get_ip_addrs;

    static int s_retry_num;

    static void example_handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
    {
        s_retry_num++;
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_err_t err = esp_wifi_connect();
        if (err == ESP_ERR_WIFI_NOT_STARTED) {
            return;
        }
        ESP_ERROR_CHECK(err);
    }

    static void example_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
    {
    }

    static void example_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
    {
        s_retry_num = 0;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        if (!example_is_our_netif(EXAMPLE_NETIF_DESC_STA, event->esp_netif)) {
            return;
        }
        ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
        if (s_semph_get_ip_addrs) {
            xSemaphoreGive(s_semph_get_ip_addrs);
        } else {
            ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
        }
    }

    static void example_wifi_start(void)
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
        // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
        esp_netif_config.if_desc = EXAMPLE_NETIF_DESC_STA;
        esp_netif_config.route_prio = 128;
        s_example_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
        esp_wifi_set_default_wifi_sta_handlers();

        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }


    static void example_wifi_stop(void)
    {
        esp_err_t err = esp_wifi_stop();
        if (err == ESP_ERR_WIFI_NOT_INIT) {
            return;
        }
        ESP_ERROR_CHECK(err);
        ESP_ERROR_CHECK(esp_wifi_deinit());
        ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_example_sta_netif));
        esp_netif_destroy(s_example_sta_netif);
        s_example_sta_netif = NULL;
    }


    static esp_err_t example_wifi_sta_do_connect(wifi_config_t wifi_config, bool wait)
    {
        if (wait) {
            s_semph_get_ip_addrs = xSemaphoreCreateBinary();
            if (s_semph_get_ip_addrs == NULL) {
                return ESP_ERR_NO_MEM;
            }
        }
        s_retry_num = 0;
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect, s_example_sta_netif));

        ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_err_t ret = esp_wifi_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
            return ret;
        }
        if (wait) {
            ESP_LOGI(TAG, "Waiting for IP(s)");
            xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
            if (s_retry_num > CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY) {
                return ESP_FAIL;
            }
        }
        return ESP_OK;
    }

    static esp_err_t example_wifi_sta_do_disconnect()
    {
        ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect));
        ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip));
        ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect));
        if (s_semph_get_ip_addrs) {
            vSemaphoreDelete(s_semph_get_ip_addrs);
        }
        return esp_wifi_disconnect();
    }

    static void example_wifi_shutdown(void)
    {
        example_wifi_sta_do_disconnect();
        example_wifi_stop();
    }
public:
    static esp_err_t example_wifi_connect(void)
    {
        ESP_LOGI(TAG, "Start example_connect.");
        example_wifi_start();
        
        
        wifi_config_t wifi_config = wifi_config_t{
            .sta = {
                .threshold = {
                    .authmode = WIFI_AUTH_WPA2_PSK,
                },
            },
        };

        std::string ssid = CONFIG_WIFI_SSID;
        std::copy(ssid.begin(), ssid.end(), std::begin(wifi_config.sta.ssid));
        std::string pass = CONFIG_WIFI_PASSWORD;
        std::copy(pass.begin(), pass.end(), std::begin(wifi_config.sta.password));

        return example_wifi_sta_do_connect(wifi_config, true);
    }

};

esp_netif_t * Wifi::s_example_sta_netif;
SemaphoreHandle_t Wifi::s_semph_get_ip_addrs;
int Wifi::s_retry_num;