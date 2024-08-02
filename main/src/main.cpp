#include "config.hpp"
#include "imus.hpp"
#include <stdio.h>
//#include "wifi.hpp"
#include "middleware.hpp"
//#include "udp.hpp"
#include "nvs.hpp"
#include "new_wifi.hpp"

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    Config::init();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        ESP_LOGI("NVS", "nvs_flash_erase: 0x%04x", ret);
        ret = nvs_flash_init();
        ESP_LOGI("NVS", "nvs_flash_init: 0x%04x", ret);
    }
    ESP_LOGI("NVS", "nvs_flash_init: 0x%04x", ret);
    Wifi::example_wifi_connect();
    //esp_log_level_set("NVS", ESP_LOG_NONE);
    //wifi_ap_record_t info;
    //wifi_init_sta();

    MiddleWare::init();
    ImuArray::init();
    
    return;
}