#include "config.hpp"
#include "imus.hpp"
#include <stdio.h>
#include "wifi.hpp"
#include "middleware.hpp"
#include "udp.hpp"
#include "nvs.hpp"

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    Config::init();
    //esp_log_level_set("NVS", ESP_LOG_NONE);
    //wifi_ap_record_t info;
    // Initialize NVS
    //esp_err_t ret = nvs_flash_init();
    //if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    //{
    //    ret = nvs_flash_erase();
    //    ESP_LOGI(TAG, "nvs_flash_erase: 0x%04x", ret);
    //    ret = nvs_flash_init();
    //    ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);
    //}
    //ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);
    //wifi_init_sta();

    //Udp::init();

    MiddleWare::init();
    ImuArray::init();
    
    
    //while (true)
    //{
        //ESP_LOGI(TAG, "Heap free size:%d", xPortGetFreeHeapSize());
        //ret = esp_wifi_sta_get_ap_info(&info);
        //ESP_LOGI(TAG, "esp_wifi_sta_get_ap_info: 0x%04x", ret);
        //if (ret == 0){}
            //ESP_LOGI(TAG, "SSID: %s", info.ssid);
        //else
        //{
            //wifi_init_sta();
        //}
        //vTaskDelay(5000);
    //}
    return;
}