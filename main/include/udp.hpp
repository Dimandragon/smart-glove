#pragma once

#include <sys/param.h>
#include "small_mutex.hpp"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cstring"
#include "esp_system.h"
#include "esp_event.h"
 
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <queue>
#include <utility>


static const char *TAG = "udp";
//-------------------------------------------------------------
class Udp{
private:
    static SmallMutex mutex;
    static void udp_task(void *pvParameters);
    static std::queue<std::pair<void *, size_t>> sending_data;
    static int sockfd;
    static struct sockaddr_in servaddr;
    static struct sockaddr_in cliaddr;
    
public:
    static void init(){
        //ESP_ERROR_CHECK(example_connect());
        xTaskCreate(udp_task, "udp", 4096, NULL, 5, NULL);
    }

    static void sendStringAsync(std::string & data){
        mutex.lock();
        uint8_t * buffer = new uint8_t[data.size()];
        const void * source = (const void *)(data.data());
        std::memcpy(buffer, source, data.size());

        sending_data.push({(void*)buffer, data.size()});
        mutex.unlock();
    }
    
    static void sendString(std::string & data){
        mutex.lock();
        uint8_t * buffer = new uint8_t[data.size()];
        const void * source = (const void *)(data.data());
        std::memcpy(buffer, source, data.size());

        std::pair<void *, size_t> data_for_send = {(void*)buffer, data.size()};
        sendto(sockfd, buffer, data.size(),  
          0, (struct sockaddr*) &servaddr,  sizeof(servaddr));
        mutex.unlock();
    }
};



