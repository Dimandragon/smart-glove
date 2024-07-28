#include "udp.hpp"
#include <iostream>

SmallMutex Udp::mutex = SmallMutex();
std::queue<std::pair<void *, size_t>> Udp::sending_data = {};
int Udp::sockfd;
struct sockaddr_in Udp::servaddr;
struct sockaddr_in Udp::cliaddr;

void Udp::udp_task(void *pvParameters)
{
    TickType_t xLastWakeTime;

    ESP_LOGI(TAG, "Create socket...\n");
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0 ) {
        ESP_LOGE(TAG, "socket not created\n");
        vTaskDelete(NULL);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    //Заполнение информации о клиенте
    cliaddr.sin_family    = AF_INET; // IPv4
    cliaddr.sin_addr.s_addr = INADDR_ANY;
    cliaddr.sin_port = htons(CONFIG_CLIENT_PORT);
  //Свяжем сокет с адресом клиента
    if (bind(sockfd, (const struct sockaddr *)&cliaddr,  sizeof(struct sockaddr_in)) < 0 )
    {
        ESP_LOGE(TAG, "socket not binded\n");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "socket was binded\n");
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = inet_addr(CONFIG_SERVER_IP);
    servaddr.sin_port = htons(CONFIG_SERVER_PORT);
    xLastWakeTime = xTaskGetTickCount();
    for(short i=0;;i++)
    {
      while(!Udp::sending_data.empty()){
        Udp::mutex.lock();
        std::pair<void *, size_t> sending_package = Udp::sending_data.front();
        Udp::sending_data.pop();
        //std::cout << "send package with size " <<  sending_package.second << "to "<< 
          //servaddr.sin_addr.s_addr << ": " << servaddr.sin_port << std::endl;
        //int a = 8;
        sendto(sockfd, sending_package.first, sending_package.second,  
          0, (struct sockaddr*) &servaddr,  sizeof(servaddr));
        Udp::mutex.unlock();
        operator delete[] (sending_package.first, sending_package.second);
      }
      vTaskDelayUntil( &xLastWakeTime, (TickType_t)500);
      //body
    }
    shutdown(sockfd, 0);
    close(sockfd);
    vTaskDelete(NULL);
}