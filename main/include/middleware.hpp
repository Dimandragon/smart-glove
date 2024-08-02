#pragma once

#include <cstddef>
#include <small_mutex.hpp>
#include <map>
#include <glove.pb.h>
#include "config.hpp"
#include "mbedtls/asn1.h"
#include <sys/socket.h>
#include <sstream>
#include <utility>
#include <inttypes.h>
#include <string>
#include <sstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <google/protobuf/map.h>
#include "imus.hpp"
#include <iostream>
#include <websockets.hpp>

#define PROTOBUF_NAMESPACE_ID google::protobuf

class MiddleWare
{
private:
    static size_t len;
    static SmallMutex mutex;
    static SmallMutex mutex2;

    MiddleWare(){}
    ~MiddleWare(){}
    MiddleWare(const MiddleWare &) {}
    MiddleWare &operator=(MiddleWare &) = default;
    static TaskHandle_t task_handle;

public:
    static void init(){
        ImuArray::setCallback(imusCB);
    }

    static void imusCB(ImuArray::ImuArrayDataPack data){
        glove::IMUPack sending_data;
        std::string cs_pin_name;
        int controller_id = Config::GetFieldInt("controller_id");
        for (int i = 0; i < data.size(); i++){
            glove::IMU * data_t = sending_data.add_data();
            data_t->set_xaccel(data[i].accel.x);
            data_t->set_yaccel(data[i].accel.y);
            data_t->set_zaccel(data[i].accel.y);
            data_t->set_xangle(data[i].gyro.x);
            data_t->set_yangle(data[i].gyro.y);
            data_t->set_zangle(data[i].gyro.z);
            data_t->set_board_number(controller_id);
            data_t->set_number((controller_id * 6) + i);
            cs_pin_name.clear();
            std::stringstream cs_pin_name_stream;
            cs_pin_name_stream << "cs" << i+1 << "_pin";

            std::stringstream pin_name;
            pin_name << "gpio" << Config::GetFieldInt(cs_pin_name_stream.str());
            data_t->set_connectionpin(pin_name.str()); //todo
            //std::cout << "i: " << i << " gyro.x: " << data[i].gyro.x 
            //    << " gyro.y: " << data[i].gyro.y 
            //    << " gyro.x: "<< data[i].gyro.z << std::endl;
        }
        std::string sending_string;
        sending_data.SerializeToString(&sending_string);
        WebSockets::sendData((const char*)sending_string.data(), sending_string.size());
        std::cout << "sended data to websockets" << std::endl;
    }
};