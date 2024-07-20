#pragma once
#include "map"
#include "string"
#include "sdkconfig.h"
#include "small_mutex.hpp"

class Config{
public:
    static Config & getInstance(){
        mutex.lock();
        if (instance){
            mutex.unlock();
            return *instance;
        }
        else{
            instance = new Config();
            mutex.unlock();
            return *instance;
        }
    }

    static void init(){
        mutex.lock();
        if (!instance){
            instance = new Config();
        }
        else{
            delete instance;
            instance = new Config();
        }
        mutex.unlock();
    }

    static int GetFieldInt(const std::string & key){
        return data_int[key];
    }

    static std::string GetFieldString(const std::string & key){
        return data_string[key];
    }

private: 
    static SmallMutex mutex;
    static std::map<std::string, std::string> data_string;
    static std::map<std::string, int> data_int;

    Config() {
        data_int["miso1_pin"] = CONFIG_GPIO_MISO1;
        data_int["miso2_pin"] = CONFIG_GPIO_MISO2;
        data_int["mosi1_pin"] = CONFIG_GPIO_MOSI1;
        data_int["mosi2_pin"] = CONFIG_GPIO_MOSI2;
        data_int["sclk1_pin"] = CONFIG_GPIO_SCLK1;
        data_int["sclk2_pin"] = CONFIG_GPIO_SCLK2;
        data_int["cs1_pin"] = CONFIG_GPIO_CS1;
        data_int["cs2_pin"] = CONFIG_GPIO_CS2;
        data_int["cs3_pin"] = CONFIG_GPIO_CS3;
        data_int["cs4_pin"] = CONFIG_GPIO_CS4;
        data_int["cs5_pin"] = CONFIG_GPIO_CS5;
        data_int["cs6_pin"] = CONFIG_GPIO_CS6;
    }        

    // Private constructor
    ~Config() {}
    Config(const Config&);                 
    // Prevent copy-construction
    Config& operator=(const Config&);      
    // Prevent assignment
    static Config * instance;
};

SmallMutex Config::mutex;
Config * Config::instance = 0;
std::map<std::string, std::string> Config::data_string = {};
std::map<std::string, int> Config::data_int = {};