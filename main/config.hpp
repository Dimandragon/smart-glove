#pragma once
#include "map"
#include "string"
#include "sdkconfig.h"

class Config{
    static Config & getInstance(){
        if (instance){
            return *instance;
        }
        else{
            instance = new Config();
            return *instance;
        }
    }

    static void init(){
        instance = new Config();
    }

private: 
    Config() {
        
    }                              
    // Private constructor
    ~Config() {}
    Config(const Config&);                 
    // Prevent copy-construction
    Config& operator=(const Config&);      
    // Prevent assignment
    static Config * instance;
};

Config * Config::instance = 0;
std::map<std::string, std::string> Config::data_string = {};
std::map<std::string, int> Config::data_int = {};