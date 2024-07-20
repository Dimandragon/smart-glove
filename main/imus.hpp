#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "sdkconfig.h"
#include <cstdint>
#include <cstdlib>
#include "config.hpp"

class ImuArray{
    static ImuArray & getInstance(){
        if (instance){
            return *instance;
        }
        else{
            instance = new ImuArray();
            return *instance;
        }
    }

    static void init(){
        instance = new ImuArray();
    }

private: 
    static spi_device_handle_t spi1;
    static spi_device_handle_t spi2;
    static spi_device_handle_t spi3;
    static spi_device_handle_t spi4;
    static spi_device_handle_t spi5;
    static spi_device_handle_t spi6;

    ImuArray() {
        spi_bus_config_t buscfg1 = {
            .mosi_io_num = Config::GetFieldInt("mosi1_pin"),
            .miso_io_num = Config::GetFieldInt("miso1_pin"),
            .sclk_io_num = Config::GetFieldInt("sclk1_pin"),
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };

        spi_bus_config_t buscfg2 = {
            .mosi_io_num = Config::GetFieldInt("mosi2_pin"),
            .miso_io_num = Config::GetFieldInt("miso2_pin"),
            .sclk_io_num = Config::GetFieldInt("sclk2_pin"),
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };

        spi_device_interface_config_t devcfg1 = {
            .mode = 0,                              // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs1_pin"),             // CS pin
            .queue_size = 1,                        // We want to be able to queue 1 transactions at a time
        };
        spi_device_interface_config_t devcfg2 = {
            .mode = 0,                              
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs2_pin"),
            .queue_size = 1,                       
        };
        spi_device_interface_config_t devcfg3 = {
            .mode = 0,                              
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs3_pin"),
            .queue_size = 1,                        
        };
        spi_device_interface_config_t devcfg4 = {
            .mode = 0,                             
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs4_pin"),
            .queue_size = 1,                        
        };
        spi_device_interface_config_t devcfg5 = {
            .mode = 0,                              
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs5_pin"),             // CS pin
            .queue_size = 1,                        
        };
        spi_device_interface_config_t devcfg6 = {
            .mode = 0,                              
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs6_pin"),             // CS pin
            .queue_size = 1,                        
        };

        spi_host_device_t host1 = (spi_host_device_t)(SPI_HOST_MAX - 2);
        spi_host_device_t host2 = (spi_host_device_t)(SPI_HOST_MAX - 1);

        // Initialize the SPI bus
        int ret = spi_bus_initialize(host1, &buscfg1, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);

        ret = spi_bus_initialize(host2, &buscfg2, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);

        // Add device to bus
        ret = spi_bus_add_device(host1, &devcfg1, &spi1);
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host1, &devcfg2, &spi2);
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host1, &devcfg3, &spi3);
        ESP_ERROR_CHECK(ret);

        ret = spi_bus_add_device(host2, &devcfg4, &spi4);
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host2, &devcfg5, &spi5);
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host2, &devcfg6, &spi6);
        ESP_ERROR_CHECK(ret);
    }                              
    // Private constructor
    ~ImuArray() {}
    ImuArray(const ImuArray&);                 
    // Prevent copy-construction
    ImuArray& operator=(const ImuArray&);      
    // Prevent assignment
    static ImuArray * instance;
};

ImuArray * ImuArray::instance = 0;

