#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "driver/spi_master.h"
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
    ImuArray() {
        spi_bus_config_t buscfg1 = {
            .mosi_io_num = CONFIG_GPIO_MOSI1,
            .miso_io_num = CONFIG_GPIO_MISO1,
            .sclk_io_num = CONFIG_GPIO_SCLK1,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };

        spi_bus_config_t buscfg2 = {
            .mosi_io_num = CONFIG_GPIO_MOSI2,
            .miso_io_num = CONFIG_GPIO_MISO2,
            .sclk_io_num = CONFIG_GPIO_SCLK2,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };

        spi_device_interface_config_t devcfg1 = {
            .mode = 0,                              // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
            .spics_io_num = CONFIG_GPIO_CS,             // CS pin
            .queue_size = 1,                        // We want to be able to queue 1 transactions at a time
        };
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

