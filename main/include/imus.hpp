#pragma once

#include "esp_rom_sys.h"
#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "imus.hpp"
#include "sdkconfig.h"
#include <cstdint>
#include <cstdlib>
#include "config.hpp"
#include "esp_log.h"
#include <cstring>
#include "utils.hpp"

#include "bmi160.h"
#include "bmi160_defs.h"
#include "array"
#include <queue>
#include <functional>
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <esp_task_wdt.h>
#include <glove.pb.h>

static void IRAM_ATTR Imu1Interrupt(void * param){

    SemaphoreHandle_t drdy = *((SemaphoreHandle_t*)param);
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(drdy, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
    }
}

class ImuArray{
public:
    static SemaphoreHandle_t drdy;

    struct AccelGyroData {
        struct bmi160_sensor_data accel;
	    struct bmi160_sensor_data gyro;
    };

    using ImuArrayDataPack = std::array<AccelGyroData, 6>;

    static bool queueEmpty(){
        return messusrments_queue.empty();
    }

    static void pushMessurment(ImuArrayDataPack mess){
        messusrments_queue.push(mess);
    }

    static ImuArrayDataPack getMessurment(){
        auto res  = messusrments_queue.front();
        messusrments_queue.pop();
        return res;
    }

    static void setCallback(auto cb){
        callback = cb;
    }

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

    struct SpiBuss{
        spi_bus_config_t bus;
        std::array<spi_device_interface_config_t, 3> devices;
        std::array<spi_device_handle_t, 3> handles; 
    };

    static std::array<bmi160_dev, 6> sensors;
    //static std::array<bmi160_int_settg, 1> interrupts_configs;
private: 
    static std::queue<ImuArrayDataPack> messusrments_queue;
    static std::function<void(ImuArrayDataPack)> callback;

    //static std::array<FusionAhrs, 4> arhs_array;

    constexpr static uint16_t CMD_READ  = 0x01;
    constexpr static uint16_t CMD_WRITE = 0x00;

    static const char * TAG;

    static std::array<SpiBuss, 2> spi_config;

    static int8_t user_spi_read(spi_device_handle_t spi, uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
        esp_err_t ret;
        spi_transaction_t t;
        std::memset(&t, 0, sizeof(t));
    
        // Memory allocation with 32-bit alignment for more efficient transactions with DMA
        uint8_t *recv_data = (uint8_t*) heap_caps_aligned_alloc(32, len, MALLOC_CAP_DMA);
    
        if (recv_data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for buffers");
            return -1;
        }
    
        // Setting up transaction
        t.length = 8 * len;
        t.cmd = CMD_READ;
        t.addr = reg_addr;
        t.rx_buffer = recv_data;
    
        ret = spi_device_polling_transmit(spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
            heap_caps_free(recv_data);
            return -1;
        }
    
        // Copy the received data
        std::memcpy(read_data, recv_data, len);
    
        // Freeing allocated memory
        heap_caps_free(recv_data);
    
    #ifdef __DEBUG__
        ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
        print_byte_array("read_data", read_data, len);
    #endif
        return 0;
    }
    
    static int8_t user_spi_write(spi_device_handle_t spi, uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
        esp_err_t ret;
        spi_transaction_t t;
        std::memset(&t, 0, sizeof(t));
    
        // Memory allocation with 32-bit alignment for more efficient transactions with DMA
        uint8_t *send_data = (uint8_t*) heap_caps_aligned_alloc(32, len, MALLOC_CAP_DMA);
    
        if (send_data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for the buffer");
            return -1;
        }
        
    
        // Сopy information for sending
        std::memcpy(send_data, write_data, len);
    
        // Setting up transaction
        t.length = 8 * len;
        t.cmd = CMD_WRITE;
        t.addr = reg_addr;
        t.tx_buffer = send_data;
    
        ret = spi_device_polling_transmit(spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
            heap_caps_free(send_data);
            return -1;
        }
    
        // Freeing allocated memory
        heap_caps_free(send_data);
    
    #ifdef __DEBUG__
        ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
        print_byte_array("write_data", write_data, len);
    #endif
        return 0;
    }

    /*struct Angles{
        double x = 0;
        double y = 0;
        double z = 0;
    };*/

    static void ImusTask(void * pvParameters){
        //esp_task_wdt_init(30, false);
	    double accel_sensitivity = 16384.0;    // g
	    double gyro_sensitivity = 131.2 / 4.0; // Deg/Sec
        ImuArrayDataPack data;
        int64_t time = esp_timer_get_time();
        int64_t logging_time = esp_timer_get_time();
        for(;;){
            int64_t time_old = time;
            time = esp_timer_get_time();
            double dt = (time - time_old) / 1000000.;
            bool fl = false;
            if (time - logging_time > 500000){
                fl = true;
                logging_time = time;
            }
            for (int i = 0; i < sensors.size(); i++){
                int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &(data[i].accel), &(data[i].gyro), &sensors[i]);
                if (ret != BMI160_OK) {
                    ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
                }

                data[i].accel.x = (double)data[i].accel.x / accel_sensitivity;
                data[i].accel.y = (double)data[i].accel.y / accel_sensitivity;
                data[i].accel.z = (double)data[i].accel.z / accel_sensitivity;
                data[i].gyro.x = (double)data[i].gyro.x / gyro_sensitivity;
                data[i].gyro.y = (double)data[i].gyro.y / gyro_sensitivity;
                data[i].gyro.z = (double)data[i].gyro.z / gyro_sensitivity;

                //FusionVector gyro = {(float)data[i].gyro.x, (float)data[i].gyro.y, (float)data[i].gyro.z};
                //FusionVector accel = {(float)data[i].accel.x, (float)data[i].accel.y, (float)data[i].accel.z};

                //FusionAhrsUpdateNoMagnetometer(&(arhs_array[i]), gyro, accel, dt);

                //const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&(arhs_array[i])));

                if (fl){
                    //ESP_LOGI(TAG, "ANGLES number %d : Roll=%f, Pitch=%f, Yaw=%f", i, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
                    //ESP_LOGI(TAG, "GYRO: x=%f, y=%f, z=%f", (double)data[i].gyro.x, (double)data[i].gyro.y, (double)data[i].gyro.z); 
                    //logging_time = time;
                }
                #ifdef __DEBUG__
                ESP_LOGI(TAG, "RAW DATA:");
                ESP_LOGI(TAG, "ACCEL: x=%f, y=%f, z=%f", (double)data[i].accel.x, (double)data[i].accel.y, (double)data[i].accel.z);
                ESP_LOGI(TAG, "GYRO: x=%f, y=%f, z=%f", (double)data[i].gyro.x, (double)data[i].gyro.y, (double)data[i].gyro.z); 
                //todo
                #endif 
            }
            try{
                callback(data);
            }
            catch(const std::bad_function_call & err){
                messusrments_queue.push(data);
            }
            taskYIELD();
            //vTaskDelay(100);
        }
    }

    static void user_delay_ms(uint32_t period) {
	    esp_rom_delay_us(period*1000);
    };

    ImuArray() {
        spi_bus_config_t bus1 = {
            .mosi_io_num = Config::GetFieldInt("mosi1_pin"),
            .miso_io_num = Config::GetFieldInt("miso1_pin"),
            .sclk_io_num = Config::GetFieldInt("sclk1_pin"),
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };
        spi_bus_config_t bus2 = {
            .mosi_io_num = Config::GetFieldInt("mosi2_pin"),
            .miso_io_num = Config::GetFieldInt("miso2_pin"),
            .sclk_io_num = Config::GetFieldInt("sclk2_pin"),
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        };
        

        std::cout << "Mosi1: " << Config::GetFieldInt("mosi1_pin")
                << " Miso1: " << Config::GetFieldInt("miso1_pin")
                << " sclk1: " << Config::GetFieldInt("sclk1_pin")
                << " Mosi2: " << Config::GetFieldInt("mosi2_pin")
                << " Miso2: " << Config::GetFieldInt("miso2_pin")
                << " sclk2: " << Config::GetFieldInt("sclk2_pin") << std::endl;

        spi_device_interface_config_t device1 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs1_pin"), // CS pin
            .queue_size = 1,
        };
         
        spi_device_interface_config_t device2 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs2_pin"), // CS pin
            .queue_size = 1, 
        };
         

        spi_device_interface_config_t device3 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs3_pin"), // CS pin
            .queue_size = 1,
        };  

        spi_device_interface_config_t device4 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs4_pin"), // CS pin
            .queue_size = 1,  
        };

        spi_device_interface_config_t device5 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs5_pin"), // CS pin
            .queue_size = 1,  
        };

        spi_device_interface_config_t device6 = {
            .command_bits = 1,                      // R/W
            .address_bits = 7, 
            .mode = 0,                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
            .clock_speed_hz = 10 * 1000 * 1000,    // 10 MHz
            .spics_io_num = Config::GetFieldInt("cs6_pin"), // CS pin
            .queue_size = 1,  
        };

        std::cout << "cs1_pin " << Config::GetFieldInt("cs1_pin")
                  << " cs2_pin "<< Config::GetFieldInt("cs2_pin")
                  << " cs3_pin "<< Config::GetFieldInt("cs3_pin")
                  << " cs4_pin "<< Config::GetFieldInt("cs4_pin")
                  << " cs5_pin "<< Config::GetFieldInt("cs5_pin")
                  << " cs6_pin "<< Config::GetFieldInt("cs6_pin")
                  << std::endl;
        
        spi_config[0].bus = bus1;
        spi_config[1].bus = bus2;

        std::array<spi_device_interface_config_t, 3> devices_array1 = {device1, device2, device3};
        std::array<spi_device_interface_config_t, 3> devices_array2 = {device4, device5, device6};

        spi_config[0].devices = devices_array1;
        spi_config[1].devices = devices_array2;

        spi_host_device_t host1 = (spi_host_device_t)(SPI_HOST_MAX - 2);
        spi_host_device_t host2 = (spi_host_device_t)(SPI_HOST_MAX - 1);
        esp_err_t ret;
        
        
        // Initialize the SPI bus
        ret = spi_bus_initialize(host1, &(spi_config[0].bus), SPI_DMA_DISABLED);
        ESP_ERROR_CHECK(ret);

        ret = spi_bus_initialize(host2, &(spi_config[1].bus), SPI_DMA_DISABLED);
        ESP_ERROR_CHECK(ret);

        // Add device to bus
        ret = spi_bus_add_device(host1, &(spi_config[0].devices[0]), &(spi_config[0].handles[0]));
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host1, &(spi_config[0].devices[1]), &(spi_config[0].handles[1]));
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host1, &(spi_config[0].devices[2]), &(spi_config[0].handles[2]));
        ESP_ERROR_CHECK(ret);

        ret = spi_bus_add_device(host2, &(spi_config[1].devices[0]), &(spi_config[1].handles[0]));
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host2, &(spi_config[1].devices[1]), &(spi_config[1].handles[1]));
        ESP_ERROR_CHECK(ret);
        ret = spi_bus_add_device(host2, &(spi_config[1].devices[2]), &(spi_config[1].handles[2]));
        ESP_ERROR_CHECK(ret);
        using spiopT = std::function<int8_t(uint8_t, uint8_t, uint8_t*, uint16_t)>;
        
        spiopT sensor1Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[0].handles[0], dev_addr, reg_addr, read_data, len);
        };
        spiopT sensor2Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[0].handles[1], dev_addr, reg_addr, read_data, len);
        };
        spiopT sensor3Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[0].handles[2], dev_addr, reg_addr, read_data, len);
        };
        spiopT sensor4Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[1].handles[0], dev_addr, reg_addr, read_data, len);
        };
        spiopT sensor5Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[1].handles[1], dev_addr, reg_addr, read_data, len);
        };
        spiopT sensor6Read = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
            return user_spi_read(spi_config[1].handles[2], dev_addr, reg_addr, read_data, len);
        };

        spiopT sensor1Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[0].handles[0], dev_addr, reg_addr, write_data, len);
        };
        spiopT sensor2Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[0].handles[1], dev_addr, reg_addr, write_data, len);
        };
        spiopT sensor3Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[0].handles[2], dev_addr, reg_addr, write_data, len);
        };
        spiopT sensor4Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[1].handles[0], dev_addr, reg_addr, write_data, len);
        };
        spiopT sensor5Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[1].handles[1], dev_addr, reg_addr, write_data, len);
        };
        spiopT sensor6Write = [&](uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len){
            return user_spi_write(spi_config[1].handles[2], dev_addr, reg_addr, write_data, len);
        };

        sensors[0] = {
            .id = static_cast<uint8_t>(spi_config[0].devices[0].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<0, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor1Read),
            .write = get_fn_ptr<1, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor1Write),
            .delay_ms = user_delay_ms,
        };
        
	    ret = bmi160_init(&(sensors[0]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[0].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[1] = {
            .id = static_cast<uint8_t>(spi_config[0].devices[1].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<2, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor2Read),
            .write = get_fn_ptr<3, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor2Write),
            .delay_ms = user_delay_ms,
        };
	    ret = bmi160_init(&(sensors[1]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[1].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }
        sensors[2] = {
            .id = static_cast<uint8_t>(spi_config[0].devices[2].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<4, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor3Read),
            .write = get_fn_ptr<5, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor3Write),
            .delay_ms = user_delay_ms,
        };
	    ret = bmi160_init(&(sensors[2]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[2].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }
        sensors[3] = {
            .id = static_cast<uint8_t>(spi_config[1].devices[0].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<6, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Read),
            .write = get_fn_ptr<7, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Write),
            .delay_ms = user_delay_ms,
        };
        
	    ret = bmi160_init(&(sensors[3]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[3].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }
        sensors[4] = {
            .id = static_cast<uint8_t>(spi_config[1].devices[1].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<6, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Read),
            .write = get_fn_ptr<7, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Write),
            .delay_ms = user_delay_ms,
        };
        
	    ret = bmi160_init(&(sensors[4]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[4].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }
        sensors[5] = {
            .id = static_cast<uint8_t>(spi_config[1].devices[2].spics_io_num),
            .intf = BMI160_SPI_INTF,
            .read = get_fn_ptr<6, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Read),
            .write = get_fn_ptr<7, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Write),
            .delay_ms = user_delay_ms,
        };
        
	    ret = bmi160_init(&(sensors[5]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[5].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        for (auto & sensor: sensors){
            // Config Accel
	        sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	        sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
	        sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	        sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

            // Config Gyro
	        sensor.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
	        sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS; // -250 --> +250[Deg/Sec]
	        sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	        sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

            ret = bmi160_set_sens_conf(&sensor);
	        if (ret != BMI160_OK) {
	        	ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
	        }
	        ESP_LOGI(TAG, "bmi160_set_sens_conf");
        }

        //for (int i = 0; i < arhs_array.size(); i++){
        //    FusionAhrsInitialise(&(arhs_array[i]));
        //}

        /*gpio_num_t int1_gpio = static_cast<gpio_num_t>(Config::GetFieldInt("int1_pin"));
        gpio_reset_pin(int1_gpio);
        gpio_set_direction(int1_gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(int1_gpio, GPIO_PULLDOWN_ONLY);

        esp_err_t err = gpio_install_isr_service(0);
        if (err == ESP_ERR_INVALID_STATE) {
          ESP_LOGW("ISR", "GPIO isr service already installed");
        };
        gpio_isr_handler_add(int1_gpio, Imu1Interrupt, &drdy);

        gpio_set_intr_type(int1_gpio, GPIO_INTR_POSEDGE);

        struct bmi160_int_settg int_config;
        int_config.int_channel = BMI160_INT_CHANNEL_1;
        int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;
        int_config.int_pin_settg.output_en = BMI160_ENABLE;
        int_config.int_pin_settg.output_mode = 0;
        int_config.int_pin_settg.output_type = 1;
        int_config.int_pin_settg.edge_ctrl = 1;
        int_config.int_pin_settg.input_en = 0;
        int_config.int_pin_settg.latch_dur = 0;
        interrupts_configs[0] = int_config;
        ret = bmi160_set_int_config(&(interrupts_configs[0]), &(sensors[0]));
        if (ret != BMI160_OK) {
            ESP_LOGE(TAG, "Failed to configure BMI160 data ready interrupt");
        }
        
        gpio_intr_enable(int1_gpio);*/

        xTaskCreatePinnedToCore(ImusTask, "Imus Task", 1024*4, NULL, 15, NULL, 0);
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
const char * ImuArray::TAG = "IMU_LOG";
std::array<ImuArray::SpiBuss, 2> ImuArray::spi_config;
std::array<bmi160_dev, 6> ImuArray::sensors;
//std::array<bmi160_int_settg, 1> ImuArray::interrupts_configs;
std::queue<ImuArray::ImuArrayDataPack> ImuArray::messusrments_queue;
std::function<void(ImuArray::ImuArrayDataPack)> ImuArray::callback;
SemaphoreHandle_t ImuArray::drdy;
//std::array<FusionAhrs, 4> ImuArray::arhs_array;