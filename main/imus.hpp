#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"
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


class ImuArray{
public:
    struct AccelGyroData {
        struct bmi160_sensor_data accel;
	    struct bmi160_sensor_data gyro;
    };
    
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

private: 
    constexpr static uint16_t CMD_READ  = 0x01;
    constexpr static uint16_t CMD_WRITE = 0x00;

    static std::array<bmi160_dev, 6> sensors;

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

    static void ImusTask(void * pvParameters){
        struct AccelGyroData data;
        for(;;){
            for (auto & sensor: sensors){
                int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &data.accel, &data.gyro, &sensor);
                if (ret != BMI160_OK) {
                    ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
                }
                #ifdef __DEBUG__
                ESP_LOGI(TAG, "RAW DATA:");
                ESP_LOGI(TAG, "ACCEL: x=%f, y=%f, z=%f", (double)accel.x, (double)accel.y, (double)accel.z);
                ESP_LOGI(TAG, "GYRO: x=%f, y=%f, z=%f", (double)gyro.x, (double)gyro.y, (double)gyro.z); 
                //todo
                #endif
        
                taskYIELD();
            }
        }
    }

    static void user_delay_ms(uint32_t period) {
	    esp_rom_delay_us(period*1000);
    };

    ImuArray() {
        spi_bus_config_t bus1;
        spi_bus_config_t bus2;

        spi_device_interface_config_t device1;
        spi_device_interface_config_t device2;
        spi_device_interface_config_t device3;
        spi_device_interface_config_t device4;
        spi_device_interface_config_t device5;
        spi_device_interface_config_t device6;

        bus1.mosi_io_num = Config::GetFieldInt("mosi1_pin");
        bus1.miso_io_num = Config::GetFieldInt("miso1_pin");
        bus1.sclk_io_num = Config::GetFieldInt("sclk1_pin");
        bus1.quadwp_io_num = -1;
        bus1.quadhd_io_num = -1;
        bus1.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

        bus2.mosi_io_num = Config::GetFieldInt("mosi2_pin");
        bus2.miso_io_num = Config::GetFieldInt("miso2_pin");
        bus2.sclk_io_num = Config::GetFieldInt("sclk2_pin");
        bus2.quadwp_io_num = -1;
        bus2.quadhd_io_num = -1;
        bus2.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

        device1.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device1.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device1.spics_io_num = Config::GetFieldInt("cs1_pin"); // CS pin
        device1.queue_size = 1;  

        device2.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device2.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device2.spics_io_num = Config::GetFieldInt("cs2_pin"); // CS pin
        device2.queue_size = 1;  

        device3.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device3.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device3.spics_io_num = Config::GetFieldInt("cs3_pin"); // CS pin
        device3.queue_size = 1;  

        device4.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device4.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device4.spics_io_num = Config::GetFieldInt("cs4_pin"); // CS pin
        device4.queue_size = 1;  

        device5.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device5.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device5.spics_io_num = Config::GetFieldInt("cs5_pin"); // CS pin
        device5.queue_size = 1;  

        device6.mode = 0;                             // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        device6.clock_speed_hz = 10 * 1000 * 1000;    // 10 MHz
        device6.spics_io_num = Config::GetFieldInt("cs6_pin"); // CS pin
        device6.queue_size = 1;  
        
        spi_config[0].bus = bus1;
        spi_config[1].bus = bus2;

        spi_config[0].handles = {0, 0, 0};
        spi_config[1].handles = {0, 0, 0};

        std::array<spi_device_interface_config_t, 3> devices_array1 = {device1, device2, device3};
        std::array<spi_device_interface_config_t, 3> devices_array2 = {device4, device5, device6};

        spi_config[0].devices = devices_array1;
        spi_config[1].devices = devices_array2;

        spi_config[0].handles = {0, 0, 0};
        spi_config[1].handles = {0, 0, 0};

        spi_host_device_t host1 = (spi_host_device_t)(SPI_HOST_MAX - 2);
        spi_host_device_t host2 = (spi_host_device_t)(SPI_HOST_MAX - 1);

        // Initialize the SPI bus
        int ret = spi_bus_initialize(host1, &(spi_config[0].bus), SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);

        ret = spi_bus_initialize(host2, &(spi_config[1].bus), SPI_DMA_CH_AUTO);
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

        sensors[0].id = spi_config[0].devices[0].spics_io_num;
        sensors[0].intf = BMI160_SPI_INTF;
        sensors[0].read = get_fn_ptr<0, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor1Read);
        sensors[0].write = get_fn_ptr<1, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor1Write);
        sensors[0].delay_ms = user_delay_ms;
	    ret = bmi160_init(&(sensors[0]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[0].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[1].id = spi_config[0].devices[1].spics_io_num;
        sensors[1].intf = BMI160_SPI_INTF;
        sensors[1].read = get_fn_ptr<2, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor2Read);
        sensors[1].write = get_fn_ptr<3, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor2Write);
        sensors[1].delay_ms = user_delay_ms;
	    ret = bmi160_init(&(sensors[1]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[1].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[2].id = spi_config[0].devices[2].spics_io_num;
        sensors[2].intf = BMI160_SPI_INTF;
        sensors[2].read = get_fn_ptr<4, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor3Read);
        sensors[2].write = get_fn_ptr<5, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor3Write);
        sensors[2].delay_ms = user_delay_ms;
	    ret = bmi160_init(&(sensors[2]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[2].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[3].id = spi_config[1].devices[0].spics_io_num;
        sensors[3].intf = BMI160_SPI_INTF;
        sensors[3].read = get_fn_ptr<6, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Read);
        sensors[3].write = get_fn_ptr<7, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor4Write);
        sensors[3].delay_ms = user_delay_ms;
	    ret = bmi160_init(&(sensors[3]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[3].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[4].id = spi_config[1].devices[1].spics_io_num;
        sensors[4].intf = BMI160_SPI_INTF;
        sensors[4].read = get_fn_ptr<8, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor5Read);
        sensors[4].write = get_fn_ptr<9, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor5Write);
        sensors[4].delay_ms = user_delay_ms;
	    ret = bmi160_init(&(sensors[4]));

        if (ret == BMI160_OK) {
	    	ESP_LOGI(TAG, "BMI160 initialization success !");
	    	ESP_LOGI(TAG, "Chip ID 0x%X", sensors[4].chip_id);
	    } else {
	    	ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
	    }

        sensors[5].id = spi_config[1].devices[2].spics_io_num;
        sensors[5].intf = BMI160_SPI_INTF;
        sensors[5].read = get_fn_ptr<10, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor6Read);
        sensors[5].write = get_fn_ptr<11, int8_t, uint8_t, uint8_t, uint8_t*, uint16_t>(sensor6Write);
        sensors[5].delay_ms = user_delay_ms;
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
	        sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	        sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
	        sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	        sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

            ret = bmi160_set_sens_conf(&sensor);
	        if (ret != BMI160_OK) {
	        	ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
	        }
	        ESP_LOGI(TAG, "bmi160_set_sens_conf");
        }

        xTaskCreate(ImusTask, "Imus Task", 1024*4, NULL, 1, NULL);
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