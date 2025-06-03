#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ce/sensor/sht4x.h>

static uint8_t sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
static uint16_t sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;


static uint8_t check_crc(const uint8_t* data, uint8_t len, uint8_t checksum);

ce_error_t ce_sht4x_init(i2c_master_dev_handle_t *ret_handle) {
    i2c_master_bus_config_t sht4x_i2c_config = {
        .i2c_port = -1,
        .sda_io_num = SHT4X_I2C_SDA,
        .scl_io_num = SHT4X_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    i2c_master_bus_handle_t sht4x_i2c_bus_handle;
    if (i2c_new_master_bus(&sht4x_i2c_config, &sht4x_i2c_bus_handle) != ESP_OK) {
        printf("i2c_new_master_bus failed\n");
        return CE_ERROR_SENSOR_INIT;
    }
    printf("I2C bus initialized\n");

    // General Call Reset 수행
    printf("Performing I2C general call reset...\n");
    sht4x_general_call_reset(sht4x_i2c_bus_handle);
    vTaskDelay(pdMS_TO_TICKS(10));  // 리셋 후 대기

    // SHT4x 디바이스 설정
    i2c_device_config_t sht4x_i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = SHT4X_ADDRESS,
        .scl_speed_hz = 100000,
    };

    if (i2c_master_bus_add_device(sht4x_i2c_bus_handle, &sht4x_i2c_device_config, ret_handle) != ESP_OK) {
        printf("Failed to add SHT4x device\n");
        return CE_ERROR_SENSOR_INIT;
    }

    // 소프트 리셋 수행
    printf("Performing soft reset...\n");
    if (sht4x_soft_reset(*ret_handle) != ESP_OK) {
        printf("Soft reset failed\n");
        return CE_ERROR_SENSOR_INIT;
    }

    // 시리얼 넘버 읽기 시도
    printf("Attempting to read SHT4x serial number...\n");
    uint32_t serial;
    if (sht4x_read_serial(*ret_handle, &serial) != ESP_OK) {
        printf("Failed to read serial number\n");
        return CE_ERROR_SENSOR_INIT;
    }

    printf("SHT4x initialized successfully, serial: 0x%08lX\n", serial);
    return CE_OK;
}


int16_t sht4x_soft_reset(i2c_master_dev_handle_t i2c_dev) {
    uint8_t cmd = SHT4X_CMD_RESET;
    int16_t ret = i2c_master_transmit(i2c_dev, &cmd, 1, 1000);
    if (ret == ESP_OK) {
        // 리셋 후 최소 1ms 대기
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ret;
}


int16_t sht4x_measure_blocking_read(i2c_master_dev_handle_t i2c_dev, float* temperature, float* humidity) {
    int16_t ret;
    const int MAX_RETRIES = 3;  // 최대 재시도 횟수
    
    // 1. 측정 명령 전송
    ret = i2c_master_transmit(i2c_dev, &sht4x_cmd_measure, 1, 1000);
    if (ret != ESP_OK) {
        printf("Failed to send measurement command: %d\n", ret);
        return ret;
    }

    // 2. 측정 완료 대기 (명확한 시간 지정)
    vTaskDelay(pdMS_TO_TICKS(20));  // 10ms 대기

    // 3. 데이터 읽기 시도 (재시도 로직 포함)
    uint8_t data[6];
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        // 읽기 시도
        ret = i2c_master_receive(i2c_dev, data, sizeof(data), 1000);
        if (ret == ESP_OK) {
            break;  // 성공하면 루프 종료
        }
        
        printf("Read attempt %d failed, error: %d\n", retry + 1, ret);
        vTaskDelay(pdMS_TO_TICKS(5));  // 5ms 대기 후 재시도
    }

    if (ret != ESP_OK) {
        printf("All read attempts failed\n");
        return ret;
    }

    // 4. 데이터 처리
    // 온도 데이터 (첫 번째 2바이트 + CRC)
    uint16_t temp_raw = ((uint16_t)data[0] << 8) | data[1];
    
    // 습도 데이터 (두 번째 2바이트 + CRC)
    uint16_t hum_raw = ((uint16_t)data[3] << 8) | data[4];

    // 변환
    *temperature = (((21875 * (int32_t)temp_raw) >> 13) - 45000) / 1000.0;
    *humidity = (((15625 * (int32_t)hum_raw) >> 13) - 6000) / 1000.0;

    // printf("temperature: %.2f, humidity: %.2f\n", *temperature, *humidity);

    return ESP_OK;
}


// I2C General Call Reset 함수 추가
int16_t sht4x_general_call_reset(i2c_master_bus_handle_t bus_handle) {
    i2c_master_dev_handle_t temp_dev_handle;
    i2c_device_config_t temp_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = I2C_GENERAL_CALL_ADDR,
        .scl_speed_hz = 100000,
    };
    
    if (i2c_master_bus_add_device(bus_handle, &temp_dev_config, &temp_dev_handle) != ESP_OK) {
        return ESP_FAIL;
    }
    
    uint8_t reset_cmd = I2C_GENERAL_CALL_RESET;
    int16_t ret = i2c_master_transmit(temp_dev_handle, &reset_cmd, 1, 1000);
    
    i2c_master_bus_rm_device(temp_dev_handle);
    
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));  // 리셋 후 대기
    }
    return ret;
}


int16_t sht4x_measure(i2c_master_dev_handle_t i2c_dev) {
    return i2c_master_transmit(i2c_dev, &sht4x_cmd_measure, 1, 1000 / portTICK_PERIOD_MS);
}

// CRC-8 체크 함수 추가
static uint8_t check_crc(const uint8_t* data, uint8_t len, uint8_t checksum) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;  // CRC-8 polynomial
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc == checksum;
}

int16_t sht4x_read(i2c_master_dev_handle_t i2c_dev, int32_t* temperature, int32_t* humidity) {
    uint8_t data[6];  // 2바이트 온도 + 1바이트 CRC + 2바이트 습도 + 1바이트 CRC
    int16_t ret;
    
    // 데이터만 읽기 (측정 명령 보내지 않음)
    ret = i2c_master_receive(i2c_dev, data, sizeof(data), 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    // CRC 체크
    if (!check_crc(data, 2, data[2]) || !check_crc(data + 3, 2, data[5])) {
        return ESP_ERR_INVALID_CRC;
    }

    // 온도 데이터 변환 (첫 번째 2바이트)
    uint16_t temp_raw = ((uint16_t)data[0] << 8) | data[1];
    *temperature = ((21875 * (int32_t)temp_raw) >> 13) - 45000;

    // 습도 데이터 변환 (두 번째 2바이트)
    uint16_t hum_raw = ((uint16_t)data[3] << 8) | data[4];
    *humidity = ((15625 * (int32_t)hum_raw) >> 13) - 6000;

    return ESP_OK;
}


void sht4x_enable_low_power_mode(uint8_t enable_low_power_mode) {
    if (enable_low_power_mode) {
        sht4x_cmd_measure = SHT4X_CMD_MEASURE_LPM;
        sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_LPM_USEC;
    } else {
        sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
        sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;
    }
}

// 시리얼 넘버 읽기 함수 수정
int16_t sht4x_read_serial(i2c_master_dev_handle_t i2c_dev, uint32_t* serial) {
    uint8_t cmd = SHT4X_CMD_READ_SERIAL;
    uint8_t data[6];  // 2바이트 시리얼 + 1바이트 CRC + 2바이트 시리얼 + 1바이트 CRC
    int16_t ret;

    // 시리얼 넘버 읽기 명령 전송
    ret = i2c_master_transmit(i2c_dev, &cmd, 1, 1000);
    if (ret != ESP_OK) {
        printf("Failed to send serial number command\n");
        return ret;
    }

    // 데이터 수신 전 약간의 대기
    vTaskDelay(pdMS_TO_TICKS(10));

    // 6바이트 데이터 읽기
    ret = i2c_master_receive(i2c_dev, data, sizeof(data), 1000);
    if (ret != ESP_OK) {
        printf("Failed to receive serial number data\n");
        return ret;
    }

    // CRC 체크 (필요시 구현)
    
    // 시리얼 넘버 조합
    *serial = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
              ((uint32_t)data[3] << 8) | data[4];

    return ESP_OK;
}


uint8_t sht4x_get_configured_address(void) {
    return SHT4X_ADDRESS;
}

void i2c_scanner(i2c_master_bus_handle_t bus_handle) {
    printf("\nScanning I2C bus...\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    uint8_t dummy_data = 0;  // 실제 데이터를 가진 버퍼
    
    for (uint8_t addr_first_digit = 0; addr_first_digit < 8; addr_first_digit++) {
        printf("%d0:", addr_first_digit);
        for (uint8_t addr_second_digit = 0; addr_second_digit < 16; addr_second_digit++) {
            uint8_t address = (addr_first_digit << 4) | addr_second_digit;
            
            if (address < 0x08 || address > 0x77) {
                printf("   ");
                continue;
            }

            i2c_master_dev_handle_t temp_dev_handle;
            i2c_device_config_t temp_dev_config = {
                .dev_addr_length = I2C_ADDR_BIT_7,
                .device_address = address,
                .scl_speed_hz = 100000,
            };
            
            if (i2c_master_bus_add_device(bus_handle, &temp_dev_config, &temp_dev_handle) == ESP_OK) {
                // 유효한 데이터로 transmit 시도
                esp_err_t ret = i2c_master_transmit(temp_dev_handle, &dummy_data, 1, 100);
                i2c_master_bus_rm_device(temp_dev_handle);
                
                if (ret == ESP_OK) {
                    printf(" %02X", address);
                } else {
                    printf(" --");
                }
            } else {
                printf(" --");
            }
        }
        printf("\n");
    }
}