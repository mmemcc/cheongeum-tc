#ifndef SENSOR_KX022ACR
#define SENSOR_KX022ACR

#include <stdint.h>

#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <ce/util/error.h>


#define KX022ACR_SPI_PIN_MOSI 11
#define KX022ACR_SPI_PIN_MISO 13
#define KX022ACR_SPI_PIN_CLK 12
#define KX022ACR_SPI_PIN_CS0 10

#define KX022ACR_I2C_PIN_SDA 13
#define KX022ACR_I2C_PIN_SCL 12

#define KX022ACR_I2C_ADDR 0x1E

#define I2C_MASTER_NUM 0

// KX022ACR Register
#define WHO_AM_I     0x0F
#define CNTL1        0x18
#define CNTL2        0x19
#define CNTL3        0x1A
#define ODCNTL       0x1B
#define BUF_CNTL1    0x3A
#define BUF_CNTL2    0x3B
#define BUF_STATUS_1 0x3C
#define BUF_STATUS_2 0x3D
#define BUF_READ     0x3F
#define BUF_CLEAR    0x3E
#define INC1         0x1C

#define KX022ACR_SAMPLE_RATE 1600
#define KX022ACR_SAMPLE_SIZE 6
#define KX022ACR_MAX_BUF_SIZE 6*1024*1024 // 6MB
#define KX022ACR_MAX_SAMPLES (KX022ACR_MAX_BUF_SIZE / KX022ACR_SAMPLE_SIZE)

// extern int16_t *psram_buffer;


ce_error_t ce_kx022acr_pin_set_spi(spi_device_handle_t * kx022acr_device_handle);

ce_error_t ce_kx022acr_write_reg_spi(spi_device_handle_t kx022acr_device_handle, uint8_t reg_addr, uint8_t reg_data);

ce_error_t ce_kx022acr_read_reg_spi(spi_device_handle_t kx022acr_device_handle, uint8_t reg_addr, void *reg_data);

ce_error_t ce_kx022acr_init_spi(void);

ce_error_t ce_kx022acr_pin_set_i2c(i2c_master_dev_handle_t *ret_handle);

ce_error_t ce_kx022acr_write_reg_i2c(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data);

ce_error_t ce_kx022acr_read_reg_i2c(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t rx_len);

ce_error_t ce_kx022acr_init_i2c(void);

#endif