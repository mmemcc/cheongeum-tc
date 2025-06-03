#ifndef SHT4X_H
#define SHT4X_H

#include <stdint.h>

#include <ce/util/error.h>
#include <ce/tc/esp32.h>

#define SHT4X_CMD_RESET     0x94
#define SHT4X_CMD_MEASURE_HPM 0xFD
#define SHT4X_CMD_MEASURE_LPM 0xE0
#define SHT4X_CMD_READ_SERIAL 0x89
#define SHT4X_CMD_DURATION_USEC 1000
#define I2C_GENERAL_CALL_ADDR 0x00
#define I2C_GENERAL_CALL_RESET 0x06

#define SHT4X_ADDRESS 0x44

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32


#define STATUS_OK 0
#define STATUS_ERR_BAD_DATA (-1)
#define STATUS_CRC_FAIL (-2)
#define STATUS_UNKNOWN_DEVICE (-3)
#define SHT4X_MEASUREMENT_DURATION_USEC 10 /* 10ms "high repeatability" */
#define SHT4X_MEASUREMENT_DURATION_LPM_USEC \
    2500 /* 2.5ms "low repeatability"       \
          */

#define SHT4X_I2C_SDA I2C_SDA
#define SHT4X_I2C_SCL I2C_SCL

ce_error_t ce_sht4x_init(i2c_master_dev_handle_t *ret_handle);

/**
 * Detects if a sensor is connected by reading out the ID register.
 * If the sensor does not answer or if the answer is not the expected value,
 * the test fails.
 *
 * @return 0 if a sensor was detected
 */
int16_t sht4x_probe(i2c_master_dev_handle_t i2c_dev);

/**
 * Starts a measurement and then reads out the results. This function blocks
 * while the measurement is in progress. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
int16_t sht4x_measure_blocking_read(i2c_master_dev_handle_t i2c_dev, float* temperature, float* humidity);

/**
 * Starts a measurement in high precision mode. Use sht4x_read() to read out the
 * values, once the measurement is done. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 *
 * @return     0 if the command was successful, else an error code.
 */
int16_t sht4x_measure(i2c_master_dev_handle_t i2c_dev);

/**
 * Reads out the results of a measurement that was previously started by
 * sht4x_measure(). If the measurement is still in progress, this function
 * returns an error.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
int16_t sht4x_read(i2c_master_dev_handle_t i2c_dev, int32_t* temperature, int32_t* humidity);

/**
 * Enable or disable the SHT's low power mode
 *
 * @param enable_low_power_mode 1 to enable low power mode, 0 to disable
 */
void sht4x_enable_low_power_mode(uint8_t enable_low_power_mode);

/**
 * Read out the serial number
 *
 * @param serial    the address for the result of the serial number
 * @return          0 if the command was successful, else an error code.
 */
int16_t sht4x_read_serial(i2c_master_dev_handle_t i2c_dev, uint32_t* serial);

/**
 * Return the driver version
 *
 * @return Driver version string
 */
const char* sht4x_get_driver_version(void);

/**
 * Returns the configured SHT4x address.
 *
 * @return SHT4x_ADDRESS
 */
uint8_t sht4x_get_configured_address(void);

void i2c_scanner(i2c_master_bus_handle_t bus_handle);

int16_t sht4x_soft_reset(i2c_master_dev_handle_t i2c_dev);

int16_t sht4x_general_call_reset(i2c_master_bus_handle_t bus_handle);

#endif