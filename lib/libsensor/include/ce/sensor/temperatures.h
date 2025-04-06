#ifndef SENSOR_TEMPERATURES_H
#define SENSOR_TEMPERATURES_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


#include <ce/util/error.h>

#define TMEP_ADC_CHANNLE_1 0
#define TMEP_ADC_CHANNLE_2 1
#define TMEP_ADC_CHANNLE_3 2
#define TMEP_ADC_CHANNLE_4 3
#define TMEP_ADC_CHANNLE_5 4
#define TMEP_ADC_CHANNLE_6 5

#define TMEP_SENSOR_R_25_DEGREE 10000.0
#define TEMP_SENSOR_R_PARALLEL 10000.0

#define TEMP_REP_VOLTAE 3.3

#define HUMI_TEMP_UART_TX 17
#define HUMI_TEMP_UART_RX 18
#define HUMI_TEMP_UART_BAUD_RATE 9600
#define HUMI_TEMP_UART_PORT_NUM 1

typedef struct {
    uint8_t address;
    uint8_t function_code;
    uint8_t inital_addr_l;
    uint8_t inital_addr_h;
    uint8_t data_length_l;
    uint8_t data_length_h;
    uint8_t check_l;
    uint8_t check_h;
} ce_humi_temp_tx_frame_t;

typedef struct {
    uint8_t address;
    uint8_t function_code;
    uint8_t return_byte;
    uint8_t data_humi[2];
    uint8_t data_temp[2];
    uint8_t check_l;
    uint8_t check_h;
} ce_humi_temp_rx_frame_t;

extern QueueHandle_t ce_temperatures_queue_global;

ce_error_t ce_temperatures_init(void);



#endif