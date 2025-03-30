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


extern QueueHandle_t ce_temperatures_queue_global;

ce_error_t ce_temperatures_init(void);



#endif