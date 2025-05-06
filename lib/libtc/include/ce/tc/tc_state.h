#ifndef TC_STATE_H
#define TC_STATE_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

typedef struct {
    uint8_t mqtt_connected;
    uint8_t wifi_connected;
    uint8_t alarm_connected;
    uint8_t warning_connected;
    uint8_t machine_condition;
    SemaphoreHandle_t tc_state_mutex;
} tc_state_t;

extern tc_state_t tc_state_global;

#endif
