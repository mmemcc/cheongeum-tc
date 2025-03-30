#ifndef OPRTATION_MODE_H
#define OPRTATION_MODE_H



#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#define SYSTEM_MODE_DEBUG_BIT 0
#define SYSTEM_MODE_DEFAULT_BIT 1
#define SYSTEM_MODE_SAFE_BIT 2


extern EventGroupHandle_t ce_mode_event_group_global;


#define CE_OPERATION_TASK_PERIOD 1000
#define CE_SERVER_TASK_PERIOD 1000
#define CE_SENSOR_TASK_PERIOD 1000
#define CE_TC_TASK_PERIOD 1000
#define CE_CONTROL_RELAY_TASK_PERIOD 1000
#define CE_CONSOLE_TASK_PERIOD 1000

#define CE_OPERATION_TASK_PRIORITY 5
#define CE_SERVER_TASK_PRIORITY 5
#define CE_SENSOR_TASK_PRIORITY 5
#define CE_TC_TASK_PRIORITY 5
#define CE_CONTROL_RELAY_TASK_PRIORITY 5
#define CE_CONSOLE_TASK_PRIORITY 5

#define CE_OPERATION_TASK_STACK_SIZE 2048
#define CE_SERVER_TASK_STACK_SIZE 2048
#define CE_SENSOR_TASK_STACK_SIZE 2048
#define CE_TC_TASK_STACK_SIZE 2048
#define CE_CONTROL_RELAY_TASK_STACK_SIZE 2048
#define CE_CONSOLE_TASK_STACK_SIZE 2048

extern TaskHandle_t ce_kx022acr_task_handle;
extern TaskHandle_t ce_temperatures_task_handle;


typedef enum 
{
    SYSTEM_MODE_DEBUG = 0,
    SYSTEM_MODE_DEFAULT = 1,
    SYSTEM_MODE_SAFE = 2
} ce_mode_t;

typedef enum 
{
    DEBUG_OFF = 0,
    DEBUG_ON = 1
} ce_debug_flag_t;




void ce_mode_set(ce_mode_t new_mode);
ce_mode_t ce_mode_get();

void ce_mode_init(void);

#endif