


#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <ce/operation/mode.h>


static SemaphoreHandle_t ce_mode_mutex;
static ce_debug_flag_t ce_debug_flag = DEBUG_ON;
static ce_mode_t ce_current_mode = SYSTEM_MODE_DEBUG;


void ce_mode_set(ce_mode_t new_mode)
{
    xSemaphoreTake(ce_mode_mutex, portMAX_DELAY);
    ce_current_mode = new_mode;
    if (new_mode == SYSTEM_MODE_DEBUG)
    {
        ce_debug_flag = DEBUG_ON;
    }
    else
    {
        ce_debug_flag = DEBUG_OFF;
    }
    xSemaphoreGive(ce_mode_mutex);
}

ce_mode_t ce_mode_get()
{
    ce_mode_t ret;
    xSemaphoreTake(ce_mode_mutex, portMAX_DELAY);
    ret = ce_current_mode;
    xSemaphoreGive(ce_mode_mutex);
    return ret;
}

static void ce_mode_task(void *pvParameters)
{
    while (1)
    {
        // TODO: 모드 판단하는 코드 작성

        // 테스트용
        ce_mode_t new_mode = SYSTEM_MODE_DEBUG;
        ce_mode_set(new_mode);

        vTaskDelay(pdMS_TO_TICKS(CE_OPERATION_TASK_PERIOD));

    }
}


void ce_mode_init(void)
{
    ce_mode_mutex = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(ce_mode_task, "ce_mode_task", CE_OPERATION_TASK_STACK_SIZE, NULL, CE_OPERATION_TASK_PRIORITY, NULL, 0);
    
}