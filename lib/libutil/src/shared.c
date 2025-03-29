
#include <ce/util/shared.h>
#include <string.h>

ce_error_t ce_global_update(void * value_for_update, void * data, size_t size, SemaphoreHandle_t mutex)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(value_for_update, data, size);
        xSemaphoreGive(mutex);
        return CE_OK;
    }
    return CE_ERROR_MUTEX_TAKE;
}