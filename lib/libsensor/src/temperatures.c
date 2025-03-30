
#include <stdint.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>

#include <ce/sensor/temperatures.h>
#include <ce/relay/control.h>

SemaphoreHandle_t temp_sample_sem;
esp_timer_handle_t temp_sample_timer;
TaskHandle_t ce_temperatures_task_handle;

void temp_sample_timer_cb(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(temp_sample_sem, &high_task_wakeup);
    portYIELD_FROM_ISR();
}

ce_error_t ce_temperatures_read(float * temp, uint8_t sensor_ch, adc_oneshot_unit_handle_t adc_handle)
{
    int adc_raw[0][0];
    esp_err_t ret = ESP_OK;
    ret = adc_oneshot_read(adc_handle, sensor_ch, &adc_raw[0][0]);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_READ;
    }

    float voltage = (TEMP_REP_VOLTAE*adc_raw[0][0])/4095.0;
    float resistance = (TEMP_REP_VOLTAE*TMEP_SENSOR_R_25_DEGREE)/voltage - TMEP_SENSOR_R_25_DEGREE;
    float temp_c = 1.0/((1.0/(273.15+25.0)) + (log(resistance/TMEP_SENSOR_R_25_DEGREE)/3435.0)) - 273.15;
    *temp = temp_c;
    return CE_OK;

}


ce_error_t ce_temperatures_set_pin(adc_oneshot_unit_handle_t *adc1_handle)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = false,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, adc1_handle);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    adc_oneshot_chan_cfg_t adc1_chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_1, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_2, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_3, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_4, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_5, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_6, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    return CE_OK;
}

static void ce_temperatures_task(void *params)
{
    adc_oneshot_unit_handle_t adc1_handle = *((adc_oneshot_unit_handle_t *)params);
    float temp[6];
    while (1)
    {
        EventBits_t event_bits = xEventGroupWaitBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT2, pdTRUE, pdFALSE, portMAX_DELAY);

        while (1)
        {
            bool is_relay_changed = (xEventGroupGetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group) & RELAY_STATE_CHANGE_BIT2) ? true : false;
            if (is_relay_changed)
            {
                break;
            }

            xSemaphoreTake(temp_sample_sem, portMAX_DELAY);

            for (int i = 0; i < 6; i++)
            {
                ce_temperatures_read(&temp[i], i, adc1_handle);
            }
            
            xQueueSend(ce_temperatures_queue_global, temp, 0);
        }
    }
}

void init_temp_sampling_timer()
{
    temp_sample_sem = xSemaphoreCreateBinary();
    esp_timer_create_args_t timer_args = {
        .callback = temp_sample_timer_cb,
        .name = "kx_sample_timer"};
    esp_timer_create(&timer_args, &temp_sample_timer);
    esp_timer_start_periodic(temp_sample_timer, 1000000); // 1초
}

ce_error_t ce_temperatures_init(void)
{
    adc_oneshot_unit_handle_t adc1_handle = NULL;
    ce_error_t err = ce_temperatures_set_pin(&adc1_handle);
    if (err != CE_OK)
    {
        return err;
    }

    

    if (xTaskCreatePinnedToCore(ce_temperatures_task, "ce_temperatures_task", 4000, &adc1_handle, 5, &ce_temperatures_task_handle, 0) != pdPASS)
    {
        return CE_ERROR_TASK_CREATE;
    }

    init_temp_sampling_timer();
    return CE_OK;
}