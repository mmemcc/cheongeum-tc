
#include <stdint.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <esp_rom_sys.h>
#include <driver/uart.h>

#include <ce/sensor/temperatures.h>
#include <ce/relay/control.h>
#include <ce/operation/test.h>
#include <ce/sensor/sensors.h>

#define SAMPLE_COUNT_CURRENT 500
#define SAMPLE_COUNT_TEMP 50

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
    int adc_raw;
    esp_err_t ret = ESP_OK;
    float sum_temp = 0.0f;
    float sum_squared = 0.0f;
    float temp_samples[SAMPLE_COUNT_TEMP];

    // ret = adc_oneshot_read(adc_handle, sensor_ch, &adc_raw);
    // if (ret != ESP_OK) {
    //     return CE_ERROR_SENSOR_READ;
    // }
    // float voltage = (ADC_VREF*adc_raw)/4095.0;
    // float resistance = (voltage * TMEP_SENSOR_R_25_DEGREE) / (ADC_VREF - voltage);
    // float temp_c = 1.0/((1.0/(273.15+25.0)) + (log(resistance/TMEP_SENSOR_R_25_DEGREE)/3435.0)) - 273.15;

    // *temp = temp_c;

    // 여러 번 샘플링
    for (int i = 0; i < SAMPLE_COUNT_TEMP; i++) {
        ret = adc_oneshot_read(adc_handle, sensor_ch, &adc_raw);
        if (ret != ESP_OK) {
            return CE_ERROR_SENSOR_READ;
        }

        float voltage = (ADC_VREF * adc_raw) / 4095.0f;
        float resistance = (voltage * TMEP_SENSOR_R_25_DEGREE) / (ADC_VREF - voltage);
        float temp_c = 1.0f / ((1.0f / (273.15f + 25.0f)) + (log(resistance / TMEP_SENSOR_R_25_DEGREE) / 3435.0f)) - 273.15f;
        
        temp_samples[i] = temp_c;
        sum_temp += temp_c;
        
        // 다음 샘플링을 위한 짧은 대기
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 평균 온도 계산
    float mean_temp = sum_temp / SAMPLE_COUNT_TEMP;

    // RMS 오차 계산
    for (int i = 0; i < SAMPLE_COUNT_TEMP; i++) {
        float diff = temp_samples[i] - mean_temp;
        sum_squared += diff * diff;
    }
    float rms_error = sqrt(sum_squared / SAMPLE_COUNT_TEMP);

    // RMS 오차가 큰 샘플들을 제외하고 평균 재계산
    sum_temp = 0.0f;
    int valid_samples = 0;
    for (int i = 0; i < SAMPLE_COUNT_TEMP; i++) {
        if (fabs(temp_samples[i] - mean_temp) <= 2.0f * rms_error) {  // 2*RMS 범위 내의 샘플만 사용
            sum_temp += temp_samples[i];
            valid_samples++;
        }
    }

    if (valid_samples > 0) {
        *temp = sum_temp / valid_samples;
    } else {
        *temp = mean_temp;  // 모든 샘플이 제외된 경우 원래 평균값 사용
    }

    return CE_OK;

}

void ce_current_read(float * current, uint8_t sensor_ch, adc_oneshot_unit_handle_t adc_handle)
{
    int adc_raw;
    float sum_sq = 0.0;
    float voltage_rms = 0.0;

   for (int i = 0; i < SAMPLE_COUNT_CURRENT; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, sensor_ch, &adc_raw);
        if (ret != ESP_OK) continue;

        // ADC → 전압
        float voltage = ((float)adc_raw / 4095.0f) * ADC_VREF;

        // 센서 기준 전압(0A 기준) 보정
        float ac_voltage = voltage - ACS712_ZERO;
        voltage_rms += voltage;

        // RMS 제곱합 누적
        sum_sq += ac_voltage * ac_voltage;

        // 10kHz 샘플링
        esp_rom_delay_us(100);
    }

    // 전압 RMS → 전류 RMS
    float vrms = sqrtf(sum_sq / SAMPLE_COUNT_CURRENT);

    float vrms_offset = voltage_rms / SAMPLE_COUNT_CURRENT;
    // printf("vrms_offset: %f\n", vrms_offset);
    *current = vrms / ACS712_5A_RATIO;  // I = Vrms / (mV per A)

}

void ce_humi_temp_read(float * humi_temp, uart_port_t uart_num)
{
    ce_humi_temp_tx_frame_t frame = {
        .address = 0x00,
        .function_code = 0x03,
        .inital_addr_l = 0x00,
        .inital_addr_h = 0x00,
        .data_length_l = 0x00,
        .data_length_h = 0x02,
        .check_l = 0xC4,
        .check_h = 0x0B
    };

    uart_write_bytes(uart_num, &frame, sizeof(ce_humi_temp_tx_frame_t));


    ce_humi_temp_rx_frame_t humi_temp_rx_frame;
    int len = uart_read_bytes(uart_num, &humi_temp_rx_frame, sizeof(humi_temp_rx_frame), 1000 / portTICK_PERIOD_MS);

    humi_temp[0] = ((uint16_t)humi_temp_rx_frame.data_humi[0] << 8) | humi_temp_rx_frame.data_humi[1];
    humi_temp[1] = ((uint16_t)humi_temp_rx_frame.data_temp[0] << 8) | humi_temp_rx_frame.data_temp[1];
    // printf("len: %d, Humidity: %.2f%%, Temperature: %.2f°C\n", len, humi_temp[0] / 10.0f, humi_temp[1] / 10.0f);

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

    if (uart_driver_install(HUMI_TEMP_UART_PORT_NUM, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(HUMI_TEMP_UART_PORT_NUM, &uart_config);
    uart_set_pin(HUMI_TEMP_UART_PORT_NUM, HUMI_TEMP_UART_TX, HUMI_TEMP_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // // 온습도 센서 설정
    // ce_humi_temp_frame_t humi_temp_frame = {
    //     .address = 0x01,
    //     .function_code = 0x03,
    //     .inital_addr_l = 0x01,
    //     .inital_addr_h = 0x01,
    //     .data_length_l = 0x01,
    //     .data_length_h = 0x02,
    //     .check_l = 0x00,
    //     .check_h = 0x00
    // };

    ret = adc_oneshot_config_channel(*adc1_handle, CURRENT_ADC_CHANNLE, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }

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
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_7, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    ret = adc_oneshot_config_channel(*adc1_handle, TMEP_ADC_CHANNLE_8, &adc1_chan_config);
    if (ret != ESP_OK) {
        return CE_ERROR_SENSOR_INIT;
    }
    return CE_OK;
}

static void ce_temperatures_task_nonstop(void * params)
{
    adc_oneshot_unit_handle_t adc1_handle = *((adc_oneshot_unit_handle_t *)params);
    ce_env_sensor_data_t env_sensor_data;

    while(1) 
    {
        xSemaphoreTake(temp_sample_sem, portMAX_DELAY);

        for (int i = 0; i < 6; i++)
        {
#if SENSOR_REAL
            ce_temperatures_read(&env_sensor_data.temp[i], i+1, adc1_handle);
            // printf("temp[%d]: %f\n", i, env_sensor_data.temp[i]);

#else
            temp[i] = esp_random() % 100; // 임의 데이터 생성
#endif
        }

        ce_current_read(&env_sensor_data.current[0], CURRENT_ADC_CHANNLE, adc1_handle);
        // printf("COMP Current: %f\n", env_sensor_data.current[0]);

        env_sensor_data.humi_temp[0] = 0;
        env_sensor_data.humi_temp[1] = 0;
        // ce_humi_temp_read(env_sensor_data.humi_temp, HUMI_TEMP_UART_PORT_NUM);
        ce_temperatures_read(&env_sensor_data.humi_temp[0], TMEP_ADC_CHANNLE_8, adc1_handle);
        ce_relay_temp_control_global.current_temp = env_sensor_data.humi_temp[0];

        printf("current_temp: %f\n", ce_relay_temp_control_global.current_temp);

        xQueueSend(ce_temperatures_queue_global, &env_sensor_data, 0);
        ce_env_sensor_data_global = env_sensor_data;
    }
}

static void ce_temperatures_task_onoff(void *params)
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

    init_temp_sampling_timer();

    
    if (xTaskCreatePinnedToCore(ce_temperatures_task_nonstop, "ce_temperatures_task", 4000, &adc1_handle, 5, &ce_temperatures_task_handle, 0) != pdPASS)
    {
        return CE_ERROR_TASK_CREATE;
    }

    
    return CE_OK;
}