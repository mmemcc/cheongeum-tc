#ifndef SENSOR_SENSORS_H
#define SENSOR_SENSORS_H


#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define SENSOR_FFT_SIZE 1024
#define SENSOR_SAMPLE_RATE 1600

#define ADC_VREF 3.1

typedef struct {
    float amplitude;
    float frequency;
} fft_result_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_acc_t;

typedef struct {
    uint64_t start_time;     // 시작 시간 (us 단위)
    uint64_t elapsed_time;  // 경과 시간 (us 단위)
    uint8_t relay_state;     // 릴레이 상태 (0 또는 1)
    uint32_t samp_num_vib;   // 수집된 진동 샘플 개수
} ce_sensor_info_t;

typedef struct {
    float temp[6];           // 온도 센서 값
    float current[2];          // 전류 센서 값
    float humi_temp[2];         // 냉장고 내부 온습도 센서 값
} ce_env_sensor_data_t;

extern QueueHandle_t ce_sensors_info_global;
extern QueueHandle_t ce_acc_queue_global;
extern QueueSetHandle_t mqtt_queue_set;

#endif