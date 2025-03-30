#ifndef SENSOR_SENSORS_H
#define SENSOR_SENSORS_H


#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define SENSOR_FFT_SIZE 1024
#define SENSOR_SAMPLE_RATE 1600

typedef struct {
    float amplitude;
    float frequency;
} fft_result_t;

typedef struct {
    float x;
    float y;
    float z;
} sensor_acc_t;

typedef struct {
    uint64_t start_time;     // 시작 시간 (us 단위)
    uint64_t elapsed_time;  // 경과 시간 (us 단위)
    uint8_t relay_state;     // 릴레이 상태 (0 또는 1)
    uint32_t samp_num_vib;   // 수집된 진동 샘플 개수
} ce_sensor_info_t;

extern QueueHandle_t ce_sensors_info_global;



#endif