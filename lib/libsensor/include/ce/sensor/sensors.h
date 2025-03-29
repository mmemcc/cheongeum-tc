#ifndef SENSOR_SENSORS_H
#define SENSOR_SENSORS_H


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







#endif