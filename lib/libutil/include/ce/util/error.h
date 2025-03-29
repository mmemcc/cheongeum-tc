#ifndef UTILL_ERROR_H
#define UTILL_ERROR_H


/**
 * @brief Error codes
 * @note 에러 코드 범위
 * @note 0 ~ 99 : 기본 에러 코드
 * @note 100 ~ 199 : 릴레이이 에러 코드
 * @note 200 ~ 299 : 센서 에러 코드
 * @note 300 ~ 399 : 통신 에러 코드
 * 
*/
typedef enum 
{
    CE_OK = 0,

    CE_ERROR_MUTEX_TAKE = 1,
    CE_ERROR_MUTEX_GIVE = 2,
    CE_ERROR_TASK_CREATE = 3,
    CE_ERROR_INVALID_SIZE = 4,

    CE_ERROR_RELAY_INIT = 100,
    CE_ERROR_RELAY_SET = 101,
    
    CE_ERROR_SENSOR_INIT = 200,
    CE_ERROR_SENSOR_READ = 201,
    CE_ERROR_SENSOR_WRITE = 202,


} ce_error_t;




#endif 