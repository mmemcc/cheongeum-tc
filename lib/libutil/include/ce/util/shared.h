#ifndef UTIL_SHARED_H
#define UTIL_SHARED_H

#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


#include <ce/util/error.h>

/**
    전역변수 업데이트 함수
	@param value_for_update 업데이트할 전역변수의 주소
	@param data 업데이트할 데이터의 주소
	@param size 업데이트할 데이터의 크기
	@param mutex 업데이트할 데이터의 mutex
	@return ce_error_t
*/
ce_error_t ce_global_update(void * value_for_update, void * data, size_t size, SemaphoreHandle_t mutex);

#endif