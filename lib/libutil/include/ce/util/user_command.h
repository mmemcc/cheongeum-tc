#ifndef USER_COMMAND_H
#define USER_COMMAND_H
        
#include <esp_console.h>
#include <esp_err.h>

// 명령어 등록을 위한 초기화 함수
esp_err_t ce_cmd_init(void);

// 콘솔 태스크 시작 함수
esp_err_t ce_cmd_start_console_task(void);

#endif