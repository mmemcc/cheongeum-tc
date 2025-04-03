#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include <ce/util/error.h>

/* TODO: Relay 개수와 연결 장비에 따라 relay contol 조정 구현
    - Relay 개수: 최대 6개
    - 연결 장비 종류: Comp, Fan, Def, AUX/Light
    - Relay 제어 pin 초기화
    - Relay 제어 pin 마다 연결 장비 설정
    - 사용 안하는 pin은 초기화 하지 않음
*/


#define MAX_RELAYS 4

#define RELAY_COMP_PIN 45
#define RELAY_FAN_PIN 3
#define RELAY_DEF_PIN 19
#define RELAY_AUX_LIGHT_PIN 20

#define RELAY_COMP 0
#define RELAY_FAN 1
#define RELAY_DEF 2
#define RELAY_AUX_LIGHT 3


#define RELAY_STATE_CHANGE_BIT1 BIT0
#define RELAY_STATE_CHANGE_BIT2 BIT1
#define RELAY_STATE_CHANGE_BIT3 BIT2

typedef struct 
{
    uint8_t relay_pin; // GPIO pin number
    uint8_t relay_load; // 0: comp, 1: fan, 2: def, 3: aux/light
    bool relay_connection; // 0: not connected, 1: connected

    bool relay_state; // 0: off, 1: on
    EventGroupHandle_t relay_state_set_event_group; // Event Group
    SemaphoreHandle_t relay_mutex; // Mutex
} ce_relay_state_t;


// [comp, fan, def, aux/light]
typedef struct
{
    bool relay_connection[MAX_RELAYS]; // 0: not connected, 1: connected
    bool relay_state[MAX_RELAYS]; // 0: off, 1: on
} ce_relay_state_set_t;

extern ce_relay_state_t ce_relay_state_global[MAX_RELAYS];


extern QueueHandle_t ce_relay_state_queue_global;

/**
 * @brief 릴레이 제어 초기화
 * @param num_relays 릴레이 개수
 * @param relay_connection 연결 상태 배열
 * @return ce_error_t 에러 코드
 */
ce_error_t ce_relay_control_init(bool relay_connection[MAX_RELAYS]);

/**
 * @brief 릴레이 제어 설정
 * @note RELAY_STATE에 따라 릴레이 제어 설정
 * @note RELAY_STATE == 0: GPIO INPUT 설정
 * @note RELAY_STATE == 1: GPIO OUTPUT 설정
 * @note RELAY_STATE == -1: GPIO 설정 없음
 * @param relay_cfg 릴레이 설정 구조체
 * @return ce_error_t 에러 코드
 */
ce_error_t ce_relay_pin_set(ce_relay_state_t *relay_cfg);

ce_error_t ce_relay_init(void);

#endif