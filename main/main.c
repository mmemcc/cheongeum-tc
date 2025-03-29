
#include <stdio.h>
    
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <ce/operation/mode.h>
#include <ce/relay/control.h>
#include <ce/sensor/kx022acr.h>

// 전역 변수 초기화
ce_relay_state_t ce_relay_state_global[MAX_RELAYS];
EventGroupHandle_t ce_mode_event_group_global;

void app_main(void) {

    // mode, task 초기화화
    ce_mode_event_group_global = xEventGroupCreate();

    ce_mode_init();
    
    ce_relay_init();
    
    ce_error_t res = ce_kx022acr_init_i2c();
    if (res != CE_OK) {
        printf("res: %d\n", res);
    }
}