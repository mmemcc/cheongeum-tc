#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_console.h>
#include <driver/uart.h>

#include <ce/relay/control.h>
#include <ce/operation/test.h>
#include <ce/util/shared.h>
#include <ce/operation/mode.h>

#if RELAY_STATE == -1
    static int relay_set_cnt = 0;
#endif

ce_error_t ce_relay_control_init(bool relay_connection[MAX_RELAYS])
{
    ce_relay_state_t relay_cfg[MAX_RELAYS];

    for (uint8_t i = 0; i < MAX_RELAYS; i++) {
        // if (relay_connection[i] == false) {
        //     relay_cfg[i].relay_connection = relay_connection[i];
        //     relay_cfg[i].relay_state = false;
        //     continue;
        // }
        if (i == RELAY_COMP) {
            relay_cfg[RELAY_COMP].relay_pin = RELAY_COMP_PIN;
            relay_cfg[RELAY_COMP].relay_load = RELAY_COMP;
            relay_cfg[RELAY_COMP].relay_control_case = RELAY_CONTROL_CASE_AUTO;
            relay_cfg[RELAY_COMP].relay_control_value = false;
        } else if (i == RELAY_FAN) {
            relay_cfg[RELAY_FAN].relay_pin = RELAY_FAN_PIN;
            relay_cfg[RELAY_FAN].relay_load = RELAY_FAN;
            relay_cfg[RELAY_FAN].relay_control_case = RELAY_CONTROL_CASE_AUTO;
            relay_cfg[RELAY_FAN].relay_control_value = false;
        } else if (i == RELAY_DEF) {
            relay_cfg[RELAY_DEF].relay_pin = RELAY_DEF_PIN;
            relay_cfg[RELAY_DEF].relay_load = RELAY_DEF;
            relay_cfg[RELAY_DEF].relay_control_case = RELAY_CONTROL_CASE_AUTO;
            relay_cfg[RELAY_DEF].relay_control_value = false;
        } else if (i == RELAY_AUX_LIGHT) {
            relay_cfg[RELAY_AUX_LIGHT].relay_pin = RELAY_AUX_LIGHT_PIN;
            relay_cfg[RELAY_AUX_LIGHT].relay_load = RELAY_AUX_LIGHT;
            relay_cfg[RELAY_AUX_LIGHT].relay_control_case = RELAY_CONTROL_CASE_AUTO;
            relay_cfg[RELAY_AUX_LIGHT].relay_control_value = false;
        }
        relay_cfg[i].relay_connection = relay_connection[i];
        relay_cfg[i].relay_state = false;

        if (ce_relay_pin_set(&relay_cfg[i]) != CE_OK) {
            return CE_ERROR_RELAY_INIT;
        }
    }

    memcpy(ce_relay_state_global, relay_cfg, sizeof(relay_cfg));

    for (uint8_t i = 0; i < MAX_RELAYS; i++) {
        // if (relay_connection[i] == false) {
        //     continue;
        // }
        if (i == RELAY_COMP) {
            ce_relay_state_global[RELAY_COMP].relay_mutex = xSemaphoreCreateMutex();
            ce_relay_state_global[RELAY_COMP].relay_state_set_event_group = xEventGroupCreate();
        } else if (i == RELAY_FAN) {
            ce_relay_state_global[RELAY_FAN].relay_mutex = xSemaphoreCreateMutex();
            ce_relay_state_global[RELAY_FAN].relay_state_set_event_group = xEventGroupCreate();
        } else if (i == RELAY_DEF) {
            ce_relay_state_global[RELAY_DEF].relay_mutex = xSemaphoreCreateMutex();
            ce_relay_state_global[RELAY_DEF].relay_state_set_event_group = xEventGroupCreate();
        } else if (i == RELAY_AUX_LIGHT) {
            ce_relay_state_global[RELAY_AUX_LIGHT].relay_mutex = xSemaphoreCreateMutex();
            ce_relay_state_global[RELAY_AUX_LIGHT].relay_state_set_event_group = xEventGroupCreate();
        }
    }

    ce_relay_temp_control_global.current_temp = 0;
    ce_relay_temp_control_global.target_temp = 0;
    
    return CE_OK;
}

// static void IRAM_ATTR relay_gpio_isr_handler(void *arg)
// {
//     int pin = (int)arg;

//     // 상태 읽기
//     int level = gpio_get_level(pin);

//     // 간단하게 전역 상태 저장 (또는 Queue, EventBits 사용)
//     ce_relay_state_global[RELAY_COMP].relay_state = (level == 1);
    
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// #if RELAY_STATE == -1
//     xEventGroupSetBitsFromISR(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group,
//                               RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3,
//                               &xHigherPriorityTaskWoken);
// #endif
//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

ce_error_t ce_relay_pin_set(ce_relay_state_t *relay_cfg)
{
#if RELAY_STATE == 0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_COMP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RELAY_COMP_PIN, relay_gpio_isr_handler, (void *)RELAY_COMP_PIN);

#elif RELAY_STATE == 1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << relay_cfg->relay_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(relay_cfg->relay_pin, 0);

#elif RELAY_STATE == -1

#endif
    return CE_OK;
}


static void relay_control_task(void *pvParameters)
{
    // int level = gpio_get_level(RELAY_COMP_PIN);
    // ce_relay_state_global[RELAY_COMP].relay_state = (level == 1);
    // gpio_set_level(RELAY_FAN_PIN, 0);
    // gpio_set_level(RELAY_DEF_PIN, 0);
    // gpio_set_level(RELAY_AUX_LIGHT_PIN, 0);
    while (1)
    {
        ce_relay_state_set_t relay_state_set = {
            .relay_connection = {ce_relay_state_global[RELAY_COMP].relay_connection,
                                 ce_relay_state_global[RELAY_FAN].relay_connection,
                                 ce_relay_state_global[RELAY_DEF].relay_connection,
                                 ce_relay_state_global[RELAY_AUX_LIGHT].relay_connection},
            .relay_state = {ce_relay_state_global[RELAY_COMP].relay_state,
                            ce_relay_state_global[RELAY_FAN].relay_state,
                            ce_relay_state_global[RELAY_DEF].relay_state,
                            ce_relay_state_global[RELAY_AUX_LIGHT].relay_state},
            .relay_control_case = {ce_relay_state_global[RELAY_COMP].relay_control_case,
                                   ce_relay_state_global[RELAY_FAN].relay_control_case,
                                   ce_relay_state_global[RELAY_DEF].relay_control_case,
                                   ce_relay_state_global[RELAY_AUX_LIGHT].relay_control_case},
            .relay_control_value = {ce_relay_state_global[RELAY_COMP].relay_control_value,
                                    ce_relay_state_global[RELAY_FAN].relay_control_value,
                                    ce_relay_state_global[RELAY_DEF].relay_control_value,
                                    ce_relay_state_global[RELAY_AUX_LIGHT].relay_control_value}
        };

        // printf("relay_state_global[RELAY_COMP].relay_connection: %d\n", ce_relay_state_global[RELAY_COMP].relay_connection);
        // printf("relay_state_global[RELAY_COMP].relay_state: %d\n", ce_relay_state_global[RELAY_COMP].relay_state);
        // printf("relay_state_global[RELAY_COMP].relay_control_case: %d\n", ce_relay_state_global[RELAY_COMP].relay_control_case);
        // printf("relay_state_global[RELAY_COMP].relay_control_value: %d\n", ce_relay_state_global[RELAY_COMP].relay_control_value);

      

#if RELAY_STATE == -1
        relay_state_set.relay_state[RELAY_COMP] = false;
        relay_set_cnt++;
        if (relay_set_cnt == 10) {
            relay_state_set.relay_state[RELAY_COMP] = true;
            
            ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state_set.relay_state[RELAY_COMP], sizeof(relay_state_set.relay_state[RELAY_COMP]), ce_relay_state_global[RELAY_COMP].relay_mutex);
            // xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_BIT);
#if !SENSOR_NONSTOP
            EventBits_t bit = xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
#else
            xQueueSend(ce_relay_state_queue_global, &relay_state_set, 0);
#endif
        }
        if (relay_set_cnt == 20) {
            relay_state_set.relay_state[RELAY_COMP] = false;
            ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state_set.relay_state[RELAY_COMP], sizeof(relay_state_set.relay_state[RELAY_COMP]), ce_relay_state_global[RELAY_COMP].relay_mutex);
            // xEventGroupClearBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_BIT);
#if !SENSOR_NONSTOP
            xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
#else
            xQueueSend(ce_relay_state_queue_global, &relay_state_set, 0);
#endif
            relay_set_cnt = 0;
        }
#elif RELAY_STATE == 0
        // ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state, sizeof(relay_state), ce_relay_state_global[RELAY_COMP].relay_mutex);
        // EventBits_t bit = xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
        xQueueSend(ce_relay_state_queue_global, &relay_state_set, 0);
#elif RELAY_STATE == 1
        // if (ce_relay_state_global[RELAY_COMP].relay_state == true) {
        //     gpio_set_level(RELAY_COMP_PIN, 0);
        //     ce_relay_state_global[RELAY_COMP].relay_state = false;
        // } else {
        //     gpio_set_level(RELAY_COMP_PIN, 1);
        //     ce_relay_state_global[RELAY_COMP].relay_state = true;
        // }

        
        switch (relay_state_set.relay_control_case[RELAY_COMP]) {
            case RELAY_CONTROL_CASE_MANUAL:
                gpio_set_level(RELAY_COMP_PIN, relay_state_set.relay_control_value[RELAY_COMP]);
                ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state_set.relay_control_value[RELAY_COMP], sizeof(relay_state_set.relay_control_value[RELAY_COMP]), ce_relay_state_global[RELAY_COMP].relay_mutex);
                break;
            case RELAY_CONTROL_CASE_AUTO:
                bool state = false;
                if (ce_relay_temp_control_global.current_temp > ce_relay_temp_control_global.target_temp + 2) 
                {
                    state = true;
                    gpio_set_level(RELAY_COMP_PIN, 1);
                } 
                else if (ce_relay_temp_control_global.current_temp < ce_relay_temp_control_global.target_temp - 2)
                {
                    state = false;
                    gpio_set_level(RELAY_COMP_PIN, 0);
                }
                relay_state_set.relay_state[RELAY_COMP] = state;
                ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state_set.relay_state[RELAY_COMP], sizeof(relay_state_set.relay_state[RELAY_COMP]), ce_relay_state_global[RELAY_COMP].relay_mutex);
                break;
            default:
                break;
        }

        switch (relay_state_set.relay_control_case[RELAY_FAN]) {
            case RELAY_CONTROL_CASE_MANUAL:
                gpio_set_level(RELAY_FAN_PIN, relay_state_set.relay_control_value[RELAY_FAN]);
                ce_global_update(&ce_relay_state_global[RELAY_FAN].relay_state, &relay_state_set.relay_control_value[RELAY_FAN], sizeof(relay_state_set.relay_control_value[RELAY_FAN]), ce_relay_state_global[RELAY_FAN].relay_mutex);
                break;
            case RELAY_CONTROL_CASE_AUTO:
                bool state = false;
                if (ce_relay_temp_control_global.current_temp > ce_relay_temp_control_global.target_temp + 2) 
                {
                    state = true;
                    gpio_set_level(RELAY_FAN_PIN, 1);
                } 
                else if (ce_relay_temp_control_global.current_temp < ce_relay_temp_control_global.target_temp - 2) 
                {
                    state = false;
                    gpio_set_level(RELAY_FAN_PIN, 0);
                }
                relay_state_set.relay_state[RELAY_FAN] = state;
                ce_global_update(&ce_relay_state_global[RELAY_FAN].relay_state, &relay_state_set.relay_state[RELAY_FAN], sizeof(relay_state_set.relay_state[RELAY_FAN]), ce_relay_state_global[RELAY_FAN].relay_mutex);
                break;
            default:
                break;
        }

        switch (relay_state_set.relay_control_case[RELAY_DEF]) {
            case RELAY_CONTROL_CASE_MANUAL:
                gpio_set_level(RELAY_DEF_PIN, relay_state_set.relay_control_value[RELAY_DEF]);
                ce_global_update(&ce_relay_state_global[RELAY_DEF].relay_state, &relay_state_set.relay_control_value[RELAY_DEF], sizeof(relay_state_set.relay_control_value[RELAY_DEF]), ce_relay_state_global[RELAY_DEF].relay_mutex);
                break;
            case RELAY_CONTROL_CASE_AUTO:
                break;
            default:
                break;
        }

        switch (relay_state_set.relay_control_case[RELAY_AUX_LIGHT]) {
            case RELAY_CONTROL_CASE_MANUAL:
                gpio_set_level(RELAY_AUX_LIGHT_PIN, relay_state_set.relay_control_value[RELAY_AUX_LIGHT]);
                ce_global_update(&ce_relay_state_global[RELAY_AUX_LIGHT].relay_state, &relay_state_set.relay_control_value[RELAY_AUX_LIGHT], sizeof(relay_state_set.relay_control_value[RELAY_AUX_LIGHT]), ce_relay_state_global[RELAY_AUX_LIGHT].relay_mutex);
                break;
            case RELAY_CONTROL_CASE_AUTO:
                break;
            default:
                break;
        }

        xQueueSend(ce_relay_state_queue_global, &relay_state_set, 0);

#endif

        vTaskDelay(pdMS_TO_TICKS(CE_CONTROL_RELAY_TASK_PERIOD));
    }
}

ce_error_t ce_relay_init(void)
{
    ce_error_t err = CE_OK;
    bool relay_connection[MAX_RELAYS] = {true, false, false, false};
    
    err = ce_relay_control_init(relay_connection);

    if (xTaskCreatePinnedToCore(relay_control_task, "relay_control_task", CE_CONTROL_RELAY_TASK_STACK_SIZE, NULL, CE_CONTROL_RELAY_TASK_PRIORITY, NULL, 0) != pdPASS) {
        err = CE_ERROR_TASK_CREATE;
    }
    return err;
}