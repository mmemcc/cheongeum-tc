
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

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
        if (relay_connection[i] == false) {
            continue;
        }
        if (i == RELAY_COMP) {
            relay_cfg[RELAY_COMP].relay_pin = RELAY_COMP_PIN;
            relay_cfg[RELAY_COMP].relay_load = RELAY_COMP;
        } else if (i == RELAY_FAN) {
            relay_cfg[RELAY_FAN].relay_pin = RELAY_FAN_PIN;
            relay_cfg[RELAY_FAN].relay_load = RELAY_FAN;
        } else if (i == RELAY_DEF) {
            relay_cfg[RELAY_DEF].relay_pin = RELAY_DEF_PIN;
            relay_cfg[RELAY_DEF].relay_load = RELAY_DEF;
        } else if (i == RELAY_AUX_LIGHT) {
            relay_cfg[RELAY_AUX_LIGHT].relay_pin = RELAY_AUX_LIGHT_PIN;
            relay_cfg[RELAY_AUX_LIGHT].relay_load = RELAY_AUX_LIGHT;
        }
        relay_cfg[i].relay_connection = relay_connection[i];
        relay_cfg[i].relay_state = false;

        if (ce_relay_pin_set(&relay_cfg[i]) != CE_OK) {
            return CE_ERROR_RELAY_INIT;
        }
    }

    memcpy(ce_relay_state_global, relay_cfg, sizeof(relay_cfg));

    for (uint8_t i = 0; i < MAX_RELAYS; i++) {
        if (relay_connection[i] == false) {
            continue;
        }
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
    
    return CE_OK;
}

static void IRAM_ATTR relay_gpio_isr_handler(void *arg)
{
    int pin = (int)arg;

    // 상태 읽기
    int level = gpio_get_level(pin);

    // 간단하게 전역 상태 저장 (또는 Queue, EventBits 사용)
    ce_relay_state_global[RELAY_COMP].relay_state = (level == 1);  // 전역 변수로 예시 (주의: atomic하게 다뤄야 안전)
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group,
                              RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3,
                              &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

ce_error_t ce_relay_pin_set(ce_relay_state_t *relay_cfg)
{
#if RELAY_STATE == 0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_COMP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RELAY_COMP_PIN, relay_gpio_isr_handler, (void *)RELAY_COMP_PIN);

#elif RELAY_STATE == 1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_FAN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

#elif RELAY_STATE == -1

#endif
    return CE_OK;
}


static void relay_control_task(void *pvParameters)
{
    bool relay_state = false;

    while (1)
    {
#if RELAY_STATE == -1
        relay_set_cnt++;
        if (relay_set_cnt == 10) {
            relay_state = true;
            
            ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state, sizeof(relay_state), ce_relay_state_global[RELAY_COMP].relay_mutex);
            // xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_BIT);
            EventBits_t bit = xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
        }
        if (relay_set_cnt == 20) {
            relay_state = false;
            ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state, sizeof(relay_state), ce_relay_state_global[RELAY_COMP].relay_mutex);
            // xEventGroupClearBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_BIT);
            xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
            relay_set_cnt = 0;
        }
#elif RELAY_STATE == 0
        // ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_state, &relay_state, sizeof(relay_state), ce_relay_state_global[RELAY_COMP].relay_mutex);
        // EventBits_t bit = xEventGroupSetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT1 | RELAY_STATE_CHANGE_BIT2 | RELAY_STATE_CHANGE_BIT3);
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