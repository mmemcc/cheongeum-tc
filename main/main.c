
#include <stdio.h>
    
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <esp_netif.h>

#include <ce/operation/mode.h>
#include <ce/relay/control.h>
#include <ce/sensor/kx022acr.h>
#include <ce/server/wifi.h>
#include <ce/sensor/temperatures.h>
#include <ce/sensor/sensors.h>
#include <ce/server/server_upload.h>
#include <ce/server/mqtt_server.h>
#include <ce/tc/display.h>
#include <ce/tc/tc_state.h>
// 전역 변수 초기화
ce_relay_state_t ce_relay_state_global[MAX_RELAYS];
ce_relay_temp_control_t ce_relay_temp_control_global;
ce_env_sensor_data_t ce_env_sensor_data_global;
tc_state_t tc_state_global;

EventGroupHandle_t ce_mode_event_group_global;
QueueHandle_t ce_temperatures_queue_global;
QueueHandle_t ce_sensors_info_global;
QueueHandle_t ce_acc_queue_global;
QueueHandle_t ce_relay_state_queue_global;
QueueSetHandle_t mqtt_queue_set;


void app_main(void) {

    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());

    tc_state_global.tc_state_mutex = xSemaphoreCreateMutex();

    // TODO: TC 상태 관리 초기화 필요
    tc_state_global.mqtt_connected = 0;
    tc_state_global.wifi_connected = 0;
    tc_state_global.alarm_connected = 0;
    tc_state_global.warning_connected = 0;
    tc_state_global.machine_condition = 0;

    // mode, task 초기화화
    ce_mode_event_group_global = xEventGroupCreate();
    ce_relay_temp_control_global.relay_temp_mutex = xSemaphoreCreateMutex();
    ce_mode_init();
    
    wifi_init_sta();

    
    ce_sensors_info_global = xQueueCreate(2, sizeof(ce_sensor_info_t));
    ce_temperatures_queue_global = xQueueCreate(2, sizeof(ce_env_sensor_data_t));
    ce_acc_queue_global = xQueueCreate(10, sizeof(sensor_acc_t));
    ce_relay_state_queue_global = xQueueCreate(1, sizeof(ce_relay_state_set_t));
    

    mqtt_queue_set = xQueueCreateSet(20);
    xQueueAddToSet(ce_acc_queue_global, mqtt_queue_set);
    xQueueAddToSet(ce_temperatures_queue_global, mqtt_queue_set);
    xQueueAddToSet(ce_relay_state_queue_global, mqtt_queue_set);
    
    // if(ce_st7789_init() != CE_OK) {
    //     printf("ce_st7789_init failed\n");
    // }

    ce_relay_init();
    // if(ce_kx022acr_init_i2c() != CE_OK) {
    //     printf("ce_kx022acr_init_i2c failed\n");
    // }
    if(ce_kx022acr_init_spi() != CE_OK) {
        printf("ce_kx022acr_init_spi failed\n");
    }
    if(ce_temperatures_init() != CE_OK) {
        printf("ce_temperatures_init failed\n");
    }
    // if(ce_server_upload_init() != CE_OK) {
    //     printf("ce_server_upload_init failed\n");
    // }

    mqtt_init();
}