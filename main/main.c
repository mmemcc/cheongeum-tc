
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

// 전역 변수 초기화
ce_relay_state_t ce_relay_state_global[MAX_RELAYS];
EventGroupHandle_t ce_mode_event_group_global;
QueueHandle_t ce_temperatures_queue_global;
QueueHandle_t ce_sensors_info_global;
QueueHandle_t ce_acc_queue_global;
QueueSetHandle_t mqtt_queue_set;


void app_main(void) {

    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());

    // mode, task 초기화화
    ce_mode_event_group_global = xEventGroupCreate();

    ce_mode_init();
    
    ce_relay_init();

    wifi_init_sta();

    
    ce_sensors_info_global = xQueueCreate(2, sizeof(ce_sensor_info_t));
    ce_temperatures_queue_global = xQueueCreate(2, sizeof(float)*6);
    ce_acc_queue_global = xQueueCreate(10, sizeof(sensor_acc_t));

    mqtt_queue_set = xQueueCreateSet(20);
    xQueueAddToSet(ce_acc_queue_global, mqtt_queue_set);
    xQueueAddToSet(ce_temperatures_queue_global, mqtt_queue_set);

    if(ce_kx022acr_init_i2c() != CE_OK) {
        printf("ce_kx022acr_init_i2c failed\n");
    }
    if(ce_temperatures_init() != CE_OK) {
        printf("ce_temperatures_init failed\n");
    }
    // if(ce_server_upload_init() != CE_OK) {
    //     printf("ce_server_upload_init failed\n");
    // }

    mqtt_init();
}