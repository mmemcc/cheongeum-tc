#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cJSON.h>

#include <ce/sensor/kx022acr.h>
#include <ce/sensor/sensors.h>
#include <ce/sensor/temperatures.h>
#include <ce/relay/control.h>
#include <ce/server/mqtt_server.h>
#include <ce/util/shared.h>
#include <ce/tc/tc_state.h>

#if SENSOR_NONSTOP
    int16_t *psram_buffer;
#endif

#define MQTT_ACC_DATA_SIZE 1600

static const char *TAG = "mqtt";

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    uint8_t mqtt_connected = 0;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        
        msg_id = esp_mqtt_client_subscribe(client, "esp32/target_temp", 1);
        ESP_LOGI(TAG, "sent subscribe successful for target_temp, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "esp32/manual_relay", 1);
        ESP_LOGI(TAG, "sent subscribe successful for manual_relay, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "esp32/mode", 1);
        ESP_LOGI(TAG, "sent subscribe successful for mode, msg_id=%d", msg_id);

        mqtt_connected = 1;
        ce_global_update(&tc_state_global.mqtt_connected, &mqtt_connected, sizeof(uint8_t), tc_state_global.tc_state_mutex);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = 2;
        ce_global_update(&tc_state_global.mqtt_connected, &mqtt_connected, sizeof(uint8_t), tc_state_global.tc_state_mutex);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        mqtt_connected = 4;
        ce_global_update(&tc_state_global.mqtt_connected, &mqtt_connected, sizeof(uint8_t), tc_state_global.tc_state_mutex);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        mqtt_connected = 4;
        ce_global_update(&tc_state_global.mqtt_connected, &mqtt_connected, sizeof(uint8_t), tc_state_global.tc_state_mutex);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        mqtt_connected = 4;
        ce_global_update(&tc_state_global.mqtt_connected, &mqtt_connected, sizeof(uint8_t), tc_state_global.tc_state_mutex);

        cJSON *root = cJSON_Parse(event->data);
        if (root == NULL) {
            ESP_LOGE(TAG, "Failed to parse JSON data");
            return;
        }

        if (strncmp(event->topic, "esp32/target_temp", event->topic_len) == 0) {
            // 목표 온도 데이터 처리
            cJSON *target_temp = cJSON_GetObjectItem(root, "target_temp");
            if (target_temp != NULL && target_temp->type == cJSON_Number) {
                float temp_value = (float)target_temp->valuedouble;
                ce_global_update(&ce_relay_temp_control_global.target_temp, &temp_value, sizeof(temp_value), ce_relay_temp_control_global.relay_temp_mutex);
                ESP_LOGI(TAG, "목표 온도 수신: %f", temp_value);
            } else {
                ESP_LOGE(TAG, "Invalid target temperature data");
            }
            cJSON_Delete(root);
        }
        else if (strncmp(event->topic, "esp32/manual_relay", event->topic_len) == 0) {
            // 수동 릴레이 제어 데이터 처리
            cJSON *relay_num = cJSON_GetObjectItem(root, "relay");
            uint8_t relay_num_value = relay_num->valuestring[5] - '0' - 1;
            cJSON *relay_control = cJSON_GetObjectItem(root, "control");
            uint8_t relay_control_value = relay_control->valueint;

            ESP_LOGI(TAG, "수동 릴레이 제어 값 수신: relay_num=%d, relay_control=%d", relay_num_value, relay_control_value);

            ce_global_update(&ce_relay_state_global[relay_num_value].relay_control_value, &relay_control_value, sizeof(relay_control_value), ce_relay_state_global[relay_num_value].relay_mutex);
            cJSON_Delete(root);
        }
        else if (strncmp(event->topic, "esp32/mode", event->topic_len) == 0) {
            // 모드 제어 데이터 처리
            cJSON *mode = cJSON_GetObjectItem(root, "mode");
            cJSON *relay_num = cJSON_GetObjectItem(root, "relay");
            uint8_t relay_num_value = relay_num->valuestring[5] - '0' - 1;
            char *mode_value_string = mode->valuestring;
            bool mode_value = RELAY_CONTROL_CASE_AUTO;
            if (strcmp(mode_value_string, "auto") == 0) {
                mode_value = RELAY_CONTROL_CASE_AUTO;
            }
            else if (strcmp(mode_value_string, "manual") == 0) {
                mode_value = RELAY_CONTROL_CASE_MANUAL;
            }
            ESP_LOGI(TAG, "모드 제어 값 수신: relay_num=%d, mode_value=%d", relay_num_value, mode_value);
            ce_global_update(&ce_relay_state_global[relay_num_value].relay_control_case, &mode_value, sizeof(mode_value), ce_relay_state_global[relay_num_value].relay_mutex);
            
            cJSON_Delete(root);
        }
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void ce_mqtt_task(void *params)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)params;
    size_t cap = 1024 * 1024;
    char *psram_buffer = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!psram_buffer)
    {
        ESP_LOGE("MQTT", "PSRAM 할당 실패!");
        vTaskDelete(NULL); // 또는 return;
    }
    int count = 0;
    sensor_acc_t acc_data;
    ce_env_sensor_data_t env_data;
    size_t psram_len = 0;

    char *json_init = "{\"device_id\":\"esp32-01\",\"accel_data\":[";

    strcpy(psram_buffer, json_init);
    psram_len = strlen(psram_buffer);

    uint64_t start_time = esp_timer_get_time();
    uint64_t elapsed_us = 0;
    
    while (1)
    {
        QueueSetMemberHandle_t activated = xQueueSelectFromSet(mqtt_queue_set, portMAX_DELAY);

        if (activated == ce_acc_queue_global)
        {
            if (xQueueReceive(ce_acc_queue_global, &acc_data, 0))
            {
                char sample[64];
                elapsed_us = esp_timer_get_time() - start_time;
                snprintf(sample, sizeof(sample),
                         "{\"us\":%llu,\"x\":%d,\"y\":%d,\"z\":%d}%s",
                         elapsed_us, acc_data.x, acc_data.y, acc_data.z,
                         (count == MQTT_ACC_DATA_SIZE - 1) ? "" : ",");
                // printf("x:%u,y:%u,z:%u\n", acc_data.x, acc_data.y, acc_data.z);

                size_t sample_len = strlen(sample);

                strcpy(psram_buffer + psram_len, sample);
                psram_len += sample_len;
                count++;

                if (count >= MQTT_ACC_DATA_SIZE)
                {
                    strcpy(psram_buffer + psram_len, "]}");
                    psram_len += 2;
                    int massage = esp_mqtt_client_publish(client, "esp32/accel", psram_buffer, psram_len, 1, 0);
                    heap_caps_free(psram_buffer);
                    count = 0;
                    psram_buffer = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
                    strcpy(psram_buffer, json_init);
                    psram_len = strlen(psram_buffer);
                }
            }
        }
        
        else if (activated == ce_temperatures_queue_global)
        {
            if (xQueueReceive(ce_temperatures_queue_global, &env_data, 0))
            {
                char sample[256];
                elapsed_us = esp_timer_get_time() - start_time;
                snprintf(sample, sizeof(sample),
                         "{\"us\":%llu,\"temp1\":%.2f,\"temp2\":%.2f,\"temp3\":%.2f,\"temp4\":%.2f,\"temp5\":%.2f,\"temp6\":%.2f,\"current\":%.2f,\"humi_temp1\":%.2f,\"humi_temp2\":%.2f}",
                         elapsed_us, env_data.temp[0], env_data.temp[1], env_data.temp[2], env_data.temp[3], env_data.temp[4], env_data.temp[5], env_data.current[0], env_data.humi_temp[0], env_data.humi_temp[1]);
                int massage_temp = esp_mqtt_client_publish(client, "esp32/env", sample, strlen(sample), 1, 0);
            }
        }
        else if (activated == ce_relay_state_queue_global)
        {
            ce_relay_state_set_t relay_state;
            if (xQueueReceive(ce_relay_state_queue_global, &relay_state, 0))
            {
                char sample[100];
                elapsed_us = esp_timer_get_time() - start_time;
                snprintf(sample, sizeof(sample),
                         "{\"us\":%llu,\"relay1\":[%d,%d,%d],\"relay2\":[%d,%d,%d],\"relay3\":[%d,%d,%d],\"relay4\":[%d,%d,%d]}",
                         elapsed_us, relay_state.relay_connection[0], relay_state.relay_state[0], relay_state.relay_control_case[0],
                          relay_state.relay_connection[1], relay_state.relay_state[1], relay_state.relay_control_case[1],
                           relay_state.relay_connection[2], relay_state.relay_state[2], relay_state.relay_control_case[2],
                            relay_state.relay_connection[3], relay_state.relay_state[3], relay_state.relay_control_case[3]);
                int massage_relay = esp_mqtt_client_publish(client, "esp32/relay", sample, strlen(sample), 1, 0);
            }   
        }

        // 목표 온도값 서버에서 수신
        


        vTaskDelay(pdMS_TO_TICKS(1));
    }

}

void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    // char line[128];

    // if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
    //     int count = 0;
    //     printf("Please enter url of mqtt broker\n");
    //     while (count < 128) {
    //         int c = fgetc(stdin);
    //         if (c == '\n') {
    //             line[count] = '\0';
    //             break;
    //         } else if (c > 0 && c < 127) {
    //             line[count] = c;
    //             ++count;
    //         }
    //         vTaskDelay(10 / portTICK_PERIOD_MS);
    //     }
    //     mqtt_cfg.broker.address.uri = line;
    //     printf("Broker url: %s\n", line);
    // } else {
    //     ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
    //     abort();
    // }

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    if (xTaskCreatePinnedToCore(ce_mqtt_task, "ce_mqtt_task", 4096, client, 5, NULL, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ce_mqtt_task");
        return;
    }
    ESP_LOGI(TAG, "MQTT task created");
}