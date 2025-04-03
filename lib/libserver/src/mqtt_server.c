

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

#include <ce/sensor/kx022acr.h>
#include <ce/sensor/sensors.h>
#include <ce/sensor/temperatures.h>
#include <ce/relay/control.h>

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
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
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
    float temp[6];
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
                         "{\"us\":%llu,\"x\":%u,\"y\":%u,\"z\":%u}%s",
                         elapsed_us, acc_data.x, acc_data.y, acc_data.z,
                         (count == MQTT_ACC_DATA_SIZE - 1) ? "" : ",");

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
            if (xQueueReceive(ce_temperatures_queue_global, &temp, 0))
            {
                char sample[256];
                elapsed_us = esp_timer_get_time() - start_time;
                snprintf(sample, sizeof(sample),
                         "{\"us\":%llu,\"temp1\":%.2f,\"temp2\":%.2f,\"temp3\":%.2f,\"temp4\":%.2f,\"temp5\":%.2f,\"temp6\":%.2f}",
                         elapsed_us, temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
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
                         "{\"us\":%llu,\"relay1\":[%d,%d],\"relay2\":[%d,%d],\"relay3\":[%d,%d],\"relay4\":[%d,%d]}",
                         elapsed_us, relay_state.relay_connection[0], relay_state.relay_state[0],
                          relay_state.relay_connection[1], relay_state.relay_state[1],
                           relay_state.relay_connection[2], relay_state.relay_state[2],
                            relay_state.relay_connection[3], relay_state.relay_state[3]);
                int massage_relay = esp_mqtt_client_publish(client, "esp32/relay", sample, strlen(sample), 1, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}

void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://172.30.1.70",
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