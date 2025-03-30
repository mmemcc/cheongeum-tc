#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_http_client.h>
#include <esp_tls.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

#include <ce/relay/control.h>
#include <ce/sensor/sensors.h>
#include <ce/operation/mode.h>
#include <ce/util/sys.h>
#include <ce/sensor/temperatures.h>
#include <ce/sensor/kx022acr.h>

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
#define CHUNK_SIZE (128*1024)  // 64KB
#define LINE_MAX_LEN 32          // 한 줄당 최대 길이
static const char *TAG = "HTTP_CLIENT";
static const char *init_url = "http://172.30.1.79:5000/upload/init";
static const char *end_url = "http://172.30.1.79:5000/upload/end";
static const char *host_endpoint = "http://172.30.1.79:5000/upload";

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        // Clean the buffer in case of a new request
        if (output_len == 0 && evt->user_data)
        {
            // we are just starting to copy the output data into the use
            memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
        }
        /*
         *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
         *  However, event handler can also be used in case chunked encoding is used.
         */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // If user_data buffer is configured, copy the response into the buffer
            int copy_len = 0;
            if (evt->user_data)
            {
                // The last byte in evt->user_data is kept for the NULL character in case of out-of-bound access.
                copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                if (copy_len)
                {
                    memcpy(evt->user_data + output_len, evt->data, copy_len);
                }
            }
            else
            {
                int content_len = esp_http_client_get_content_length(evt->client);
                if (output_buffer == NULL)
                {
                    // We initialize output_buffer with 0 because it is used by strlen() and similar functions therefore should be null terminated.
                    output_buffer = (char *)calloc(content_len + 1, sizeof(char));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                copy_len = MIN(evt->data_len, (content_len - output_len));
                if (copy_len)
                {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                }
            }
            output_len += copy_len;
        }

        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        if (output_buffer != NULL)
        {
            // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
            // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
            ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
        }
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        esp_http_client_set_header(evt->client, "From", "user@example.com");
        esp_http_client_set_header(evt->client, "Accept", "text/html");
        esp_http_client_set_redirection(evt->client);
        break;
    }
    return ESP_OK;
}

static void ce_server_upload_task(void *params)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};

    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT3, pdTRUE, pdFALSE, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 0.1초 대기
        bool is_relay_changed = (bits & RELAY_STATE_CHANGE_BIT3) ? true : false;
        bool is_queue_full = (uxQueueMessagesWaiting(ce_sensors_info_global) > 0) ? true : false;
        printf("info len: %d\n", uxQueueMessagesWaiting(ce_sensors_info_global));
        
        if (is_relay_changed && !is_queue_full)
        {
            printf("[SERVER] Waiting for data...\n");
            continue;
        }

        printf("[SERVER] Uploading data...\n");

        // 0. 센서 tasks suspend
        vTaskSuspend(ce_kx022acr_task_handle);
        vTaskSuspend(ce_temperatures_task_handle);

        // 1. 센서 정보 수신
        ce_sensor_info_t sensor_info;
        xQueueReceive(ce_sensors_info_global, &sensor_info, portMAX_DELAY);

        extern int16_t *psram_buffer;

        // 3. 서버에 데이터 전송
        esp_http_client_config_t config_init = {
            .url = init_url, // 서버 URL
            // .event_handler = _http_event_handler, // 이벤트 핸들러
            .method = HTTP_METHOD_POST,           // POST 메소드
            .user_data = local_response_buffer,    
        };

        esp_http_client_handle_t client_info = esp_http_client_init(&config_init);
        if (client_info == NULL)
        {
            ESP_LOGE(TAG, "Failed to initialize HTTP client");
            continue;
        }

        // 3.1 sensor_info 전송
        char info_data_json[128];
        snprintf(info_data_json, sizeof(info_data_json), "{\"start_time\":%llu,\"elapsed_time\":%llu,\"relay_state\":%u,\"samp_num_vib\":%ld}",
                 sensor_info.start_time, sensor_info.elapsed_time, sensor_info.relay_state, sensor_info.samp_num_vib);
        esp_http_client_set_header(client_info, "Content-Type", "application/json");
        esp_http_client_set_post_field(client_info, info_data_json, strlen(info_data_json));
        if (esp_http_client_perform(client_info) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to perform HTTP request");
        }
        else
        {
            ESP_LOGI(TAG, "Posted sensor info: %s", info_data_json);
        }

        esp_http_client_cleanup(client_info);

        vTaskDelay(pdMS_TO_TICKS(100)); // 0.1초 대기

        // 3.2 psram_buffer 전송
        multi_heap_info_t heap_info;
        heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "PSRAM free: %d / largest: %d", heap_info.total_free_bytes, heap_info.largest_free_block);
        if (heap_info.total_free_bytes < CHUNK_SIZE)
        {
            ESP_LOGE(TAG, "Not enough PSRAM available for upload");
            continue;
        }
        
        char *send_buffer = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_SPIRAM);
        if (send_buffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for send buffer");
            continue;
        }
        send_buffer[0] = '\0';  // 문자열 시작

        memset(send_buffer, 0, CHUNK_SIZE);

        esp_http_client_config_t config_acc = {
            .url = host_endpoint, // 서버 URL
            // .event_handler = _http_event_handler, // 이벤트 핸들러
            .method = HTTP_METHOD_POST,           // POST 메소드
        };

        esp_http_client_handle_t client_acc = esp_http_client_init(&config_acc);
        esp_http_client_set_header(client_acc, "Content-Type", "text/plain");
        for (int i = 0; i < sensor_info.samp_num_vib; i++)
        {
            int16_t x = psram_buffer[i * 3 + 0];
            int16_t y = psram_buffer[i * 3 + 1];
            int16_t z = psram_buffer[i * 3 + 2];

            char line[LINE_MAX_LEN];

            snprintf(line, sizeof(line), "%d,%d,%d\n", x, y, z);

            if (strlen(send_buffer) + strlen(line) >= CHUNK_SIZE - 1) {
                // 버퍼 꽉 찼으면 전송
                esp_http_client_set_post_field(client_acc, send_buffer, strlen(send_buffer));
                esp_http_client_perform(client_acc);
                send_buffer[0] = '\0';  // 다시 초기화
                
            }
            strcat(send_buffer, line);
            vTaskDelay(pdMS_TO_TICKS(10)); // 0.01초 대기
            // taskYIELD();
        }
        if (strlen(send_buffer) > 0) {
            esp_http_client_set_post_field(client_acc, send_buffer, strlen(send_buffer));
            esp_http_client_perform(client_acc);
        }
        esp_http_client_cleanup(client_acc);
        free(send_buffer);

        // 3.3 temperatures 전송
        esp_http_client_config_t config_temp = {
            .url = host_endpoint, // 서버 URL
            // .event_handler = _http_event_handler, // 이벤트 핸들러
            .method = HTTP_METHOD_POST,           // POST 메소드
            .user_data = local_response_buffer,   
        };

        esp_http_client_handle_t client_temp = esp_http_client_init(&config_temp);
        uint32_t temp_data_num = uxQueueMessagesWaiting(ce_temperatures_queue_global);
        esp_http_client_set_header(client_temp, "Content-Type", "text/plain");
        esp_http_client_set_post_field(client_temp, "[TEMP]\n", 8); // 초기화
        esp_http_client_perform(client_temp);
        float temp_data[6];
        for (uint32_t i = 0; i < temp_data_num; i++)
        {
            xQueueReceive(ce_temperatures_queue_global, temp_data, portMAX_DELAY);
            char line[64];
            snprintf(line, sizeof(line), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     temp_data[0], temp_data[1], temp_data[2],
                     temp_data[3], temp_data[4], temp_data[5]);
            esp_http_client_set_header(client_temp, "Content-Type", "text/plain");
            esp_http_client_set_post_field(client_temp, line, strlen(line));
            esp_http_client_perform(client_temp);
        }
        esp_http_client_cleanup(client_temp);

        // 3.4 종료 알림
        esp_http_client_config_t config_end = {
            .url = end_url, // 서버 URL
            .event_handler = _http_event_handler, // 이벤트 핸들러
            .method = HTTP_METHOD_POST,           // POST 메소드
            .user_data = local_response_buffer,   
        };

        esp_http_client_handle_t client_end = esp_http_client_init(&config_end);
        char end_url[128];
        snprintf(end_url, sizeof(end_url), "%s%s", host_endpoint, "/end");
        esp_http_client_set_post_field(client_end, NULL, 0);
        esp_http_client_perform(client_end);
        esp_http_client_cleanup(client_end);

        vTaskResume(ce_kx022acr_task_handle);
        vTaskResume(ce_temperatures_task_handle);

        vTaskDelay(pdMS_TO_TICKS(100)); // 0.1초 대기
    }
    
}

ce_error_t ce_server_upload_init(void)
{
    if (xTaskCreatePinnedToCore(ce_server_upload_task, "ce_server_upload_task", 6000, NULL, 5, NULL, 1) != pdPASS)
    {
        return CE_ERROR_TASK_CREATE;
    }

    return CE_OK;
}