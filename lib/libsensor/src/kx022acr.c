

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_rom_sys.h>
#include <esp_mac.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include <ce/sensor/kx022acr.h>
#include <ce/relay/control.h>

static i2c_master_dev_handle_t ret_handle;
SemaphoreHandle_t sample_sem;
esp_timer_handle_t sample_timer;

void sample_timer_cb(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(sample_sem, &high_task_wakeup);
    portYIELD_FROM_ISR();
}

ce_error_t ce_kx022acr_pin_set_spi(spi_device_handle_t *kx022acr_device_handle)
{
    esp_err_t ret;

    spi_bus_config_t kx022acr_bus_cfg = {
        .miso_io_num = KX022ACR_SPI_PIN_MISO,
        .mosi_io_num = KX022ACR_SPI_PIN_MOSI,
        .sclk_io_num = KX022ACR_SPI_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    ret = spi_bus_initialize(SPI2_HOST, &kx022acr_bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        return CE_ERROR_SENSOR_INIT;
    }

    spi_device_interface_config_t kx022acr_device_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .spics_io_num = KX022ACR_SPI_PIN_CS0,
        .clock_speed_hz = 1 * 1000 * 1000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .queue_size = 1,
        .input_delay_ns = 0,
        .cs_ena_posttrans = 0,
    };

    ret = spi_bus_add_device(SPI2_HOST, &kx022acr_device_cfg, kx022acr_device_handle);

    if (ret != ESP_OK)
    {
        return CE_ERROR_SENSOR_INIT;
    }
    return CE_OK;
}

ce_error_t ce_kx022acr_write_reg_spi(spi_device_handle_t kx022acr_device_handle, uint8_t reg_addr, uint8_t reg_data)
{
    // esp_err_t ret;
    // spi_transaction_t trans_addr = {
    //     .length = 8, // 8 bits
    //     .tx_buffer = &reg_addr,
    //     .rx_buffer = NULL,
    // };

    // spi_transaction_t trans_data = {
    //     .length = 8,
    //     .tx_buffer = &reg_data,
    //     .rx_buffer = NULL,
    // };

    // ret = spi_device_transmit(kx022acr_device_handle, &trans_addr);
    // if (ret != ESP_OK) {
    //     return CE_ERROR_SENSOR_WRITE;
    // }
    // ret = spi_device_transmit(kx022acr_device_handle, &trans_data);
    // if (ret != ESP_OK) {
    //     return CE_ERROR_SENSOR_WRITE;
    // }

    // return CE_OK;

    esp_err_t ret;
    uint8_t tx_buf[2];
    tx_buf[0] = reg_addr & 0x7F;
    tx_buf[1] = reg_data;
    spi_transaction_t trans_addr = {
        .length = 16, // 16 bits for address and data
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    ret = spi_device_transmit(kx022acr_device_handle, &trans_addr);
    if (ret != ESP_OK)
    {
        return CE_ERROR_SENSOR_WRITE;
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms delay

    return CE_OK;
}

ce_error_t ce_kx022acr_read_reg_spi(spi_device_handle_t kx022acr_device_handle, uint8_t reg_addr, void *reg_data)
{

    // esp_err_t ret;
    // spi_transaction_t trans = {
    //     .length = 8, // 8 bits for address
    //     .tx_buffer = &reg_addr,
    //     .rx_buffer = NULL,
    // };

    // ret = spi_device_transmit(kx022acr_device_handle, &trans);
    // if (ret != ESP_OK) {
    //     return CE_ERROR_SENSOR_READ;
    // }

    // trans.length = 8; // 8 bits for data
    // trans.tx_buffer = NULL;
    // trans.rx_buffer = reg_data;

    // ret = spi_device_transmit(kx022acr_device_handle, &trans);
    // if (ret != ESP_OK) {

    //     return CE_ERROR_SENSOR_READ;
    // }

    // return CE_OK;

    esp_err_t ret;
    uint8_t tx_buf = reg_addr | 0x80; // Read 명령 + dummy
    uint8_t rx_buf;

    spi_transaction_t trans = {
        .length = 8, // 총 16비트
        .rxlength = 8,
        // .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_buffer = &tx_buf,
        .rx_buffer = &rx_buf,
    };

    ret = spi_device_transmit(kx022acr_device_handle, &trans);
    if (ret != ESP_OK)
    {
        return CE_ERROR_SENSOR_READ;
    }

    *((uint8_t *)reg_data) = rx_buf;

    printf("[SPI DEBUG] tx: 0x%X, rx: 0x%02X\n", tx_buf, rx_buf);
    return CE_OK;
}

ce_error_t ce_kx022acr_pin_set_i2c(i2c_master_dev_handle_t *ret_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = KX022ACR_I2C_PIN_SDA,
        .scl_io_num = KX022ACR_I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,

    };

    i2c_master_bus_handle_t i2c_bus_handle;
    if (i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle) != ESP_OK)
    {
        return CE_ERROR_SENSOR_INIT;
    }

    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = KX022ACR_I2C_ADDR,
        .scl_speed_hz = 350000};

    if (i2c_master_bus_add_device(i2c_bus_handle, &i2c_device_config, ret_handle) != ESP_OK)
    {
        return CE_ERROR_SENSOR_INIT;
    }

    return CE_OK;
}

ce_error_t ce_kx022acr_write_reg_i2c(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_transmit(i2c_dev, write_buf, 2, 1000 / portTICK_PERIOD_MS);
}

ce_error_t ce_kx022acr_read_reg_i2c(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t rx_len)
{
    return i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, data, rx_len, 1000 / portTICK_PERIOD_MS);
}

static void ce_kx022acr_task(void *params)
{
    // spi_device_handle_t kx022acr_device_handle = *((spi_device_handle_t *)params);
    i2c_master_dev_handle_t kx022acr_device_handle = *((i2c_master_dev_handle_t *)params);
    int16_t *psram_buffer = heap_caps_malloc(KX022ACR_MAX_BUF_SIZE, MALLOC_CAP_SPIRAM);

    while (1)
    {
        // 상태 변경 대기
        EventBits_t event_bits = xEventGroupWaitBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group, RELAY_STATE_CHANGE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        int sample_index = 0;

        int64_t start_time = esp_timer_get_time();
        int64_t elapsed_us = 0;

        while (1)
        {
            // 릴레이 상태가 바뀌면 수집 종료
            bool is_relay_changed = (xEventGroupGetBits(ce_relay_state_global[RELAY_COMP].relay_state_set_event_group) & RELAY_STATE_CHANGE_BIT) ? true : false;
            if (is_relay_changed)
            {
                elapsed_us = esp_timer_get_time() - start_time;
                printf("%llu us elapsed, %d samples\n", elapsed_us, sample_index);
                break;
            }

            xSemaphoreTake(sample_sem, portMAX_DELAY);

            // 센서 데이터 수집
            uint8_t read_samples[6];

            ce_kx022acr_read_reg_i2c(kx022acr_device_handle, BUF_READ, read_samples, 6);

            int16_t x = ((int16_t)read_samples[1] << 8) | read_samples[0];
            int16_t y = ((int16_t)read_samples[3] << 8) | read_samples[2];
            int16_t z = ((int16_t)read_samples[5] << 8) | read_samples[4];

            // elapsed_us = esp_timer_get_time() - start_time;
            // uint32_t elapsed_sec = elapsed_us / 1000000;
            // uint32_t elapsed_ms = (elapsed_us % 1000000) / 100;

            // printf("%lu.%02lu s, ", elapsed_sec, elapsed_ms);
            // printf("%llu ", elapsed_us);
            // printf("x: %d, y: %d, z: %d\n", x, y, z);

            int idx = sample_index * 3;
            psram_buffer[idx + 0] = x;
            psram_buffer[idx + 1] = y;
            psram_buffer[idx + 2] = z;

            sample_index++;
        }

        // PSRAM에 저장된 데이터 처리

        printf("- Start time: %llu us, %llu ms elapsed, %d samples\n", start_time, elapsed_us, sample_index);
        for (int i = 0; i < sample_index; i++) {
            printf("Sample %d: x: %d, y: %d, z: %d\n", i, psram_buffer[i * 3], psram_buffer[i * 3 + 1], psram_buffer[i * 3 + 2]);
        }
        
    }

    ce_error_t ce_kx022acr_init_spi(void)
    {
        ce_error_t err;
        spi_device_handle_t kx022acr_device_handle;

        err = ce_kx022acr_pin_set_spi(&kx022acr_device_handle);
        if (err != CE_OK)
        {
            return err;
        }

        // 센서 리셋
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, CNTL2, 0b10000000);
        vTaskDelay(pdMS_TO_TICKS(10));

        ce_kx022acr_write_reg_spi(kx022acr_device_handle, INC1, 0x00);

        // Standby mode 설정
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, CNTL1, 0x00);

        // Stream mode 설정
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, BUF_CNTL1, 0x28);
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, BUF_CNTL2, 0xC1);

        // Data rate 설정
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, ODCNTL, 0xCB); // 1600Hz 설정

        // 센서 동작 시작 (PC1=1, RES=1 for High Resolution)
        ce_kx022acr_write_reg_spi(kx022acr_device_handle, CNTL1, 0xC0);

        uint8_t rx_data = 0x00;

        ce_kx022acr_read_reg_spi(kx022acr_device_handle, WHO_AM_I, &rx_data);
        if (rx_data != 0xC8)
        {
            printf("[SENSOR] WHO_AM_I 확인 실패: 0x%02X\n", rx_data);
            return CE_ERROR_SENSOR_INIT;
        }

        if (xTaskCreatePinnedToCore(ce_kx022acr_task, "ce_kx022acr_task", 4096, &kx022acr_device_handle, 5, NULL, tskNO_AFFINITY) != pdPASS)
        {
            return CE_ERROR_TASK_CREATE;
        }

        return CE_OK;
    }

    void init_sampling_timer()
    {
        sample_sem = xSemaphoreCreateBinary();
        esp_timer_create_args_t timer_args = {
            .callback = sample_timer_cb,
            .name = "kx_sample_timer"};
        esp_timer_create(&timer_args, &sample_timer);
        esp_timer_start_periodic(sample_timer, 625); // 625us
    }

    ce_error_t ce_kx022acr_init_i2c(void)
    {
        ce_error_t err;

        err = ce_kx022acr_pin_set_i2c(&ret_handle);
        if (err != CE_OK)
        {
            return err;
        }

        // 센서 리셋
        ce_kx022acr_write_reg_i2c(ret_handle, CNTL2, 0b10000000);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Standby mode 설정
        ce_kx022acr_write_reg_i2c(ret_handle, CNTL1, 0x00);

        ce_kx022acr_write_reg_i2c(ret_handle, CNTL3, 0b11111111);

        // Stream mode 설정
        ce_kx022acr_write_reg_i2c(ret_handle, BUF_CNTL1, 0x28);
        ce_kx022acr_write_reg_i2c(ret_handle, BUF_CNTL2, 0xC1);

        // Data rate 설정
        ce_kx022acr_write_reg_i2c(ret_handle, ODCNTL, 0xCB); // 1600Hz 설정

        // 센서 동작 시작 (PC1=1, RES=1 for High Resolution)
        ce_kx022acr_write_reg_i2c(ret_handle, CNTL1, 0xC0);

        uint8_t rx_data = 0x00;

        ce_kx022acr_read_reg_i2c(ret_handle, WHO_AM_I, &rx_data, 1);
        if (rx_data != 0xC8)
        {
            printf("[SENSOR] WHO_AM_I 확인 실패: 0x%02X\n", rx_data);
            return CE_ERROR_SENSOR_INIT;
        }

        if (xTaskCreatePinnedToCore(ce_kx022acr_task, "ce_kx022acr_task", 6000, &ret_handle, 5, NULL, 1) != pdPASS)
        {
            return CE_ERROR_TASK_CREATE;
        }

        init_sampling_timer();

        return CE_OK;
    }
