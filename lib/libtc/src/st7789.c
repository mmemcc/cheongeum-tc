
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <driver/gpio.h>
// #include <driver/spi_master.h>
// #include <driver/ledc.h>
// #include <esp_check.h>
// #include <esp_lcd_panel_io.h>
// #include <esp_lcd_panel_interface.h>
// #include <esp_lcd_panel_vendor.h>
// #include <esp_lcd_panel_ops.h>
// #include <esp_lcd_panel_commands.h>

// #include <ce/util/error.h>
// #include <ce/tc/display.h>

// #define LCD_H_RES 172
// #define LCD_V_RES 320

// #define ST7789_CMD_RAMCTRL 0xb0
// #define ST7789_DATA_LITTLE_ENDIAN_BIT (1 << 3)

// static const char *TAG = "lcd_panel.st7789";


// typedef struct {
//     esp_lcd_panel_t base;
//     esp_lcd_panel_io_handle_t io;
//     int reset_gpio_num;
//     bool reset_level;
//     int x_gap;
//     int y_gap;
//     uint8_t fb_bits_per_pixel;
//     uint8_t madctl_val;    // save current value of LCD_CMD_MADCTL register
//     uint8_t colmod_val;    // save current value of LCD_CMD_COLMOD register
//     uint8_t ramctl_val_1;
//     uint8_t ramctl_val_2;
// } st7789_panel_t;

// static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel);
// static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel);
// static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel);
// static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
//                                           const void *color_data);
// static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
// static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
// static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
// static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
// static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool off);
// static esp_err_t panel_st7789_sleep(esp_lcd_panel_t *panel, bool sleep);

// ce_error_t ce_st7789_pin_set(esp_lcd_panel_io_handle_t *io_handle)
// {
//     esp_err_t ret;

//     spi_bus_config_t st7789_bus_cfg = {
//         .mosi_io_num = LCD_SPI_SDA,
//         .sclk_io_num = LCD_SPI_SCLK,
//         .miso_io_num = -1,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 100 * 100 * sizeof(uint16_t),
//     };

//     ret = spi_bus_initialize(SPI3_HOST, &st7789_bus_cfg, SPI_DMA_CH_AUTO);
//     if (ret != ESP_OK)
//     {
//         return CE_ERROR_SENSOR_INIT;
//     }

//     esp_lcd_panel_io_spi_config_t io_config = {
//         .dc_gpio_num = LCD_DC,
//         .cs_gpio_num = LCD_SPI_CS,
//         .pclk_hz = 20 * 1000 * 1000,
//         .spi_mode = 0,
//         .trans_queue_depth = 10,
//         .lcd_cmd_bits = 8,
//         .lcd_param_bits = 8,
//         .on_color_trans_done = NULL,
//         .user_ctx = NULL,
//     };

//     ret = esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, io_handle);

//     if (ret != ESP_OK)
//     {
//         return CE_ERROR_SENSOR_INIT;
//     }

//     // 초기 상태 설정
//     gpio_set_level(LCD_RESET, 1);
//     gpio_set_level(LCD_DC, 1);

//     // PWM 타이머 설정
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .timer_num = LEDC_TIMER_0,
//         .duty_resolution = LEDC_TIMER_8_BIT, // 0~255
//         .freq_hz = 5000,                     // PWM 주파수 5kHz
//         .clk_cfg = LEDC_AUTO_CLK};

//     ret = ledc_timer_config(&ledc_timer);
//     if (ret != ESP_OK)
//     {
//         return CE_ERROR_SENSOR_INIT;
//     }

//     // PWM 채널 설정
//     ledc_channel_config_t ledc_channel = {
//         .gpio_num = LCD_LEDA,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .channel = LEDC_CHANNEL_0,
//         .timer_sel = LEDC_TIMER_0,
//         .duty = 128, // 초기 밝기 (0~255)
//         .hpoint = 0,
//         .flags.output_invert = 0,
//         .intr_type = LEDC_INTR_DISABLE};

//     ret = ledc_channel_config(&ledc_channel);
//     if (ret != ESP_OK)
//     {
//         return CE_ERROR_SENSOR_INIT;
//     }

//     return CE_OK;
// }

// esp_err_t esp_lcd_new_panel_st7789(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
//                                    esp_lcd_panel_handle_t *ret_panel)
// {
// #if CONFIG_LCD_ENABLE_DEBUG_LOG
//     esp_log_level_set(TAG, ESP_LOG_DEBUG);
// #endif
//     esp_err_t ret = ESP_OK;
//     st7789_panel_t *st7789 = NULL;
//     ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
//     // leak detection of st7789 because saving st7789->base address
//     ESP_COMPILER_DIAGNOSTIC_PUSH_IGNORE("-Wanalyzer-malloc-leak")
//     st7789 = calloc(1, sizeof(st7789_panel_t));
//     ESP_GOTO_ON_FALSE(st7789, ESP_ERR_NO_MEM, err, TAG, "no mem for st7789 panel");

//     if (panel_dev_config->reset_gpio_num >= 0)
//     {
//         gpio_config_t io_conf = {
//             .mode = GPIO_MODE_OUTPUT,
//             .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
//         };
//         ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
//     }

//     switch (panel_dev_config->rgb_ele_order)
//     {
//     case LCD_RGB_ELEMENT_ORDER_RGB:
//         st7789->madctl_val = 0;
//         break;
//     case LCD_RGB_ELEMENT_ORDER_BGR:
//         st7789->madctl_val |= LCD_CMD_BGR_BIT;
//         break;
//     default:
//         ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported RGB element order");
//         break;
//     }

//     uint8_t fb_bits_per_pixel = 0;
//     switch (panel_dev_config->bits_per_pixel)
//     {
//     case 16: // RGB565
//         st7789->colmod_val = 0x55;
//         fb_bits_per_pixel = 16;
//         break;
//     case 18: // RGB666
//         st7789->colmod_val = 0x66;
//         // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
//         fb_bits_per_pixel = 24;
//         break;
//     default:
//         ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
//         break;
//     }

//     st7789->ramctl_val_1 = 0x00;
//     st7789->ramctl_val_2 = 0xf0; // Use big endian by default
//     if ((panel_dev_config->data_endian) == LCD_RGB_DATA_ENDIAN_LITTLE)
//     {
//         // Use little endian
//         st7789->ramctl_val_2 |= ST7789_DATA_LITTLE_ENDIAN_BIT;
//     }

//     st7789->io = io;
//     st7789->fb_bits_per_pixel = fb_bits_per_pixel;
//     st7789->reset_gpio_num = panel_dev_config->reset_gpio_num;
//     st7789->reset_level = panel_dev_config->flags.reset_active_high;
//     st7789->base.del = panel_st7789_del;
//     st7789->base.reset = panel_st7789_reset;
//     st7789->base.init = panel_st7789_init;
//     st7789->base.draw_bitmap = panel_st7789_draw_bitmap;
//     st7789->base.invert_color = panel_st7789_invert_color;
//     st7789->base.set_gap = panel_st7789_set_gap;
//     st7789->base.mirror = panel_st7789_mirror;
//     st7789->base.swap_xy = panel_st7789_swap_xy;
//     st7789->base.disp_on_off = panel_st7789_disp_on_off;
//     st7789->base.disp_sleep = panel_st7789_sleep;
//     *ret_panel = &(st7789->base);
//     ESP_LOGD(TAG, "new st7789 panel @%p", st7789);

//     return ESP_OK;

// err:
//     if (st7789)
//     {
//         if (panel_dev_config->reset_gpio_num >= 0)
//         {
//             gpio_reset_pin(panel_dev_config->reset_gpio_num);
//         }
//         free(st7789);
//     }
//     return ret;
//     ESP_COMPILER_DIAGNOSTIC_POP("-Wanalyzer-malloc-leak")
// }

// static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);

//     if (st7789->reset_gpio_num >= 0)
//     {
//         gpio_reset_pin(st7789->reset_gpio_num);
//     }
//     ESP_LOGD(TAG, "del st7789 panel @%p", st7789);
//     free(st7789);
//     return ESP_OK;
// }

// static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;

//     // perform hardware reset
//     if (st7789->reset_gpio_num >= 0)
//     {
//         gpio_set_level(st7789->reset_gpio_num, st7789->reset_level);
//         vTaskDelay(pdMS_TO_TICKS(10));
//         gpio_set_level(st7789->reset_gpio_num, !st7789->reset_level);
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
//     else
//     { // perform software reset
//         ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
//                             "io tx param failed");
//         vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
//     }

//     return ESP_OK;
// }

// static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
//                         "io tx param failed");
//     vTaskDelay(pdMS_TO_TICKS(100));
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
//         st7789->madctl_val,
//     }, 1), TAG, "io tx param failed");
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
//         st7789->colmod_val,
//     }, 1), TAG, "io tx param failed");
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ST7789_CMD_RAMCTRL, (uint8_t[]) {
//         st7789->ramctl_val_1, st7789->ramctl_val_2
//     }, 2), TAG, "io tx param failed");

//     return ESP_OK;
// }

// static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
//                                           const void *color_data)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;

//     x_start += st7789->x_gap;
//     x_end += st7789->x_gap;
//     y_start += st7789->y_gap;
//     y_end += st7789->y_gap;

//     // define an area of frame memory where MCU can access
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]){
//                                                                          (x_start >> 8) & 0xFF,
//                                                                          x_start & 0xFF,
//                                                                          ((x_end - 1) >> 8) & 0xFF,
//                                                                          (x_end - 1) & 0xFF,
//                                                                      },
//                                                   4),
//                         TAG, "io tx param failed");
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]){
//                                                                          (y_start >> 8) & 0xFF,
//                                                                          y_start & 0xFF,
//                                                                          ((y_end - 1) >> 8) & 0xFF,
//                                                                          (y_end - 1) & 0xFF,
//                                                                      },
//                                                   4),
//                         TAG, "io tx param failed");
//     // transfer frame buffer
//     size_t len = (x_end - x_start) * (y_end - y_start) * st7789->fb_bits_per_pixel / 8;
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "io tx color failed");

//     return ESP_OK;
// }

// static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     int command = 0;
//     if (invert_color_data)
//     {
//         command = LCD_CMD_INVON;
//     }
//     else
//     {
//         command = LCD_CMD_INVOFF;
//     }
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
//                         "io tx param failed");
//     return ESP_OK;
// }

// static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     if (mirror_x)
//     {
//         st7789->madctl_val |= LCD_CMD_MX_BIT;
//     }
//     else
//     {
//         st7789->madctl_val &= ~LCD_CMD_MX_BIT;
//     }
//     if (mirror_y)
//     {
//         st7789->madctl_val |= LCD_CMD_MY_BIT;
//     }
//     else
//     {
//         st7789->madctl_val &= ~LCD_CMD_MY_BIT;
//     }
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){st7789->madctl_val}, 1), TAG, "io tx param failed");
//     return ESP_OK;
// }

// static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     if (swap_axes)
//     {
//         st7789->madctl_val |= LCD_CMD_MV_BIT;
//     }
//     else
//     {
//         st7789->madctl_val &= ~LCD_CMD_MV_BIT;
//     }
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){st7789->madctl_val}, 1), TAG, "io tx param failed");
//     return ESP_OK;
// }

// static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     st7789->x_gap = x_gap;
//     st7789->y_gap = y_gap;
//     return ESP_OK;
// }

// // 밝기 조절 함수 (0~255)
// void set_backlight(uint8_t brightness)
// {
//     ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
// }


// static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     int command = 0;
//     if (on_off)
//     {
//         command = LCD_CMD_DISPON;
//     }
//     else
//     {
//         command = LCD_CMD_DISPOFF;
//     }
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
//                         "io tx param failed");
//     return ESP_OK;
// }

// static esp_err_t panel_st7789_sleep(esp_lcd_panel_t *panel, bool sleep)
// {
//     st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
//     esp_lcd_panel_io_handle_t io = st7789->io;
//     int command = 0;
//     if (sleep)
//     {
//         command = LCD_CMD_SLPIN;
//     }
//     else
//     {
//         command = LCD_CMD_SLPOUT;
//     }
//     ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
//                         "io tx param failed");
//     vTaskDelay(pdMS_TO_TICKS(100));

//     return ESP_OK;
// }


// static void ce_st7789_task(void *params)
// {
//     esp_lcd_panel_io_handle_t panel_handle = *((esp_lcd_panel_io_handle_t *)params);

//     while (1)
//     {
      
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }



// ce_error_t ce_st7789_init(void)
// {
//     ce_error_t err;

//     esp_lcd_panel_io_handle_t io_handle = NULL;
//     esp_lcd_panel_handle_t panel_handle = NULL;
//     ce_st7789_pin_set(&io_handle);
//     esp_lcd_panel_dev_config_t panel_config = {
//         .reset_gpio_num = LCD_RESET,
//         .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
//         .bits_per_pixel = 16,
//     };
//     esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);

//     if (xTaskCreatePinnedToCore(ce_st7789_task, "ce_st7789_task", 4096, &panel_handle, 5, NULL, tskNO_AFFINITY) != pdPASS)
//     {
//         return CE_ERROR_TASK_CREATE;
//     }

//     return CE_OK;
// }
