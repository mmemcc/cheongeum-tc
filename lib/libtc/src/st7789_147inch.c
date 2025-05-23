#include <string.h>
#include <stdarg.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_commands.h>

#include <ce/tc/st7789_147inch.h>
#include <ce/util/error.h>
#include <ce/relay/control.h>
#include <ce/sensor/sensors.h>
#include <ce/tc/tc_state.h>

// /* === Blink 주기 === */
// static const TickType_t blink_period = pdMS_TO_TICKS(1000);

/* === 초기값 === */
tcui_state_t tcui_init;

static const char* text_return(char *buf, size_t buf_size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(buf, buf_size, format, args);
    va_end(args);
    return buf;
}

static dyn_field_t config_status_bar = {.buf = "", .x = 66, .y = 12, .sc = 2, .sp = 1, .bg = 0xffff, .format = "%d%%"};
static dyn_field_t config_temp_internal = {.buf = "", .x = 20, .y = 70, .sc = 6, .sp = 1, .bg = COL_BG, .format = "%.1f"};
static dyn_field_t config_energy = {.buf = "", .x = DEV_START_X + 30, .y = 55, .sc = 2, .sp = 1, .bg = COL_BG, .format = "%.3f"};
static dyn_field_t config_set_temp = {.buf = "", .x = DEV_START_X + 80, .y = 50 + 28 + 4, .sc = 2, .sp = 1, .bg = COL_BG, .format = "%.1f"};
static dyn_field_t config_out_temp = {.buf = "", .x = DEV_START_X + 80, .y = 50 + 2 * (28 + 2), .sc = 2, .sp = 1, .bg = COL_BG, .format = "%.1f"};

/* ===================================================================== */
/* --------------------   ST7789 Low‑level 함수   ----------------------- */
/* ===================================================================== */
static void lcd_cmd(spi_device_handle_t spi_handle, uint8_t c)
{
    spi_transaction_t t = { .length = 8, .tx_buffer = &c };
    gpio_set_level(LCD_DC, 0);
    spi_device_polling_transmit(spi_handle, &t);
}
static void lcd_data(spi_device_handle_t spi_handle, const uint8_t* d, int len)
{
    if (!len) return;
    spi_transaction_t t = { .length = len * 8, .tx_buffer = d };
    gpio_set_level(LCD_DC, 1);
    spi_device_polling_transmit(spi_handle, &t);
}
static void lcd_set_window(spi_device_handle_t spi_handle,  uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
    uint8_t dt[4];
    x0+=X_OFF; x1+=X_OFF; y0+=Y_OFF; y1+=Y_OFF;
    lcd_cmd(spi_handle, 0x2A); dt[0]=x0>>8; dt[1]=x0; dt[2]=x1>>8; dt[3]=x1; lcd_data(spi_handle, dt,4);
    lcd_cmd(spi_handle, 0x2B); dt[0]=y0>>8; dt[1]=y0; dt[2]=y1>>8; dt[3]=y1; lcd_data(spi_handle, dt,4);
    lcd_cmd(spi_handle, 0x2C);
}
static void lcd_draw_pixel(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,uint16_t c)
{
    lcd_set_window(spi_handle, x,y,x,y);
    uint8_t dt[2]={c>>8,c};
    lcd_data(spi_handle, dt,2);
}
static void fill_rect(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t c)
{
    lcd_set_window(spi_handle, x,y,x+w-1,y+h-1);
    uint8_t line[w*2];
    for(int i=0;i<w;i++)
    { 
        line[2*i]=c>>8; 
        line[2*i+1]=c; 
    }
    for(int r=0;r<h;r++) 
        lcd_data(spi_handle, line,sizeof(line));
}
static void draw_bitmap(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,
                        const uint16_t* bm,uint16_t w,uint16_t h,uint16_t col)
{
    for(int r=0;r<h;r++)
        for(int c=0;c<w;c++)
            if(bm[r*w+c])
                lcd_draw_pixel(spi_handle, x+c,y+r,col);
}


/* ------------------------  Font 7×5  --------------------------------- */
static void draw_char(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,char ch,uint16_t col,uint8_t sc)
{
    if(ch<32||ch>127) return;
    const uint8_t* map = font5x7[ch-32];
    for(int c=0;c<5;c++){
        uint8_t bits = map[c];
        for(int r=0;r<7;r++){
            if(bits & (1<<r)){
                for(int dx=0;dx<sc;dx++)
                    for(int dy=0;dy<sc;dy++)
                        lcd_draw_pixel(spi_handle, x+c*sc+dx,y+r*sc+dy,col);
            }
        }
    }
}
static void draw_text(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,const char*s,
                      uint16_t col,uint8_t sc,uint8_t sp)
{
    while(*s){
        draw_char(spi_handle, x,y,*s,col,sc);
        x += 5*sc + sp;
        ++s;
    }
}

/* ----------------------  프로그레스 바  ------------------------------- */
static void draw_progress_bar(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,
                              uint16_t w,uint16_t h,uint8_t pct,
                              uint16_t fg,uint16_t bg,uint16_t border)
{
    fill_rect(spi_handle, x-1,y-1,w+2,h+2,border);
    fill_rect(spi_handle, x,y,w,h,bg);
    uint16_t fw = (w*pct)/100;
    if(fw) fill_rect(spi_handle, x,y,fw,h,fg);
}

/*  상태값 → 색상 / blink 필요 여부  */
static inline uint16_t status_to_color(uint8_t st)
{
    switch(st){
        case 0:  return COLOR_GREEN;
        case 1:  return COLOR_WHITE;
        case 2:  return COLOR_YELLOW;
        case 3:  return COLOR_RED;
        case 4:  return COLOR_BLUE;
        default: return COLOR_GRAY;
    }
}
static inline bool status_need_blink(uint8_t st)
{
    return (st==0 || st==2 || st==3 || st==4);
}



/* ===================================================================== */
/* -----------------  helpers (텍스트 영역 관리)  ----------------------- */
/* ===================================================================== */
static uint16_t measure_text(const char *s, uint8_t sc, uint8_t sp)
{
    size_t len = strlen(s);
    if(!len) return 0;
    return len*(5*sc+sp) - sp;
}
static void clear_text_box(spi_device_handle_t spi_handle, uint16_t x,uint16_t y,
                           const char *old_s,
                           uint8_t sc,uint8_t sp,
                           uint16_t bg)
{
    uint16_t w = measure_text(old_s,sc,sp);
    uint16_t h = 7*sc;
    if(w) fill_rect(spi_handle, x,y,w,h,bg);
}
static void clear_area(spi_device_handle_t spi_handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bg)
{
    fill_rect(spi_handle, x,y,w,h,bg);
}

/* ===================================================================== */
/* ----------------------  정적 UI (한 번만)  --------------------------- */
/* ===================================================================== */
static void draw_boxes_static(spi_device_handle_t spi_handle)
{
    /* 상단 바 & 배경 */
    fill_rect(spi_handle,0,0,LCD_W,LCD_H,COL_BG);
    fill_rect(spi_handle,0,0,LCD_W,35,COL_TOP);

    /* 로고 & 프로그레스 바 프레임 */
    draw_bitmap(spi_handle,5,4,logo_bitmap,28,28,COL_LOGO);
    draw_progress_bar(spi_handle,55,4,100,28,0,COLOR_GREEN,COLOR_GRAY,COLOR_WHITE);

    /* 전력 아이콘 & 고정 라벨 */
    draw_bitmap(spi_handle,DEV_START_X,45,current_bitmap,28,28,COLOR_WHITE);
    draw_text(spi_handle,DEV_START_X+100,55,"kwh",COLOR_WHITE,2,1);

    /* 하단 라벨 */
    draw_text(spi_handle,DEV_START_X+10,50+28+4,"Set.T:",COLOR_WHITE,2,1);
    draw_text(spi_handle,DEV_START_X+10,50+2*(28+2),"Out.T:",COLOR_WHITE,2,1);
}



/////////////////////////////  TDOD: 각각 업데이트 하는 함수로 만들기 //////////////////////////////
/* ===================================================================== */
/* -----------------  상태/디바이스 아이콘 업데이트  -------------------- */
/* ===================================================================== */
static void draw_status_icon(spi_device_handle_t spi_handle, uint16_t x, const uint16_t*bm,
                             uint8_t st, bool blink_on)
{
    bool need_blink = status_need_blink(st);
    if(!need_blink || blink_on)
        draw_bitmap(spi_handle,x,ICON_Y,bm,ICON_W,ICON_H,status_to_color(st));
    else
        fill_rect(spi_handle,x,ICON_Y,ICON_W,ICON_H,COL_TOP);
}
static void update_status_icons(spi_device_handle_t spi_handle, bool blink_on, tcui_state_t *tcui)
{
    draw_status_icon(spi_handle,ICON_WIFI_X ,wifi_bitmap  ,tcui->status_wifi ,blink_on);
    draw_status_icon(spi_handle,ICON_CLOUD_X,cloud_bitmap ,tcui->status_cloud,blink_on);
    draw_status_icon(spi_handle,ICON_ALARM_X,alarm_bitmap ,tcui->status_alarm,blink_on);
    draw_status_icon(spi_handle,ICON_WARN_X ,warning_bitmap,tcui->status_warning,blink_on);
}

static void update_device_icons(spi_device_handle_t spi_handle, bool blink_on, tcui_state_t *tcui, device_icon_t *tbl)
{
    // device_icon_t tbl[4] = {
    //     { comp_bitmap,&tcui->status_comp },
    //     { def_bitmap ,&tcui->status_def  },
    //     { fan_bitmap ,&tcui->status_fan  },
    //     { led_bitmap ,&tcui->status_led  },
    // };

    uint16_t x = tbl->x;
    uint8_t st = *tbl->st;
    bool need_blink = status_need_blink(st);
    if (!need_blink || blink_on)
        draw_bitmap(spi_handle, x, DEV_START_Y,
                    tbl->bm, DEV_ICON_W, DEV_ICON_H,
                    status_to_color(st));
    else
        fill_rect(spi_handle, x, DEV_START_Y, DEV_ICON_W, DEV_ICON_H, COL_BG);
}

static void update_text_field(spi_device_handle_t spi_handle, dyn_field_t *d, float update_value)
{
    char newstr[16];
    text_return(newstr, sizeof(newstr), d->format, update_value);
    if (strcmp(newstr, d->buf) != 0)
    {
        clear_text_box(spi_handle, d->x, d->y, d->buf, d->sc, d->sp, d->bg);
        draw_text(spi_handle, d->x, d->y, newstr, COLOR_WHITE, d->sc, d->sp);
        strcpy(d->buf, newstr);
    }
}

static void display_task(void *arg)
{
    spi_device_handle_t spi_handle = *((spi_device_handle_t *)arg);

    tcui_state_t tcui_old;
    tcui_old = tcui_init;

    while(1)
    {
        tcui_state_t tcui_update = {
            .brightness = 70,   // 우선 고정값
            .machine_condition = tc_state_global.machine_condition,   // 우선 고정값
            .temp_internal = ce_relay_temp_control_global.current_temp,
            .temp_target = ce_relay_temp_control_global.target_temp,
            .energy_consumption = ce_env_sensor_data_global.current[0] * 220 / 1000,
            .temp_outside = ce_env_sensor_data_global.temp[5],
            .unit_temp = 'C',
            .status_warning = tc_state_global.warning_connected,  // 우선 고정값
            .status_alarm = tc_state_global.alarm_connected,    // 우선 고정값
            .status_cloud = tc_state_global.mqtt_connected,
            .status_wifi = tc_state_global.wifi_connected
        };

        if (ce_relay_state_global[RELAY_COMP].relay_connection == false) {
            tcui_update.status_comp = 5;
        }
        else if (ce_relay_state_global[RELAY_COMP].relay_state == false) {
            tcui_update.status_comp = 1;
        }
        else if (ce_relay_state_global[RELAY_COMP].relay_state == true) {
            tcui_update.status_comp = 4;
        }

        if (ce_relay_state_global[RELAY_FAN].relay_connection == false) {
            tcui_update.status_fan = 5;
        }
        else if (ce_relay_state_global[RELAY_FAN].relay_state == false) {
            tcui_update.status_fan = 1;
        }
        else if (ce_relay_state_global[RELAY_FAN].relay_state == true) {
            tcui_update.status_fan = 4;
        }

        if (ce_relay_state_global[RELAY_DEF].relay_connection == false) {
            tcui_update.status_def = 5;
        }
        else if (ce_relay_state_global[RELAY_DEF].relay_state == false) {
            tcui_update.status_def = 1;
        }
        else if (ce_relay_state_global[RELAY_DEF].relay_state == true) {
            tcui_update.status_def = 4;
        }

        if (ce_relay_state_global[RELAY_AUX_LIGHT].relay_connection == false) {
            tcui_update.status_led = 5;
        }
        else if (ce_relay_state_global[RELAY_AUX_LIGHT].relay_state == false) {
            tcui_update.status_led = 1;
        }
        else if (ce_relay_state_global[RELAY_AUX_LIGHT].relay_state == true) {
            tcui_update.status_led = 4;
        }


        // tcui_update, tcui_old 비교

        if (tcui_update.temp_internal != tcui_old.temp_internal) {
            update_text_field(spi_handle, &config_temp_internal, tcui_update.temp_internal);
            tcui_old.temp_internal = tcui_update.temp_internal;
        }

        if (tcui_update.temp_target != tcui_old.temp_target) {
            update_text_field(spi_handle, &config_set_temp, tcui_update.temp_target);
            tcui_old.temp_target = tcui_update.temp_target;
        }

        if (tcui_update.temp_outside != tcui_old.temp_outside) {
            update_text_field(spi_handle, &config_out_temp, tcui_update.temp_outside);
            tcui_old.temp_outside = tcui_update.temp_outside;
        }

        if (tcui_update.energy_consumption != tcui_old.energy_consumption) {
            update_text_field(spi_handle, &config_energy, tcui_update.energy_consumption);
            tcui_old.energy_consumption = tcui_update.energy_consumption;
        }

        if (tcui_update.machine_condition != tcui_old.machine_condition) {
            update_text_field(spi_handle, &config_status_bar, tcui_update.machine_condition);
            tcui_old.machine_condition = tcui_update.machine_condition;
        }

        if (tcui_update.status_warning != tcui_old.status_warning) {
            bool blink_on = status_need_blink(tcui_update.status_warning);
            draw_status_icon(spi_handle,ICON_WARN_X ,warning_bitmap,tcui_update.status_warning, blink_on);
        }
        if (tcui_update.status_alarm != tcui_old.status_alarm) {
            bool blink_on = status_need_blink(tcui_update.status_alarm);
            draw_status_icon(spi_handle,ICON_ALARM_X ,alarm_bitmap,tcui_update.status_alarm, blink_on);
        }
        if (tcui_update.status_cloud != tcui_old.status_cloud) {
            bool blink_on = status_need_blink(tcui_update.status_cloud);
            draw_status_icon(spi_handle,ICON_CLOUD_X ,cloud_bitmap,tcui_update.status_cloud, blink_on);
        }
        if (tcui_update.status_wifi != tcui_old.status_wifi) {
            bool blink_on = status_need_blink(tcui_update.status_wifi);
            draw_status_icon(spi_handle,ICON_WIFI_X ,wifi_bitmap,tcui_update.status_wifi, blink_on);
        }

        if (tcui_update.status_comp != tcui_old.status_comp) {
            bool blink_on = status_need_blink(tcui_update.status_comp);
            device_icon_t tbl_comp = {
                .bm = comp_bitmap,
                .st = &tcui_update.status_comp,
                .x = DEVICE_ICON_COMP_X
            };
            update_device_icons(spi_handle, blink_on, &tcui_update, &tbl_comp);
        }
        if (tcui_update.status_def != tcui_old.status_def) {
            bool blink_on = status_need_blink(tcui_update.status_def);
            device_icon_t tbl_def = {
                .bm = def_bitmap,
                .st = &tcui_update.status_def,
                .x = DEVICE_ICON_DEF_X
            };
            update_device_icons(spi_handle, blink_on, &tcui_update, &tbl_def);
        }
        if (tcui_update.status_fan != tcui_old.status_fan) {
            bool blink_on = status_need_blink(tcui_update.status_fan);
            device_icon_t tbl_fan = {
                .bm = fan_bitmap,
                .st = &tcui_update.status_fan,
                .x = DEVICE_ICON_FAN_X
            };
            update_device_icons(spi_handle, blink_on, &tcui_update, &tbl_fan);
        }
        if (tcui_update.status_led != tcui_old.status_led) {
            bool blink_on = status_need_blink(tcui_update.status_led);
            device_icon_t tbl_led = {
                .bm = led_bitmap,
                .st = &tcui_update.status_led,
                .x = DEVICE_ICON_LED_X
            };
            update_device_icons(spi_handle, blink_on, &tcui_update, &tbl_led);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void lcd_init(spi_device_handle_t spi_handle)
{
    gpio_set_level(LCD_RESET,0); 
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(LCD_RESET,1); 
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(spi_handle, LCD_CMD_SWRESET); 
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(spi_handle, LCD_CMD_SLPOUT); 
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(spi_handle, LCD_CMD_COLMOD); 

    uint8_t pf[]={0x05}; 
    lcd_data(spi_handle, pf,1);

    lcd_cmd(spi_handle, LCD_CMD_MADCTL); 
    vTaskDelay(pdMS_TO_TICKS(150));

    uint8_t mad[]={0x60}; 
    lcd_data(spi_handle, mad,1);

    lcd_cmd(spi_handle, LCD_CMD_INVON);
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(spi_handle, LCD_CMD_DISPON);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static void backlight_init(void)
{
    ledc_timer_config_t tm =
        {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK
        };

    ledc_timer_config(&tm);

    ledc_channel_config_t ch =
        {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .gpio_num = LCD_LEDA,
            .duty = 0
        };

    ledc_channel_config(&ch);
}
static void set_backlight(uint8_t pct)
{
    if(pct>100) pct=100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,pct*255/100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
}



ce_error_t display_init(void)
{
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_RESET, GPIO_MODE_OUTPUT);
    
    spi_device_handle_t st7789_spi_handle;

    // printf("display_init\n");

    spi_bus_config_t lcd_spi_config = {
        .mosi_io_num = LCD_MOSI,
        .sclk_io_num = LCD_CLK,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    esp_err_t ret;

    ret = spi_bus_initialize(SPI3_HOST, &lcd_spi_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        printf("SPI bus initialization failed\n");
        return CE_ERROR_SENSOR_INIT;
    }

    spi_device_interface_config_t lcd_spi_device_config = {
        .clock_speed_hz = 10*1000*1000, 
        .mode = 0,
        .spics_io_num = LCD_CS,
        .queue_size = 7
    };

    ret = spi_bus_add_device(SPI3_HOST, &lcd_spi_device_config, &st7789_spi_handle);
    if (ret != ESP_OK)
    {
        printf("SPI device add failed\n");
        return CE_ERROR_SENSOR_INIT;
    }

    tcui_init.brightness = 50;  
    tcui_init.machine_condition = 75;
    tcui_init.temp_internal = ce_relay_temp_control_global.current_temp;
    tcui_init.temp_target = ce_relay_temp_control_global.target_temp;
    tcui_init.energy_consumption = ce_env_sensor_data_global.current[0] * 220;
    tcui_init.temp_outside = ce_env_sensor_data_global.temp[5];
    tcui_init.unit_temp = 'C';
    tcui_init.status_warning = CONNECTION_NEEDED;
    tcui_init.status_alarm = CONNECTION_NEEDED;
    tcui_init.status_cloud = CONNECTION_NEEDED;
    tcui_init.status_wifi = CONNECTION_NEEDED;
    tcui_init.status_comp = CONNECTION_NEEDED;
    tcui_init.status_def = CONNECTION_NEEDED;
    tcui_init.status_fan = CONNECTION_NEEDED;
    tcui_init.status_led = CONNECTION_NEEDED;
    
    backlight_init();
    set_backlight(tcui_init.brightness);

    lcd_init(st7789_spi_handle);

    draw_boxes_static(st7789_spi_handle);

    // 화면 초기화
    update_text_field(st7789_spi_handle, &config_temp_internal, tcui_init.temp_internal);
    update_text_field(st7789_spi_handle, &config_set_temp, tcui_init.temp_target);
    update_text_field(st7789_spi_handle, &config_out_temp, tcui_init.temp_outside);
    update_text_field(st7789_spi_handle, &config_energy, tcui_init.energy_consumption);
    update_text_field(st7789_spi_handle, &config_status_bar, tcui_init.machine_condition);

    /* D) 온도 단위 °C / °F */

    clear_area(st7789_spi_handle, 0, 140, 160, 30, COL_BG);
    if (tcui_init.unit_temp == 'C')
    {
        draw_char(st7789_spi_handle, 35, 140, 127, COLOR_WHITE, 2);
        draw_char(st7789_spi_handle, 50, 140, 'C', COLOR_WHITE, 2);
        draw_text(st7789_spi_handle, 70, 140, "(celsius)", COLOR_WHITE, 2, 1);
    }
    else
    {
        draw_char(st7789_spi_handle, 10, 140, 127, COLOR_WHITE, 2);
        draw_char(st7789_spi_handle, 25, 140, 'F', COLOR_WHITE, 2);
        draw_text(st7789_spi_handle, 40, 140, "(Fahrenheit)", COLOR_WHITE, 2, 1);
    }

    bool blink_on = status_need_blink(tcui_init.status_warning);
    draw_status_icon(st7789_spi_handle,ICON_WARN_X,warning_bitmap,tcui_init.status_warning,blink_on);

    blink_on = status_need_blink(tcui_init.status_alarm);
    draw_status_icon(st7789_spi_handle,ICON_ALARM_X,alarm_bitmap,tcui_init.status_alarm,blink_on);

    blink_on = status_need_blink(tcui_init.status_cloud);
    draw_status_icon(st7789_spi_handle,ICON_CLOUD_X,cloud_bitmap,tcui_init.status_cloud,blink_on);

    blink_on = status_need_blink(tcui_init.status_wifi);
    draw_status_icon(st7789_spi_handle,ICON_WIFI_X,wifi_bitmap,tcui_init.status_wifi,blink_on);

    device_icon_t tbl_comp = {
        .bm = comp_bitmap,
        .st = &tcui_init.status_comp,
        .x = DEVICE_ICON_COMP_X
    };
    blink_on = status_need_blink(tcui_init.status_comp);
    update_device_icons(st7789_spi_handle, blink_on, &tcui_init, &tbl_comp);

    device_icon_t tbl_def = {
        .bm = def_bitmap,
        .st = &tcui_init.status_def,
        .x = DEVICE_ICON_DEF_X
    };
    blink_on = status_need_blink(tcui_init.status_def);
    update_device_icons(st7789_spi_handle, blink_on, &tcui_init, &tbl_def);
    
    device_icon_t tbl_fan = {
        .bm = fan_bitmap,
        .st = &tcui_init.status_fan,
        .x = DEVICE_ICON_FAN_X
    };
    blink_on = status_need_blink(tcui_init.status_fan);
    update_device_icons(st7789_spi_handle, blink_on, &tcui_init, &tbl_fan);

    device_icon_t tbl_led = {
        .bm = led_bitmap,
        .st = &tcui_init.status_led,
        .x = DEVICE_ICON_LED_X
    };
    blink_on = status_need_blink(tcui_init.status_led);
    update_device_icons(st7789_spi_handle, blink_on, &tcui_init, &tbl_led);
    
    if (xTaskCreatePinnedToCore(display_task, "display_task", 4000, &st7789_spi_handle, 5, NULL, tskNO_AFFINITY) != pdPASS)
    {
        return CE_ERROR_TASK_CREATE;
    }
    
    
    return CE_OK;
}