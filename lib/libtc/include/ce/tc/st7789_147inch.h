#ifndef ST7789_147INCH_H
#define ST7789_147INCH_H

#include <stdint.h>
#include <stdbool.h>

#include <ce/tc/esp32.h>

#define LCD_MOSI SPI3_MOSI
#define LCD_CLK SPI3_CLK
#define LCD_CS SPI3_CS0

#define LCD_RESET 14
#define LCD_DC 15
#define LCD_LEDA 16

/* === LCD 해상도 === */
#define LCD_W 320
#define LCD_H 172
#define X_OFF 0
#define Y_OFF 34     /* ST7789 1.47" 240×280 → 320×172 가로모드 shift */

/* === 컬러 팔레트 (RGB565) === */
#define COL_BG    0x0000
#define COL_TOP   0x7BEF
#define COL_LOGO  0x07FF
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_GRAY    0x8410
#define COLOR_GREEN   0x07E0
#define COLOR_YELLOW  0xFFE0
#define COLOR_RED     0xF800
#define COLOR_BLUE    0x001F

/* === 위치 상수 === */
#define ICON_W   28
#define ICON_H   28
#define ICON_Y    4
#define ICON_WIFI_X   280
#define ICON_CLOUD_X  244
#define ICON_ALARM_X  208
#define ICON_WARN_X   172

#define DEV_ICON_W 28
#define DEV_ICON_H 28
#define DEV_GAP    4
#define DEV_START_X 180
#define DEV_START_Y 140

#define CONNECTION_NEEDED 0
#define NORMAL 1
#define WARNING 2
#define FAULT 3
#define RUNNING 4
#define NOT_EXIST 5



typedef struct {
    /* 일반 설정 */
    uint8_t brightness;          // 0‑100%

    /* 상단 Progress bar */
    uint8_t machine_condition;   // 0‑100 %

    /* 온도 / 에너지 */
    float temp_internal;       // °C or °F
    float temp_target;         // 설정온도
    float energy_consumption;  // kWh
    float temp_outside;        // 외기온

    /* 단위 */
    char unit_temp;           // 'C' or 'F'

    /* 상태 플래그들 */ //0 -> 연결필요(초록색 깜빡임), 1-> 정상(흰색 정적), 2-> 경고 (노랑 깜빡임), 3-> 고장(빨강 깜빡), 4-> 동작(파랑 깜빡), 5->없음 (회색 정적)
    uint8_t status_warning;      // 5:없음 2:있음 3:고장
    uint8_t status_alarm;        // 5:없음 1:있음
    uint8_t status_cloud;        // 0:초기연결필요 1:연결 2:끊김
    uint8_t status_wifi;         // 0:초기연결필요 1:연결 2:끊김
    uint8_t status_comp;         // 1:OFF 2:WARN 3:고장 4:동작중, 5:없음(회색)
    uint8_t status_def;          // 1:OFF 2:WARN 3:고장 4:동작중, 5:없음(회색)
    uint8_t status_fan;          // 1:OFF 2:WARN 3:고장 4:동작중, 5:없음(회색)
    uint8_t status_led;          // 1:OFF 2:WARN 3:고장 4:동작중, 5:없음(회색)
} tcui_state_t;

// 폰트, 비트맵 파일
const uint8_t font5x7[][5];

const uint16_t logo_bitmap[28 * 28];

const uint16_t warning_bitmap[28 * 28];
const uint16_t alarm_bitmap[28 * 28];
const uint16_t cloud_bitmap[28 * 28];
const uint16_t wifi_bitmap[28 * 28];

const uint16_t comp_bitmap[28 * 28];
const uint16_t def_bitmap[28 * 28];
const uint16_t fan_bitmap[28 * 28];
const uint16_t led_bitmap[28 * 28];

const uint16_t temp_bitmap[28 * 28];
const uint16_t current_bitmap[28 * 28];







#endif