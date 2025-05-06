#ifndef CE_TC_DISPLAY_H
#define CE_TC_DISPLAY_H

#include <ce/tc/esp32.h>


#define LCD_SPI_SDA SPI3_MOSI
#define LCD_SPI_SCLK SPI3_CLK
#define LCD_SPI_CS SPI3_CS0


ce_error_t ce_st7789_init(void);

#endif