#pragma once

#include "screen_driver.h"

#include "robot__base.h"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define LCD_HOST SPI2_HOST

class RobotLCD
{


public:
    scr_driver_t driver;
    bool switch_on;
    bool paper_drawn;

    RobotLCD(spi_bus_handle_t spi_bus, gpio_num_t pin_cs);
    virtual ~RobotLCD(){}

    void draw_wallpaper();
    void draw_color(int color);

    void update();

    void run();
private:
};
