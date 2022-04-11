#include "robot_face_screen.h"

#include <string.h>

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "logo_en_240x240_lcd.h"

static const char TAG[] = "LCD";

RobotLCD::RobotLCD(spi_bus_handle_t spi_bus, gpio_num_t pin_cs)
{
    do
    {
        spi_config_t bus_conf = {
            .miso_io_num = GPIO_NUM_13,
            .mosi_io_num = GPIO_NUM_11,
            .sclk_io_num = GPIO_NUM_12,
            .max_transfer_sz = 2 * 240 * 240 + 10,
        };
        spi_bus_handle_t spi_bus = spi_bus_create(SPI2_HOST, &bus_conf);

        scr_interface_spi_config_t spi_lcd_cfg = {
            .spi_bus = spi_bus,
            .pin_num_cs = pin_cs,
            .pin_num_dc = GPIO_NUM_1,
            .clk_freq = 40 * 1000000,
            .swap_data = 0,
        };

        scr_interface_driver_t *iface_drv;
        scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);
        if (ESP_OK != scr_find_driver(SCREEN_CONTROLLER_ST7789, &this->driver))
        {
            ESP_LOGE(TAG, "screen find failed");
            break;
        }

        scr_controller_config_t lcd_cfg = {
            .interface_drv = iface_drv,
            .pin_num_rst = GPIO_NUM_9,
            .pin_num_bckl = GPIO_NUM_0,
            .rst_active_level = 0,
            .bckl_active_level = 0,
            .width = 240,
            .height = 240,
            .offset_hor = 0,
            .offset_ver = 0,
            .rotate = (scr_dir_t)0,
        };

        if (ESP_OK != this->driver.init(&lcd_cfg))
        {
            ESP_LOGE(TAG, "screen initialize failed");
            break;
        }

        scr_info_t lcd_info;
        this->driver.get_info(&lcd_info);
        ESP_LOGI(TAG, "Screen name:%s | width:%d | height:%d", lcd_info.name, lcd_info.width, lcd_info.height);

        this->draw_color(0x000000);
        vTaskDelay(pdMS_TO_TICKS(500));
        this->draw_wallpaper();
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (0);
}

void RobotLCD::draw_wallpaper()
{
    uint16_t *pixels = (uint16_t *)heap_caps_malloc((logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (NULL == pixels)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, logo_en_240x240_lcd, (logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t));
    this->driver.draw_bitmap(0, 0, logo_en_240x240_lcd_width, logo_en_240x240_lcd_height, (uint16_t *)pixels);
    heap_caps_free(pixels);

    this->paper_drawn = true;
}

void RobotLCD::draw_color(int color)
{
    scr_info_t lcd_info;
    this->driver.get_info(&lcd_info);
    uint16_t *buffer = (uint16_t *)malloc(lcd_info.width * sizeof(uint16_t));
    if (buffer)
    {
        for (size_t i = 0; i < lcd_info.width; i++)
        {
            buffer[i] = color;
        }

        for (int y = 0; y < lcd_info.height; y++)
        {
            this->driver.draw_bitmap(0, y, lcd_info.width, 1, buffer);
        }

        free(buffer);
    }
    else
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
}

void RobotLCD::update()
{
    if (this->switch_on == false)
    {
        this->paper_drawn = false;
    }
}

static void task(RobotLCD *self)
{
    ESP_LOGD(TAG, "Start");

    while (true)
    {

    }
    ESP_LOGD(TAG, "Stop");
    self->draw_wallpaper();
    vTaskDelete(NULL);
}

void RobotLCD::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 2 * 1024, this, 5, NULL, 1);
}
