#include <stdio.h>
#include <esp_log.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "servo.h"
#include "robot.h"
#include "driver/spi_master.h"

static const char *TAG = "robot-main";
static Robot electron;

#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_12
#define PIN_NUM_CS   GPIO_NUM_10

#define PIN_NUM_DC   GPIO_NUM_1
#define PIN_NUM_RST  GPIO_NUM_9
#define PIN_NUM_BCKL GPIO_NUM_0



extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "esp32-s3 robot");

    electron.do_init();

    Servo::servoPwmTimerInit();
    Servo servo_head(LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    servo_head.servoPwmInit();
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
