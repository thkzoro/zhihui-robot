#ifndef ESP_IDF_V4_4_UTILS_H
#define ESP_IDF_V4_4_UTILS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define delay(x)	vTaskDelay(x / portTICK_PERIOD_MS)
#define delay_ms(x) delay(x)

#endif //ESP_IDF_V4_4_UTILS_H
