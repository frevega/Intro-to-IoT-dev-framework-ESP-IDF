#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "Blink_example";

#define BLUE_BLINK_GPIO GPIO_NUM_25
#define RED_BLINK_GPIO GPIO_NUM_26

bool ledState = 0;

void app_main(void)
{
    gpio_reset_pin(BLUE_BLINK_GPIO);
    gpio_set_direction(BLUE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RED_BLINK_GPIO);
    gpio_set_direction(RED_BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (true) {
        ledState = !ledState;
        gpio_set_level(BLUE_BLINK_GPIO, ledState);
        gpio_set_level(RED_BLINK_GPIO, !ledState);
        ESP_LOGI(TAG, "Turning BLUE LED %s", ledState == 1 ? "ON" : "OFF");
        ESP_LOGI(TAG, "Turning RED  LED %s\n", ledState == 0 ? "ON" : "OFF");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}