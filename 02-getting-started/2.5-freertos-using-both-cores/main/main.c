#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "sdkconfig.h"

#define BLUE_BLINK_GPIO GPIO_NUM_25
#define RED_BLINK_GPIO GPIO_NUM_26

bool blueLedState = 0;
bool redLedState = 0;

static void blueLedTask1(void *parameter)
{
    while (true)
    {
        printf("blueLedTask1\trunning on core: %d\n", xPortGetCoreID());
        gpio_set_level(BLUE_BLINK_GPIO, blueLedState);
        blueLedState = !blueLedState;
        vTaskDelay(1400 / portTICK_PERIOD_MS);
    }
}

static void redLedTask2(void *parameter)
{
    while (true)
    {
        printf("redLedTask2\trunning on core: %d\n", xPortGetCoreID());
        gpio_set_level(RED_BLINK_GPIO, redLedState);
        redLedState = !redLedState;
        vTaskDelay(700 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // gpio_pad_select_gpio() LOOK FOR THIS FUNCTION IN #include "rom/gpio.h"
    gpio_reset_pin(BLUE_BLINK_GPIO);
    gpio_set_direction(BLUE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RED_BLINK_GPIO);
    gpio_set_direction(RED_BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Create a task that will be excecuted in the blueLedTask1() function, with priority 1 and excecuted on core 0
    xTaskCreatePinnedToCore(
        blueLedTask1,       // Function to implement the task
        "Blue LED Task1",   // Name of the task
        2048,               // Stack size in words
        NULL,               // Task input parameter
        1,                  // Priority of the task
        NULL,               // Task handle
        0);                 // Core where the task should run

    // Create a task that will be excecuted in the redLedTask2() function, with priority 1 and excecuted on core 0
    xTaskCreatePinnedToCore(
        redLedTask2,        // Function to implement the task
        "Red LED Task2",    // Name of the task
        2048,               // Stack size in words
        NULL,               // Task input parameter
        1,                  // Priority of the task
        NULL,               // Task handle
        1);                // Core where the task should run
}