#include <stdio.h>
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h" // "esp_spi_flash.h" // DEPRECATED

void app_main(void)
{
    printf("\nHello world!\n");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info); // This function will give us the values of all the structure esp_chip_info_t members
    printf("This is %s chip with %d CPU cores(s), WiFi%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);
    
    uint32_t size_flash_chip;
    esp_flash_get_size(NULL, &size_flash_chip);
    printf("%luMB %s flash\n", size_flash_chip / (1024 * 1024), 
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minumun free heap size: %lu bytes\n", esp_get_minimum_free_heap_size());
}