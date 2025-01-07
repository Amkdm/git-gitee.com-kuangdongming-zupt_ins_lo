/**
 * @file start_main.c
 * @brief 
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-25
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led.h"
#include "key.h"
#include "AIR780E_4G.h"
#include "inertia.h"
#include "gps.h"
#include "Navigation_solution.h"
#include "inertia_gps_comm.h"
#include "esp_task_wdt.h"


/**
 * @brief 开机用户打印信息
 */
void begin_log(void)
{
    /* Print chip information */
    // printf("This is %s (Chip Revision %d)\n", CONFIG_IDF_TARGET, ESP_GET_CHIP_REVISION());
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");


    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

}



void app_main(void)
{
    //开机信息打印
    begin_log();
    size_t psram_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    printf("Available PSRAM: %d bytes\n", psram_size);

    //惯性器件和GPS公共部分初始化-两者共用串口2
    inertia_gps_comm_init();

    //系统心跳灯任务-蓝灯 1Hz 亮灭闪烁
    xTaskCreatePinnedToCore(heart_led_task, "heart_led_task", 4096, NULL, 5, NULL, 0);
    
    //4G模块监测任务
    xTaskCreatePinnedToCore(air780e_task, "air780e_task", 4096, NULL, 5, NULL,0 );
    
    //4G模块传输融合数据任务
    xTaskCreatePinnedToCore(send_server_data, "send_server_data", 6000, NULL, 5, NULL,0 );

    //惯性器件读取任务创建
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 6, NULL,0 );

    //gps读取任务
    //xTaskCreatePinnedToCore(gps_task, "gps_task", 6000, NULL, 7, NULL,0 );

    //导航解算任务
    xTaskCreatePinnedToCore(navigation_solution_task, "nav_solu_task", 6000, NULL, 7, NULL,1);
}


