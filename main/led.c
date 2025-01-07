/**
 * @file led.c
 * @brief 心跳灯
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-25
 * 
 * @copyright Copyright (c) 2024 
 * 
 * @par 修改日志:
 */
#include "led.h"

/**
 * @brief 系统心跳LED等配置及任务
 */
void heart_led_init(void)
{
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1<<LED_GPIO),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&led_cfg);
}


//心跳灯任务函数
void heart_led_task(void *pvParameters)
{
    heart_led_init();
    while(1)
    {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        // size_t free_heap_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
        // printf("Free heap size: %d bytes\n", free_heap_size);
    }
}


