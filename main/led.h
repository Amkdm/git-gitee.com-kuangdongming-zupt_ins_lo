/**
 * @file led.h
 * @brief 
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */
#ifndef __LED_H__
#define __LED_H__

#include <stdio.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#define LED_GPIO GPIO_NUM_4

void heart_led_task(void *pvParameters);

#endif


