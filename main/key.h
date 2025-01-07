/**
 * @file key.h
 * @brief 
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-09-10
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */
#ifndef __KEY_H__
#define __KEY_H__
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inertia_gps_comm.h"

#define KEY_GPIO GPIO_NUM_35

enum KEY_STATUS
{
    KEY_DOWN = 0,
    KEY_UP,
};


#define GET_KEY_STATUS() (gpio_get_level(KEY_GPIO))

void key_task(void *pvParameters);

#endif


