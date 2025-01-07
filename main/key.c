/**
 * @file key.c
 * @brief 
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-09-10
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */

#include "key.h"


void key_config(void)
{
    gpio_config_t key_cfg = {
        .pin_bit_mask = (((uint64_t)1 )<<KEY_GPIO),
        .pull_up_en   = GPIO_PULLUP_ENABLE, //开内部上拉
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_INPUT,    //输入墨水
        .intr_type = GPIO_INTR_DISABLE,
    };
    printf("%ld\n",(long int)(key_cfg.pin_bit_mask & 0xFFFFFFFF)) ;
    printf("%ld\n",(long int)(key_cfg.pin_bit_mask>>32)) ;
    gpio_config(&key_cfg);  
}

void key_task(void *pvParameters)
{
    uint8_t flag=0;
    key_config();
    while(1)
    {
        if(GET_KEY_STATUS() == KEY_DOWN)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            if(GET_KEY_STATUS() == KEY_DOWN)
            {
                printf("key down\n");
                if(flag)
                    UART2_TO_WT;
                else 
                    UART2_TO_AT;
                flag = !flag;

                //等待按键松开
                while(GET_KEY_STATUS() == KEY_DOWN)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }     
        }
        vTaskDelay(pdMS_TO_TICKS(30));    
    }
}