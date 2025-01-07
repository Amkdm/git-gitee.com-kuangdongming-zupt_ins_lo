/**
 * @file inertia_gps_comm.c
 * @brief 惯性器件和GPS器件共用串口的配置及其互斥相关的配置接口
 *        本模块初始需要放在惯性器件和gps初始化之前
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */
#include "inertia_gps_comm.h"




SemaphoreHandle_t uart2_mux = NULL;

int imu_rec_flag = 0;//imu接收标志位

#define UART2_RD_BUF_SIZE 1024            // 读缓冲区大小










void inertia_gps_comm_uart_init(int baud)
{
    uart_set_baudrate(UART2_PORT, baud);
}

void inertia_gps_comm_init(void)
{
    const uart_port_t uart_num = UART2_PORT;
    //串口初始化
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(uart_num, UART2_RD_BUF_SIZE * 2, 0, 0,  NULL, 0);
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));//配置串口
    uart_set_pin(uart_num, 6, 7, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);//配置引脚
   
    //串口选择IO初始化
    gpio_config_t uart_sel_cfg = {
        .pin_bit_mask = (1<<IN442),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&uart_sel_cfg);
    
    uart_sel_cfg.pin_bit_mask = (1<<EN442);
    gpio_config(&uart_sel_cfg);

    gpio_set_level(EN442, 0);
    gpio_set_level(IN442, 0);
    
    //互斥锁初始化
    uart2_mux = xSemaphoreCreateMutex();
}


/**
 * @brief 向 UART2 发送数据，并确保数据写入 UART2 稳定可靠
 *
 * @param data 数据缓冲区指针
 * @param length 数据长度
 * @return esp_err_t 返回 ESP_OK 表示传输成功，ESP_FAIL 表示失败
 */
esp_err_t uart2_write_data(const uint8_t *data, size_t length)
{
    // 写入整个数据到 UART2
    int written = uart_write_bytes(UART2_PORT, (const char *)data, length);
    
    if (written < 0)
    {
        printf("Error: UART write failed.\n");
        return ESP_FAIL;  // 发送失败
    }

    // 确保数据全部发送完成，防止数据丢失
    if (uart_wait_tx_done(UART2_PORT, pdMS_TO_TICKS(200)) != ESP_OK)
    {
        printf("Error: UART transmission timeout.\n");
        return ESP_FAIL;  // 发送超时
    }
    //uart_flush(UART2_PORT); //清除 UART2 发送接收缓冲区
    return ESP_OK;  // 发送成功
}




