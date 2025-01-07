#ifndef __INERTIA_GPS_COMM_H__
#define __INERTIA_GPS_COMM_H__

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "freertos/semphr.h"



#define IN442 GPIO_NUM_15
#define EN442 GPIO_NUM_16
#define UART2_PORT UART_NUM_2

#define  UART2_TO_WT   gpio_set_level(IN442, 0)//惯性器件
#define  UART2_TO_AT   gpio_set_level(IN442, 1)//GPS

extern SemaphoreHandle_t uart2_mux; // UART Mutex 互斥锁





void inertia_gps_comm_init(void);
void inertia_gps_comm_uart_init(int baud);

esp_err_t uart2_write_data(const uint8_t *data, size_t length);
#endif // __INERTIA_GPS_COMM_H__


