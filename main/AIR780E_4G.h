#ifndef __AIR780E_4G_H__
#define __AIR780E_4G_H__

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "gps.h"
#include "Navigation_solution.h"

#define AIR780E_MAX_LINK_NUM  6
#define RD_BUF_SIZE 1024            // 读缓冲区大小
typedef enum
{
    AIR780E_STAT_START = 0, //起始状态
    AIR780E_STAT_INIT,      //初始化
    AIR780E_STAT_CGREG,     //网络注册
    AIR780E_STAT_CGACT,     //PDP上下文激活
    AIR780E_STAT_CGATT0,    //附着状态查询
    AIR780E_STAT_CGATT1,    //附着
    AIR780E_STAT_CIICR,     //激活移动场景，获取IP地址
    AIR780E_STAT_OK,        //入网成功
    AIR780E_CONNECT_FAIL,   //入网失败
    AIR780E_CONNECT_OK,     //入网失败
} air780_stat_et;

typedef enum
{
    AIR780E_LINK_STAE_OK = 1, //起始状态
    AIR780E_LINK_STAE_FAIL = 0, //起始状态
} air780_linkstat_et;

/**
 * @brief 客户端信息
 */
typedef struct
{
    uint8_t sock_id;
    uint8_t conn_state;
    char type[5];          // UDP  TCP
    char dst_addr[30];     // 目标IP或域名
    uint16_t dst_port;
    uint16_t heart_time;   // 心跳周期
    uint32_t keep_time;
} Air780ClientStruct;

typedef struct
{
    uint8_t state; // 模块当前状态
    uint8_t rssi;  // 信号强度
    uint8_t link_stat;   // 连接服务器状态
    uint8_t imei_buff[16];
    uint8_t iccid_buff[20];
    void (*fun_recv_parse)(uint8_t sock_id, uint8_t *buff, uint16_t len); // 接收处理函数
    uart_port_t uart_port;  // 修改：明确指定 UART 端口
    float deg_sn, deg_we, hight;
    Air780ClientStruct client_list[AIR780E_MAX_LINK_NUM];
} Air780WorkStruct;

void drv_air780_init(void);
void drv_air780_reg_process(void);

uint8_t drv_get_4g_work_status(void);
uint8_t drv_get_4g_link_status(void);
void drv_air780_AT_test(void);

void drv_air780_client_connect(uint8_t sock_id, char *type, char *dst_addr, uint16_t dst_port);
int drv_air780_send_data(uint8_t sock_id, uint8_t *buff, uint16_t len);

void air780e_task(void *pvParameters);
void send_server_data(void *pvParameters);
#endif // __AIR780E_4G_H__
