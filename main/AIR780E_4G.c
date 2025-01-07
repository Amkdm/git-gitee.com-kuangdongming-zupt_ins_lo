/**
 * @file sim_4g.c
 * @brief 4g通信模块
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-28
 *
 * @copyright Copyright (c) 2024 
 *
 * @par 修改日志:
 */

#include "AIR780E_4G.h"

Air780WorkStruct g_sAir780Work = {0};


static QueueHandle_t uart_queue;    // UART 事件队列
static uint8_t *pBuff;


uint8_t drv_get_4g_work_status(void)
{
    return g_sAir780Work.state;
}

uint8_t drv_get_4g_link_status(void)
{
    return g_sAir780Work.link_stat;
}


// 获取系统启动以来的秒数
uint32_t drv_get_sec_counter_replacement(void)
{
    // 获取系统滴答数
    TickType_t ticks = xTaskGetTickCount();

    // 将滴答数转换为秒数（假设每秒 1000 个滴答）
    uint32_t seconds = ticks / configTICK_RATE_HZ;

    return seconds;
}

/*
================================================================================
描述 :
输入 :
输出 :
================================================================================
*/
void drv_air780_uart_send(char *send_buff)
{
    //	printf("***AT CMD:  %s", send_buff);
    if (g_sAir780Work.uart_port != UART_NUM_MAX)
    {
        uart_write_bytes(g_sAir780Work.uart_port, send_buff, strlen(send_buff));
    }
}


/*
================================================================================
描述 :
输入 :
输出 :
================================================================================
*/
void drv_air780_send_at(char *cmd_buff)
{
    static char send_buff[100] = {0};
    if (strlen(cmd_buff) + 10 < sizeof(send_buff))
    {
        memset(send_buff, 0, sizeof(send_buff));
        sprintf(send_buff, "AT+%s\r\n", cmd_buff);
        drv_air780_uart_send(send_buff);
    }
}

// 设置 UART 接收中断并处理数据
void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    char *pData;
    pBuff = (uint8_t *) malloc(RD_BUF_SIZE);
    if(pBuff == NULL) {
        printf("Failed to allocate memory for pBuff\n");
        return;
    }

    printf("准备进入串口1事件接收处理任务\n");
    for(;;) 
    {
        // 等待 UART 事件发生（通过队列）
        if(xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) 
        {
            bzero(pBuff, RD_BUF_SIZE);
            switch(event.type) 
            {
                // 数据接收
                case UART_DATA:
                    uart_read_bytes(UART_NUM_1, pBuff, event.size, portMAX_DELAY);
                    //printf("串口收到数据:%s\n", pBuff);
                    if ((pData = strstr((char *)pBuff, "+RECEIVE,")) != NULL) // 接收到数据
                    {
                        //printf("接收到数据:%s\n", pBuff);
                        pData = (char *)pBuff;
                        while ((pData = strstr(pData, "+RECEIVE,")) != NULL)
                        {
                            pData += strlen("+RECEIVE,");
                            uint8_t sock_id = atoi(pData);
                            //printf("sock_id=%d\n", sock_id);
                            if (sock_id < AIR780E_MAX_LINK_NUM && (pData = strstr(pData, ",")) != NULL)
                            {
                                pData += 1;
                                uint16_t data_len = atoi(pData);
                                if ((pData = strstr(pData, ":")) != NULL)
                                {
                                    pData += 3; //: 0D 0A
                                    g_sAir780Work.client_list[sock_id].keep_time = drv_get_sec_counter_replacement();
                                    //printf_hex("air780 data buff= ", (uint8_t *)pData, data_len);
                                    if (g_sAir780Work.fun_recv_parse != NULL)
                                    {
                                        g_sAir780Work.fun_recv_parse(sock_id, (uint8_t *)pData, data_len); // 接收处理
                                    }
                                    pData += data_len;
                                }
                            }
                        }
                    }
                    else if ((pData = strstr((char *)pBuff, "+CSQ: ")) != NULL) // 信号强度
                    {
                        //printf("信号强度:%s  ", pBuff);
                        pData += strlen("+CSQ: ");
                        g_sAir780Work.rssi = atoi(pData);
                        //printf("***rssi=%d\n", g_sAir780Work.rssi);
                    }
                    else if ((pData = strstr((char *)pBuff, "+CGNSINF:")) != NULL) // 定位信息
                    {
                        pData += strlen("+CGNSINF:");
                        //      printf("GPS=%s", pBuff);
                        for (uint8_t i = 0; i < 5; i++)
                        {
                            if ((pData = strstr((char *)pBuff, ",")) != NULL)
                            {
                                pData += 1;
                                switch (i)
                                {
                                case 2:
                                {
                                    g_sAir780Work.deg_sn = atof(pData); // 纬度
                                    break;
                                }
                                case 3:
                                {
                                    g_sAir780Work.deg_we = atof(pData); // 经度
                                    break;
                                }
                                case 4:
                                {
                                    g_sAir780Work.hight = atof(pData); // 海拔
                                    break;
                                }
                                }
                            }
                            else
                            {
                                break;
                            }
                        }
                        //printf("GPS: WE=%.3f deg, SN=%.3f deg, H=%.3f m\n", g_sAir780Work.deg_we, g_sAir780Work.deg_sn, g_sAir780Work.hight);
                    }
                    else if ((pData = strstr((char *)pBuff, "+CGEV: ")) != NULL) // PDP变化
                    {
                        pData += strlen("+CGEV: ");
                        if (strstr(pData, "DEACT") != NULL) // 移动场景去活
                        {
                            //printf("移动场景去活，重新激活,");
                            drv_air780_send_at("CIPSHUT"); // 去激活
                            vTaskDelay(100);
                            g_sAir780Work.state = AIR780E_STAT_START;
                            //printf("===RDY net_state=AIR780_STATE_START\n");
                        }
                        else if (strstr(pData, "ACT") != NULL) // 移动场景激活
                        {
                            if (g_sAir780Work.state == AIR780E_STAT_CIICR)
                            {
                                g_sAir780Work.state = AIR780E_STAT_OK;
                            }
                        }
                    }
                    else if ((pData = strstr((char *)pBuff, "+CGREG:")) != NULL) // 注册查询
                    {
                        //printf("注册查询讯息：");
                        pData += strlen("+CGREG:");
                        if ((pData = strstr((char *)pBuff, ",1")) != NULL || (pData = strstr((char *)pBuff, ",5")) != NULL) // 注册成功
                        {
                            g_sAir780Work.state = AIR780E_STAT_CGACT;
                            //printf("=== +CGREG:1\n");
                        }
                    }
                    else if ((pData = strstr((char *)pBuff, "+CGACT: ")) != NULL && (pData = strstr((char *)pBuff, ",1")) != NULL) // PDP查询
                    {
                        //printf("pdp查询讯息：");
                        g_sAir780Work.state = AIR780E_STAT_CGATT0;
                        //printf("=== AIR780_STAT_CGATT0\n");
                    }
                    else if ((pData = strstr((char *)pBuff, "+CGATT: 1")) != NULL) // 附着查询
                    {
                        //printf("附着查询讯息：");
                        g_sAir780Work.state = AIR780E_STAT_CGATT1;
                        //printf("=== +CGATT: 1\n");
                    }
                    else if ((pData = strstr((char *)pBuff, ", CONNECT OK")) != NULL) // 连接检测  || (pData=strstr((char*)pBuff, ", ALREADY CONNECT"))!=NULL
                    {
                        //printf("连接服务器成功：");
                        uint8_t sock_id = 0;
                        pData--;
                        //printf("### pData=%s\n", pData);
                        if (pData[0] >= '0' && pData[0] <= '9')
                        {
                            sock_id = pData[0] - '0';
                            //printf("=== air780 sock_id=%d connect ok!\n", sock_id);
                            if (sock_id < AIR780E_MAX_LINK_NUM)
                            {
                                g_sAir780Work.client_list[sock_id].conn_state = true;
                                g_sAir780Work.client_list[sock_id].keep_time = drv_get_sec_counter_replacement();
                            }
                        }
                        g_sAir780Work.link_stat = AIR780E_LINK_STAE_OK;
                    }
                    else if ((pData = strstr((char *)pBuff, ", CONNECT FAIL")) != NULL) // 连接检测  || (pData=strstr((char*)pBuff, ", ALREADY CONNECT"))!=NULL
                    {
                        //printf("连接失败：");
                        g_sAir780Work.link_stat = AIR780E_LINK_STAE_FAIL;
                    }
                    else if ((pData = strstr((char *)pBuff, ",CLOSE OK")) != NULL) // 关闭
                    {
                        //printf("连接关闭：");
                        uint8_t sock_id = 0;
                        pData--;
                        if (pData[0] >= '0' && pData[0] <= '9')
                        {
                            sock_id = pData[0] - '0';
                            //printf("=== sock_id=%d close ok!\n", sock_id);
                            if (sock_id < AIR780E_MAX_LINK_NUM)
                            {
                                g_sAir780Work.client_list[sock_id].conn_state = false;
                            }
                        }
                    }
                    else if ((pData = strstr((char *)pBuff, "+PDP: DEACT")) != NULL) // PDP失效
                    {
                        //printf("PDP失效，去激活：");
                        drv_air780_send_at("CIPSHUT"); // 去激活
                        vTaskDelay(100);
                        g_sAir780Work.state = AIR780E_STAT_START;
                        //printf("===RDY net_state=AIR780_STATE_START\n");
                    }
                    else if (pBuff[2] >= '0' && pBuff[2] <= '9' && g_sAir780Work.imei_buff[0] == 0) // IMEI检测
                    {
                        //printf("IMEI检测：");
                        bool is_number = true;
                        pBuff += 2;
                        for (uint8_t i = 0; i < 15; i++)
                        {
                            if (pBuff[i] < '0' || pBuff[i] > '9')
                            {
                                is_number = false;
                                break;
                            }
                        }
                        if (is_number == true)
                        {
                            memset(g_sAir780Work.imei_buff, 0, sizeof(g_sAir780Work.imei_buff));
                            memcpy(g_sAir780Work.imei_buff, pBuff, 15);
                            //printf("imei=%s\n", g_sAir780Work.imei_buff);
                        }
                    }
                    else if ((pData = strstr((char *)pBuff, "+ICCID: ")) != NULL) // ICCID检测
                    {
                        pData += strlen("+ICCID: ");
                        memset(g_sAir780Work.iccid_buff, 0, sizeof(g_sAir780Work.iccid_buff));
                        memcpy(g_sAir780Work.iccid_buff, pData, 20);
                        //printf("iccid=%s\n", g_sAir780Work.iccid_buff);
                    }
                    // 在这里你可以进一步处理接收到的数据
                    break;

                // 其他事件可以在这里处理
                case UART_FIFO_OVF:
                    //printf("UART FIFO Overflow\n");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    //printf("UART Buffer Full\n");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;

                case UART_PARITY_ERR:
                    //printf("UART Parity Error\n");
                    break;

                case UART_FRAME_ERR:
                    //printf("UART Frame Error\n");
                    break;

                default:
                    break;
            }
        }
    }
    free(pBuff);
    vTaskDelete(NULL);
}

void drv_air780_init(void)
{
    g_sAir780Work.uart_port = UART_NUM_1; // 修改：使用 UART1

    // 配置 UART 参数
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // 安装 UART 驱动，设置缓冲区大小
    uart_driver_install(g_sAir780Work.uart_port, 1024 * 2, 0, 20,  &uart_queue, 0);
    uart_param_config(g_sAir780Work.uart_port, &uart_config);
    uart_set_pin(g_sAir780Work.uart_port, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 修改：设置 GPIO 引脚
    

    // 创建一个任务来处理 UART 事件
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}



/*
================================================================================
描述 :
输入 :
输出 :
================================================================================
*/
void drv_air780_set_client(uint8_t index, char *dst_addr, uint16_t port, char *type)
{
    if (index < AIR780E_MAX_LINK_NUM)
    {
        Air780ClientStruct *pClient = &g_sAir780Work.client_list[index];
        if (strlen(dst_addr) < sizeof(pClient->dst_addr))
        {
            strcpy(pClient->dst_addr, dst_addr);
            pClient->sock_id = index;
            pClient->dst_port = port;
            strcpy(pClient->type, type);
            pClient->conn_state = 0;
            pClient->heart_time = 60;
        }
    }
}

/*
================================================================================
描述 :
输入 :
输出 :
================================================================================
*/
void drv_air780_fun_register(void (*fun_recv_parse)(uint8_t chn, uint8_t *buff, uint16_t len))
{
    g_sAir780Work.fun_recv_parse = fun_recv_parse;
}




/*
================================================================================
描述 : 数据发送
输入 :
输出 :
================================================================================
*/
int drv_air780_send_data(uint8_t sock_id, uint8_t *buff, uint16_t len)
{
    char cmd_buff[30] = {0};
    if (sock_id < AIR780E_MAX_LINK_NUM)
    {
        //printf("id[%d] connect stat:%d",sock_id,g_sAir780Work.client_list[sock_id].conn_state);
        if (g_sAir780Work.client_list[sock_id].conn_state > 0)
        {
            // 发送 AT 指令准备发送数据
            sprintf(cmd_buff, "AT+CIPSEND=%u,%u\r\n", sock_id, len);
            drv_air780_uart_send(cmd_buff);

            // 发送实际数据
            if (g_sAir780Work.uart_port != UART_NUM_MAX)
            {
                uart_write_bytes(g_sAir780Work.uart_port, (const char *)buff, len);
                return len;
            }
        }
    }
    return 0;
}

/*
================================================================================
描述 : 客户端连接
输入 :
输出 :
================================================================================
*/
void drv_air780_client_connect(uint8_t sock_id, char *type, char *dst_addr, uint16_t dst_port)
{
    static char cmd_buff[100] = {0};

    sprintf(cmd_buff, "CIPSTART=%d,\"%s\",\"%s\",%d", sock_id, type, dst_addr, dst_port);
    drv_air780_send_at(cmd_buff);
    //  printf("connect=%s\n", cmd_buff);
}

/*
================================================================================
描述 : 客户端断开
输入 :
输出 :
================================================================================
*/
void drv_air780_close(uint8_t sock_id)
{
    if (sock_id > AIR780E_MAX_LINK_NUM)
        return;
    char cmd_buff[30] = {0};
    sprintf(cmd_buff, "CIPCLOSE=%d", sock_id);
    drv_air780_send_at(cmd_buff);
    if (sock_id == AIR780E_MAX_LINK_NUM) // 全部关闭
    {
        for (uint8_t i = 0; i < AIR780E_MAX_LINK_NUM; i++)
        {
            g_sAir780Work.client_list[i].conn_state = 0;
        }
    }
    else
    {
        g_sAir780Work.client_list[sock_id].conn_state = 0;
    }
}





/*
================================================================================
描述 : 客户端连接管理任务
输入 :
输出 :
================================================================================
*/
void drv_air780_connect_process(void)
{
    uint32_t now_sec_time = drv_get_sec_counter_replacement();
    for (uint8_t i = 0; i < AIR780E_MAX_LINK_NUM; i++)
    {
        Air780ClientStruct *pClient = &g_sAir780Work.client_list[i];
        if (pClient->dst_port > 0) // 有连接需求
        {
            if (pClient->conn_state == 0)
            {
                drv_air780_client_connect(pClient->sock_id, pClient->type, pClient->dst_addr, pClient->dst_port);
            }
            else
            {
                int det_time = now_sec_time - pClient->keep_time;
                if (det_time > pClient->heart_time) // 心跳超时
                {
                    printf("sock_id=%d, heart time out!\n", i);
                    drv_air780_close(pClient->sock_id);
                }
            }
        }
    }
}

/*
================================================================================
描述 : 网络注册函数
输入 :
输出 :
================================================================================
*/
void drv_air780_reg_process(void)
{
    static uint32_t last_sec_time = 0, wait_time = 2;
    //  static char cmd_buff[100]={0};
    uint32_t now_sec_time = drv_get_sec_counter_replacement();
    if (now_sec_time - last_sec_time > wait_time)
    {
        switch (g_sAir780Work.state)
        {
            case AIR780E_STAT_START:
            {
                drv_air780_uart_send("ATE0\r\n");
                vTaskDelay(100);

                drv_air780_send_at("RESET"); // 复位模块
                g_sAir780Work.state = AIR780E_STAT_INIT;
                wait_time = 5;
                break;
            }
            case AIR780E_STAT_INIT:
            {
                drv_air780_uart_send("ATE0\r\n");
                vTaskDelay(20);
                drv_air780_send_at("CSQ"); // 查询信号强度
                vTaskDelay(20);
                drv_air780_send_at("CGNSPWR=1"); // 打开GPS
                vTaskDelay(20);
                drv_air780_send_at("CGNSAID=31,1,1,1"); // 使能位置辅助定位
                g_sAir780Work.state = AIR780E_STAT_CGREG;
                break;
            }
            case AIR780E_STAT_CGREG: // 查询网络注册状态
            {
                drv_air780_send_at("CGREG?"); // 查询网络连接信息
                wait_time = 3;
                break;
            }
            case AIR780E_STAT_CGACT: // 查询PDP状态
            {
                drv_air780_send_at("CGACT?"); // 查询网络连接信息
                wait_time = 3;
                break;
            }
            case AIR780E_STAT_CGATT0: // 查询附着状态
            {
                drv_air780_send_at("CGATT?"); // 查询网络连接信息
                wait_time = 3;
                break;
            }
            case AIR780E_STAT_CGATT1: // 已附着
            {
                //printf("已附着\n");
                drv_air780_send_at("CIPMUX=1"); // 多路连接
                vTaskDelay(30);
                drv_air780_send_at("CIPQSEND=0"); // 快发模式
                vTaskDelay(30);
                drv_air780_send_at("CSTT"); // 设置APN      =\"cmnet\",\"\",\"\"
                wait_time = 2;
                g_sAir780Work.state = AIR780E_STAT_CIICR;
                break;
            }
            case AIR780E_STAT_CIICR:
            {
                //printf("激活移动场景获取IP地址\n");
                drv_air780_send_at("CIICR"); ////激活移动场景,获取IP地址
                vTaskDelay(100);

                //printf("1s后查询IP地址\n");
                drv_air780_send_at("CIFSR"); // 查询IP地址
                g_sAir780Work.state = AIR780E_STAT_OK;
                wait_time = 3;
                break;
            }
            case AIR780E_STAT_OK: // 入网成功
            {
                //printf("入网成功\n");
                static uint8_t counts = 0;
                if (counts++ % 2 == 0)
                {
                    if (g_sAir780Work.imei_buff[0] == 0)
                    {
                        drv_air780_send_at("CGSN");
                    }
                    else if (g_sAir780Work.iccid_buff[0] == 0)
                    {
                        drv_air780_send_at("ICCID");
                    }
                    else
                    {
                        drv_air780_send_at("CSQ"); // 查询信号强度
                    }
                }
                else
                {
                    drv_air780_connect_process();
                }
                wait_time = 3;
                break;
            }
        }
        last_sec_time = drv_get_sec_counter_replacement();
    }
}

void drv_air780_AT_test(void)
{
    printf("制造商查询:\n");
    drv_air780_send_at("CGMI"); // 查询制造商
    vTaskDelay(500);

    printf("模块型号:\n");
    drv_air780_send_at("CGMM"); // 查询制造商
    vTaskDelay(500);    

    printf("模块版本信息查询:\n");
    drv_air780_send_at("CGMR"); // 查询制造商
    vTaskDelay(500);

    printf("查询IMEI:\n");
    drv_air780_send_at("CGSN"); // 查询制造商
    vTaskDelay(500);

    printf("查询SIM卡ICCID号码:\n");
    drv_air780_send_at("ICCID"); // 查询制造商
    vTaskDelay(500);

    printf("查询IMSI(国际移动台用户识别码):\n");
    drv_air780_send_at("CIMI"); // 查询制造商
    vTaskDelay(500);

    printf("产品信息:\n");
    drv_air780_send_at("ATI"); // 查询制造商
    vTaskDelay(500);

    printf("模块FIRMWARE版本:\n");
    drv_air780_send_at("VER"); // 查询制造商
    vTaskDelay(500);

}


void air780e_task(void *pvParameters)
{
    drv_air780_init();

    //drv_air780_AT_test();   

    while(1)
    {
        //printf("Hello world!\n"); 
        drv_air780_reg_process();
        //printf("%s任务栈使用率:%d\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}


void send_server_data(void *pvParameters)
{
    uint8_t send_data[1024] = {0};
    do{
        vTaskDelay(100);
    }while(drv_get_4g_work_status() != AIR780E_STAT_OK);

    do 
    {
        drv_air780_client_connect(0,"TCP","8.134.180.47",5000);
        vTaskDelay(1000);
    }while(drv_get_4g_link_status() != AIR780E_LINK_STAE_OK);

    //vTaskDelay(2000);
    printf("开始发送数据给服务器\n");
    while(1)
    {
        memset(send_data,0,sizeof(send_data));
        //sprintf((char *)send_data,"x_pos:%.6lf y_pos:%.6lf,z_pos:%.6lf",x_h_last[0],x_h_last[1],x_h_last[2]);
        drv_air780_send_data(0,send_data,strlen((char *)send_data));
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

