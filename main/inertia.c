/**
 * @file inertia.c
 * @brief 惯性器件驱动文件
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */
#include "inertia.h"

static const char *IMU_TAG = "imu";

uint16_t imu_read_reg(uint8_t reg)
{
    esp_err_t ret = -1;
    int read_temp_cnt;
    uint8_t imu_data[128] = {0};
    int read_size = 0;
    //协议格式： FF AA 27 xx 00
    uint8_t imu_cmd_set_rate[5] = {0xFF,0xAA,0x27,0x00,0x00};
    imu_cmd_set_rate[3] = reg;
    UART2_TO_WT; //选择 IMU惯性器件模块
    uart_flush(UART2_PORT);
    ret = uart2_write_data(imu_cmd_set_rate,5); //发送指令
    if(ret == ESP_OK)
    {
        //ESP_LOGI(IMU_TAG,"IMU read temp success\n");
    }
    else    
    {
        ESP_LOGE(IMU_TAG,"IMU read temp failed\n");
        return 0;
    }

    //读取串口
    read_temp_cnt = 100;
    while(read_temp_cnt--)
    {
        //读取数据
        memset(imu_data,0,sizeof(imu_data));
        read_size = uart_read_bytes(UART2_PORT, imu_data, sizeof(imu_data), pdMS_TO_TICKS(10));
        if(read_size == -1)
        {
            ESP_LOGE(IMU_TAG,"Failed to read from UART2\n");
            
            //vTaskDelay(10 / portTICK_PERIOD_MS);  // 每秒读取一次
            return 0;;
        }
        else if(imu_data[0] != 0x55 && imu_data[1] != 0x71)
        {
            continue;
        }
        else if(imu_data[0] == 0x55 && imu_data[1] == 0x71)
        {
            ESP_LOGI(IMU_TAG,"IMU read reg success");
            //ESP_LOGI(IMU_TAG,"IMU Read Temp %d bytes from UART2:\n\t", read_size);
            for(int i=0;i<read_size;i++)
            {
                printf("0x%02x ",imu_data[i]);
            }
            printf("\n\n");

           return ((imu_data[5]<<8) | imu_data[4]);
        }

    }   
    return 0;
}


/**
 * @brief 
 * @param  rate  1HZ  2HZ  5HZ 10HZ 20HZ 50HZ 100HZ 200HZ        
 */

/**
 * @brief 设置IMU模块的回传速率
 * @param  rate 1HZ  2HZ  5HZ 10HZ 20HZ 50HZ 100HZ 200HZ         
 * @return esp_err_t 
 */
esp_err_t imu_set_rate(imu_sample_rate_et rate)
{
    esp_err_t ret = -1;
    //协议格式： FF AA 03 RATE 00
    uint8_t imu_cmd_set_rate[5] = {0xFF,0xAA,0x04,0x00,0x00};
    imu_cmd_set_rate[3] = rate;
    ret = uart2_write_data(imu_cmd_set_rate,5); //发送指令
    if(ret == ESP_OK)
    {
        //ESP_LOGI(IMU_TAG,"IMU set rate success\n");
    }
    else    
    {
        ESP_LOGE(IMU_TAG,"IMU set rate failed\n");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));

    // 保存配置命令，FF AA 00 SAVE 00
    uint8_t save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    ret = uart2_write_data(save_cmd,5);
    if(ret == ESP_OK)
    {
        //ESP_LOGI(IMU_TAG,"IMU save set success\n");
    }
    else    
    {
        ESP_LOGE(IMU_TAG,"IMU save set failed\n");
    }
    return ret;
}

esp_err_t imu_read_tempture(void)
{
    esp_err_t ret = -1;
    //协议格式： FF AA 27 40 00
    uint8_t imu_cmd_read_temp[5] = {0xFF,0xAA,0x27,0x04,0x00};
    ret = uart2_write_data(imu_cmd_read_temp,5); //发送指令
    if(ret == ESP_OK)
    {
        //ESP_LOGI(IMU_TAG,"IMU read temp success\n");
    }
    else    
    {
        ESP_LOGE(IMU_TAG,"IMU read temp failed\n");
    }
    return ret;
}


void imu_pwr_io_init(void)
{
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1<<GPIO_NUM_5),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    gpio_set_level(GPIO_NUM_5, 0);//默认关闭
}


// 定义加速度、角速度和角度数据的结构体
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float angle_x;
    float angle_y;
    float angle_z;
} ImuData;

// 辅助函数，将2字节的原始数据转换为有符号16位整数
static int16_t bytes_to_int16(uint8_t low_byte, uint8_t high_byte) {
    return (int16_t)(high_byte << 8 | low_byte);
}

// 解析IMU数据（20字节），存入结构体
void parse_imu_data(const uint8_t* data, ImuData* imu) {
    // 按照协议从字节数据解析出加速度、角速度和角度
    imu->accel_x = bytes_to_int16(data[2], data[3]) / 32768.0f * 16.0f;  // 单位 g
    imu->accel_y = bytes_to_int16(data[4], data[5]) / 32768.0f * 16.0f;
    imu->accel_z = bytes_to_int16(data[6], data[7]) / 32768.0f * 16.0f;

    imu->gyro_x = bytes_to_int16(data[8], data[9]) / 32768.0f * 2000.0f;  // 单位 deg/s
    imu->gyro_y = bytes_to_int16(data[10], data[11]) / 32768.0f * 2000.0f;
    imu->gyro_z = bytes_to_int16(data[12], data[13]) / 32768.0f * 2000.0f;

    imu->angle_x = bytes_to_int16(data[14], data[15]) / 32768.0f * 180.0f;  // 单位 deg
    imu->angle_y = bytes_to_int16(data[16], data[17]) / 32768.0f * 180.0f;
    imu->angle_z = bytes_to_int16(data[18], data[19]) / 32768.0f * 180.0f;
}


#define DEG_TO_RAD (M_PI / 180.0) // 角度转换为弧度的常数
#define QUEUE_SIZE 100 // 队列大小


void imu_task(void *pvParameters)
{
    ImuData imu_data_struct;
    int read_size;
    uint32_t read_cnt = 0;
    uint8_t imu_data[50] = {0};

    //开启IMU模块电源
    imu_pwr_io_init();

    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1)
    {
        xSemaphoreTake(uart2_mux, portMAX_DELAY);  // 获取 UART2 互斥锁

        inertia_gps_comm_uart_init(115200);
        UART2_TO_WT; //选择 IMU惯性器件模块
        //vTaskDelay(pdMS_TO_TICKS(100));
        //uart_flush(UART2_PORT);
        memset(imu_data,0,sizeof(imu_data));
        read_size = uart_read_bytes(UART2_PORT, imu_data, 30, pdMS_TO_TICKS(10));

        xSemaphoreGive(uart2_mux);  // 释放 UART2 互斥锁
        //printf("read size:%d\n",read_size);
        if(read_size == -1)
        {
            ESP_LOGE(IMU_TAG,"Failed to read from UART2\n");
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        else if(read_size == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        else if(imu_data[0] != 0x55 || read_size < 20)
        {
            //vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        else 
        {
            //这里读到了一组20Byte的数据，需要调用接口解析
            parse_imu_data(imu_data, &imu_data_struct);

            // 上一轮刚填满 QUEUE_SIZE个数据即read_cnt从0~99下表都填充了一遍
            if(read_cnt == QUEUE_SIZE)
            {
                // 表示上一轮填队列满了，这回先所有数据左移一列
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < QUEUE_SIZE-1; j++) {
                        u_data[i][j] = u_data[i][j + 1]; // 0<-1  1<-2  2<-3  3<-4  4<-5  5<-6...... 98<-99  99空出来
                    }
                }
                for (int j = 0; j < QUEUE_SIZE-1; j++) {
                    Angle_data[0][j] = Angle_data[0][j + 1];
                }

                //调试打印
                // printf("\n\n"); 
                // for (int i = 0; i < ROWS; i++) {
                //     for (int j = 0; j < 10; j++) {
                //         printf("%.2f ",u_data[i][j]);
                //     }
                //     printf("\n");
                // }

                read_cnt = QUEUE_SIZE-1; //继续99
            }

            u_data[0][read_cnt] = imu_data_struct.accel_x;//x轴加速度 
            u_data[1][read_cnt] = imu_data_struct.accel_y;//y轴加速度
            u_data[2][read_cnt] = imu_data_struct.accel_z;//z轴加速度
            u_data[3][read_cnt] = imu_data_struct.gyro_x;//x轴角速度
            u_data[4][read_cnt] = imu_data_struct.gyro_y;//y轴角速度
            u_data[5][read_cnt] = imu_data_struct.gyro_z;//z轴角速度
            Angle_data[0][read_cnt] = imu_data_struct.angle_z;//z轴角度

            // 将新读取的加速度数据缩放到SI单位
            for (int i = 0; i < 3; i++) {
                u_data[i][read_cnt] *= get_cur_g();
            }

            // 将新读取的角速度从°/s转换为rad/s
            for (int i = 3; i < ROWS; i++) {
                u_data[i][read_cnt] *= DEG_TO_RAD;
            }

            // 将新读取的角度从°转换为rad
            Angle_data[0][read_cnt] *= DEG_TO_RAD;

            // 更新总列数 total_cols++;
            Navigation_solution_set_total_cols(Navigation_solution_get_total_cols()+1);
            if(Navigation_solution_get_total_cols() > QUEUE_SIZE )
            {
                Navigation_solution_set_total_cols(QUEUE_SIZE+1); 
            }

            // 更新当前列索引
            if (read_cnt < QUEUE_SIZE) {
                read_cnt++;
            }
            
            //调试打印
            //printf("read_cnt:%d  total_cols:%d\n",(int)read_cnt,Navigation_solution_get_total_cols());
            
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}



