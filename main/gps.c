/**
 * @file gps.c
 * @brief gps定位模块
 * @author kdm (19397266@qq.com)
 * @version 1.0
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024  
 * 
 * @par 修改日志:
 */
#include "gps.h"


#define BUFFER_SIZE 800
gps_info_t gps_data={0};

// 将度分格式转换为小数度格式
double convert_to_decimal(double degree_minute,char dir) {
    int degrees = (int)(degree_minute / 100);   
    double minutes = degree_minute - degrees * 100;
    if(dir == 'S' || dir == 'W')
        return -(degrees + (minutes / 60));
    return degrees + (minutes / 60);  
}

void convert_to_standard_time(char* nmea_time,char *nmea_date) {
    int hours, minutes;
    float seconds;
    int year;
    int month;
    int day;
    // 从字符串中解析小时、分钟和秒
    sscanf(nmea_time, "%2d%2d%f", &hours, &minutes, &seconds);
    sscanf(nmea_date, "%2d%2d%2d", &day, &month, &year);
    hours += 8;
    if (hours >= 24) {
        hours -= 24;  // 调整小时

        // 简单日期加1逻辑（不考虑月末或闰年等情况）
        day += 1;

        // 如果是月底需要月份增加
        if ((day > 30 && (month == 4 || month == 6 || month == 9 || month == 11)) ||
            (day > 31 && (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12))) {
            day = 1;
            month += 1;

            // 如果月份为12月底，年份增加
            if (month > 12) {
                month = 1;
                year += 1;
            }
        }
    }

    sprintf(nmea_time,"%2d:%2d:%2d",hours, minutes, (int)seconds);
    sprintf(nmea_date,"20%2d-%2d-%2d",year, month,day );
    //printf("Standard Time: %02d:%02d:%06.3f\n", hours, minutes, seconds);
}


void parse_gpgsa(const char *line, gps_info_t *gps) {
    char *token;
    int index = 0;

    token = strtok(line, ",");
    while (token != NULL) {
        if (index == 1) {
            gps->mode = *token;
        } else if (index == 2) {
            gps->fix_type = atoi(token);
        } else if (index >= 3 && index <= 14) {
            if (*token != '\0') {
                gps->sat_used[index - 3] = atoi(token);
            } else {
                gps->sat_used[index - 3] = 0; // 没有卫星编号
            }
        } else if (index == 15) {
            gps->pdop = atof(token);
        } else if (index == 16) {
            gps->hdop_gsa = atof(token);
        } else if (index == 17) {
            gps->vdop = atof(token);
        }
        token = strtok(NULL, ",");
        index++;
    }
}


void parse_gps_data(const char *nmea_data, gps_info_t *gps) {
    char buffer[BUFFER_SIZE];
    strncpy(buffer, nmea_data, BUFFER_SIZE); // 复制数据
    char *line = strtok(buffer, "\n");  // 分解每一行

    while (line != NULL) {
        if (strncmp(line, "$GNGGA", 6) == 0) {
            sscanf(line, "$GNGGA,%10[^,],%lf,%c,%lf,%c,%d,%d,%f,%f",
                gps->time_gga, &gps->latitude, &gps->lat_dir,
                &gps->longitude, &gps->lon_dir,
                &gps->fix_quality, &gps->num_satellites, &gps->hdop, &gps->altitude);
         

            // 将经纬度转换为小数度
            gps->latitude = convert_to_decimal(gps->latitude,gps->lat_dir);
            gps->longitude = convert_to_decimal(gps->longitude,gps->lon_dir);

        } else if (strncmp(line, "$GNRMC", 6) == 0) {
            sscanf(line, "$GNRMC,%10[^,],A,%lf,%c,%lf,%c,%lf,%lf,%6[^,]",
                gps->time_rmc, &gps->latitude, &gps->lat_dir, 
                &gps->longitude, &gps->lon_dir, &gps->speed, &gps->course, gps->date);


            convert_to_standard_time(gps->time_rmc,gps->date);

            // 将经纬度转换为小数度
            gps->latitude = convert_to_decimal(gps->latitude,gps->lat_dir);
            gps->longitude = convert_to_decimal(gps->longitude,gps->lon_dir);

        } else if (strncmp(line, "$GPGSA", 6) == 0 || strncmp(line, "$BDGSA", 6) == 0) {
            sscanf(line, "$GPGSA,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f",
                &gps->mode, &gps->fix_type, &gps->sat_used[0], &gps->sat_used[1],
                &gps->sat_used[2], &gps->sat_used[3], &gps->sat_used[4],
                &gps->sat_used[5], &gps->sat_used[6], &gps->sat_used[7],
                &gps->sat_used[8], &gps->sat_used[9], &gps->sat_used[10],
                &gps->pdop, &gps->hdop_gsa, &gps->vdop);
                // parse_gpgsa(line, gps);  

        } else if (strncmp(line, "$GPGSV", 6) == 0 || strncmp(line, "$BDGSV", 6) == 0) {
            int num_sentences, sentence_num, satellites_in_view;
            sscanf(line, "$GPGSV,%d,%d,%d", &num_sentences, &sentence_num, &satellites_in_view);
            gps->sat_in_view = satellites_in_view;

            // 提取卫星信息
            int sat_id, elevation, azimuth, snr;
            for (int i = 0; i < 4; i++) {
                sscanf(line, "$GPGSV,%*d,%*d,%*d,%d,%d,%d,%d", &sat_id, &elevation, &azimuth, &snr);
                gps->sat_info[sentence_num*4 + i][0] = sat_id;
                gps->sat_info[sentence_num*4 + i][1] = elevation;
                gps->sat_info[sentence_num*4 + i][2] = azimuth;
                gps->sat_info[sentence_num*4 + i][3] = snr;
            }

        } else if (strncmp(line, "$GPTXT", 6) == 0) {
            sscanf(line, "$GPTXT,%*d,%*d,%*d,%19s", gps->antenna_status);
        }

        line = strtok(NULL, "\n");  // 处理下一行
    }
}












/**
 * @brief AT指令发送函数
 * @param  at_data          
 */
esp_err_t gps_send_cmd(uint8_t *at_data)
{
    return uart2_write_data(at_data, strlen((char *)at_data));
}


void gps_pwr_io_init(void)
{
    //AT_EN
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1<<GPIO_NUM_8),
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    gpio_set_level(GPIO_NUM_8, 0);

    //GPS_EN
    led_cfg.pin_bit_mask = (1<<GPIO_NUM_3);
    gpio_config(&led_cfg);

    gpio_set_level(GPIO_NUM_3, 0);
}


void gps_task(void *pvParameters)
{
    int read_size;
    
    uint8_t imu_data[BUFFER_SIZE]={0};

    //1、使能GPS模块
    gps_pwr_io_init();

    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    while (1)
    {
        xSemaphoreTake(uart2_mux, portMAX_DELAY);  // 获取 UART2 互斥锁
        inertia_gps_comm_uart_init(9600);
        UART2_TO_AT;  // 选择 GPS 模块
        //vTaskDelay(pdMS_TO_TICKS(10)); 
        uart_flush(UART2_PORT);
        memset(imu_data,0,sizeof(imu_data));
        read_size = uart_read_bytes(UART2_PORT, imu_data, sizeof(imu_data), pdMS_TO_TICKS(200));
        xSemaphoreGive(uart2_mux);  // 释放 UART2 互斥锁

        parse_gps_data((char *)imu_data,&gps_data);
#if 0
        printf("GPS read %d bytes from UART2:\n", read_size);
        for(int i=0;i<read_size;i++)
        {
            printf("%c",imu_data[i]);
        }
        printf("\n\n");
        
        // printf("GGA Time: %s, Latitude: %.6f %c, Longitude: %.6f %c\n",
        // gps_data.time_gga, gps_data.latitude, gps_data.lat_dir,
        // gps_data.longitude, gps_data.lon_dir);
        printf("RMC时间: %s, 速度: %.2f节, 航向: %.2f\n",
            gps_data.time_rmc , gps_data.speed, gps_data.course);
        printf("经度:%.6f %c,纬度:%.6f %c\n", gps_data.longitude,gps_data.lon_dir,gps_data.latitude,gps_data.lat_dir);
        printf("可见卫星数:%d,PDOP:%.2f,HDOP:%.2f,VDOP:%.2f\n", gps_data.sat_in_view,gps_data.pdop,gps_data.hdop,gps_data.vdop);
        for(int i=0;i<gps_data.sat_in_view;i++)
        {
            printf("%d ",gps_data.sat_used[i]);
        }
        
        if(gps_data.fix_quality == 1)
        {
            //数据有效
            if(gps_data.sat_in_view>9)
            {
                printf("大概率在室外 %.3f\n",gps_data.hdop );

                if(gps_data.hdop < 3.0)
                {
                    printf("就在室外\n");
                }
                else 
                {
                    printf("可能在建筑边沿\n");
                }
            }
            else 
            {
                printf("大概率在室内\n");
                if(gps_data.hdop < 4.0)
                {
                    printf("可能在建筑边沿\n");
                }
                else 
                {
                    printf("就在室内\n");
                }
            }
        }
        else if(gps_data.fix_quality != 1)
        {
            //数据无效
            printf("无效定位数据，可能在室内\n");
        }
        printf("\n\n");
#endif
        vTaskDelay(pdMS_TO_TICKS(3000));  // 每3秒读取一次
    }
}

