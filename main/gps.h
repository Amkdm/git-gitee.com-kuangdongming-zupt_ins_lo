#ifndef __GPS_H__
#define __GPS_H__

#include <stdio.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "inertia_gps_comm.h"

/**
 * @brief 存放从GPS模块获取到数据的提取信息
 */
typedef struct {
    // GGA数据
    char time_gga[11];      // 时间 (HHMMSS.sss)
    double latitude;        // 纬度
    char lat_dir;           // 纬度方向 (N/S)
    double longitude;       // 经度
    char lon_dir;           // 经度方向 (E/W)
    int fix_quality;        // 定位状态
    int num_satellites;     // 卫星数量
    float hdop;             // 水平精度
    float altitude;         // 海拔高度
    
    // RMC数据
    char time_rmc[16];      // 时间 (HHMMSS.sss)
    char date[13];           // 日期 (DDMMYY)
    double speed;           // 速度 (节)
    double course;          // 航向

    // GSA数据
    char mode;              // 模式 (A/M)
    int fix_type;           // 定位类型 (2D/3D)
    int sat_used[12];       // 使用的卫星编号
    int sat_used_num;       // 使用的卫星数量
    float pdop;             // PDOP
    float hdop_gsa;         // HDOP
    float vdop;             // VDOP

    // GSV数据
    int sat_in_view;        // 可见卫星数量
    int sat_info[12][4];    // 卫星信息：编号, 仰角, 方位角, 信号强度

    // 其他数据
    char antenna_status[20];  // 天线状态信息
} gps_info_t;

extern gps_info_t gps_data; // 存放GPS模块获取到的数据

void gps_task(void *pvParameters);

#endif /* __INERTIA_H__ */

