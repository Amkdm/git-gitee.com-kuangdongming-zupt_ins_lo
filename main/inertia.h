#ifndef __INERTIA_H__
#define __INERTIA_H__

#include <stdio.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "inertia_gps_comm.h"
#include "esp_log.h"
#include "Navigation_solution.h"
#include "../../components/zupt_ins_mode/src/zero_velocity_detector.h"

//IMU回传频率
typedef enum
{
    SAMPLE_RATE_1HZ=3,//按照手册填写  
    SAMPLE_RATE_2HZ=4,
    SAMPLE_RATE_5HZ=5,
    SAMPLE_RATE_10HZ=6,
    SAMPLE_RATE_20HZ=7,
    SAMPLE_RATE_50HZ=8,
    SAMPLE_RATE_100HZ=9,
    SAMPLE_RATE_200HZ=11, 
}imu_sample_rate_et;

void imu_task(void *pvParameters);

#endif /* __INERTIA_H__ */


