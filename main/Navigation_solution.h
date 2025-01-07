#ifndef __NAVIGATION_SOLUTION_H__
#define __NAVIGATION_SOLUTION_H__

#include <stdio.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "../../components/zupt_ins_mode/src/zero_velocity_detector.h"// #include "zero_velocity_detector.h"
#include "../../components/zupt_ins_mode/src/ZUPTaidedINS.h"// #include "ZUPTaidedINS.h"

#define QUEUE_SIZE 100 // 队列大小

extern precision** u_data;
extern precision** Angle_data; 
extern int can_solution_flag;

void Navigation_solution_set_total_cols(int cur_cols);
int Navigation_solution_get_total_cols(void);
void navigation_solution_task(void *pvParameters);

#endif // __NAVIGATION_SOLUTION_H__

