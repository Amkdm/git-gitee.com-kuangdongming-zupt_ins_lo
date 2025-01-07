#ifndef ZERO_VELOCITY_DETECTOR_H
#define ZERO_VELOCITY_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "nav_val_define.h"


// ��������
precision get_cur_g(void);//��ȡ��ǰ�������ٶ�
void zero_velocity_detector(const precision** u, uint32_t window_size, bool* zupt, precision* T);
void GLRT(const precision** u, uint32_t window_size, precision* T);
void MV(const precision** u, uint32_t window_size, precision* T);
void MAG(const precision** u, uint32_t window_size, precision* T);
void ARE(const precision** u, uint32_t window_size, precision* T);
void initialize_detector(precision latitude, precision altitude); //��ʼ�����ټ����
precision calculate_gravity(precision latitude, precision altitude); //�����������ٶ�

#endif // ZERO_VELOCITY_DETECTOR_H
