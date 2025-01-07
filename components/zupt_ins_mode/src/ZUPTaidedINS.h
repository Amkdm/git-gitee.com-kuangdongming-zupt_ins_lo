#ifndef ZUPT_AIDED_INS_H
#define ZUPT_AIDED_INS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "nav_val_define.h"

//函数声明
//初始化卡尔曼滤波器的函数。给定了初始状态协方差阵P，初始过程噪声协方差阵Q，初始测量噪声协方差阵R和观测矩阵H。
void init_filter(precision P[9][9], precision Q[6][6], precision R[3][3], precision H[3][9]); 

//初始化输出数据长度的函数
void init_vec(uint32_t N, precision P[9][9], precision*** x_h, precision*** cov, precision Id[9][9]);

// 释放内存的函数
void free_memory(precision** x_h, precision** cov, precision* zupt, precision* T, int cols);

//计算导航解算初始状态的函数，它对导航系统进行了一个简单的初始对准，其中系统的横滚角和俯仰角是根据前20次采样的加速度计读数推算得到的。
void init_Nav_eq(const precision** u, uint32_t sample_count, precision* x, precision quat[4], const precision** Angle);

//惯性导航系统的机械化导航方程解算，忽略了高阶项。
//void Navigation_equations(const precision* x, const precision** u, precision* q, precision* y);
void Navigation_equations(const precision* x, const precision** u, precision* q, precision* y, const precision yaw);

//通过当前方向和比力向量计算状态转移矩阵F和过程噪声增益矩阵G的函数
void state_matrix(const precision quat[4], const precision** u, precision** F, precision** G);

//用卡尔曼滤波估计的系统误差来修正估计的导航状态的函数。
//void comp_internal_states(precision* x_h, const precision* dx, precision* q_in);
void comp_internal_states(precision* x_h, const precision* dx, precision* q_in, const precision yaw);

// 功能函数声明
//给定欧拉角，输出从t系旋转到b系所需的旋转矩阵
void Rt2b(const precision ang[3], precision R[3][3]);

//将旋转矩阵转化为四元数向量的函数
void dcm2q(const precision R[3][3], precision q[4]);

//将四元数向量转化为旋转矩阵的函数
void q2dcm(const precision q[4], precision** R);

//计算反正切的函数
precision custom_atan2(precision y, precision x);

//更新卡尔曼滤波P矩阵并赋值给cov
void updateMatrixP(precision** F, precision** G, precision P[9][9], precision Q[6][6], precision** cov, int current_cols);

//计算卡尔曼滤波增益矩阵K
void calculate_kalman_gain(precision H[3][9], precision P[9][9], precision R[3][3], precision** K);

//更新ZUPT生效时的卡尔曼滤波矩阵P并赋值给cov
void update_state_covariance(precision** K, precision H[3][9], precision P[9][9], precision I[9][9], precision** cov, int current_cols);

#endif // ZUPT_AIDED_INS_H
