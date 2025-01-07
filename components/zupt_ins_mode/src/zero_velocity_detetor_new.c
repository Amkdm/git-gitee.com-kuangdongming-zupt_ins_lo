// zero_velocity_detector.c
#include "zero_velocity_detector.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// 定义相关变量，详见 nav_val_define.h
int detector_type = DETECTOR_TYPE_GLRT;
precision sigma_a = 0.01;
precision sigma_g = 0.1 * M_PI / 180;
precision g;
precision gamma1 = 0.3e6;

precision get_cur_g(void)
{  
    return g;
}

void zero_velocity_detector(const precision** u, uint32_t window_size, bool* zupt, precision* T) {

    // 根据检测器类型运行相应的检测器
    switch (detector_type) {
    case DETECTOR_TYPE_GLRT:
        GLRT(u, window_size, T);
        break;
    case DETECTOR_TYPE_MV:
        MV(u, window_size, T);
        break;
    case DETECTOR_TYPE_MAG:
        MAG(u, window_size, T);
        break;
    case DETECTOR_TYPE_ARE:
        ARE(u, window_size, T);
        break;
    default:
        GLRT(u, window_size, T);
        break;
    }

    //// 检查检测统计量 T 是否低于检测器阈值
    //if (*T < gamma1) {
    //    *zupt = true;
    //}
    // 检查检测统计量 T 是否低于检测器阈值
    if (*T < gamma1) {
        for (uint32_t w = 0; w < window_size; w++) {
            *zupt = true;
            zupt += 1;
        }
    }
    else 
    {
        for (uint32_t w = 0; w < window_size; w++) {
            *zupt = false;
            zupt += 1;
        }
    }
}

void GLRT(const precision** u, uint32_t window_size, precision* T) {
    precision sigma2_a = sigma_a * sigma_a;
    precision sigma2_g = sigma_g * sigma_g;
    uint32_t W = window_size;

    *T = 0.0f;

    // 计算ya_m
    precision ya_m[3] = { 0.0f, 0.0f, 0.0f };
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t j = 0; j < W; j++) {
            ya_m[i] += u[i][j];
        }
        ya_m[i] /= W;
    }

    // 计算T
    for (uint32_t l = 0; l < W; l++) {
        precision tmp[3];
        precision norm_ya_m = 0.0f;

        // 计算ya_m的模
        for (uint32_t i = 0; i < 3; i++) {
            norm_ya_m += ya_m[i] * ya_m[i];
        }
        norm_ya_m = sqrtf(norm_ya_m);

        // 计算tmp向量
        for (uint32_t i = 0; i < 3; i++) {
            tmp[i] = u[i][l] - g * ya_m[i] / norm_ya_m;
        }

        // 计算u(4:6,l)'*u(4:6,l)
        precision u_gyro_squared = 0.0f;
        for (uint32_t i = 3; i < 6; i++) {
            u_gyro_squared += u[i][l] * u[i][l];
        }

        // 累加到T
        *T += u_gyro_squared / sigma2_g;

        precision tmp_squared = 0.0f;
        for (uint32_t i = 0; i < 3; i++) {
            tmp_squared += tmp[i] * tmp[i];
        }

        *T += tmp_squared / sigma2_a;
    }

    *T /= W;
}

void MV(const precision** u, uint32_t window_size, precision* T) {
    precision sigma2_a = sigma_a * sigma_a;
    uint32_t W = window_size;

    *T = 0.0f;

    precision ya_m[3] = { 0 };
    for (uint32_t l = 0; l < W; l++) {
        for (int i = 0; i < 3; i++) {
            ya_m[i] += u[l][i];
        }
    }
    for (int i = 0; i < 3; i++) {
        ya_m[i] /= W;
    }

    for (uint32_t l = 0; l < W; l++) {
        precision tmp[3];
        for (int i = 0; i < 3; i++) {
            tmp[i] = u[l][i] - ya_m[i];
        }
        for (int i = 0; i < 3; i++) {
            *T += tmp[i] * tmp[i];
        }
    }
    *T /= (sigma2_a * W);
}

void MAG(const precision** u, uint32_t window_size, precision* T) {
    precision sigma2_a = sigma_a * sigma_a;
    uint32_t W = window_size;

    *T = 0.0f;

    for (uint32_t l = 0; l < W; l++) {
        precision norm_a = sqrt(u[l][0] * u[l][0] + u[l][1] * u[l][1] + u[l][2] * u[l][2]);
        *T += (norm_a - g) * (norm_a - g);
    }
    *T /= (sigma2_a * W);
}

void ARE(const precision** u, uint32_t window_size, precision* T) {
    precision sigma2_g = sigma_g * sigma_g;
    uint32_t W = window_size;

    *T = 0.0f;

    for (uint32_t l = 0; l < W; l++) {
        precision norm_g = sqrt(u[l][3] * u[l][3] + u[l][4] * u[l][4] + u[l][5] * u[l][5]);
        *T += norm_g * norm_g;
    }
    *T /= (sigma2_g * W);
}

// 计算重力加速度
precision calculate_gravity(precision latitude, precision altitude) {
    latitude = M_PI / 180 * latitude;
    precision gamma1 = 9.780327 * (1 + 0.0053024 * sin(latitude) * sin(latitude) - 0.0000058 * sin(2 * latitude) * sin(2 * latitude));
    return gamma1 - ((3.0877e-6) - (0.004e-6) * sin(latitude) * sin(latitude)) * altitude + (0.072e-12) * altitude * altitude;
}

// 在初始化时调用 calculate_gravity 函数
void initialize_detector(precision latitude, precision altitude) {
    g = calculate_gravity(latitude, altitude);
}
