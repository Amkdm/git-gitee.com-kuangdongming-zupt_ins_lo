#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> 
#include "ZUPTaidedINS.h"

// 定义全局变量
precision sigma_acc[3] = { 0.5 * 1, 0.5 * 1, 0.5 * 1 };
precision sigma_gyro[3] = { 0.5 * (1 * (M_PI / 180)), 0.5 * (1 * (M_PI / 180)), 0.5 * (1 * (M_PI / 180)) };
precision sigma_vel[3] = { 0.01, 0.01, 0.01 };

void init_vec(uint32_t N, precision P[9][9], precision*** x_h, precision*** cov, precision Id[9][9]) {
    // 分配内存
    *x_h = (precision**)malloc(N * sizeof(precision*));
    *cov = (precision**)malloc(N * sizeof(precision*));
    if (*x_h == NULL || *cov == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }

    for (uint32_t i = 0; i < N; i++) {
        (*x_h)[i] = (precision*)malloc(9 * sizeof(precision));
        (*cov)[i] = (precision*)malloc(9 * sizeof(precision));
        if ((*x_h)[i] == NULL || (*cov)[i] == NULL) {
            fprintf(stderr, "Memory allocation failed\n");
            exit(EXIT_FAILURE);
        }
    }

    // 初始化 x_h 和 cov
    for (uint32_t i = 0; i < N; i++) {
        for (int j = 0; j < 9; j++) {
            (*x_h)[i][j] = (precision)0.0;
            (*cov)[i][j] = (precision)0.0;
        }
    }

    // 初始化单位矩阵 Id
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            Id[i][j] = (i == j) ? (precision)1.0 : (precision)0.0;
        }
    }

    // 将 P 的对角元素复制到 cov 的第一列
    for (int i = 0; i < 9; i++) {
        (*cov)[0][i] = P[i][i];
    }
}

void free_memory(precision** x_h, precision** cov, precision* zupt, precision* T, int cols) {
    // 释放 x_h 的内存
    for (int i = 0; i < cols; i++) {
        free(x_h[i]); // 释放 x_h 的每一行
    }
    free(x_h); // 释放 x_h 本身

    // 释放 cov 的内存
    for (int i = 0; i < cols; i++) {
        free(cov[i]); // 释放 cov 的每一行
    }
    free(cov); // 释放 cov 本身

    // 释放 zupt 和 T 的内存
    free(zupt);
    free(T);
}

void init_filter(precision P[9][9], precision Q[6][6], precision R[3][3], precision H[3][9]) {
    // 初始化 P 矩阵
    memset(P, 0, 9 * 9 * sizeof(precision));
    P[0][0] = SIGMA2_INIT_POSX;
    P[1][1] = SIGMA2_INIT_POSY;
    P[2][2] = SIGMA2_INIT_POSZ;
    P[3][3] = SIGMA2_INIT_VELX;
    P[4][4] = SIGMA2_INIT_VELY;
    P[5][5] = SIGMA2_INIT_VELZ;
    P[6][6] = SIGMA2_INIT_ROLL;
    P[7][7] = SIGMA2_INIT_PITCH;
    P[8][8] = SIGMA2_INIT_YAW;

    // 初始化 Q 矩阵
    memset(Q, 0, 6 * 6 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        Q[i][i] = sigma_acc[i] * sigma_acc[i];
        Q[i + 3][i + 3] = sigma_gyro[i] * sigma_gyro[i];
    }

    // 初始化 R 矩阵
    memset(R, 0, 3 * 3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        R[i][i] = sigma_vel[i] * sigma_vel[i];
    }

    // 初始化 H 矩阵
    memset(H, 0, 3 * 9 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        H[i][i + 3] = 1.0;
    }
}


void init_Nav_eq(const precision** u, uint32_t sample_count, precision* x, precision quat[4], const precision** Angle) {
    // 假设系统在前 sample_count 个样本期间是静止的，从前 sample_count 个加速度计读数中计算初始横滚和俯仰角
    precision f_u = 0.0, f_v = 0.0, f_w = 0.0;
    for (uint32_t i = 0; i < sample_count; i++) {
        f_u += u[0][i];
        f_v += u[1][i];
        f_w += u[2][i];
    }
    f_u /= sample_count;
    f_v /= sample_count;
    f_w /= sample_count;

    precision roll = atan2(-f_v, -f_w);
    precision sqrt_result = sqrt(f_v * f_v + f_w * f_w);
    precision pitch = custom_atan2(f_u, sqrt_result);

    // 设置姿态向量
    precision attitude[3] = { roll, pitch, Angle[0][0]};

    // 计算与初始姿态对应的四元数
    precision Rb2t[3][3];
    Rt2b(attitude, Rb2t);
    dcm2q(Rb2t, quat);

    // 设置初始状态向量
    for (int i = 0; i < 9; i++) {
        x[i] = 0.0;
    }

    x[0] = INITIAL_POSX;
    x[1] = INITIAL_POSY;
    x[2] = INITIAL_POSZ;
    x[6] = roll;
    x[7] = pitch;
    x[8] = Angle[0][0];
}


/**
 * @brief 计算从坐标系 t 旋转到坐标系 b 的旋转矩阵，给定欧拉角。
 *
 * @param[in]  ang  欧拉角 [roll, pitch, heading]
 * @param[out] R    旋转矩阵
 */
void Rt2b(const precision ang[3], precision R[3][3]) {
    // 计算滚转角的余弦和正弦
    precision cr = cos(ang[0]);
    precision sr = sin(ang[0]);

    // 计算俯仰角的余弦和正弦
    precision cp = cos(ang[1]);
    precision sp = sin(ang[1]);

    // 计算航向角的余弦和正弦
    precision cy = cos(ang[2]);
    precision sy = sin(ang[2]);

    // 填充旋转矩阵 R
    R[0][0] = cy * cp;
    R[0][1] = sy * cp;
    R[0][2] = -sp;

    R[1][0] = -sy * cr + cy * sp * sr;
    R[1][1] = cy * cr + sy * sp * sr;
    R[1][2] = cp * sr;

    R[2][0] = sy * sr + cy * sp * cr;
    R[2][1] = -cy * sr + sy * sp * cr;
    R[2][2] = cp * cr;

    // 动态分配内存来存储转置矩阵
    precision* R_transposed = (precision*)malloc(3 * 3 * sizeof(precision));
    if (R_transposed == NULL) {
        // 处理内存分配失败的情况
        return;
    }

    // 转置矩阵 R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_transposed[j * 3 + i] = R[i][j];
        }
    }

    // 将转置后的矩阵复制回 R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = R_transposed[i * 3 + j];
        }
    }

    // 释放动态分配的内存
    free(R_transposed);

}

/**
 * @brief 将方向余弦矩阵（旋转矩阵）转换为四元数向量。
 *
 * @param[in]  R  旋转矩阵的指针
 * @param[out] q  四元数向量的指针
 */
void dcm2q(const precision R[3][3], precision q[4]) {
    precision T = 1 + R[0][0] + R[1][1] + R[2][2];

    if (T > 1e-8) {
        precision S = 0.5 / sqrt(T);
        q[3] = 0.25 / S; // qw
        q[0] = (R[2][1] - R[1][2]) * S; // qx
        q[1] = (R[0][2] - R[2][0]) * S; // qy
        q[2] = (R[1][0] - R[0][1]) * S; // qz
    }
    else {
        if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            precision S = sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; // S=4*qx
            q[3] = (R[2][1] - R[1][2]) / S; // qw
            q[0] = 0.25 * S; // qx
            q[1] = (R[0][1] + R[1][0]) / S; // qy
            q[2] = (R[0][2] + R[2][0]) / S; // qz
        }
        else if (R[1][1] > R[2][2]) {
            precision S = sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; // S=4*qy
            q[3] = (R[0][2] - R[2][0]) / S; // qw
            q[0] = (R[0][1] + R[1][0]) / S; // qx
            q[1] = 0.25 * S; // qy
            q[2] = (R[1][2] + R[2][1]) / S; // qz
        }
        else {
            precision S = sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; // S=4*qz
            q[3] = (R[1][0] - R[0][1]) / S; // qw
            q[0] = (R[0][2] + R[2][0]) / S; // qx
            q[1] = (R[1][2] + R[2][1]) / S; // qy
            q[2] = 0.25 * S; // qz
        }
    }
}

void Navigation_equations(const precision* x, const precision** u, precision* q, precision* y, const precision yaw) {
    // 分配内存给输出向量
    memset(y, 0, 9 * sizeof(precision));

    // 获取系统的采样周期
    precision Ts = SampleTimeFrequency;

    // 更新四元数向量 "q" 给定的角速度测量值
    precision* w_tb = (precision*)malloc(3 * sizeof(precision));
    w_tb[0] = u[3][0];
    w_tb[1] = u[4][0];
    w_tb[2] = u[5][0];

    precision P = w_tb[0] * Ts;
    precision Q = w_tb[1] * Ts;
    precision R = w_tb[2] * Ts;

    precision** OMEGA = (precision**)malloc(4 * sizeof(precision*));
    for (int i = 0; i < 4; i++) {
        OMEGA[i] = (precision*)malloc(4 * sizeof(precision));
    }

    OMEGA[0][0] = 0.5 * 0;
    OMEGA[0][1] = 0.5 * R;
    OMEGA[0][2] = -0.5 * Q;
    OMEGA[0][3] = 0.5 * P;
    OMEGA[1][0] = -0.5 * R;
    OMEGA[1][1] = 0.5 * 0;
    OMEGA[1][2] = 0.5 * P;
    OMEGA[1][3] = 0.5 * Q;
    OMEGA[2][0] = 0.5 * Q;
    OMEGA[2][1] = -0.5 * P;
    OMEGA[2][2] = 0.5 * 0;
    OMEGA[2][3] = 0.5 * R;
    OMEGA[3][0] = -0.5 * P;
    OMEGA[3][1] = -0.5 * Q;
    OMEGA[3][2] = -0.5 * R;
    OMEGA[3][3] = 0.5 * 0;

    precision v = sqrt(P * P + Q * Q + R * R);

    if (v != 0) {
        precision cos_v2 = cos(v / 2);
        precision sin_v2 = sin(v / 2);
        precision* q_new = (precision*)malloc(4 * sizeof(precision));
        precision** omega_term = (precision**)malloc(4 * sizeof(precision*));

        for (int i = 0; i < 4; i++) {
            omega_term[i] = (precision*)malloc(4 * sizeof(precision));
        }

        // 初始化 new_q, omega_term
        for (int i = 0; i < 4; i++) {
            q_new[i] = 0.0;
            for (int j = 0; j < 4; j++) {
                omega_term[i][j] = (i == j) ? cos_v2 : 0.0 + (2 / v) * sin_v2 * OMEGA[i][j];
            }
        }

        for (int i = 0; i < 4; i++) {
            precision temp = 0.0;
            for (int j = 0; j < 4; j++) {
                temp += omega_term[i][j] * q[j];
            }
            q_new[i] = temp;
        }
        memcpy(q, q_new, 4 * sizeof(precision));
        free(q_new);

        // 归一化四元数
        precision norm_q = 0.0;
        for (int i = 0; i < 4; i++) {
            norm_q += q[i] * q[i];
        }
        norm_q = sqrt(norm_q);

        for (int i = 0; i < 4; i++) {
            q[i] /= norm_q;
        }

        for (int i = 0; i < 4; i++) {
            free(omega_term[i]);
        }
        free(omega_term);
    }



    // 使用更新后的四元数获取导航系统的姿态（欧拉角）
    precision** Rb2t = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        Rb2t[i] = (precision*)malloc(3 * sizeof(precision));
    }

	q2dcm(q, Rb2t);

    // 计算滚转角
    y[6] = atan2(Rb2t[2][1], Rb2t[2][2]);

	// 计算俯仰角
    y[7] = -atan(Rb2t[2][0] / sqrt(1 - Rb2t[2][0] * Rb2t[2][0]));

	// 计算偏航角
    //y[8] = custom_atan2(Rb2t[1][0], Rb2t[0][0]);
    y[8] = yaw;

    // 假设 y 是一个包含 9 个元素的数组
    precision R_tmp[3][3];
    Rt2b(&y[6], R_tmp);

    // 将 DCM 转换为四元数
    dcm2q(R_tmp, q);

    q2dcm(q, Rb2t);

    // 更新位置和速度状态，使用测量的比力和新计算的姿态
    precision g_t[3] = { 0, 0, g };

    // 将比力向量转换到导航坐标系
    precision* f_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        f_t[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            f_t[i] += Rb2t[i][j] * u[j][0];
        }
    }

    // 减去（加上）重力，得到导航坐标系中的加速度
    precision* acc_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        acc_t[i] = f_t[i] + g_t[i];
    }

    // 状态空间模型矩阵
    precision** A = (precision**)malloc(6 * sizeof(precision*));
    for (int i = 0; i < 6; i++) {
        A[i] = (precision*)malloc(6 * sizeof(precision));
    }

    A[0][0] = 1;
    A[0][1] = 0;
    A[0][2] = 0;
    A[0][3] = Ts;
    A[0][4] = 0;
    A[0][5] = 0;
    A[1][0] = 0;
    A[1][1] = 1;
    A[1][2] = 0;
    A[1][3] = 0;
    A[1][4] = Ts;
    A[1][5] = 0;
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;
    A[2][3] = 0;
    A[2][4] = 0;
    A[2][5] = Ts;
    A[3][0] = 0;
    A[3][1] = 0;
    A[3][2] = 0;
    A[3][3] = 1;
    A[3][4] = 0;
    A[3][5] = 0;
    A[4][0] = 0;
    A[4][1] = 0;
    A[4][2] = 0;
    A[4][3] = 0;
    A[4][4] = 1;
    A[4][5] = 0;
    A[5][0] = 0;
    A[5][1] = 0;
    A[5][2] = 0;
    A[5][3] = 0;
    A[5][4] = 0;
    A[5][5] = 1;

    precision** B = (precision**)malloc(6 * sizeof(precision*));
    for (int i = 0; i < 6; i++) {
        B[i] = (precision*)malloc(3 * sizeof(precision));
    }

    B[0][0] = (Ts * Ts) / 2;
    B[0][1] = 0;
    B[0][2] = 0;
    B[1][0] = 0;
    B[1][1] = (Ts * Ts) / 2;
    B[1][2] = 0;
    B[2][0] = 0;
    B[2][1] = 0;
    B[2][2] = (Ts * Ts) / 2;
    B[3][0] = Ts;
    B[3][1] = 0;
    B[3][2] = 0;
    B[4][0] = 0;
    B[4][1] = Ts;
    B[4][2] = 0;
    B[5][0] = 0;
    B[5][1] = 0;
    B[5][2] = Ts;

    // 更新位置和速度估计
    for (int i = 0; i < 6; i++) {
        y[i] = 0;
        for (int j = 0; j < 6; j++) {
            y[i] += A[i][j] * x[j];
        }
        for (int j = 0; j < 3; j++) {
            y[i] += B[i][j] * acc_t[j];
        }
    }
    // 释放动态分配的内存
    free(w_tb);
    for (int i = 0; i < 4; i++) {
        free(OMEGA[i]);
    }
    free(OMEGA);
    for (int i = 0; i < 3; i++) {
        free(Rb2t[i]);
    }
    free(Rb2t);
    free(f_t);
    free(acc_t);
    for (int i = 0; i < 6; i++) {
        free(A[i]);
    }
    free(A);
    for (int i = 0; i < 6; i++) {
        free(B[i]);
    }
    free(B);
}


/**
 * @brief 将四元数向量转换为方向余弦矩阵（旋转矩阵）
 *
 * @param[in] q 四元数向量
 * @param[out] R 方向余弦矩阵（旋转矩阵）
 */
void q2dcm(const precision q[4], precision** R) {
    precision p[6] = { 0 };

    // 计算四元数的平方
    for (int i = 0; i < 4; i++) {
        p[i] = q[i] * q[i];
    }

    // 计算 p[4] = q2^2 + q3^2
    p[4] = p[1] + p[2];

    // 计算 p[5] = 2 / (q0^2 + q3^2 + q2^2)
    if (p[0] + p[3] + p[4] != 0) {
        p[5] = 2 / (p[0] + p[3] + p[4]);
    }
    else {
        p[5] = 0;
    }

    // 计算方向余弦矩阵的对角元素
    R[0][0] = 1 - p[5] * p[4];
    R[1][1] = 1 - p[5] * (p[0] + p[2]);
    R[2][2] = 1 - p[5] * (p[0] + p[1]);

    // 计算方向余弦矩阵的非对角元素
    p[0] = p[5] * q[0];
    p[1] = p[5] * q[1];
    p[4] = p[5] * q[2] * q[3];
    p[5] = p[0] * q[1];

    R[0][1] = p[5] - p[4];
    R[1][0] = p[5] + p[4];

    p[4] = p[1] * q[3];
    p[5] = p[0] * q[2];

    R[0][2] = p[5] + p[4];
    R[2][0] = p[5] - p[4];

    p[4] = p[0] * q[3];
    p[5] = p[1] * q[2];

    R[1][2] = p[5] - p[4];
    R[2][1] = p[5] + p[4];
}


/**
 * @brief 计算状态转移矩阵 F 和过程噪声输入矩阵 G。
 *
 * @param[in]  quat  当前姿态的四元数
 * @param[in]  u_h   当前时刻的传感器测量值
 * @param[out] F     状态转移矩阵
 * @param[out] G     过程噪声输入矩阵
 */
void state_matrix(const precision quat[4], const precision** u, precision** F, precision** G) {
    // 将四元数转换为旋转矩阵
    precision** Rb2t = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        Rb2t[i] = (precision*)malloc(3 * sizeof(precision));
    }
    q2dcm(quat, Rb2t);

    // 将测量力转换为导航坐标系中的力
    precision* f_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        f_t[i] = 0;
        for (int j = 0; j < 3; j++) {
            f_t[i] += Rb2t[i][j] * u[j][0];
        }
    }

    // 创建特定力向量的反对称矩阵
    precision** St = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        St[i] = (precision*)malloc(3 * sizeof(precision));
    }
    St[0][0] = 0; St[0][1] = -f_t[2]; St[0][2] = f_t[1];
    St[1][0] = f_t[2]; St[1][1] = 0; St[1][2] = -f_t[0];
    St[2][0] = -f_t[1]; St[2][1] = f_t[0]; St[2][2] = 0;

    // 零矩阵
    precision** O = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        O[i] = (precision*)calloc(3, sizeof(precision));
    }

    // 单位矩阵
    precision** I = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        I[i] = (precision*)malloc(3 * sizeof(precision));
    }
    I[0][0] = 1; I[0][1] = 0; I[0][2] = 0;
    I[1][0] = 0; I[1][1] = 1; I[1][2] = 0;
    I[2][0] = 0; I[2][1] = 0; I[2][2] = 1;

    // 状态转移矩阵 Fc
    precision** Fc = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        Fc[i] = (precision*)calloc(9, sizeof(precision));
    }
    Fc[0][3] = 1; Fc[1][4] = 1; Fc[2][5] = 1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Fc[i + 3][j + 6] = St[i][j];
        }
    }

    // 噪声增益矩阵 Gc
    precision** Gc = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        Gc[i] = (precision*)calloc(6, sizeof(precision));
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Gc[i + 3][j] = Rb2t[i][j];
            Gc[i + 6][j + 3] = -Rb2t[i][j];
        }
    }

    // 计算离散时间状态转移矩阵 F
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            F[i][j] = (i == j) ? 1.0 + SampleTimeFrequency * Fc[i][j] : SampleTimeFrequency * Fc[i][j];
        }
    }

    // 计算离散时间噪声增益矩阵 G
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            G[i][j] = SampleTimeFrequency * Gc[i][j];
        }
    }

    // 释放动态分配的内存
    for (int i = 0; i < 3; i++) {
        free(Rb2t[i]);
        free(St[i]);
        free(O[i]);
        free(I[i]);
    }
    free(Rb2t);
    free(St);
    free(O);
    free(I);

    for (int i = 0; i < 9; i++) {
        free(Fc[i]);
        free(Gc[i]);
    }
    free(Fc);
    free(Gc);
    free(f_t);
}


void comp_internal_states(precision* x_h, const precision* dx, precision* q_in, const precision yaw) {
    // 动态分配旋转矩阵 R
    precision** R = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        R[i] = (precision*)malloc(3 * sizeof(precision));
    }
    q2dcm(q_in, R);

    // 修正状态向量
    for (int i = 0; i < 9; i++) {
        x_h[i] += dx[i];
    }

    // 动态分配 epsilon 和 OMEGA
    precision* epsilon = (precision*)malloc(3 * sizeof(precision));
    precision** OMEGA = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        OMEGA[i] = (precision*)malloc(3 * sizeof(precision));
    }

    epsilon[0] = dx[6];
    epsilon[1] = dx[7];
    epsilon[2] = dx[8];

    OMEGA[0][0] = 0; OMEGA[0][1] = -epsilon[2]; OMEGA[0][2] = epsilon[1];
    OMEGA[1][0] = epsilon[2]; OMEGA[1][1] = 0; OMEGA[1][2] = -epsilon[0];
    OMEGA[2][0] = -epsilon[1]; OMEGA[2][1] = epsilon[0]; OMEGA[2][2] = 0;

    // 动态分配 tempR
    precision** tempR = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        tempR[i] = (precision*)malloc(3 * sizeof(precision));
    }

    // 计算 tempR
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tempR[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                tempR[i][j] += ((i == k ? 1.0 : 0.0) - OMEGA[i][k]) * R[k][j];
            }
        }
    }

    // 复制 tempR 到 R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = tempR[i][j];
        }
    }

    // 从修正后的旋转矩阵中获取修正后的横滚、俯仰和航向
    x_h[6] = atan2(R[2][1], R[2][2]);
    x_h[7] = -atan(R[2][0] / sqrt(1 - R[2][0] * R[2][0]));
    //x_h[8] = atan2(R[1][0], R[0][0]);
	x_h[8] = yaw;

    // 假设 y 是一个包含 9 个元素的数组
    precision R_tmp[3][3];
    Rt2b(&x_h[6], R_tmp); // y[6] 对应 y(7) 因为 C 语言数组从 0 开始

    // 将 DCM 转换为四元数
    dcm2q(R_tmp, q_in);
    //q2dcm(q_in, R);

    // 释放动态分配的内存
    for (int i = 0; i < 3; i++) {
        free(R[i]);
        free(OMEGA[i]);
        free(tempR[i]);
    }
    free(R);
    free(OMEGA);
    free(tempR);
    free(epsilon);
}

precision custom_atan2(precision y, precision x) {
    if (x > 0) return atan(y / x);
    if (x < 0 && y >= 0) return atan(y / x) + M_PI;
    if (x < 0 && y < 0) return atan(y / x) - M_PI;
    if (x == 0 && y > 0) return M_PI / 2;
    if (x == 0 && y < 0) return -M_PI / 2;
    return 0; // x == 0 && y == 0
}

void updateMatrixP(precision** F, precision** G, precision P[9][9], precision Q[6][6], precision** cov, int current_cols) {
    // 更新滤波器状态协方差矩阵 P
    precision** P_temp = (precision**)malloc(9 * sizeof(precision*));
    precision** P_new = (precision**)malloc(9 * sizeof(precision*));
    precision** GQG = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        P_temp[i] = (precision*)malloc(9 * sizeof(precision));
        P_new[i] = (precision*)malloc(9 * sizeof(precision));
        GQG[i] = (precision*)malloc(9 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            P_temp[i][j] = 0.0;
            P_new[i][j] = 0.0;
            GQG[i][j] = 0.0;
        }
    }

    // 计算 F * P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                P_temp[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // 计算 F * P * F'
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                P_new[i][j] += P_temp[i][k] * F[j][k];
            }
        }
    }

    // 计算 G * Q * G'
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 6; k++) {
                GQG[i][j] += G[i][k] * Q[k][k] * G[j][k];
            }
        }
    }

    // 更新 P 矩阵
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = P_new[i][j] + GQG[i][j];
        }
    }

    // 动态分配内存定义 P 的转置矩阵
    precision** P_transpose = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        P_transpose[i] = (precision*)malloc(9 * sizeof(precision));
    }

    // 计算 P 的转置矩阵
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P_transpose[j][i] = P[i][j];
        }
    }

    // 计算 P = (P + P') / 2
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = (P[i][j] + P_transpose[i][j]) / 2.0;
        }
    }

    // 释放 P_transpose 矩阵内存
    for (int i = 0; i < 9; i++) {
        free(P_transpose[i]);
    }
    free(P_transpose);

    // 存储状态协方差矩阵 P 的对角元素
    for (int i = 0; i < 9; i++) {
        cov[current_cols][i] = P[i][i];
    }

    // 释放临时矩阵的内存
    for (int i = 0; i < 9; i++) {
        free(P_temp[i]);
        free(P_new[i]);
        free(GQG[i]);
    }
    free(P_temp);
    free(P_new);
    free(GQG);
}

void calculate_kalman_gain(precision H[3][9], precision P[9][9], precision R[3][3], precision** K) {
    // 动态分配临时矩阵 HP, HPH 和 HPHR
    precision** HP = (precision**)malloc(3 * sizeof(precision*));
    precision** HPH = (precision**)malloc(3 * sizeof(precision*));
    precision** HPHR = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        HP[i] = (precision*)malloc(9 * sizeof(precision));
        HPH[i] = (precision*)malloc(3 * sizeof(precision));
        HPHR[i] = (precision*)malloc(3 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            HP[i][j] = 0.0;
        }
        for (int j = 0; j < 3; j++) {
            HPH[i][j] = 0.0;
            HPHR[i][j] = 0.0;
        }
    }

    // 计算 H * P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    // 计算 H * P * H'
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 9; k++) {
                HPH[i][j] += HP[i][k] * H[j][k];
            }
        }
    }

    // 计算 H * P * H' + R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            HPHR[i][j] = HPH[i][j] + R[i][j];
        }
    }

    // 高斯消元法求解线性方程组 (P * H') / (H * P * H' + R)

    // 计算 P * H'
    precision** PHt = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        PHt[i] = (precision*)malloc(3 * sizeof(precision));
        for (int j = 0; j < 3; j++) {
            PHt[i][j] = 0.0;
            for (int k = 0; k < 9; k++) {
                PHt[i][j] += P[i][k] * H[j][k];
            }
        }
    }

    // 高斯消元法求解 K = PHt * inv(HPHR)
    for (int i = 0; i < 3; i++) {
        // 归一化主对角线元素
        precision diag = HPHR[i][i];
        for (int j = 0; j < 3; j++) {
            HPHR[i][j] /= diag;
        }
        for (int j = 0; j < 9; j++) {
            PHt[j][i] /= diag;
        }

        // 消去其他行的当前列
        for (int k = 0; k < 3; k++) {
            if (k != i) {
                precision factor = HPHR[k][i];
                for (int j = 0; j < 3; j++) {
                    HPHR[k][j] -= factor * HPHR[i][j];
                }
                for (int j = 0; j < 9; j++) {
                    PHt[j][k] -= factor * PHt[j][i];
                }
            }
        }
    }

    // 将 PHt 的结果赋值给 K
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = PHt[i][j];
        }
    }


    // 释放分配的内存
    for (int i = 0; i < 3; i++) {
        free(HP[i]);
        free(HPH[i]);
        free(HPHR[i]);
    }
    free(HP);
    free(HPH);
    free(HPHR);

    for (int i = 0; i < 9; i++) {
        free(PHt[i]);
    }
    free(PHt);
}

void update_state_covariance(precision** K, precision H[3][9], precision P[9][9], precision I[9][9], precision** cov, int current_cols) {
    // 动态分配 KH 矩阵
    precision** KH = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        KH[i] = (precision*)malloc(9 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            KH[i][j] = 0.0;
        }
    }

    // 计算 KH 矩阵
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    // 动态分配临时矩阵 P_copy 并复制 P 矩阵的值
    precision** P_copy = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        P_copy[i] = (precision*)malloc(9 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            P_copy[i][j] = P[i][j];
        }
    }

    // 更新滤波器状态协方差矩阵 P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = 0.0;
            for (int k = 0; k < 9; k++) {
                P[i][j] += (I[i][k] - KH[i][k]) * P_copy[k][j];
            }
        }
    }

    //再次赋值给 P_copy
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P_copy[i][j] = P[j][i];
        }
    }

    // 确保滤波器状态协方差矩阵 P 是对称的
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = (P[i][j] + P_copy[i][j]) / 2.0;
        }
    }

    // 释放临时矩阵 P_copy 的内存
    for (int i = 0; i < 9; i++) {
        free(P_copy[i]);
    }
    free(P_copy);


    // 存储状态协方差矩阵 P 的对角元素
    for (int i = 0; i < 9; i++) {
        cov[current_cols][i] = P[i][i];
    }

    // 释放动态分配的内存
    for (int i = 0; i < 9; i++) {
        free(KH[i]);
    }
    free(KH);
}   