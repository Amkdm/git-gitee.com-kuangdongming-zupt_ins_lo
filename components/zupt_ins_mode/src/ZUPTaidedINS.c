#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> 
#include "ZUPTaidedINS.h"

// ����ȫ�ֱ���
precision sigma_acc[3] = { 0.5 * 1, 0.5 * 1, 0.5 * 1 };
precision sigma_gyro[3] = { 0.5 * (1 * (M_PI / 180)), 0.5 * (1 * (M_PI / 180)), 0.5 * (1 * (M_PI / 180)) };
precision sigma_vel[3] = { 0.01, 0.01, 0.01 };

void init_vec(uint32_t N, precision P[9][9], precision*** x_h, precision*** cov, precision Id[9][9]) {
    // �����ڴ�
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

    // ��ʼ�� x_h �� cov
    for (uint32_t i = 0; i < N; i++) {
        for (int j = 0; j < 9; j++) {
            (*x_h)[i][j] = (precision)0.0;
            (*cov)[i][j] = (precision)0.0;
        }
    }

    // ��ʼ����λ���� Id
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            Id[i][j] = (i == j) ? (precision)1.0 : (precision)0.0;
        }
    }

    // �� P �ĶԽ�Ԫ�ظ��Ƶ� cov �ĵ�һ��
    for (int i = 0; i < 9; i++) {
        (*cov)[0][i] = P[i][i];
    }
}

void free_memory(precision** x_h, precision** cov, precision* zupt, precision* T, int cols) {
    // �ͷ� x_h ���ڴ�
    for (int i = 0; i < cols; i++) {
        free(x_h[i]); // �ͷ� x_h ��ÿһ��
    }
    free(x_h); // �ͷ� x_h ����

    // �ͷ� cov ���ڴ�
    for (int i = 0; i < cols; i++) {
        free(cov[i]); // �ͷ� cov ��ÿһ��
    }
    free(cov); // �ͷ� cov ����

    // �ͷ� zupt �� T ���ڴ�
    free(zupt);
    free(T);
}

void init_filter(precision P[9][9], precision Q[6][6], precision R[3][3], precision H[3][9]) {
    // ��ʼ�� P ����
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

    // ��ʼ�� Q ����
    memset(Q, 0, 6 * 6 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        Q[i][i] = sigma_acc[i] * sigma_acc[i];
        Q[i + 3][i + 3] = sigma_gyro[i] * sigma_gyro[i];
    }

    // ��ʼ�� R ����
    memset(R, 0, 3 * 3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        R[i][i] = sigma_vel[i] * sigma_vel[i];
    }

    // ��ʼ�� H ����
    memset(H, 0, 3 * 9 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        H[i][i + 3] = 1.0;
    }
}


void init_Nav_eq(const precision** u, uint32_t sample_count, precision* x, precision quat[4], const precision** Angle) {
    // ����ϵͳ��ǰ sample_count �������ڼ��Ǿ�ֹ�ģ���ǰ sample_count �����ٶȼƶ����м����ʼ����͸�����
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

    // ������̬����
    precision attitude[3] = { roll, pitch, Angle[0][0]};

    // �������ʼ��̬��Ӧ����Ԫ��
    precision Rb2t[3][3];
    Rt2b(attitude, Rb2t);
    dcm2q(Rb2t, quat);

    // ���ó�ʼ״̬����
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
 * @brief ���������ϵ t ��ת������ϵ b ����ת���󣬸���ŷ���ǡ�
 *
 * @param[in]  ang  ŷ���� [roll, pitch, heading]
 * @param[out] R    ��ת����
 */
void Rt2b(const precision ang[3], precision R[3][3]) {
    // �����ת�ǵ����Һ�����
    precision cr = cos(ang[0]);
    precision sr = sin(ang[0]);

    // ���㸩���ǵ����Һ�����
    precision cp = cos(ang[1]);
    precision sp = sin(ang[1]);

    // ���㺽��ǵ����Һ�����
    precision cy = cos(ang[2]);
    precision sy = sin(ang[2]);

    // �����ת���� R
    R[0][0] = cy * cp;
    R[0][1] = sy * cp;
    R[0][2] = -sp;

    R[1][0] = -sy * cr + cy * sp * sr;
    R[1][1] = cy * cr + sy * sp * sr;
    R[1][2] = cp * sr;

    R[2][0] = sy * sr + cy * sp * cr;
    R[2][1] = -cy * sr + sy * sp * cr;
    R[2][2] = cp * cr;

    // ��̬�����ڴ����洢ת�þ���
    precision* R_transposed = (precision*)malloc(3 * 3 * sizeof(precision));
    if (R_transposed == NULL) {
        // �����ڴ����ʧ�ܵ����
        return;
    }

    // ת�þ��� R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_transposed[j * 3 + i] = R[i][j];
        }
    }

    // ��ת�ú�ľ����ƻ� R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = R_transposed[i * 3 + j];
        }
    }

    // �ͷŶ�̬������ڴ�
    free(R_transposed);

}

/**
 * @brief ���������Ҿ�����ת����ת��Ϊ��Ԫ��������
 *
 * @param[in]  R  ��ת�����ָ��
 * @param[out] q  ��Ԫ��������ָ��
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
    // �����ڴ���������
    memset(y, 0, 9 * sizeof(precision));

    // ��ȡϵͳ�Ĳ�������
    precision Ts = SampleTimeFrequency;

    // ������Ԫ������ "q" �����Ľ��ٶȲ���ֵ
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

        // ��ʼ�� new_q, omega_term
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

        // ��һ����Ԫ��
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



    // ʹ�ø��º����Ԫ����ȡ����ϵͳ����̬��ŷ���ǣ�
    precision** Rb2t = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        Rb2t[i] = (precision*)malloc(3 * sizeof(precision));
    }

	q2dcm(q, Rb2t);

    // �����ת��
    y[6] = atan2(Rb2t[2][1], Rb2t[2][2]);

	// ���㸩����
    y[7] = -atan(Rb2t[2][0] / sqrt(1 - Rb2t[2][0] * Rb2t[2][0]));

	// ����ƫ����
    //y[8] = custom_atan2(Rb2t[1][0], Rb2t[0][0]);
    y[8] = yaw;

    // ���� y ��һ������ 9 ��Ԫ�ص�����
    precision R_tmp[3][3];
    Rt2b(&y[6], R_tmp);

    // �� DCM ת��Ϊ��Ԫ��
    dcm2q(R_tmp, q);

    q2dcm(q, Rb2t);

    // ����λ�ú��ٶ�״̬��ʹ�ò����ı������¼������̬
    precision g_t[3] = { 0, 0, g };

    // ����������ת������������ϵ
    precision* f_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        f_t[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            f_t[i] += Rb2t[i][j] * u[j][0];
        }
    }

    // ��ȥ�����ϣ��������õ���������ϵ�еļ��ٶ�
    precision* acc_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        acc_t[i] = f_t[i] + g_t[i];
    }

    // ״̬�ռ�ģ�;���
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

    // ����λ�ú��ٶȹ���
    for (int i = 0; i < 6; i++) {
        y[i] = 0;
        for (int j = 0; j < 6; j++) {
            y[i] += A[i][j] * x[j];
        }
        for (int j = 0; j < 3; j++) {
            y[i] += B[i][j] * acc_t[j];
        }
    }
    // �ͷŶ�̬������ڴ�
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
 * @brief ����Ԫ������ת��Ϊ�������Ҿ�����ת����
 *
 * @param[in] q ��Ԫ������
 * @param[out] R �������Ҿ�����ת����
 */
void q2dcm(const precision q[4], precision** R) {
    precision p[6] = { 0 };

    // ������Ԫ����ƽ��
    for (int i = 0; i < 4; i++) {
        p[i] = q[i] * q[i];
    }

    // ���� p[4] = q2^2 + q3^2
    p[4] = p[1] + p[2];

    // ���� p[5] = 2 / (q0^2 + q3^2 + q2^2)
    if (p[0] + p[3] + p[4] != 0) {
        p[5] = 2 / (p[0] + p[3] + p[4]);
    }
    else {
        p[5] = 0;
    }

    // ���㷽�����Ҿ���ĶԽ�Ԫ��
    R[0][0] = 1 - p[5] * p[4];
    R[1][1] = 1 - p[5] * (p[0] + p[2]);
    R[2][2] = 1 - p[5] * (p[0] + p[1]);

    // ���㷽�����Ҿ���ķǶԽ�Ԫ��
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
 * @brief ����״̬ת�ƾ��� F �͹�������������� G��
 *
 * @param[in]  quat  ��ǰ��̬����Ԫ��
 * @param[in]  u_h   ��ǰʱ�̵Ĵ���������ֵ
 * @param[out] F     ״̬ת�ƾ���
 * @param[out] G     ���������������
 */
void state_matrix(const precision quat[4], const precision** u, precision** F, precision** G) {
    // ����Ԫ��ת��Ϊ��ת����
    precision** Rb2t = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        Rb2t[i] = (precision*)malloc(3 * sizeof(precision));
    }
    q2dcm(quat, Rb2t);

    // ��������ת��Ϊ��������ϵ�е���
    precision* f_t = (precision*)malloc(3 * sizeof(precision));
    for (int i = 0; i < 3; i++) {
        f_t[i] = 0;
        for (int j = 0; j < 3; j++) {
            f_t[i] += Rb2t[i][j] * u[j][0];
        }
    }

    // �����ض��������ķ��Գƾ���
    precision** St = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        St[i] = (precision*)malloc(3 * sizeof(precision));
    }
    St[0][0] = 0; St[0][1] = -f_t[2]; St[0][2] = f_t[1];
    St[1][0] = f_t[2]; St[1][1] = 0; St[1][2] = -f_t[0];
    St[2][0] = -f_t[1]; St[2][1] = f_t[0]; St[2][2] = 0;

    // �����
    precision** O = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        O[i] = (precision*)calloc(3, sizeof(precision));
    }

    // ��λ����
    precision** I = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        I[i] = (precision*)malloc(3 * sizeof(precision));
    }
    I[0][0] = 1; I[0][1] = 0; I[0][2] = 0;
    I[1][0] = 0; I[1][1] = 1; I[1][2] = 0;
    I[2][0] = 0; I[2][1] = 0; I[2][2] = 1;

    // ״̬ת�ƾ��� Fc
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

    // ����������� Gc
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

    // ������ɢʱ��״̬ת�ƾ��� F
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            F[i][j] = (i == j) ? 1.0 + SampleTimeFrequency * Fc[i][j] : SampleTimeFrequency * Fc[i][j];
        }
    }

    // ������ɢʱ������������� G
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 6; j++) {
            G[i][j] = SampleTimeFrequency * Gc[i][j];
        }
    }

    // �ͷŶ�̬������ڴ�
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
    // ��̬������ת���� R
    precision** R = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        R[i] = (precision*)malloc(3 * sizeof(precision));
    }
    q2dcm(q_in, R);

    // ����״̬����
    for (int i = 0; i < 9; i++) {
        x_h[i] += dx[i];
    }

    // ��̬���� epsilon �� OMEGA
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

    // ��̬���� tempR
    precision** tempR = (precision**)malloc(3 * sizeof(precision*));
    for (int i = 0; i < 3; i++) {
        tempR[i] = (precision*)malloc(3 * sizeof(precision));
    }

    // ���� tempR
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tempR[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                tempR[i][j] += ((i == k ? 1.0 : 0.0) - OMEGA[i][k]) * R[k][j];
            }
        }
    }

    // ���� tempR �� R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = tempR[i][j];
        }
    }

    // �����������ת�����л�ȡ������ĺ���������ͺ���
    x_h[6] = atan2(R[2][1], R[2][2]);
    x_h[7] = -atan(R[2][0] / sqrt(1 - R[2][0] * R[2][0]));
    //x_h[8] = atan2(R[1][0], R[0][0]);
	x_h[8] = yaw;

    // ���� y ��һ������ 9 ��Ԫ�ص�����
    precision R_tmp[3][3];
    Rt2b(&x_h[6], R_tmp); // y[6] ��Ӧ y(7) ��Ϊ C ��������� 0 ��ʼ

    // �� DCM ת��Ϊ��Ԫ��
    dcm2q(R_tmp, q_in);
    //q2dcm(q_in, R);

    // �ͷŶ�̬������ڴ�
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
    // �����˲���״̬Э������� P
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

    // ���� F * P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                P_temp[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // ���� F * P * F'
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                P_new[i][j] += P_temp[i][k] * F[j][k];
            }
        }
    }

    // ���� G * Q * G'
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 6; k++) {
                GQG[i][j] += G[i][k] * Q[k][k] * G[j][k];
            }
        }
    }

    // ���� P ����
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = P_new[i][j] + GQG[i][j];
        }
    }

    // ��̬�����ڴ涨�� P ��ת�þ���
    precision** P_transpose = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        P_transpose[i] = (precision*)malloc(9 * sizeof(precision));
    }

    // ���� P ��ת�þ���
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P_transpose[j][i] = P[i][j];
        }
    }

    // ���� P = (P + P') / 2
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = (P[i][j] + P_transpose[i][j]) / 2.0;
        }
    }

    // �ͷ� P_transpose �����ڴ�
    for (int i = 0; i < 9; i++) {
        free(P_transpose[i]);
    }
    free(P_transpose);

    // �洢״̬Э������� P �ĶԽ�Ԫ��
    for (int i = 0; i < 9; i++) {
        cov[current_cols][i] = P[i][i];
    }

    // �ͷ���ʱ������ڴ�
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
    // ��̬������ʱ���� HP, HPH �� HPHR
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

    // ���� H * P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    // ���� H * P * H'
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 9; k++) {
                HPH[i][j] += HP[i][k] * H[j][k];
            }
        }
    }

    // ���� H * P * H' + R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            HPHR[i][j] = HPH[i][j] + R[i][j];
        }
    }

    // ��˹��Ԫ��������Է����� (P * H') / (H * P * H' + R)

    // ���� P * H'
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

    // ��˹��Ԫ����� K = PHt * inv(HPHR)
    for (int i = 0; i < 3; i++) {
        // ��һ�����Խ���Ԫ��
        precision diag = HPHR[i][i];
        for (int j = 0; j < 3; j++) {
            HPHR[i][j] /= diag;
        }
        for (int j = 0; j < 9; j++) {
            PHt[j][i] /= diag;
        }

        // ��ȥ�����еĵ�ǰ��
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

    // �� PHt �Ľ����ֵ�� K
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = PHt[i][j];
        }
    }


    // �ͷŷ�����ڴ�
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
    // ��̬���� KH ����
    precision** KH = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        KH[i] = (precision*)malloc(9 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            KH[i][j] = 0.0;
        }
    }

    // ���� KH ����
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    // ��̬������ʱ���� P_copy ������ P �����ֵ
    precision** P_copy = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        P_copy[i] = (precision*)malloc(9 * sizeof(precision));
        for (int j = 0; j < 9; j++) {
            P_copy[i][j] = P[i][j];
        }
    }

    // �����˲���״̬Э������� P
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = 0.0;
            for (int k = 0; k < 9; k++) {
                P[i][j] += (I[i][k] - KH[i][k]) * P_copy[k][j];
            }
        }
    }

    //�ٴθ�ֵ�� P_copy
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P_copy[i][j] = P[j][i];
        }
    }

    // ȷ���˲���״̬Э������� P �ǶԳƵ�
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            P[i][j] = (P[i][j] + P_copy[i][j]) / 2.0;
        }
    }

    // �ͷ���ʱ���� P_copy ���ڴ�
    for (int i = 0; i < 9; i++) {
        free(P_copy[i]);
    }
    free(P_copy);


    // �洢״̬Э������� P �ĶԽ�Ԫ��
    for (int i = 0; i < 9; i++) {
        cov[current_cols][i] = P[i][i];
    }

    // �ͷŶ�̬������ڴ�
    for (int i = 0; i < 9; i++) {
        free(KH[i]);
    }
    free(KH);
}   