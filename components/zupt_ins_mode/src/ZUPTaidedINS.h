#ifndef ZUPT_AIDED_INS_H
#define ZUPT_AIDED_INS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "nav_val_define.h"

//��������
//��ʼ���������˲����ĺ����������˳�ʼ״̬Э������P����ʼ��������Э������Q����ʼ��������Э������R�͹۲����H��
void init_filter(precision P[9][9], precision Q[6][6], precision R[3][3], precision H[3][9]); 

//��ʼ��������ݳ��ȵĺ���
void init_vec(uint32_t N, precision P[9][9], precision*** x_h, precision*** cov, precision Id[9][9]);

// �ͷ��ڴ�ĺ���
void free_memory(precision** x_h, precision** cov, precision* zupt, precision* T, int cols);

//���㵼�������ʼ״̬�ĺ��������Ե���ϵͳ������һ���򵥵ĳ�ʼ��׼������ϵͳ�ĺ���Ǻ͸������Ǹ���ǰ20�β����ļ��ٶȼƶ�������õ��ġ�
void init_Nav_eq(const precision** u, uint32_t sample_count, precision* x, precision quat[4], const precision** Angle);

//���Ե���ϵͳ�Ļ�е���������̽��㣬�����˸߽��
//void Navigation_equations(const precision* x, const precision** u, precision* q, precision* y);
void Navigation_equations(const precision* x, const precision** u, precision* q, precision* y, const precision yaw);

//ͨ����ǰ����ͱ�����������״̬ת�ƾ���F�͹��������������G�ĺ���
void state_matrix(const precision quat[4], const precision** u, precision** F, precision** G);

//�ÿ������˲����Ƶ�ϵͳ������������Ƶĵ���״̬�ĺ�����
//void comp_internal_states(precision* x_h, const precision* dx, precision* q_in);
void comp_internal_states(precision* x_h, const precision* dx, precision* q_in, const precision yaw);

// ���ܺ�������
//����ŷ���ǣ������tϵ��ת��bϵ�������ת����
void Rt2b(const precision ang[3], precision R[3][3]);

//����ת����ת��Ϊ��Ԫ�������ĺ���
void dcm2q(const precision R[3][3], precision q[4]);

//����Ԫ������ת��Ϊ��ת����ĺ���
void q2dcm(const precision q[4], precision** R);

//���㷴���еĺ���
precision custom_atan2(precision y, precision x);

//���¿������˲�P���󲢸�ֵ��cov
void updateMatrixP(precision** F, precision** G, precision P[9][9], precision Q[6][6], precision** cov, int current_cols);

//���㿨�����˲��������K
void calculate_kalman_gain(precision H[3][9], precision P[9][9], precision R[3][3], precision** K);

//����ZUPT��Чʱ�Ŀ������˲�����P����ֵ��cov
void update_state_covariance(precision** K, precision H[3][9], precision P[9][9], precision I[9][9], precision** cov, int current_cols);

#endif // ZUPT_AIDED_INS_H
