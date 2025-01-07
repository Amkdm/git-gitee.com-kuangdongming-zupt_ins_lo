#ifndef NAV_VAL_DEFINE_H_
#define NAV_VAL_DEFINE_H_

// ��double����Ϊprecision
typedef double precision;

// ��ȡ���������е�������������ٶ�+������ٶȣ�
#define ROWS 6

// ����ȫ�ֱ��� latitude �� altitude
extern precision latitude; // γ��
extern precision altitude; // �߶�

// ����Բ����Pi
#define M_PI 3.14159265358979323846

// ���ټ���㷨��ر���
extern int detector_type; // ���ټ��������
extern int window_size;   // ���ټ�ⴰ�ڴ�С
extern precision sigma_a; // ���ٶȼ�������׼��[m/s^2]  �����������ٶ�̽�����Լ��ٶȼ����ݵļ����ֵ
extern precision sigma_g; // ������������׼��[rad/s]  �����������ٶ�̽���������������ݵļ����ֵ
extern precision g;       // �������ٶ�[m/s^2]
extern precision gamma1;   // ���ټ�����ļ����ֵ�����ͳ��ֵ�������ֵ�����϶���������״̬

/// ���ټ��������
#define DETECTOR_TYPE_GLRT 0
#define DETECTOR_TYPE_MV 1
#define DETECTOR_TYPE_MAG 2
#define DETECTOR_TYPE_ARE 3

// ����������ر���
// �������������������ͳ�ʼ״̬Э�������Q��R��P�����á�
// �����������������Ϊ�ԽǾ����������þ�����Ϊ��׼�

extern precision sigma_acc[3];// ���ڽ�ģ���ٶȼ�����(x y zƽ̨����ϵ)���������ٶȼ����[m/s^2]�Ĺ�������
extern precision sigma_gyro[3];// ���ڽ�ģ����������(x y zƽ̨����ϵ)���������������[rad/s]�Ĺ�������
extern precision sigma_vel[3];// α���ٸ��²�������Э����(R)  �ٶ���Ϊ�Խ���

// ��ʼ��λ��
#define INITIAL_HEADING 0.0

/// ����������ϵ�µĳ�ʼλ��
#define INITIAL_POSX 0.0
#define INITIAL_POSY 0.0
#define INITIAL_POSZ 0.0

// �����������ٶ�ǰ���ٸ������Ǿ�ֹ�ģ��Դ�����ʼ������Ǻ͸�����
#define SAMPLE_COUNT 20 

// ����������Ƶ��
#define SampleTimeFrequency (1.0 / 100.0)

// ��ʼ״̬����P�ĶԽ�ֵ�趨
// λ�ã�x y z��������ϵ��[m] ֵΪ��1e-5
#define SIGMA2_INIT_POSX (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_POSY (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_POSZ (precision)(0.00001 * 0.00001)

// �ٶȣ�x y z��������ϵ��[m/s] ֵΪ��1e-5
#define SIGMA2_INIT_VELX (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_VELY (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_VELZ (precision)(0.00001 * 0.00001)

// ��̬����� ���� ��λ��[rad] ֵΪ��0.00175��pi/180*[0.1 0.1 0.1])
#define SIGMA2_INIT_ROLL  (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))
#define SIGMA2_INIT_PITCH (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))
#define SIGMA2_INIT_YAW   (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))


#endif /* NAV_VAL_DEFINE_H_ */
