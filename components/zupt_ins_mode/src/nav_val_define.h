#ifndef NAV_VAL_DEFINE_H_
#define NAV_VAL_DEFINE_H_

// 将double定义为precision
typedef double precision;

// 读取传感器序列的行数（三轴加速度+三轴角速度）
#define ROWS 6

// 定义全局变量 latitude 和 altitude
extern precision latitude; // 纬度
extern precision altitude; // 高度

// 定义圆周率Pi
#define M_PI 3.14159265358979323846

// 零速检测算法相关变量
extern int detector_type; // 零速检测器类型
extern int window_size;   // 零速检测窗口大小
extern precision sigma_a; // 加速度计噪声标准差[m/s^2]  用来控制零速度探测器对加速度计数据的检测阈值
extern precision sigma_g; // 陀螺仪噪声标准差[rad/s]  用来控制零速度探测器对陀螺仪数据的检测阈值
extern precision g;       // 重力加速度[m/s^2]
extern precision gamma1;   // 零速检测器的检测阈值，如果统计值低于这个值，则认定处于零速状态

/// 零速检测器类型
#define DETECTOR_TYPE_GLRT 0
#define DETECTOR_TYPE_MV 1
#define DETECTOR_TYPE_MAG 2
#define DETECTOR_TYPE_ARE 3

// 导航解算相关变量
// 过程噪声、测量噪声和初始状态协方差矩阵Q、R和P的设置。
// 假设所有三个矩阵均为对角矩阵，所有设置均定义为标准差。

extern precision sigma_acc[3];// 用于建模加速度计噪声(x y z平台坐标系)和其他加速度计误差[m/s^2]的过程噪声
extern precision sigma_gyro[3];// 用于建模陀螺仪噪声(x y z平台坐标系)和其他陀螺仪误差[rad/s]的过程噪声
extern precision sigma_vel[3];// 伪零速更新测量噪声协方差(R)  假定其为对角阵。

// 初始方位角
#define INITIAL_HEADING 0.0

/// 东北天坐标系下的初始位置
#define INITIAL_POSX 0.0
#define INITIAL_POSY 0.0
#define INITIAL_POSZ 0.0

// 样本数量，假定前多少个样本是静止的，以此来初始化横滚角和俯仰角
#define SAMPLE_COUNT 20 

// 传感器采样频率
#define SampleTimeFrequency (1.0 / 100.0)

// 初始状态矩阵P的对角值设定
// 位置（x y z导航坐标系）[m] 值为：1e-5
#define SIGMA2_INIT_POSX (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_POSY (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_POSZ (precision)(0.00001 * 0.00001)

// 速度（x y z导航坐标系）[m/s] 值为：1e-5
#define SIGMA2_INIT_VELX (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_VELY (precision)(0.00001 * 0.00001)
#define SIGMA2_INIT_VELZ (precision)(0.00001 * 0.00001)

// 姿态（横滚 俯仰 方位）[rad] 值为：0.00175（pi/180*[0.1 0.1 0.1])
#define SIGMA2_INIT_ROLL  (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))
#define SIGMA2_INIT_PITCH (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))
#define SIGMA2_INIT_YAW   (precision)((M_PI / 180.0 * 0.1) * (M_PI / 180.0 * 0.1))


#endif /* NAV_VAL_DEFINE_H_ */
