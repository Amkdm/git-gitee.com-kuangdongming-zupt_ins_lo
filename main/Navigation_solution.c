#include "Navigation_solution.h"

precision** u_data = NULL;
precision** Angle_data = NULL; 

int window_size = 5; // 零速检测滑动窗口大小
const int N = 10;

static int total_cols = 0;//总列数 改值的累计需要在惯性器件数据读取任务中即 inertia.c中更新 这里提供接口给外部设置改值

static precision** x_h;
static precision** cov;

/**
 * @brief 提供给外部的接口，设置总列数
 * @param  cur_cols             
 */
void Navigation_solution_set_total_cols(int cur_cols)
{
    total_cols = cur_cols;
}

/**
 * @brief   提供给外部的接口，获取总列数
 * @return int 
 */
int Navigation_solution_get_total_cols(void)
{
    return total_cols;
}

/**
 * @brief 申请用于存放传感器采集数据的内存
 */
static void Navigation_solution_initialize_dataset() 
{
    // 初始化动态数组
    u_data = (precision**)malloc(ROWS * sizeof(precision*)); // 加速度和角速度
    Angle_data = (precision**)malloc(sizeof(precision*)); // 角度Z(°)
    Angle_data[0] = (precision*)malloc(QUEUE_SIZE * sizeof(precision));
    for (int i = 0; i < ROWS; i++) {
        u_data[i] = (precision*)malloc(QUEUE_SIZE * sizeof(precision));
    }
}

/**
 * @brief 释放用于存放传感器采集数据的内存 一般不需要姿态解算 任务才会调用
 * @return * void 
 */
static void Navigation_solution_free_dataset(void) 
{
    for (int i = 0; i < ROWS; i++) {
        free(u_data[i]);
    }
    free(u_data);
    free(Angle_data[0]);
    free(Angle_data);
}


/**
 * @brief 导航解算任务
 * @param  pvParameters     
 */
void navigation_solution_task(void *pvParameters)
{
    int do_imu_flag = 0;//是否做惯导标志

    //等待GPS和IMU启动完成

    //等待GPS数据有效

    //先选择任意经纬度初始化一个重力加速度  zero_velocity_detector_new.c中的函数
    initialize_detector(22,20);

    // 分配读取数据集的内存空间
    Navigation_solution_initialize_dataset();

    //————————初始化最终解算结果存储数组————————
	// 定义最终的 zupt 和 T 数组，长度为N
	//  zupt：零速检测结果（true or false） T：零速检测阈值（根据这个阈值和预设阈值比较得到zupt的值）
    bool* zupt = (bool*)malloc(N * sizeof(bool));
    for (int i = 0; i < N; i++) {
		zupt[i] = false;
	}
    precision* T = (precision*)malloc(N * sizeof(precision));

    // 导航解算部分矩阵初始化 初始化滤波器状态协方差矩阵 P，过程噪声协方差矩阵 Q，伪测量噪声协方差 R 和观测矩阵 H
    precision P[9][9], Q[6][6], R[3][3], H[3][9];
    precision Id[9][9];
    precision quat[4];
    init_filter(P, Q, R, H);
    
    // 分配向量--主要时用于存放最终结果的数组 x_h 和 cov
    init_vec(N, P, &x_h, &cov, Id);

    precision** F = (precision**)malloc(9 * sizeof(precision*));
    precision** G = (precision**)malloc(9 * sizeof(precision*));
    for (int i = 0; i < 9; i++) {
        F[i] = (precision*)malloc(9 * sizeof(precision));
        G[i] = (precision*)malloc(6 * sizeof(precision));
    }
    printf("存放数据的数组已经分配内存\n");


    //——————开始解算流程——————
    while(1)
    {
        if(gps_data.fix_quality){
            //gps数据有效才会根据经纬度初始化检测器
            initialize_detector(gps_data.latitude,gps_data.longitude);
        }

        //如果在室内 则需要做惯导
        if(gps_data.fix_quality == 0){
            do_imu_flag = 1;
        }else{
            //数据有效，再看可见卫星数量   导航精度够不够
            if(gps_data.sat_in_view>9)
            {
                //大概率在室外
                if(gps_data.hdop < 3.0)
                {
                    do_imu_flag = 0;//不需要做IMU惯导，直接使用GPS经纬度数据 
                }
                else 
                {
                    do_imu_flag = 0; //也不需要惯导
                }
            }
            else 
            {
                //printf("大概率在室内\n");
                if(gps_data.hdop < 3.0)
                {
                    do_imu_flag = 0; //也不需要惯导
                }
                else 
                {
                    do_imu_flag = 1; //需要惯导
                }
            }
        }

        //switch(do_imu_flag)
        #if 1

        //暂时固定只用惯导！！！！
        switch(1)
        {
            //不需要做惯导
            case 0:    
            {
                //直接使用GPS数据
                //使用GPS数据解算后的结果    
                break;
            }
            
            //需要做惯导
            case 1:
            {
                // 只有先等待QUEUE_SIZE次采样后（即缓冲区满）才开始进行零速检测和导航解算。
                // 随后来一个解算一次，相当于实时解算（但是输出延后了QUEUE_SIZE次采样的时间），相当于t时刻是在解算得到t-QUEUE_SIZE时刻的数据。
                if(total_cols == QUEUE_SIZE)
                {
                    // 在刚存满缓冲区时，进行初始化横滚角、俯仰角计算（需要用到SampleCount个数据，这个数据必须小于QUEUE_SIZE），并根据初始值赋值给x_h[0]。
                    // 动态分配内存
                    precision** u_init = (precision**)malloc(3 * sizeof(precision*));
                    for (int i = 0; i < 3; i++) {
                        u_init[i] = (precision*)malloc(SAMPLE_COUNT * sizeof(precision));
                        for (int j = 0; j < SAMPLE_COUNT; j++) {
                            u_init[i][j] = u_data[i][j];
                        }
                    }

                    // 调用 init_Nav_eq 函数
                    init_Nav_eq((const precision**)u_init, SAMPLE_COUNT, x_h[0], quat, Angle_data);

                    // 释放分配的内存
                    for (int i = 0; i < 3; i++) {
                        free(u_init[i]);
                    }
                    free(u_init);

                    //零速监测
                    zero_velocity_detector(u_data, window_size , zupt + total_cols - QUEUE_SIZE, T + total_cols - QUEUE_SIZE);

                }
                else if (total_cols > QUEUE_SIZE) 
                {
                    // 超过QUEUE_SIZE个值后即刻开始导航解算更新
                    //零速监测
                    zero_velocity_detector(u_data, window_size , zupt + total_cols - QUEUE_SIZE, T + total_cols - QUEUE_SIZE);

                    //姿态解算的正式部分
                    {

                        // 复制一个k次采样的向量存储传感器值。
                        precision** u_h = (precision**)malloc(6 * sizeof(precision*));
                        for (int i = 0; i < 6; i++) {
                            u_h[i] = (precision*)malloc(1 * sizeof(precision));
                            u_h[i][0] = u_data[i][0];
                        }

                        // 更新导航方程  x_h_new是结果  用x_h[0]作为基数，更新x_h[1]
                        precision* x_h_new = (precision*)malloc(9 * sizeof(precision));
                        Navigation_equations(x_h[total_cols - QUEUE_SIZE - 1], u_h, quat, x_h_new, Angle_data[0][0]);
                        memcpy(x_h[total_cols - QUEUE_SIZE], x_h_new, 9 * sizeof(precision));
                        free(x_h_new);

                        // 更新状态转移矩阵
                        state_matrix(quat, u_h, F, G);
 
                        // 更新卡尔曼滤波状态协方差矩阵 P 并将其对角元素幅值给cov
			            updateMatrixP(F, G, P, Q, cov, total_cols - QUEUE_SIZE);

                        // 零速度更新
                        if (zupt[total_cols - QUEUE_SIZE]) {
                            //printf("非零速\r\n");
                            // 动态分配卡尔曼滤波增益矩阵 K
                            precision** K = (precision**)malloc(9 * sizeof(precision*));
                            for (int i = 0; i < 9; i++) {
                                K[i] = (precision*)malloc(3 * sizeof(precision));
                                for (int j = 0; j < 3; j++) {
                                    K[i][j] = 0.0;
                                }
                            }

                            //计算卡尔曼滤波增益矩阵 K
                            calculate_kalman_gain(H, P, R, K);

                            // 动态分配预测误差向量 z
                            precision* z = (precision*)malloc(3 * sizeof(precision));
                            for (int i = 0; i < 3; i++) {
                                z[i] = -x_h[total_cols - QUEUE_SIZE][i + 3];
                            }

                            // 动态分配导航状态的扰动向量 dx
                            precision* dx = (precision*)malloc(9 * sizeof(precision));
                            for (int i = 0; i < 9; i++) {
                                dx[i] = 0.0;
                                for (int j = 0; j < 3; j++) {
                                    dx[i] += K[i][j] * z[j];
                                }
                            }

                            // 使用估计的扰动修正导航状态
                            comp_internal_states(x_h[total_cols - QUEUE_SIZE], dx, quat, Angle_data[0][0]);
                            
                            // 更新状态协方差矩阵P
                            update_state_covariance(K, H, P, Id, cov, total_cols - QUEUE_SIZE);

                            // 释放动态分配的内存
                            for (int i = 0; i < 9; i++) {
                                free(K[i]);
                            }
                            free(K);
                            free(z);
                            free(dx);
                        }
                        else 
                        {
                            //printf("零速\n");
                        }
                        // 释放 u_h 的内存
                        for (int i = 0; i < 6; i++) {
                            free(u_h[i]);
                        }
                        free(u_h);
                    }

                    //在这里拿出x_h[total_cols - QUEUE_SIZE]的值，这是最新的解算结果，但是是延后了QUEUE_SIZE次采样的时间的。
                    //具体是：x_h[total_cols - QUEUE_SIZE][0]为y轴位置  x_h[total_cols - QUEUE_SIZE][1]为x轴位置 这就是要的平面定位结果 其他行不用管 都不需要。
                    //即假如传感器采样率100Hz，那么对于QUEUE_SIZE=100的话，x_h的第[total_cols - QUEUE_SIZE]行是在解算得到t=0s的位置数据，但是是在t=1s时刻输出的
                    //也就是说，前1秒没有数据输出，一秒以后才有前一秒的解算结果。延后了1秒
                    //想要降低延迟，可以减小QUEUE_SIZE的值，但是不能小于Sample_count的值，因为需要Sample_count个数据进行初始化
                    //在这个项目中，将Sample_count设定为20，也就是需要20次采样的数据来完成初始化，所以不论怎样读取的传感器数据队列都至少得有这么多个，最好给点余量。
                    for (int i = 0; i < N - 1; i++) 
                    {
                        // 上移 x_h 数组
                        memcpy(x_h[i], x_h[i + 1], 9 * sizeof(precision));
                        // 上移 cov 数组
                        memcpy(cov[i], cov[i + 1], 9 * sizeof(precision));
                        // 上移 T 数组
                        T[i] = T[i + 1];
                        // 上移 zupt 数组
                        zupt[i] = zupt[i + 1];
                    }


                    //使用IMU数据解算后的结果x_h[total_cols - QUEUE_SIZE]
                    printf("解算完成 %.6lf  %.6lf  %.6lf \n",x_h[total_cols - QUEUE_SIZE][0],x_h[total_cols - QUEUE_SIZE][1],x_h[total_cols - QUEUE_SIZE][2]);   
                   
                }
                break;     
            }    
        }
        #endif

        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    

    // 释放F和G矩阵的内存
    for (int i = 0; i < 9; i++) {
        free(F[i]);
        free(G[i]);
    }
    free(F);
    free(G);

    Navigation_solution_free_dataset(); // 释放数据集内存
}



