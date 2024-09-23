#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H



/**
  * @brief          一阶卡尔曼滤波器
  * @param[in]      None
  * @retval         None
  */
typedef struct
{
    float A;//状态转移矩阵
    float B;//输入矩阵
    float Q;//过程噪声协方差矩阵
    float R;//测量噪声协方差矩阵
    float H;//状态观测矩阵

    float X_prior;//先验估计
    float X_posterior;//后验估计
    float X_last;//上次估计
    float P_prior;
    float P_last;
    float U;
    float U_last;

    float Kalman_K;
}Kalman_fir_t;



void Kalman_fir_init(Kalman_fir_t *kalman, float T_Q, float T_R);
float KalmanFilter_fir(Kalman_fir_t *kalman, float z_data);



/*********************************************************************************
  * @brief          二阶卡尔曼滤波器
  * @param[in]      None
  * @retval         None
  ******************************************************************************** 
  * @attention      二阶卡尔曼提供了位移，速度，到加速度的状态估计，同时可以将加速度计作为系统输入，轮式里程计作为测量值做数据融合检验
  * 
  *  A[2][2] 状态转移矩阵，代表了x、v两个变量的状态转化关系(以下t为delta_t，即时间常数)
  *
  *          x = x    x = v*t 
  *          v = 0*x  v = v   
  * 
  *          由上式易得：
  * 
  *          A[2][2]={{1   t},
  *                   {0   1}}
  *
  *  B[2]    输入矩阵，输入为加速度a,代表了a与x、v的关系
  *
  *          x = 1/2*t^2*a
  *          v = a*t
  * 
  *          则B[2]={ 0.5*t^2 ,
  *                   t       }
  * 
  *  P.P_prior[2][2]  先验状态估计误差的协方差矩阵
  *  P.P[2][2]        后验状态估计误差协方差矩阵
  *          
  *          KalmanFilter一般形式表达式   x_hat(k) = x_hat_prior(k) + K(k)(z(k) - Hm * x_hat_prior(k))
  *            
  *          先验估计误差:  e_prior(k) = x(k) - x_hat_prior(k) = A*e(k-1)+w(K-1)  
  *          先验估计误差存在是由于噪声的存在，其期望是噪声w和v期望的线性组合
  * 
  *          后验估计误差：  e(k) = x(k) - x_hat(k)
  *          Tr(P(k))协方差矩阵的迹，代表每一个状态变量估计误差的方差加和，它越小，说明估计值越准确
  *          所以，求最优的卡尔曼增益K(k)就是求最小的Tr(P(k))
  *          最后，求出P(k) = (I - K(k) * Hm) * P_prior(k)
  * 
  * 
  * 
  *  Q[2][2] 过程噪声协方差矩阵
  * 
  *          由卡尔曼滤波器状态方程x(k)=Ax(k-1)+Bu(k-1)+w(k-1)，可得：
  *          w为过程噪声，其服从高斯分布(期望为0，协方差矩阵为Q)，产生于物理系统中的环境噪声
  * 
  *          如果过程噪声是相互独立的：
  * 
  *          Q[2][2]={{位置方差       0    },
  *                  {    0       速度方差 }}
  *    
  *   R      测量协方差矩阵，本应是一个二维对角矩阵（参考Q[2][2]）,但是为了简化模型并兼顾系统的性能和稳定性将R简化，
  *          但是，如果测量过程中使用了更多不同的传感器或测量源，或者这些传感器的噪声特性不同，也需要根据实际情况设计R矩阵
  *          再者，R和Q阵代表了对实际测量和物理模型的信任度，将R简化也有利于调整Q阵参数使其快速收敛
  * 
  */
typedef struct 
{
    float x, v;//状态向量

}StateVector;

typedef struct
{
    float P[2][2];//协方差矩阵
} CovMatrix;

typedef struct 
{
    StateVector X_prior;//先验估计状态向量
    StateVector X_now;//后验估计状态向量
    StateVector out;//输出后验估计状态向量
    CovMatrix P_prior;//先验状态协方差矩阵
    CovMatrix P;//后验估计协方差矩阵

    float a;//输入向量
    
    float time_const;

    float A[2][2];//状态转移矩阵
    float B[2];//输入矩阵
    float Q[2][2];//过程噪声协方差矩阵
    float R;//测量噪声协方差矩阵
    float Hm[2];

    double K[2];

} Kalman_sec_t;

void Kalman_sec_init(Kalman_sec_t *kalman, float Qc_1, float Qc_2);
void KalmanFilter_sec(Kalman_sec_t *kalman, float distance, float accel);

#endif
