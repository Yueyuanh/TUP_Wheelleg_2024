/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Yue Yuanhao 
 * @version V0.0.1
 * @date    2023/12/29
 * @brief   KalmanFilter test
 ******************************************************************************
 * @date    2023/12/29  一阶KalmanFilter
 *          2023/12/30  二阶KalmanFilter
 * 
 * 
 * 
**/

#include "KalmanFilter.h"


/***************************************************************************************************
* @name   Kalman_fir_init
* @brief  创建一个一阶卡尔曼滤波器
* @param  kalman : 滤波器
* @param  T_Q : 系统噪声协方差
* @param  T_R : 测量噪声协方差  
* @retval none
* @attention 
*   R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
*   反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
****************************************************************************************************
*/

void Kalman_fir_init(Kalman_fir_t *kalman, float T_Q,float T_R)
{
    kalman->X_last = 0;
    kalman->P_last = 0;

    kalman->Q = T_Q;
    kalman->R = T_R;

    kalman->A = 1;
    kalman->B = 0;
    kalman->H = 1;
    kalman->X_prior = kalman->X_last;
}


/**
* @name   KalmanFilter_fir
* @brief  卡尔曼滤波器
* @param  kalman : 滤波器
* @param  z_data : 待滤波数据
* @retval 滤波后的数据
* @attention
*  A=1 B=0 H=1 I=1
*  z_data是系统输入，即带测量值
*  X_posterior 为后验估计，即最终输出
*  以下为卡尔曼滤波的五大核心公式     
*/

float KalmanFilter_fir(Kalman_fir_t *kalman,float z_data)
{
    //先验状态估计
    //x_hat[k]_prior = A * x_hat[k-1] + B * u[k-1]
    kalman->X_prior = kalman->A * kalman->X_last + kalman->B * kalman->U_last;
    
    //先验状态估计误差协方差矩阵
    //P[k]_prior = A * P[k-1] * A' +Q
    kalman->P_prior = kalman->A * kalman->P_last + kalman->Q;

    //计算卡尔曼增益
    //K[k] = ( P[k]_prior * Hm' ) / ( Hm * P[k]_prior * Hm' + R )
    kalman->Kalman_K = (kalman->P_prior * kalman->H) / (kalman->H * kalman->P_prior + kalman->R);

    //计算后验状态估计
    //x_hat[k] = x_hat[k]_prior + K[k] * ( z[k] - Hm * x_hat[k]_prior)
    kalman->X_posterior = kalman->X_prior + kalman->Kalman_K*(z_data - kalman->H * kalman->X_prior);

    //更新后验状态估计误差协方差矩阵
    //P[k] = ( I - K[k] * Hm )* P[k]_prior
    kalman->P_last = (1 - kalman->Kalman_K) * kalman->P_prior;

    return kalman->X_posterior;
} 

/****************************************************************************************************
* @name   Kalman_sec_init
* @brief  创建一个二阶阶卡尔曼滤波器
* @param  kalman : 滤波器
* @param  Qc_1: 对角矩阵1
* @param  Qc_2: 对角矩阵2  
* @retval none
* @attention 
*   R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
*   反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
*****************************************************************************************************
*/
void Kalman_sec_init(Kalman_sec_t *kalman, float Qc_1,float Qc_2)
{
    kalman->time_const = 0.002;
    //初始状态
    kalman->X_now.x = 0;
    kalman->X_now.v = 0;
    kalman->a = 0;

    //状态误差矩阵初始化，这个后面会迭代
    kalman->P.P[0][0] = 1;kalman->P.P[0][1] = 0;
    kalman->P.P[1][0] = 0;kalman->P.P[1][1] = 1;
   
 
    //状态转移矩阵初始化
    kalman->A[0][0] =1;kalman->A[0][1] =kalman->time_const;//time_const
    kalman->A[1][0] =0;kalman->A[1][1] =1;

    //输入矩阵初始化
    kalman->B[0] = 0.5f * (kalman->time_const) * (kalman->time_const);
    kalman->B[1] = kalman->time_const;

    //过程噪声初始化，对角初始化
    kalman->Q[0][0] = Qc_1;kalman->Q[0][1] = 0;
    kalman->Q[1][0] = 0   ;kalman->Q[1][1] = Qc_2;

    //测量噪声初始化，将R作为常数参考值
    kalman->R = 1;
}

/**
* @name   KalmanFilter_sen
* @brief  二阶卡尔曼滤波器
* @param  kalman   : 滤波器
* @param  distance : 待滤波数据
* @param  accel    : 系统输入/融合传感器数据（加速度）
* @retval 滤波/融合后的数据
* @attention
*  
*  以下为卡尔曼滤波的五大核心公式
*     二阶卡尔曼提供了位移，速度，到加速度的状态估计，同时可以将加速度计作为系统输入，轮式里程计作为测量值做数据融合检验
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

void KalmanFilter_sec(Kalman_sec_t *kalman,float distance,float accel)
{
		float dt=0.002f;

    //先验估计、状态预测
    kalman->X_prior.x =
        kalman->A[0][0] * kalman->X_now.x + kalman->A[0][1] * kalman->X_now.v + kalman->B[0] * accel;
    kalman->X_prior.v =
        kalman->A[1][0] * kalman->X_now.x + kalman->A[1][1] * kalman->X_now.v + kalman->B[1] * accel;

//    //先验状态误差估计
//    kalman->P_prior.P[0][0] = kalman->A[0][0] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->Q[0][0];
//    kalman->P_prior.P[0][1] = kalman->A[0][1] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->Q[0][1];
//    kalman->P_prior.P[1][0] = kalman->A[1][0] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->Q[1][0];
//    kalman->P_prior.P[1][1] = kalman->A[1][1] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->Q[1][1];

		// 先验状态误差估计
		kalman->P_prior.P[0][0] = kalman->A[0][0] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->A[0][1] * (kalman->A[0][0] * kalman->P.P[0][1] + kalman->A[0][1] * kalman->P.P[1][1]) + kalman->Q[0][0];
		kalman->P_prior.P[0][1] = kalman->A[0][0] * (kalman->A[0][0] * kalman->P.P[0][1] + kalman->A[0][1] * kalman->P.P[1][1]) + kalman->A[0][1] * (kalman->A[0][0] * kalman->P.P[0][0] + kalman->A[0][1] * kalman->P.P[1][0]) + kalman->Q[0][1];
		kalman->P_prior.P[1][0] = kalman->A[1][0] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->A[1][1] * (kalman->A[1][0] * kalman->P.P[0][1] + kalman->A[1][1] * kalman->P.P[1][1]) + kalman->Q[1][0];
		kalman->P_prior.P[1][1] = kalman->A[1][0] * (kalman->A[1][0] * kalman->P.P[0][1] + kalman->A[1][1] * kalman->P.P[1][1]) + kalman->A[1][1] * (kalman->A[1][0] * kalman->P.P[0][0] + kalman->A[1][1] * kalman->P.P[1][0]) + kalman->Q[1][1];


    //更新kalman_gain
    double S = kalman->P_prior.P[0][0] + kalman->R;
    kalman->K[0] = kalman->P_prior.P[0][0] / S;
    kalman->K[1] = kalman->P_prior.P[1][0] / S;

    //更新后验估计
    kalman->X_now.x = kalman->X_prior.x + kalman->K[0] * (distance - kalman->X_prior.x);
    kalman->X_now.v = kalman->X_prior.v + kalman->K[1] * (distance - kalman->X_prior.x);

    //更新后验估计误差协方差矩阵
    kalman->P.P[0][0] = (1 - kalman->K[0]) * kalman->P_prior.P[0][0];
    kalman->P.P[0][1] = (1 - kalman->K[0]) * kalman->P_prior.P[0][1];
    kalman->P.P[1][0] = (1 - kalman->K[1]) * kalman->P_prior.P[1][0];
    kalman->P.P[1][1] = (1 - kalman->K[1]) * kalman->P_prior.P[1][1];

    //输出向量
    kalman->out.x = kalman->X_now.x;
    kalman->out.v = kalman->X_now.v;

}
