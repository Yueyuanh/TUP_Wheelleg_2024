#ifndef FILTER_H
#define FILTER_H

#include "struct_typedef.h"

/**
  * @brief          梯度微分器结构体
  * @author        	Lu Yi
  */

typedef struct{   
	fp32 y[2];
	fp32 u[2];
	fp32 last_input;
} differ_type_def;

void differentiator_init(differ_type_def *differ);
float differ_calc(differ_type_def *differ ,float input,float bandwidth,float time_cons,float cutoff);


/**
  * @brief          一阶低通滤波器结构体
  * @author         
  * @param[in]      微分结构体
  * @param[in]      控制带宽
  * @param[in]      时间常数
  * @param[in]      输入
  * @retval         微分
  */

typedef struct
{
	fp32 input;			    //输入数据
	fp32 output;		    //输出数据
	fp32 num;			      //滤波参数
	fp32 frame_period;	//时间常数

} LPF_type_def;

void LPF_init(LPF_type_def *LPF, fp32 frame_period,fp32 num);
fp32 LPF_calc(LPF_type_def *LPF, fp32 input);



#endif
