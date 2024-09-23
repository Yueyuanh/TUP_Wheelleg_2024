#include "ladrc_feedforward.h"
#include "bsp_dwt.h"
/*wc:控制器带宽 b0:控制器增益 Wo:观测器带宽 maxout:电机最大输入
	w:前馈带宽    gain:前馈增益
  开始调前四步时设置gain=0		
	1.先分别把wo和wc设置成5上下和15上下
	2.调整b0至无超调无高频振荡
	3.增加wo和wc(保持3wc<=wo<=5wc)(非硬性限制)
	4.配合3微调b0使无超调无高频振荡
	5.设置gain=1,在wc附近调整w
	6.微调gain和w,至性能最优
	*/
void LADRC_FDW_t::Init(fp32 ladrc_fdw_param[5],fp32 max_out)
{
	own_wc = ladrc_fdw_param[0];
	own_b0 = ladrc_fdw_param[1];
	own_wo = ladrc_fdw_param[2];
	own_w  = ladrc_fdw_param[3];
	own_gain = ladrc_fdw_param[4];
	
	if(max_out != NULL){
		own_max_out = max_out;
	}
	
	own_fdb = u = own_set = own_gyro = 0.0f;
	z1 = z2 = 0.0f;
	time_cons = 0.002;//采样率
	
	init_flag = 1;
}

void LADRC_FDW_t::MaxOutInit(fp32 input_max_out)
{
	own_max_out = input_max_out;
}

fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //使用梯形法离散化
 
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  
  return differ->y[1];
}

fp32 LADRC_FDW_t::FDW_Calc(fp32 measure, fp32 set, fp32 gyro)
{
//	time_cons = DWT_GetDeltaT(&DWT_CNT);
	own_set_last = own_set;
	own_set = set;
	own_fdb = measure;
	own_err = rad_format(set - measure);
	own_gyro = gyro;
	if(abs(own_set - own_set_last) > 6.0f)//如果发生了跳变,防止产生过大的前馈量  //fabs
	{
		differ1.y[0] = 0;
		differ1.y[1] = 0;
		differ1.u[0] = own_set;
		differ1.u[1] = own_set;
		
		differ2.y[0] = 0;
		differ2.y[1] = 0;
		differ2.u[0] = 0;
		differ2.u[1] = 0;
	}
	dif1 = differentiator(&differ1,own_w,time_cons,set);
	dif2 = differentiator(&differ2,own_w,time_cons,dif1);
	
	//带前馈的ladrc算法，先把前馈增益置0，调好控制器，再调节前馈器，前馈增益一般不大于1.
	
	//零阶保持法离散化积分器
	z2 += time_cons * (own_wo * own_wo) * (own_gyro - z1);
	z1 += time_cons * ((own_b0 * u) + z2 + (2 * own_wo) * (own_gyro - z1));
	u = (own_wc * own_wc * own_err + 2 * own_wc * (own_gain * dif1 - z1) - z2 + own_gain * dif2) / own_b0;
	LimitMax(u,own_max_out);
	
	return u;
}

fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w)
{ 
  //使用梯形法离散化
 
  lpf->u[0] = lpf->u[1];	
  lpf->u[1] = input; 
  lpf->y[0] = lpf->y[1];
  lpf->y[1] = (lpf->u[1]+lpf->u[0])/(2/time_cons + w)*w - lpf->y[0]*(w - 2/time_cons)/(2/time_cons + w);
  
  return lpf->y[1];
}
