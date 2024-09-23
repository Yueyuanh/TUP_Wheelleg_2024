#ifndef __LADRC_FEEDFORWARD_H
#define __LADRC_FEEDFORWARD_H

#include "struct_typedef.h"
#include "ladrc.h"

#ifdef __cplusplus
extern "C"{
#endif
		
#ifdef __cplusplus
typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} differ_type_def;	

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} lpf_type_def;

class	LADRC_FDW_t:public LADRC_t 
{
  private:
	  differ_type_def differ1;
	  differ_type_def differ2;
	  fp32 dif1;
	  fp32 dif2; 
	  fp32 own_w;//前馈带宽
	  fp32 own_gain;//前馈增益
		
	  //输入
	  fp32 own_set_last;
	
		uint32_t DWT_CNT;
    float dt;
	public:
		void Init(fp32 ladrc_fdw_param[5],fp32 max_out);
		void MaxOutInit(fp32 input_max_out);
	  fp32 FDW_Calc(fp32 measure, fp32 set, fp32 gyro);
};
	
extern fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w);
#endif

#ifdef __cplusplus
}
#endif

#endif
