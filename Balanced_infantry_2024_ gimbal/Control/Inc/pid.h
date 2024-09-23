#ifndef __PID_H
#define __PID_H

#include "struct_typedef.h"
#include "user_lib.h"
#ifdef __cplusplus
extern "C"{
#endif

#define DEBUG 0
#ifdef __cplusplus
enum PID_MODE
{
	PID_POSITION = 0,
	PID_DELTA,
};	

//pid改进方式选择
enum PID_Improvement_e
{
	PID_IMPROVE_NONE = 0x00,                // 0000 0000
  PID_Integral_Limit = 0x01,              // 0000 0001
  PID_Derivative_On_Measurement = 0x02,   // 0000 0010
  PID_Trapezoid_Intergral = 0x04,         // 0000 0100
  PID_Proportional_On_Measurement = 0x08, // 0000 1000
  PID_OutputFilter = 0x10,                // 0001 0000
  PID_ChangingIntegrationRate = 0x20,     // 0010 0000
  PID_DerivativeFilter = 0x40,            // 0100 0000
  PID_ErrorHandle = 0x80,                 // 1000 0000
};	
	
class PID_t
{
	public:
		uint8_t own_mode;
		uint8_t init_flag;
		fp32 own_Kp;
	  fp32 own_Ki;
	  fp32 own_Kd;
	  fp32 own_max_out;  //最大输出
	  fp32 own_max_iout; //最大积分输出
//		fp32 dead_band;
    fp32 own_set; //设定值
    fp32 own_fdb; //当前值
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
		fp32 i_term;
//		fp32 last_ref;
	
		//improve parameter
//		PID_Improvement_e improve;
	  
		uint32_t DWT_CNT;
    float dt;
//		
//		float forward_in,last_forward_in;
//		float forward_out,forward_gain;
		
		void IntegralLimit();
		void ForwardFeed();
		void TrapezoidIntergral();
		void ChangingIntergrationRate();
		void Init(uint8_t mode,const fp32 PID[3],fp32 max_out,fp32 max_iout);
		void MaxOutInit(fp32 input_max_out);
		void ImproveTypeInit(PID_Improvement_e input_improve);
	  fp32 Calc(fp32 measure,fp32 set);
	  void Clear(void);
};

#endif	
	
#ifdef __cplusplus
}
#endif

#endif
