#include "pid.h"
#include "bsp_dwt.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_t::Init(uint8_t mode,const fp32 PID[3],fp32 max_out,fp32 max_iout)
{
	own_mode = mode;
	own_Kp = PID[0];
	own_Ki = PID[1];
	own_Kd = PID[2];
	if(max_out != NULL){
		own_max_out = max_out;
	}
	own_max_iout = max_iout;
	Dbuf[0] = Dbuf[1] = Dbuf[2] =0.0f;
	error[0] = error[1] =error[2] = Pout = Dout = Iout = out =0.0f;
	
//	improve = PID_IMPROVE_NONE;
	init_flag = 1;
}

void PID_t::MaxOutInit(fp32 input_max_out)
{
	own_max_out = input_max_out;
}

//void PID_t::ImproveTypeInit(PID_Improvement_e input_improve)
//{
//	improve = input_improve;
//}

fp32 PID_t::Calc(fp32 measure,fp32 set)
{
#if DEBUG
	dt = DWT_GetDeltaT(&DWT_CNT); //获取两次pid计算的时间间隔
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = measure;
	error[0] = set - measure;
	//计算
	Pout = own_Kp * error[0];
	i_term = own_Ki * error[0] * dt;
	Dout = own_Kd * (error[0] - error[1]) / dt;
	
//	ForwardFeed();
	IntegralLimit();
	Iout += i_term;
	out = Pout + Iout + Dout ;//+ forward_out;
	LimitMax(out,own_max_out);
	
#else	
	error[2] = error[1];
	error[1] = error[0];
	own_set = set;
	own_fdb = measure;
	error[0] = set - measure;
	if(own_mode == PID_POSITION)
	{
		Pout = own_Kp * error[0];
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = error[0] - error[1];
		Dout = own_Kd *Dbuf[0];
		LimitMax(Iout,own_max_iout);
		out = Pout + Iout + Dout;
    LimitMax(out,own_max_out);
	}
	else if(own_mode == PID_DELTA)
	{
		Pout = own_Kp * (error[0] - error[1]);
		Iout += own_Ki * error[0];
		Dbuf[2] = Dbuf[1];
		Dbuf[1] = Dbuf[0];
		Dbuf[0] = (error[0] - 2.0f *error[1] + error[2]);
		Dout = own_Kd * Dbuf[0];
		out = Pout + Iout + Dout;
		LimitMax(out,own_max_out);
	}
#endif
	return out;
}

void PID_t::IntegralLimit()
{
	float temp_output, temp_iout;
  temp_iout = Iout + i_term;
  temp_output = Pout + Iout + Dout;
	
	if (abs(temp_output) > own_max_out){
    if (error[0] * Iout > 0){ // 积分却还在累积
      i_term = 0; // 当前积分项置零
    }
  }

  if (temp_iout > own_max_iout){
    i_term = 0;
    Iout = own_max_iout;
  }else if (temp_iout < -own_max_iout){
    i_term = 0;
    Iout = -own_max_iout;
  }
}

//void PID_t::ForwardFeed()
//{
//	forward_in = own_set;
//	forward_out = (1.7419f*(forward_in - last_forward_in)/dt + 4.0272f*forward_in) * forward_gain;
//	last_forward_in = forward_in;
//}

////梯形积分
//void PID_t::TrapezoidIntergral()
//{
//	i_term = own_Ki * ((error[0]+error[1])*0.5f) * dt;
//}

//变速积分
void PID_t::ChangingIntergrationRate()
{
	
}

void PID_t::Clear(void)
{
	error[0] = error[1] = error[2] = 0.0f;
	Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
	out = Pout = Iout = Dout =0.0f;
//	own_fdb = own_set = 0.0f;
}


	
