#include "ladrc.h"

void LADRC_t::Init(fp32 wc,fp32 b0,fp32 wo,fp32 max_out)
{
	own_wc = wc;
	own_wo = wo;
	own_b0 = b0;
	
	if(max_out != NULL){
		own_max_out = max_out;
	}
	own_fdb = u = own_set = own_gyro = 0.0f;
	z1 = z2 = 0.0f;
	time_cons = 0.002;//������
	
	init_flag = 1;
}		

void LADRC_t::MaxOutInit(fp32 input_max_out)
{
	own_max_out = input_max_out;
}

fp32 LADRC_t::Calc(fp32 measure, fp32 set, fp32 gyro)
{
	fp32 err;
	err = set - measure;

	own_set = set;
	own_fdb = measure;
	own_err = rad_format(err);
	own_gyro = gyro;
	
	//��ױ��ַ���ɢ��������
  z2 += time_cons * (own_wo * own_wo) * (own_gyro - z1);
	z1 += time_cons * ((own_b0 * u) + z2 + (2 * own_wo) * (own_gyro - z1));
	u = (own_wc * own_wc * own_err - 2 * own_wc * z1 - z2) / own_b0;
	LimitMax(u,own_max_out);
	
	return u;
}



