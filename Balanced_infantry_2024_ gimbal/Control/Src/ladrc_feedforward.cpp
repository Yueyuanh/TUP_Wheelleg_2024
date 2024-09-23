#include "ladrc_feedforward.h"
#include "bsp_dwt.h"
/*wc:���������� b0:���������� Wo:�۲������� maxout:����������
	w:ǰ������    gain:ǰ������
  ��ʼ��ǰ�Ĳ�ʱ����gain=0		
	1.�ȷֱ��wo��wc���ó�5���º�15����
	2.����b0���޳����޸�Ƶ��
	3.����wo��wc(����3wc<=wo<=5wc)(��Ӳ������)
	4.���3΢��b0ʹ�޳����޸�Ƶ��
	5.����gain=1,��wc��������w
	6.΢��gain��w,����������
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
	time_cons = 0.002;//������
	
	init_flag = 1;
}

void LADRC_FDW_t::MaxOutInit(fp32 input_max_out)
{
	own_max_out = input_max_out;
}

fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //ʹ�����η���ɢ��
 
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
	if(abs(own_set - own_set_last) > 6.0f)//�������������,��ֹ���������ǰ����  //fabs
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
	
	//��ǰ����ladrc�㷨���Ȱ�ǰ��������0�����ÿ��������ٵ���ǰ������ǰ������һ�㲻����1.
	
	//��ױ��ַ���ɢ��������
	z2 += time_cons * (own_wo * own_wo) * (own_gyro - z1);
	z1 += time_cons * ((own_b0 * u) + z2 + (2 * own_wo) * (own_gyro - z1));
	u = (own_wc * own_wc * own_err + 2 * own_wc * (own_gain * dif1 - z1) - z2 + own_gain * dif2) / own_b0;
	LimitMax(u,own_max_out);
	
	return u;
}

fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w)
{ 
  //ʹ�����η���ɢ��
 
  lpf->u[0] = lpf->u[1];	
  lpf->u[1] = input; 
  lpf->y[0] = lpf->y[1];
  lpf->y[1] = (lpf->u[1]+lpf->u[0])/(2/time_cons + w)*w - lpf->y[0]*(w - 2/time_cons)/(2/time_cons + w);
  
  return lpf->y[1];
}
