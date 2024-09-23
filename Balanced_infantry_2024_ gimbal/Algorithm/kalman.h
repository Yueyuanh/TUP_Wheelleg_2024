#ifndef __KALMAN_H
#define __KALMAN_H

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class extKalman_t
{
	public:
		float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	  float B;
    float Q;
    float R;
    float H;
	
	  void KalmanCreate(float T_Q,float T_R);
	  float KalmanFilter(float dat);
};	
	
#endif

#ifdef __cplusplus
}	
#endif

#endif
