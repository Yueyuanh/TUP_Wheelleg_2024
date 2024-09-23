#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "motor.h"
#include "message_center.h"
#include "robot_def.h"
#include "struct_typedef.h"
#include "vision.h"
#include "led.h"
#include "debug.h"
#include "imu.h"

//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.003f
#define GIMBAL_INIT_YAW_SPEED       0.00025f
//��ʼ�Ƕ�ֵ�趨
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f
//��������ʼ������
#define INIT_PARAM 0
#define NORMAL_PARAM 1
#define AUTO_PARAM 2

//��̨�������Ʋ���
#define YAW_NORMAL_WC 23
#define YAW_NORMAL_B0 0.007f
#define YAW_NORMAL_WO 90
#define YAW_NORMAL_W  25
#define YAW_NORMAL_GAIN 1

#define PIT_NORMAL_WC 30
#define PIT_NORMAL_B0 0.008f
#define PIT_NORMAL_WO 120
#define PIT_NORMAL_W  20
#define PIT_NORMAL_GAIN 1

//��̨��ʼ������
#define YAW_INIT_WC 25
#define YAW_INIT_B0 0.0058f
#define YAW_INIT_WO 100

#define PIT_INIT_WC 15
#define PIT_INIT_B0 0.0055f
#define PIT_INIT_WO 60

//��̨�������
#define YAW_AUTO_WC 16
#define YAW_AUTO_B0 0.0065f
#define YAW_AUTO_WO 80
#define YAW_AUTO_W  25
#define YAW_AUTO_GAIN 0.2f

#define PIT_AUTO_WC 17
#define PIT_AUTO_B0 0.0065f
#define PIT_AUTO_WO 85
#define PIT_AUTO_W  17
#define PIT_AUTO_GAIN 0.3f
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

class gimbal_t
{
	private:
		/*****��������Ϣ*****/
		fp32 roll_gyro,pitch_gyro,yaw_gyro;						//������ٶȡ��������ǻ�ȡ
		fp32 yaw_absolute_set_rad,pit_absolute_set_rad; 		//���ԽǶ��趨ֵ��rad��
		fp32 yaw_absolute_rad,pit_absolute_rad;					//���ԽǶ�ֵ��rad�����������ǻ�ȡ
		/*****��������Ϣ*****/
		fp32 add_yaw,add_pit;									//������
	
		/*****��ԽǶ��趨ֵ��Ϣ����ʹ��������ʱʹ�ã����������LADRC�����������ٶ���ϢӦ�ӵ�����API�л�ȡ*****/
		fp32 yaw_relative_set,pit_relative_set;  				//YAW�ᡢPIT����ԽǶ��趨ֵ
		/*****������Ϣ*****/
		uint16_t yaw_ecd_angle;                  				//YAW���ۻ�����ֵ����Ϊͬ����1��2������Ҫ���������ɾ��
	
		/*****�Ӿ��ǶȻ�ȡ����ʽѡ��*****/
#if Auto_Type == HANDLE_LPF
		lpf_type_def yaw_vision_lpf,pitch_vision_lpf;			//һ�׵�ͨ�˲��ṹ��
#elif Auto_Type == HANDLE_KALMAN
			
#endif
	public:
		gimbal_t();
		/*****��̨���Ե�Ԫ*****/
		Test_Module_t gimbal_test;
		/*****YAW�ᡢPIT����ʵ��*****/
		DJIMotorInstance yaw_motor;
		DJIMotorInstance pit_motor;
		/*****��Ϣʵ��*****/
		Gimbal_Pub_Msg_t gimbal_msg;
		
		/*****��Ϣ��ȡ����*****/
		void BasicInfoUpdate();
		void OperationInfoUpdate();
		void AutoInfoUpdate();
		/*****��ʼ������*****/
		void GimbalControllerInit(uint8_t type);
		void InitInfoUpdate();
		void JudgeInitState();
		/*****���ƺ���*****/
		void NormalControl();
		void RelativeControl();
		void ZeroForceControl();

};	
	














#endif
void GimbalInit(void);
void GimbalTask(void);
#ifdef __cplusplus
}
#endif

#endif
