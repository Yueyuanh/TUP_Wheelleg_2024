#ifndef __REVOLVER_H
#define __REVOLVER_H

#include "struct_typedef.h"
#include "message_center.h"
#include "robot_def.h"
#include "motor.h"
#include "PID.h"

#define AN_BULLET 2*PI/WHOLE_CIRCLE_BULLET_NUM  //�������������Ӧת���Ƕ� = 2PI / ����һ��Ȧ�ӵ�����

#if BOARD_PLACE == ON_GIMBAL
//Ħ����
#define FIRC_SPEED_PID_KP 15.5f
#define FIRC_SPEED_PID_KI 0.0f
#define FIRC_SPEED_PID_KD 5.5f
#define FIRC_SPEED_PID_MAX_IOUT 5000.0f

//AM3508����
//#define FIRC_SPEED_PID_KP 3.0f
//#define FIRC_SPEED_PID_KI 20.0f
//#define FIRC_SPEED_PID_KD 0.0f
//#define FIRC_SPEED_PID_MAX_IOUT 2100.0f

//���̲���
#define STIR_SPEED_PID_KP 900.0f //1000.0f //3000  2650 1400 1000
#define STIR_SPEED_PID_KI 2.0f //10.0f //0.10f
#define STIR_SPEED_PID_KD 200.0f //0.0f //20.0f
#define STIR_SPEED_PID_MAX_OUT 9000.0f
#define STIR_SPEED_PID_MAX_IOUT 5000.0f

#define STIR_POSITION_PID_KP 50 //15.1f //40.0f//11.1f//0.0015f//0.0008f��0.003
#define STIR_POSITION_PID_KI 0.0f//0.5f //0.010f
#define STIR_POSITION_PID_KD 2500 //0.0f //10.0f
#define STIR_POSITION_PID_MAX_OUT 20.0f
#define STIR_POSITION_PID_MAX_IOUT 10.0f
	
#endif

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus 
//�趨�ٶ�ѡ��
enum FircSpeed_type_t
{
	LOW_SPEED,
	MID_SPEED,
	HIGH_SPEED,
};

class Revolver_t
{
	private:
		/*****���̱���*****/
		fp32 stir_angle,stir_angle_set;  //����λ���������͵�ǰ��
		fp32 stir_speed,stir_speed_set;	 //�����ٶ��������͵�ǰ��
		/*****Ħ����������*****/
		fp32 l_firc_speed_set[FIRC_STAGE],r_firc_speed_set[FIRC_STAGE];  //�ٶ�����
		fp32 l_firc_ramp_set[FIRC_STAGE],r_firc_ramp_set[FIRC_STAGE];		 //б���ٶ�����
		fp32 vfic_set;														//Ħ����ת������
	
#if FIRC_STAGE == 2 		//����Ħ����
		fp32 second_firc_speed_set,second_firc_speed_ramp_set;
#endif
		Stir_Mode_t last_stir_mode; 			//�ϴ�ģʽ
		uint16_t unfinish_time;           //���δ���ʱ��
		
	public:
		Revolver_t();
		/*****���ʵ��*****/
		DJIMotorInstance firc_l[FIRC_STAGE],firc_r[FIRC_STAGE],stir_motor_gun[REVOLVER_NUM];
		/*****��Ϣʵ��*****/
		Revolver_Pub_Msg_t revolver_msg;
		
		//
		int fire_time,fire_time_last;
		int fire_Hz;

		/*****��Ϣ���º���*****/
		void InfoUpdate();
		/*****���ƺ����ú���*****/
		void ControlSet();      			//��������			
		void FircControl();						//Ħ���ֿ���
		void StirControl();						//���̿���
		void FircSpeedSet();					//Ħ�����ٶ�����
		void DifferentFirSpeedSet(FircSpeed_type_t speed_type);		//��ͬ�ٶ�ѡ������
};

#endif
extern void RevolverInit(void);
extern void RevolverTask(void);
#ifdef __cplusplus
}
#endif

#endif
