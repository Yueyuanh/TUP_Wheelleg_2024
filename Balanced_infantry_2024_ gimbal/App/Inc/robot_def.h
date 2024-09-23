#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "board_def.h"

#define OFF_SET 0
#define ON_SET 1
/****************����������*******************/
#define TASK_INIT_TIME 300  //���������ʼ��ʱ��

/***���䵯������***/
#define BULLET_17 0  //17mm����
#define BULLET_42 1  //42mm����
#define NO_BULLET 2  //�޵���
#define ROBOT_SHOOT_TYPE BULLET_17

/***����ʹ�÷�ʽ***/
#define HANDLE_LPF 0     //һ�ڵ�ͨ�˲�
#define HANDLE_KALMAN 1  //�������˲�
#define Auto_Type HANDLE_LPF

/***���鶨��***/
#define All_Mecanum 0         //����
#define All_Omnidirectional 1 //ȫ����
#define CHASSIS_TYPE All_Mecanum

/***�������ݿ������***/
#define POWER_CONTROL_TYPE OFF_SET

/****************��̨����*******************/
#define YAW_OFFSET 5452   //YAW���趨��ֵ
#define PIT_OFFSET 6050   //PIT���趨��ֵ

#define MOTOR_TO_YAW_RADIO 1 //�����YAW��Ƕ�ת����
#define MOTOR_TO_PIT_RADIO 1 //�����PIT��Ƕ�ת����
#define PIT_TRANSMISSION_RADIO 1  //PIT�ᴫ����

#define MAX_YAW_RELATIVE 3.141f  //YAW�������λ//MOTOR_ECD_TO_RAD*2048.0f
#define MIN_YAW_RELATIVE -3.141f //YAW����С��λ//-MOTOR_ECD_TO_RAD*2048.0f

#define MAX_PIT_RELATIVE 0.45f   //PIT�������λ
#define MIN_PIT_RELATIVE -0.51f  //PIT����С��λ
/****************��������*******************/
#define NORMAL_MAX_CHASSIS_SPEED_X 600  //�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 600  //�����˶��������ƽ���ٶ�

// �����˵����޸ĵĲ���,��λΪmm(����)������������������δʹ��
#define WHEEL_BASE 350              // �������(ǰ�����˷���)
#define TRACK_WIDTH 300             // �����־�(����ƽ�Ʒ���)
#define CENTER_GIMBAL_OFFSET_X 0    // ��̨��ת���ľ���̼������ĵľ���,ǰ����,��̨λ��������ʱĬ����Ϊ0
#define CENTER_GIMBAL_OFFSET_Y 0    // ��̨��ת���ľ���̼������ĵľ���,���ҷ���,��̨λ��������ʱĬ����Ϊ0
#define RADIUS_WHEEL 60             // ���Ӱ뾶
#define REDUCTION_RATIO_WHEEL 19.0f // ������ٱ�,��Ϊ�������������ת�ӵ��ٶȶ������������ٶȹ������ת��

/****************��������*******************/
#define USE_SENSOR OFF_SET //���̴��Ƿ�ʹ�ô�����

/***����һ��Ȧ�ӵ�����***/
#define WHOLE_CIRCLE_BULLET_NUM 8 
/***������Ƶ����***/

#define LOW_FREQUENCE 2
#define MID_FREQUENCE 5
#define HIGH_FREQUENCE 20
#define INIT_FREQUENCE HIGH_FREQUENCE

/***������������***/
#define REVOLVER_NUM 1  //����
#define MAIN_REVOLVER 0 //������id
#define ASSIST_REVOLVER 1  //��������id

/***Ħ������������***/
#define FIRC_STAGE 1  	//Ħ���ּ���
#define FIRST_STAGE 0		//һ��Ħ����
#define SECONE_STAGE 1  //����Ħ����

/***Ħ����ת������***/
#define FIRST_STAGE_RPM_SET  8200 //һ��ת������
#define SECOND_STAGE_RPM_SET 2000 //����Ħ��������

/***���ȼ������趨�������ã�***/
#define LOW_RPM_SET 8200   //��ת��//5000
#define MID_RPM_SET 7000   //��ת��
#define HIGH_RPM_SET 9000  //��ת��
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//
	
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif
