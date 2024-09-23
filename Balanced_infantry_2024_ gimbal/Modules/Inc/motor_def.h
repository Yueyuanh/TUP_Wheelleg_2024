#ifndef __MOTOR_DEF_H
#define __MOTOR_DEF_H

#include "pid.h"
#include "ladrc_feedforward.h"
#include "device_monitor.h"

//���ص������
#define TOTAL_MOTOR_SUM 16

//�����ٱȶ��壨�ó˷����������    5.2: -> 57/11   3.7: -> 63/17    14:->3969/289  19:->3591/187  27:->3249/121 71: -> 226223/3179
#define ONE_FOURTEEN 0.07142857f  //1��14
#define ONE_NINETEEN 0.05263158f  //1��19
#define ONE_OUT_OF_THIRTYSIX 0.02777778f  //1��36
#define ONE_OUT_OF_SEVENTYONE 0.01408451f  //1��71
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
//�����ת����--(GM6020����ʱ��Ϊ��)
enum Motor_Rotate_Direction_e
{
	STOP = 0,	POSITIVE_DIRECT = 1,
	NEGATIVE_DIRECT = -1,
};

//�������
enum Motor_Type_e
{
	NO_MOTOR,
	M3508,
	AM3508,
	GM6020,
	M2006,
};

//����������
enum Control_Type_e
{
	OPEN_LOOP,
	//PID
	SINGLE_LOOP,
	CASCADE_LOOP,
	//LADRC
	LADRC_CONTROL,
	LADRC_FDW_CONTROL,
};

//����������
enum Reduction_Drive_Type_e
{
	DIRECT_DRIVE,
	RATIO_1_TO_14,
	RATIO_1_TO_19,
	RATIO_1_TO_36,
	RATIO_1_TO_71,
};

//������
typedef struct
{
	//PID
	PID_t speed_PID;
	PID_t angle_PID;
	//LADRC
	LADRC_t ladrc;
	LADRC_FDW_t ladrc_fdw;
	//���⻷����
	fp32 target_value,now_value,pid_speed;
	int16_t send_current;
}Motor_Controller_s;
		
#endif
	
#ifdef __cplusplus
}
#endif

#endif
