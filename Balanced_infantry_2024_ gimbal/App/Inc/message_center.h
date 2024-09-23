#ifndef __MESSAGE_CENTER_H
#define __MESSAGE_CENTER_H

#include "struct_typedef.h"
#include "led.h"
//�������
//#define MAX_SUBJECT 4

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	
enum Msg_Type_e 
{
	SYSPUB,
	GIMBALPUB,
	INSPUB,
	CHASSISPUB,
	REVOLVERPUB,
	UIPUB,
	MONITORPUB,
	//add_more
	
	TOTALNUM,
};

enum MessageErrorCode
{
	NO_MSG_ERROR,
	INVALID_MSG_TYPE,
	NULL_POINTER,
};

//ģʽ��Ϣ
enum Control_Mode_t
{
	ZERO_FORCE = 0,	DT7_MISSING, INIT			,	NORMAL, RELATIVE_ANGLE,
	SPIN 					, AUTO				, SPIN_AUTO	,
	NO_FOLLOW_YAW ,
};

enum Firc_Mode_t
{
	FORBID_FIRE, CLOSE , OPEN,
};

enum Stir_Mode_t
{
	NO_MOVE, ALLOW_MOVE,
	REVERSE_MOVE, //��ת
	SHOOT_1_BULLET,  //����
	SHOOT_3_BULLET, //����
	SHOOT_FIXED_FIRE_RATE, //����
};

//���̱�־λ
typedef struct
{
	uint8_t up_thumb_flag,down_thumb_flag;
	//��̨����
	uint8_t auto_aim_flag,turn_round_flag;
	//�˶�����
	uint8_t super_cap_flag,spin_flag;
	//��������
	uint8_t heat_limit_flag,auto_fir_flag,fir_flag;
	//��������
	uint8_t	reset_flag,side_flag;//����
	uint8_t ster_flag;//����

	//����״̬����
}Key_Flag_t;

typedef struct
{
	Control_Mode_t mode;
	Firc_Mode_t fir_mode;
	Stir_Mode_t stir_mode;
	Key_Flag_t key_flag;

	//�趨ֵ
	fp32 add_yaw,add_pit;
	fp32 vx_set,vy_set;
	uint8_t change_mode_flag;
	
	//����ģʽ
	uint8_t rc_chassis_init;	//0������   1����ʼ��
	uint8_t rc_leg_state;		//0: С��� 1������ 2��վ��
	uint8_t rc_restart;      //0: ��Ч   1��C������
	uint8_t rc_lying_flag;	  //0: ��Ч		1��ſ��ǰ����־λ

	uint8_t rc_sideway_flag; //0: ����   1������
	uint8_t rc_spin_flag;		//0������   1��С����		
	uint8_t rc_jump_flag;		//0������   1����Ծ
	uint8_t rc_super_flag;		//0������   1: ������������

	fp32 chassis_phi0,chassis_phi1,chassis_phi2,chassis_phi3,chassis_phi4,chassis_phi5;
	fp32 L0;

	uint8_t rc_chassis_power_limit;//��������

  fp32 offset_firc_v;

}Sys_Pub_Msg_t;

typedef struct
{
	fp32 yaw_relative_angle,pit_relative_angle;
	fp32 yaw_num,yaw_relative_angle_sum;
	fp32 yaw_relative_angle_last;
}Gimbal_Pub_Msg_t;

typedef struct
{
		uint8_t chassis_init;	//0������   1����ʼ��
		uint8_t leg_state;		//0: С��� 1������ 2��վ��
		uint8_t restart;      //0: ��Ч   1��C������
		uint8_t lying_flag;	  //0: ��Ч		1��ſ��ǰ����־λ

		uint8_t sideway_flag; //0: ����   1������
		uint8_t spin_flag;		//0������   1��С����		
		uint8_t jump_flag;		//0������   1����Ծ
		uint8_t chassis_power_limit;//��������

		fp32 chassis_x_speed_set;		//���������ٶ��趨
		fp32 chassis_y_speed_set;		//���̲����ٶ��趨
		fp32 chassis_x_distance_set;//���������ƶ������趨
		fp32 chassis_y_distance_set;//���̲����ƶ������趨
		fp32 chassis_z_angle_set;		//���̽Ƕ��趨
		fp32 chassis_z_num;					//������תȦ��
		fp32 chassis_real_power;		//����ʵʱ����
		fp32 chassis_surplus_energy;//����ʣ������

		uint8_t gimbal_update_flag; //��̨���±�־

		//���ֱ�־λ
		bool_t rc_ctrl_down;
		bool_t rc_ctrl_up;

}Chassis_Pub_Msg_t;

typedef struct
{
	uint8_t rest_heat;  //ʣ������
	fp32 shoot_rate;  //��Ƶ
	uint8_t if_stuck;
	uint8_t shoot_finish,reserve_time;
	int16_t firc_speed_set;

}Revolver_Pub_Msg_t;

typedef struct
{
	fp32 x_coordinate,y_coordinate;
	fp32 pre_x_coordinate,pre_y_coordinate;
	fp32 follow_radius;
}UI_Pub_Msg_t;

typedef struct
{
	Led_State_e state;
	
}Monitor_Pub_Msg_t;





class Message_Center_t
{
	public:
		//������ʼ���Ľṹ��ָ��
		Sys_Pub_Msg_t *sys_msg;
		Gimbal_Pub_Msg_t *gimbal_msg;
		Chassis_Pub_Msg_t *chassis_msg;
		Revolver_Pub_Msg_t *revolver_msg;
		UI_Pub_Msg_t *ui_msg;
		Monitor_Pub_Msg_t *monitor_msg;

		//ָ���ʼ��
		MessageErrorCode PointerInit(void *msg,Msg_Type_e type);
		bool CheckPointerEmpty();
		void SetLedState(Led_State_e input_state,uint8_t priority);
};


bool CheckMessageCenter();


Message_Center_t *CenterPointer();


Sys_Pub_Msg_t *SysPointer();
Gimbal_Pub_Msg_t *GimbalPointer();
Chassis_Pub_Msg_t *ChassisPointer();
Revolver_Pub_Msg_t *RevolverPointer();
UI_Pub_Msg_t *UiPointer();
Monitor_Pub_Msg_t *MonitorPointer(); 

#endif
	
#ifdef __cplusplus
}
#endif

#endif
 
