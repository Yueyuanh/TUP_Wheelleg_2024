#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "message_center.h"
#include "user_lib.h"
#include "robot_def.h"
#include "PID.h"
#include "motor.h"
#include "super_cap.h"
#include "operation_def.h"
#include "remote_control.h"
#include "bsp_can.h"


#define pi 3.1415926f
#define rad2angle 57.3f
#define angle2rad 1/57.3f

#define LEG_1 0.14f
#define LEG_2 0.28f
#define LEG_0 0.20f





//���̽ǶȻ�����
#define CHASSIS_MOTOR_YAW_PID_KP 14.0f
#define CHASSIS_MOTOR_YAW_PID_KI 0.0f 
#define CHASSIS_MOTOR_YAW_PID_KD 2.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_OUT 9.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_IOUT 0.2f			

//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN      0.01f      
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE   0.0f    
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define MAX_WHEEL_SPEED 4.0f	//���̵������ٶ�


#define RC_MODE_CHANGE         chassis.rc_ctrl->rc.s[0]==1
#define RC_CHASSIS_INIT        chassis.rc_ctrl->rc.s[0]==3
#define RC_ZERO_FORCE          chassis.rc_ctrl->rc.s[0]==2
#define RC_S_RIGHT						 chassis.rc_ctrl->rc.s[0]

#define RC_STAND_UP            chassis.rc_ctrl->rc.s[1]==1
#define RC_STAND               chassis.rc_ctrl->rc.s[1]==3
#define RC_STAND_DOWN          chassis.rc_ctrl->rc.s[1]==2
#define RC_S_LEFT							 chassis.rc_ctrl->rc.s[1]

#define RC_SIDEWAY						 chassis.rc_ctrl->rc.ch[0]
#define RC_FORWARD						 chassis.rc_ctrl->rc.ch[1]
#define RC_YAW						     chassis.rc_ctrl->rc.ch[2]
#define RC_PITCH						 	 chassis.rc_ctrl->rc.ch[3]
#define RC_ADDITION						 chassis.rc_ctrl->rc.ch[4]










//A�����C�嵥�����
#if BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS)

	#define MAX_WHEEL_SPEED 4.0f	//���̵������ٶ�

	#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
	#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
	//�����ٶ�ת��Ϊ����
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
	//�����ٶȻ�����
	#define CHASSIS_MOTOR_SPEED_PID_KP 8000.0f//10000.0f
	#define CHASSIS_MOTOR_SPEED_PID_KI 0.0f	
	#define CHASSIS_MOTOR_SPEED_PID_KD  2.0f//2.7f
	#define M3508_MOTOR_SPEED_PID_MAX_IOUT   1000.0f
	
#endif
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//���̵�����
enum Chassis_Wheel_Type_e
{
	LF, LB,  //0��1
	RB,	RF,  //2��3
};

class chassis_t
{
	private:
		/*****������ƽ����ת�ٶ��趨ֵ*****/


//		uint8_t chassis_init;	//0������   1����ʼ��
//		uint8_t leg_state;		//0: С��� 1������ 2��վ��
//		uint8_t restart;      //0: ��Ч   1��C������
//		uint8_t lying_flag;	  //0: ��Ч		1��ſ��ǰ����־λ

//		uint8_t sideway_flag; //0: ����   1������
//		uint8_t spin_flag;		//0������   1��С����		
//		uint8_t jump_flag;		//0������   1����Ծ
//		uint8_t chassis_power_limit;//��������

//		fp32 chassis_x_speed_set;		//���������ٶ��趨
//		fp32 chassis_y_speed_set;		//���̲����ٶ��趨
//		fp32 chassis_x_distance_set;//���������ƶ������趨
//		fp32 chassis_y_distance_set;//���̲����ƶ������趨
//		fp32 chassis_z_angle_set;		//���̽Ƕ��趨
//		fp32 chassis_real_power;		//����ʵʱ����
//		fp32 chassis_surplus_energy;//����ʣ������

//		uint8_t gimbal_update_flag; //��̨���±�־







		/*****�����ٶȣ�˳������Ϊ��LF��LB��RB��RF��*****/
		fp32 	wheel_speed[4];   							//�������ٶȽ���������ֵ��ٶȣ����֣�
		fp32 	speed_set[4],speed[4];						//�ٶ��趨ֵ�͵�ǰ�ٶ�ֵ
		int16_t symbol[4];  								//������ת�����־
		fp32 	chassis_max_power;							//���̹�������
		fp32 	chassis_power_buffer;						//���̻�������
		fp32  cap_v_out;									//�����ѹ
		fp32  real_scale_k;
	public:
		chassis_t();
		const RC_ctrl_t *rc_ctrl; //ң����ָ��
		PID_t chassis_angle_pid;	//���̸���Ƕ�pid

		CAN_Rx_Instance_t chassis_can;				//����ͨ��CANʵ��ָ��


		int16_t rc_addition_last;
		int16_t rc_forward_last;
		int16_t rc_sideway_last;
		int16_t rc_yaw_last;
		int16_t rc_pitch_last;
		bool_t  rc_s_right_last;




		/*****��Ϣʵ��*****/
		Chassis_Pub_Msg_t chassis_msg;  

		void SendMsgUpdate(); 								//������Ϣ����

	  /*****�������ú���*****/
		void chassis_feedback(void);
		void chassis_control_state(void);
		void chassis_control_loop(void);
		void chassis_player(void);

		void chassis_zero_control_set(void);//������������
		void chassis_sideway_control_set(void); //���̲���ģʽ
    void chassis_stand_down_mode(void);//С���ģʽ
		void chassis_stand_mode(void);//��������ģʽ
		void chassis_stand_up_mode(void);//վ��ģʽ
    void chassis_jump_mode(void);//��Ծģʽ
		void chassis_auto_length(void);//�ȳ�����Ӧ
};
	



#endif
void ChassisTask(void);
void ChassisInit(void);

#ifdef __cplusplus
}
#endif

#endif
