#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "message_center.h"
#include "remote_control.h"
#include "ladrc_feedforward.h"
#include "operation_def.h"
#include "robot_def.h"


#define RC_MODE_CHANGE         sys.system_rc_ctrl->rc.s[0]==1
#define RC_CHASSIS_INIT        sys.system_rc_ctrl->rc.s[0]==3
#define RC_ZERO_FORCE          sys.system_rc_ctrl->rc.s[0]==2
#define RC_S_RIGHT						 sys.system_rc_ctrl->rc.s[0]

#define RC_STAND_UP            sys.system_rc_ctrl->rc.s[1]==1
#define RC_STAND               sys.system_rc_ctrl->rc.s[1]==3
#define RC_STAND_DOWN          sys.system_rc_ctrl->rc.s[1]==2
#define RC_S_LEFT							 sys.system_rc_ctrl->rc.s[1]

#define RC_SIDEWAY						 sys.system_rc_ctrl->rc.ch[0]
#define RC_FORWARD						 sys.system_rc_ctrl->rc.ch[1]
#define RC_YAW						     sys.system_rc_ctrl->rc.ch[2]
#define RC_PITCH						 	 sys.system_rc_ctrl->rc.ch[3]
#define RC_ADDITION						 sys.system_rc_ctrl->rc.ch[4]




#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
extern int16_t test_current;
class system_t
{
	private:
		Control_Mode_t last_mode; //��һ�ε�ģʽ
		/*****��ͨ�˲��ṹ��*****/
		lpf_type_def yaw_lpf;  
	  lpf_type_def pitch_lpf;
	
		/*****ң����������*****/
		fp32 vx_set_channel, vy_set_channel;
		fp32 vx_ramp_set,vy_ramp_set;
		fp32 mouse_yaw, mouse_pitch;
		int16_t yaw_channel, pitch_channel;
		int8_t key_yaw;
		int8_t steering_mode;
	
		/*****���ģʽ����*****/
		uint8_t debug_shoot_flag,debug_shoot_mode_set; //����ʹ�ñ���
		uint8_t key_shoot_mode_set,key_shoot_flag; //���̿��Ʊ���
		uint16_t press_shoot_time; //������������ʱ��
	
		/*****����״̬����*****/
		uint16_t postive_thumb_wheel_state_time,negative_thumb_wheel_state_time; //����״ֵ̬ʱ��
		uint8_t postive_thumb_wheel_state,negative_thumb_wheel_state;	//��������״ֵ̬

		/*****���˱�������*****/
		uint16_t stand_up_state_time; //�ϲ�״ֵ̬ʱ��
		uint16_t stand_up_state_time_last; //�ϲ�״ֵ̬ʱ��
		uint16_t stand_state,stand_state_last;

		/*****�����������*****/
		uint8_t last_mouse_l_press;
		uint8_t last_key_g_press;
		uint8_t last_key_shift_press;
		uint8_t last_key_z_press;
		uint8_t last_key_x_press;
		uint8_t last_key_q_press;
		uint8_t last_key_e_press;
		uint8_t last_key_c_press;
		uint8_t last_key_v_press;
		uint8_t last_key_f_press;
		uint8_t last_key_ctrl_press;
		uint8_t last_sideway_rc;

		



	public:
		system_t();
		const RC_ctrl_t *system_rc_ctrl; //ң����ָ��
		Sys_Pub_Msg_t sys_pub; //������Ϣʵ��
	
		/***ģʽ����***/
		void RobotModeSet(); //��������
	  void ThumbWheelModeSet(); //״̬����
		void KeyBoardModeSet();
		/***�����Ϳ���������***/
		void CalControlQuantity();
		void KeyBoardQuantitySet();
		void RemoteQuantitySet();
};
	
#endif

void SysInit(void);
void SystemTask(void);	
static void C_RESET(void);

#ifdef __cplusplus
}
#endif

#endif	

