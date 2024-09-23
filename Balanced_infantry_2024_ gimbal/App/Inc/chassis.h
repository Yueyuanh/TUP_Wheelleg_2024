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





//底盘角度环参数
#define CHASSIS_MOTOR_YAW_PID_KP 14.0f
#define CHASSIS_MOTOR_YAW_PID_KI 0.0f 
#define CHASSIS_MOTOR_YAW_PID_KD 2.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_OUT 9.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_IOUT 0.2f			

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN      0.01f      
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE   0.0f    
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define MAX_WHEEL_SPEED 4.0f	//底盘电机最大速度


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










//A板或者C板单板控制
#if BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS)

	#define MAX_WHEEL_SPEED 4.0f	//底盘电机最大速度

	#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
	#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
	//四轮速度转化为车体
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
	//底盘速度环参数
	#define CHASSIS_MOTOR_SPEED_PID_KP 8000.0f//10000.0f
	#define CHASSIS_MOTOR_SPEED_PID_KI 0.0f	
	#define CHASSIS_MOTOR_SPEED_PID_KD  2.0f//2.7f
	#define M3508_MOTOR_SPEED_PID_MAX_IOUT   1000.0f
	
#endif
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//底盘电机编号
enum Chassis_Wheel_Type_e
{
	LF, LB,  //0、1
	RB,	RF,  //2、3
};

class chassis_t
{
	private:
		/*****机器人平移旋转速度设定值*****/


//		uint8_t chassis_init;	//0：无力   1：初始化
//		uint8_t leg_state;		//0: 小板凳 1：正常 2：站立
//		uint8_t restart;      //0: 无效   1：C板重启
//		uint8_t lying_flag;	  //0: 无效		1：趴着前进标志位

//		uint8_t sideway_flag; //0: 正对   1：侧身
//		uint8_t spin_flag;		//0：正常   1：小陀螺		
//		uint8_t jump_flag;		//0：正常   1：跳跃
//		uint8_t chassis_power_limit;//功率上限

//		fp32 chassis_x_speed_set;		//底盘正向速度设定
//		fp32 chassis_y_speed_set;		//底盘侧向速度设定
//		fp32 chassis_x_distance_set;//底盘正向移动距离设定
//		fp32 chassis_y_distance_set;//底盘侧向移动距离设定
//		fp32 chassis_z_angle_set;		//底盘角度设定
//		fp32 chassis_real_power;		//底盘实时功率
//		fp32 chassis_surplus_energy;//底盘剩余能量

//		uint8_t gimbal_update_flag; //云台更新标志







		/*****四轮速度（顺序依次为：LF、LB、RB、RF）*****/
		fp32 	wheel_speed[4];   							//机器人速度解算出单个轮的速度（四轮）
		fp32 	speed_set[4],speed[4];						//速度设定值和当前速度值
		int16_t symbol[4];  								//四轮旋转方向标志
		fp32 	chassis_max_power;							//底盘功率上限
		fp32 	chassis_power_buffer;						//底盘缓冲能量
		fp32  cap_v_out;									//输出电压
		fp32  real_scale_k;
	public:
		chassis_t();
		const RC_ctrl_t *rc_ctrl; //遥控器指针
		PID_t chassis_angle_pid;	//底盘跟随角度pid

		CAN_Rx_Instance_t chassis_can;				//底盘通信CAN实例指针


		int16_t rc_addition_last;
		int16_t rc_forward_last;
		int16_t rc_sideway_last;
		int16_t rc_yaw_last;
		int16_t rc_pitch_last;
		bool_t  rc_s_right_last;




		/*****信息实例*****/
		Chassis_Pub_Msg_t chassis_msg;  

		void SendMsgUpdate(); 								//发送信息更新

	  /*****控制设置函数*****/
		void chassis_feedback(void);
		void chassis_control_state(void);
		void chassis_control_loop(void);
		void chassis_player(void);

		void chassis_zero_control_set(void);//底盘无力控制
		void chassis_sideway_control_set(void); //底盘侧向模式
    void chassis_stand_down_mode(void);//小板凳模式
		void chassis_stand_mode(void);//正常操作模式
		void chassis_stand_up_mode(void);//站立模式
    void chassis_jump_mode(void);//跳跃模式
		void chassis_auto_length(void);//腿长自适应
};
	



#endif
void ChassisTask(void);
void ChassisInit(void);

#ifdef __cplusplus
}
#endif

#endif
