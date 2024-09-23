#ifndef __REVOLVER_H
#define __REVOLVER_H

#include "struct_typedef.h"
#include "message_center.h"
#include "robot_def.h"
#include "motor.h"
#include "PID.h"

#define AN_BULLET 2*PI/WHOLE_CIRCLE_BULLET_NUM  //单发拨盘输出轴应转动角度 = 2PI / 拨盘一整圈子弹个数

#if BOARD_PLACE == ON_GIMBAL
//摩擦轮
#define FIRC_SPEED_PID_KP 15.5f
#define FIRC_SPEED_PID_KI 0.0f
#define FIRC_SPEED_PID_KD 5.5f
#define FIRC_SPEED_PID_MAX_IOUT 5000.0f

//AM3508参数
//#define FIRC_SPEED_PID_KP 3.0f
//#define FIRC_SPEED_PID_KI 20.0f
//#define FIRC_SPEED_PID_KD 0.0f
//#define FIRC_SPEED_PID_MAX_IOUT 2100.0f

//拨盘参数
#define STIR_SPEED_PID_KP 900.0f //1000.0f //3000  2650 1400 1000
#define STIR_SPEED_PID_KI 2.0f //10.0f //0.10f
#define STIR_SPEED_PID_KD 200.0f //0.0f //20.0f
#define STIR_SPEED_PID_MAX_OUT 9000.0f
#define STIR_SPEED_PID_MAX_IOUT 5000.0f

#define STIR_POSITION_PID_KP 50 //15.1f //40.0f//11.1f//0.0015f//0.0008f、0.003
#define STIR_POSITION_PID_KI 0.0f//0.5f //0.010f
#define STIR_POSITION_PID_KD 2500 //0.0f //10.0f
#define STIR_POSITION_PID_MAX_OUT 20.0f
#define STIR_POSITION_PID_MAX_IOUT 10.0f
	
#endif

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus 
//设定速度选择
enum FircSpeed_type_t
{
	LOW_SPEED,
	MID_SPEED,
	HIGH_SPEED,
};

class Revolver_t
{
	private:
		/*****拨盘变量*****/
		fp32 stir_angle,stir_angle_set;  //拨盘位置设置量和当前量
		fp32 stir_speed,stir_speed_set;	 //拨盘速度设置量和当前量
		/*****摩擦轮设置量*****/
		fp32 l_firc_speed_set[FIRC_STAGE],r_firc_speed_set[FIRC_STAGE];  //速度设置
		fp32 l_firc_ramp_set[FIRC_STAGE],r_firc_ramp_set[FIRC_STAGE];		 //斜坡速度设置
		fp32 vfic_set;														//摩擦轮转速设置
	
#if FIRC_STAGE == 2 		//二级摩擦轮
		fp32 second_firc_speed_set,second_firc_speed_ramp_set;
#endif
		Stir_Mode_t last_stir_mode; 			//上次模式
		uint16_t unfinish_time;           //射击未完成时间
		
	public:
		Revolver_t();
		/*****电机实例*****/
		DJIMotorInstance firc_l[FIRC_STAGE],firc_r[FIRC_STAGE],stir_motor_gun[REVOLVER_NUM];
		/*****信息实例*****/
		Revolver_Pub_Msg_t revolver_msg;
		
		//
		int fire_time,fire_time_last;
		int fire_Hz;

		/*****信息更新函数*****/
		void InfoUpdate();
		/*****控制和设置函数*****/
		void ControlSet();      			//控制设置			
		void FircControl();						//摩擦轮控制
		void StirControl();						//拨盘控制
		void FircSpeedSet();					//摩擦轮速度设置
		void DifferentFirSpeedSet(FircSpeed_type_t speed_type);		//不同速度选择设置
};

#endif
extern void RevolverInit(void);
extern void RevolverTask(void);
#ifdef __cplusplus
}
#endif

#endif
